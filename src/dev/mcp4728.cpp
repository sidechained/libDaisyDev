#include "dev/mcp4728.h"

using namespace daisy;

void Mcp4728::Init(Config config)
{
    i2c_       = config.i2c;
    // I2CHandle expects 7-bit addresses (it performs the left-shift internally),
    // so store the raw 7-bit address here.
    addr_8bit_ = config.address;

    // Specific initialization settings (Volatile only)
    uint8_t vref = 0;   // 0 = VDD
    uint8_t pd1  = 0;   // Power-down bit 1
    uint8_t pd0  = 0;   // Power-down bit 0
    uint8_t gx   = 0;   // Gain bit (0 = 1x)
    uint16_t initial_val = 0; 

    for(int ch = 0; ch < 4; ++ch)
    {
        uint8_t buf[3];
        
        // Byte 1: 0 1 0 0 0 DAC1 DAC0 UDAC
        // 0x40 is "Write DAC Input Register" (Volatile)
        buf[0] = 0x40 | (ch << 1); 

        // Byte 2: VREF PD1 PD0 Gx D11 D10 D9 D8
        buf[1] = (vref << 7) | (pd1 << 6) | (pd0 << 5) | (gx << 4) | ((initial_val >> 8) & 0x0F);

        // Byte 3: D7...D0
        buf[2] = (uint8_t)(initial_val & 0xFF);

        // No 50ms delay needed for volatile writes
        i2c_->TransmitBlocking(addr_8bit_, buf, 3, 10);
    }
}

// write to all 4 channels in a single I2C transaction using the Fast Write command
// relies on correct initialisation above including VREF=0 (VDD), PD=00 (Power down Normal), Gx=0 (Gain 1x) settings for each channel
I2CHandle::Result Mcp4728::FastWrite(uint16_t a, uint16_t b, uint16_t c, uint16_t d)
{
    if(!i2c_)
        return I2CHandle::Result::ERR;

    uint16_t vals[4] = {a, b, c, d};
    uint8_t data[8];
    for(int i = 0; i < 4; i++)
    {
        // 12-bit value: 0 to 4095
        uint16_t val = vals[i] > 4095 ? 4095 : vals[i];

    uint8_t low  = (uint8_t)(val & 0xFF);
    uint8_t high = (uint8_t)((val >> 8) & 0x0F);

    // Per Dwigen implementation: pack high nibble then low byte for each channel
    data[i * 2]     = high;
    data[i * 2 + 1] = low;
    }

    // Transmit the 8-byte burst to update all 4 channels
    auto res = i2c_->TransmitBlocking(addr_8bit_, data, 8, 10);
    if(res != I2CHandle::Result::OK)
        return res;

    // Some drivers (and the Dwigen implementation) issue a General-Call
    // SWUPDATE to address 0x00 to latch buffered DAC values into outputs.
    // The SWUPDATE command byte used by that driver is 0x08.
    // Send a one-byte general call to trigger the SWUPDATE.
    uint8_t swupdate_cmd = 0x08; // MCP4728_GENERAL_SWUPDATE
    return i2c_->TransmitBlocking(0x00, &swupdate_cmd, 1, 10);
}

// DMA variant: non-blocking transfer using I2C DMA. Uses an internal
// DMA-friendly buffer allocated in D2 memory.
I2CHandle::Result Mcp4728::FastWriteDMA(uint16_t a,
                                        uint16_t b,
                                        uint16_t c,
                                        uint16_t d,
                                        I2CHandle::CallbackFunctionPtr callback,
                                        void* callback_context)
{
    if(!i2c_)
        return I2CHandle::Result::ERR;

    // DMA buffer must live in D2 memory. Allocate a static buffer with the
    // project's DMA attribute so it is safe for the HAL DMA engine.
    static uint8_t DMA_BUFFER_MEM_SECTION dma_buf[8];

    uint16_t vals[4] = {a, b, c, d};
    for(int i = 0; i < 4; i++)
    {
        uint16_t val = vals[i] > 4095 ? 4095 : vals[i];
        uint8_t low  = (uint8_t)(val & 0xFF);
        uint8_t high = (uint8_t)((val >> 8) & 0x0F);

        // same packing as FastWrite (high nibble, low byte)
        dma_buf[i * 2]     = high;
        dma_buf[i * 2 + 1] = low;
    }

    // Queue a DMA transfer. TransmitDma will return OK if the job was queued
    // (it may block briefly if the DMA queue is busy, per I2CHandle contract).
    auto res = i2c_->TransmitDma(addr_8bit_, dma_buf, 8, callback, callback_context);
    if(res != I2CHandle::Result::OK)
    {
        return res;
    }

    // Optionally the SWUPDATE general-call may be required to latch buffered
    // values into outputs. Doing this via DMA would require a separate small
    // DMA/buffer; instead we perform the SWUPDATE via a blocking transmit
    // after queueing the DMA. This keeps semantics similar to the blocking
    // FastWrite path. If you prefer fully non-blocking SWUPDATE, pass a
    // callback and issue SWUPDATE there.
    uint8_t swupdate_cmd = 0x08; // MCP4728_GENERAL_SWUPDATE
    // Blocking transmit of SWUPDATE to general call address
    return i2c_->TransmitBlocking(0x00, &swupdate_cmd, 1, 10);
}
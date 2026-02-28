#include "dev/mcp4728.h"

using namespace daisy;

void Mcp4728::Init(Config config)
{
    i2c_       = config.i2c;
    // I2CHandle expects 7-bit addresses (it performs the left-shift internally),
    // so store the raw 7-bit address here.
    addr_8bit_ = config.address;
}

// write to all 4 channels in a single I2C transaction using the Fast Write command
// including VREF=1 (Internal 2.048V), PD=00 (Power down Normal), Gx=0 (Gain 1x) settings for each channel
I2CHandle::Result Mcp4728::FastWrite(uint16_t a, uint16_t b, uint16_t c, uint16_t d)
{
    if(!i2c_)
        return I2CHandle::Result::ERR;

    // Some example code in this repo (EnvelopeExample) uses the 3-byte
    // Multi-Write command sequence and has been confirmed working on
    // hardware. If the device ACKs but outputs don't change, it's likely
    // the Fast Write packet format or VREF bits. To maximize compatibility
    // with working code, implement the Multi-Write per-channel update here
    // (3 bytes per channel) which mirrors EnvelopeExample's transmitDacValue
    // behavior.

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
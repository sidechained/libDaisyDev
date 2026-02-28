#pragma once
#ifndef DSY_MCP4728_H
#define DSY_MCP4728_H

#include "daisy_core.h"
#include "per/i2c.h"

namespace daisy
{
/** * @brief Driver for the MCP4728 4-Channel 12-bit I2C DAC.
 * Optimized for FastWrite updates.
 */
class Mcp4728
{
  public:
    Mcp4728() : i2c_(nullptr), addr_8bit_(0) {}
    ~Mcp4728() {}

    /** @brief Configuration structure for the MCP4728 */
    struct Config
    {
        uint8_t    address; /**< 7-bit address (Default 0x60) */
        I2CHandle *i2c;     /**< Pointer to initialized I2C peripheral */
    };

    /** @brief Initializes the driver with the provided configuration */
    void Init(Config config);

  /** * @brief Updates all 4 channels using the FastWrite command.
   * Values are clipped to 12-bit (0-4095).
   * This is a blocking I2C transaction.
   * Returns the I2C transmission result (OK or ERR) so callers can probe ACKs.
   */
  I2CHandle::Result FastWrite(uint16_t a, uint16_t b, uint16_t c, uint16_t d);

  private:
    I2CHandle *i2c_;
    uint8_t    addr_8bit_;
};

} // namespace daisy

#endif
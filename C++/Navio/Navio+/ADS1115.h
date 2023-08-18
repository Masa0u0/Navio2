#pragma once

#include <string>

#ifdef DEBUG_ADS1115
#define debug(M, ...) fprintf(stderr, "DEBUG %s:%d: " M "\n", __FILE__, __LINE__, ##__VA_ARGS__)
#else
#define debug(M, ...)
#endif

#define ADS1115_ADDRESS_ADDR_GND 0x48  // address pin low (GND)
#define ADS1115_ADDRESS_ADDR_VDD 0x49  // address pin high (VCC)
#define ADS1115_ADDRESS_ADDR_SDA 0x4A  // address pin tied to SDA pin
#define ADS1115_ADDRESS_ADDR_SCL 0x4B  // address pin tied to SCL pin
#define ADS1115_DEFAULT_ADDRESS ADS1115_ADDRESS_ADDR_GND

#define ADS1115_RA_CONVERSION 0x00
#define ADS1115_RA_CONFIG 0x01
#define ADS1115_RA_LO_THRESH 0x02
#define ADS1115_RA_HI_THRESH 0x03

#define ADS1115_OS_SHIFT 15
#define ADS1115_OS_INACTIVE 0x00 << ADS1115_OS_SHIFT
#define ADS1115_OS_ACTIVE 0x01 << ADS1115_OS_SHIFT

#define ADS1115_MUX_SHIFT 12
#define ADS1115_MUX_P0_N1 0x00 << ADS1115_MUX_SHIFT /* default */
#define ADS1115_MUX_P0_N3 0x01 << ADS1115_MUX_SHIFT
#define ADS1115_MUX_P1_N3 0x02 << ADS1115_MUX_SHIFT
#define ADS1115_MUX_P2_N3 0x03 << ADS1115_MUX_SHIFT
#define ADS1115_MUX_P0_NG 0x04 << ADS1115_MUX_SHIFT
#define ADS1115_MUX_P1_NG 0x05 << ADS1115_MUX_SHIFT
#define ADS1115_MUX_P2_NG 0x06 << ADS1115_MUX_SHIFT
#define ADS1115_MUX_P3_NG 0x07 << ADS1115_MUX_SHIFT

#define ADS1115_PGA_SHIFT 9
#define ADS1115_PGA_6P144 0x00 << ADS1115_PGA_SHIFT
#define ADS1115_PGA_4P096 0x01 << ADS1115_PGA_SHIFT
#define ADS1115_PGA_2P048 0x02 << ADS1115_PGA_SHIFT  // default
#define ADS1115_PGA_1P024 0x03 << ADS1115_PGA_SHIFT
#define ADS1115_PGA_0P512 0x04 << ADS1115_PGA_SHIFT
#define ADS1115_PGA_0P256 0x05 << ADS1115_PGA_SHIFT
#define ADS1115_PGA_0P256B 0x06 << ADS1115_PGA_SHIFT
#define ADS1115_PGA_0P256C 0x07 << ADS1115_PGA_SHIFT

#define ADS1115_MV_6P144 0.187500
#define ADS1115_MV_4P096 0.125000
#define ADS1115_MV_2P048 0.062500  // default
#define ADS1115_MV_1P024 0.031250
#define ADS1115_MV_0P512 0.015625
#define ADS1115_MV_0P256 0.007813
#define ADS1115_MV_0P256B 0.007813
#define ADS1115_MV_0P256C 0.007813

#define ADS1115_MODE_SHIFT 8
#define ADS1115_MODE_CONTINUOUS 0x00 << ADS1115_MODE_SHIFT
#define ADS1115_MODE_SINGLESHOT 0x01 << ADS1115_MODE_SHIFT  // default

#define ADS1115_RATE_SHIFT 5
#define ADS1115_RATE_8 0x00 << ADS1115_RATE_SHIFT
#define ADS1115_RATE_16 0x01 << ADS1115_RATE_SHIFT
#define ADS1115_RATE_32 0x02 << ADS1115_RATE_SHIFT
#define ADS1115_RATE_64 0x03 << ADS1115_RATE_SHIFT
#define ADS1115_RATE_128 0x04 << ADS1115_RATE_SHIFT  // default
#define ADS1115_RATE_250 0x05 << ADS1115_RATE_SHIFT
#define ADS1115_RATE_475 0x06 << ADS1115_RATE_SHIFT
#define ADS1115_RATE_860 0x07 << ADS1115_RATE_SHIFT

#define ADS1115_COMP_MODE_SHIFT 4
#define ADS1115_COMP_MODE_HYSTERESIS 0x00 << ADS1115_COMP_MODE_SHIFT  // default
#define ADS1115_COMP_MODE_WINDOW 0x01 << ADS1115_COMP_MODE_SHIFT

#define ADS1115_COMP_POL_SHIFT 3
#define ADS1115_COMP_POL_ACTIVE_LOW 0x00 << ADS1115_COMP_POL_SHIFT  // default
#define ADS1115_COMP_POL_ACTIVE_HIGH 0x01 << ADS1115_COMP_POL_SHIFT

#define ADS1115_COMP_LAT_SHIFT 2
#define ADS1115_COMP_LAT_NON_LATCHING 0x00 << ADS1115_COMP_LAT_SHIFT  // default
#define ADS1115_COMP_LAT_LATCHING 0x01 << ADS1115_COMP_LAT_SHIFT

#define ADS1115_COMP_QUE_SHIFT 0
#define ADS1115_COMP_QUE_ASSERT1 0x00 << ADS1115_COMP_SHIFT
#define ADS1115_COMP_QUE_ASSERT2 0x01 << ADS1115_COMP_SHIFT
#define ADS1115_COMP_QUE_ASSERT4 0x02 << ADS1115_COMP_SHIFT
#define ADS1115_COMP_QUE_DISABLE 0x03  // default

#include "../Common/I2Cdev.h"

class ADS1115
{
public:
  /**
   * @brief ADS1115 constructor
   * Default gain is 4096
   * Default multiplexer is between GND and P0
   * Default comparator mode is disabled
   * Default mode is singleshot
   * @param address
   */
  explicit ADS1115(uint8_t address = ADS1115_DEFAULT_ADDRESS);

  /** Verify the I2C connection.
   * @return True if connection is valid, false otherwise
   */
  bool testConnection();

  /**
   * @brief Get data from Conversion Register
   *
   * @return Little-Endian result
   */
  int16_t getConversion();

  /**
   * @brief Update Operational Status
   *
   * @param Desired Status
   */
  void setOpStatus(uint16_t op);

  /**
   * @brief Check which Multiplexer is selected
   *
   * @return Multiplexer status
   */
  uint16_t getMultiplexer();

  /**
   * @brief Choose the multiplexer
   *
   * @param Desired multiplexer
   */
  void setMultiplexer(uint16_t mux);

  /**
   * @brief Get current gain
   *
   * @return Current Gain
   */
  uint16_t getGain();

  /**
   * @brief Set gain
   *
   * @param gain
   */
  void setGain(uint16_t gain);

  /**
   * @brief Get mode
   *
   * @return mode
   */
  uint16_t getMode();

  /**
   * @brief Set mode
   *
   * @param mode
   */
  void setMode(uint16_t mode);

  /**
   * @brief Get rate
   *
   * @return rate
   */
  uint16_t getRate();

  /**
   * @brief Set rate
   *
   * @param rate
   */
  void setRate(uint16_t rate);

  /**
   * @brief Get content of conversion register in mV. It gets converted using current gain
   * @see setGain
   *
   * @return Last conversion in mV
   */
  float getMilliVolts();

  /**
   * @brief set comparator mode
   *
   * @param comparator
   */
  void setComparatorMode(uint16_t comparatorMode);

  /**
   * @brief Set Comparator polarity
   *
   * @param polarity
   */
  void setComparatorPolarity(uint16_t polarit);

  /**
   * @brief Set comparator latch status
   *
   * @param latch
   */
  void setComparatorLatchEnabled(uint16_t latchStatus);

  /**
   * @brief Set Comparator Queue Mode
   *
   * @param queue
   */
  void setComparatorQueueMode(uint16_t queueMode);

private:
  /**
   * @brief Call it if you updated ConfigRegister
   */
  void updateConfigRegister();

  uint8_t address;

  struct configRegister
  {
    uint16_t status;
    uint16_t mux;
    uint16_t gain;
    uint16_t mode;
    uint16_t rate;
    uint16_t comparator;
    uint16_t polarity;
    uint16_t latch;
    uint16_t queue;
  } config;
};

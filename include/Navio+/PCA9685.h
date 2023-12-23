#pragma once

#include <cinttypes>

#define PCA9685_DEFAULT_ADDRESS 0x40  // All address pins low, Navio default

#define PCA9685_RA_MODE1 0x00
#define PCA9685_RA_MODE2 0x01
#define PCA9685_RA_LED0_ON_L 0x06
#define PCA9685_RA_LED0_ON_H 0x07
#define PCA9685_RA_LED0_OFF_L 0x08
#define PCA9685_RA_LED0_OFF_H 0x09
#define PCA9685_RA_ALL_LED_ON_L 0xFA
#define PCA9685_RA_ALL_LED_ON_H 0xFB
#define PCA9685_RA_ALL_LED_OFF_L 0xFC
#define PCA9685_RA_ALL_LED_OFF_H 0xFD
#define PCA9685_RA_PRE_SCALE 0xFE

#define PCA9685_MODE1_RESTART_BIT 7
#define PCA9685_MODE1_EXTCLK_BIT 6
#define PCA9685_MODE1_AI_BIT 5
#define PCA9685_MODE1_SLEEP_BIT 4
#define PCA9685_MODE1_SUB1_BIT 3
#define PCA9685_MODE1_SUB2_BIT 2
#define PCA9685_MODE1_SUB3_BIT 1
#define PCA9685_MODE1_ALLCALL_BIT 0

#define PCA9685_MODE2_INVRT_BIT 4
#define PCA9685_MODE2_OCH_BIT 3
#define PCA9685_MODE2_OUTDRV_BIT 2
#define PCA9685_MODE2_OUTNE1_BIT 1
#define PCA9685_MODE2_OUTNE0_BIT 0

class PCA9685
{
public:
  /** PCA9685 constructor.
   * @param address I2C address
   * @see PCA9685_DEFAULT_ADDRESS
   */
  explicit PCA9685(uint8_t address = PCA9685_DEFAULT_ADDRESS);

  /** Power on and prepare for general usage.
   * This method reads prescale value stored in PCA9685 and calculate frequency based on it.
   * Then it enables auto-increment of register address to allow for faster writes.
   * And finally the restart is performed to enable clocking.
   */
  void initialize();

  /** Verify the I2C connection.
   * @return True if connection is valid, false otherwise
   */
  bool testConnection();

  /** Put PCA9685 to sleep mode thus turning off the outputs.
   * @see PCA9685_MODE1_SLEEP_BIT
   */
  void sleep();

  /** Disable sleep mode and start the outputs.
   * @see PCA9685_MODE1_SLEEP_BIT
   * @see PCA9685_MODE1_RESTART_BIT
   */
  void restart();

  /** Calculate prescale value based on the specified frequency and write it to the device.
   * @return Frequency in Hz
   * @see PCA9685_RA_PRE_SCALE
   */
  float getFrequency();

  /** Calculate prescale value based on the specified frequency and write it to the device.
   * @param Frequency in Hz
   * @see PCA9685_RA_PRE_SCALE
   */
  void setFrequency(float frequency);

  /** Set channel start offset of the pulse and it's length
   * @param Channel number (0-15)
   * @param Offset (0-4095)
   * @param Length (0-4095)
   * @see PCA9685_RA_LED0_ON_L
   */
  void setPWM(uint8_t channel, uint16_t offset, uint16_t length);

  /** Set channel's pulse length
   * @param Channel number (0-15)
   * @param Length (0-4095)
   * @see PCA9685_RA_LED0_ON_L
   */
  void setPWM(uint8_t channel, uint16_t length);

  /** Set channel's pulse length in milliseconds
   * @param Channel number (0-15)
   * @param Length in milliseconds
   * @see PCA9685_RA_LED0_ON_L
   */
  void setPWMmS(uint8_t channel, float length_mS);

  /** Set channel's pulse length in microseconds
   * @param Channel number (0-15)
   * @param Length in microseconds
   * @see PCA9685_RA_LED0_ON_L
   */
  void setPWMuS(uint8_t channel, float length_uS);

  /** Set start offset of the pulse and it's length for all channels
   * @param Offset (0-4095)
   * @param Length (0-4095)
   * @see PCA9685_RA_ALL_LED_ON_L
   */
  void setAllPWM(uint16_t offset, uint16_t length);

  /** Set pulse length for all channels
   * @param Length (0-4095)
   * @see PCA9685_RA_ALL_LED_ON_L
   */
  void setAllPWM(uint16_t length);

  /** Set pulse length in milliseconds for all channels
   * @param Length in milliseconds
   * @see PCA9685_RA_ALL_LED_ON_L
   */
  void setAllPWMmS(float length_mS);

  /** Set pulse length in microseconds for all channels
   * @param Length in microseconds
   * @see PCA9685_RA_ALL_LED_ON_L
   */
  void setAllPWMuS(float length_uS);

private:
  uint8_t devAddr;
  float frequency;
};

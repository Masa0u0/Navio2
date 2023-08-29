#include <cmath>

#include "./LSM9DS1.h"

#define DEVICE_ACC_GYRO "/dev/spidev0.3"
#define DEVICE_MAGNETOMETER "/dev/spidev0.2"

#define READ_FLAG 0x80
#define MULTIPLE_READ 0x40

#define G_SI 9.80665
#define DEG2RAD (M_PI / 180.)

LSM9DS1::LSM9DS1() : spi_dev_imu_(DEVICE_ACC_GYRO), spi_dev_mag_(DEVICE_MAGNETOMETER)
{
}

/*-----------------------------------------------------------------------------------------------
                                    REGISTER READ & WRITE
usage: use these methods to read and write LSM9DS1 registers over SPI
-----------------------------------------------------------------------------------------------*/

uint32_t LSM9DS1::WriteReg(SPIdev& spi_dev, uint8_t WriteAddr, uint8_t WriteData)
{
  uint8_t tx[2] = { WriteAddr, WriteData };
  uint8_t rx[2] = { 0 };
  spi_dev.transfer(tx, rx, 2);
  return rx[1];
}

uint32_t LSM9DS1::ReadReg(SPIdev& spi_dev, uint8_t ReadAddr)
{
  return WriteReg(spi_dev, ReadAddr | READ_FLAG, 0x00);
}

void LSM9DS1::ReadRegsImu(uint8_t ReadAddr, uint8_t* ReadBuf, uint32_t Bytes)
{
  uint8_t tx[255] = { 0 };
  uint8_t rx[255] = { 0 };

  tx[0] = ReadAddr | READ_FLAG;

  spi_dev_imu_.transfer(tx, rx, Bytes + 1);

  for (uint32_t i = 0; i < Bytes; ++i)
    ReadBuf[i] = rx[i + 1];

  // usleep(50);
}

void LSM9DS1::ReadRegsMag(uint8_t ReadAddr, uint8_t* ReadBuf, uint32_t Bytes)
{
  uint8_t tx[255] = { 0 };
  uint8_t rx[255] = { 0 };

  tx[0] = ReadAddr | READ_FLAG | MULTIPLE_READ;

  spi_dev_mag_.transfer(tx, rx, Bytes + 1);

  for (uint32_t i = 0; i < Bytes; ++i)
    ReadBuf[i] = rx[i + 1];

  // usleep(50);
}

/*-----------------------------------------------------------------------------------------------
                                TEST CONNECTION
usage: call this function to know if SPI and LSM9DS1 are working correctly.
returns true if Accel/Gyro and Magnetometer answers
-----------------------------------------------------------------------------------------------*/

bool LSM9DS1::probe()
{
  uint8_t responseXG, responseM;
  responseXG = ReadReg(spi_dev_imu_, LSM9DS1XG_WHO_AM_I);
  responseM = ReadReg(spi_dev_mag_, LSM9DS1M_WHO_AM_I);
  if (responseXG == WHO_AM_I_ACC_GYRO && responseM == WHO_AM_I_MAG)
    return true;
  else
    return false;
}

bool LSM9DS1::initialize()
{
  //--------Accelerometer and Gyroscope---------
  // enable the 3-axes of the gyroscope
  WriteReg(spi_dev_imu_, LSM9DS1XG_CTRL_REG4, BITS_XEN_G | BITS_YEN_G | BITS_ZEN_G);
  // configure the gyroscope
  WriteReg(spi_dev_imu_, LSM9DS1XG_CTRL_REG1_G, BITS_ODR_G_952HZ | BITS_FS_G_2000DPS);
  usleep(200);

  // enable the three axes of the accelerometer
  WriteReg(spi_dev_imu_, LSM9DS1XG_CTRL_REG5_XL, BITS_XEN_XL | BITS_YEN_XL | BITS_ZEN_XL);
  // configure the accelerometer-specify bandwidth selection with Abw
  WriteReg(spi_dev_imu_, LSM9DS1XG_CTRL_REG6_XL, BITS_ODR_XL_952HZ | BITS_FS_XL_16G);
  usleep(200);

  //------------Magnetometer----------------
  WriteReg(spi_dev_mag_, LSM9DS1M_CTRL_REG1_M, BITS_TEMP_COMP | BITS_OM_HIGH | BITS_ODR_M_80HZ);
  WriteReg(spi_dev_mag_, LSM9DS1M_CTRL_REG2_M, BITS_FS_M_16Gs);
  // continuous conversion mode
  WriteReg(spi_dev_mag_, LSM9DS1M_CTRL_REG3_M, BITS_MD_CONTINUOUS);
  WriteReg(spi_dev_mag_, LSM9DS1M_CTRL_REG4_M, BITS_OMZ_HIGH);
  WriteReg(spi_dev_mag_, LSM9DS1M_CTRL_REG5_M, 0x00);
  usleep(200);

  set_gyro_scale(BITS_FS_G_2000DPS);
  set_acc_scale(BITS_FS_XL_16G);
  set_mag_scale(BITS_FS_M_16Gs);
  return true;
}

void LSM9DS1::update()
{
  uint8_t response[6];
  int16_t bit_data[3];

  // Read temperature
  // ReadRegsImu(LSM9DS1XG_OUT_TEMP_L, &response[0], 2);
  // temperature = (float)(((int16_t)response[1] << 8) | response[0]) / 256. + 25.;

  // Read accelerometer
  ReadRegsImu(LSM9DS1XG_OUT_X_L_XL, &response[0], 6);
  for (uint32_t i = 0; i < 3; ++i)
  {
    bit_data[i] = ((int16_t)response[2 * i + 1] << 8) | response[2 * i];
  }
  _ax = G_SI * ((float)bit_data[0] * acc_scale);
  _ay = G_SI * ((float)bit_data[1] * acc_scale);
  _az = G_SI * ((float)bit_data[2] * acc_scale);

  // Read gyroscope
  ReadRegsImu(LSM9DS1XG_OUT_X_L_G, &response[0], 6);
  for (uint32_t i = 0; i < 3; ++i)
  {
    bit_data[i] = ((int16_t)response[2 * i + 1] << 8) | response[2 * i];
  }
  _gx = DEG2RAD * ((float)bit_data[0] * gyro_scale);
  _gy = DEG2RAD * ((float)bit_data[1] * gyro_scale);
  _gz = DEG2RAD * ((float)bit_data[2] * gyro_scale);

  // Read magnetometer
  ReadRegsMag(LSM9DS1M_OUT_X_L_M, &response[0], 6);
  for (uint32_t i = 0; i < 3; ++i)
  {
    bit_data[i] = ((int16_t)response[2 * i + 1] << 8) | response[2 * i];
  }
  _mx = 100. * ((float)bit_data[0] * mag_scale);
  _my = 100. * ((float)bit_data[1] * mag_scale);
  _mz = 100. * ((float)bit_data[2] * mag_scale);

  // Change rotation of LSM9DS1 like in MPU-9250
  rotate();
}

void LSM9DS1::rotate()
{
  float replacement_acc, replacement_gyro;

  replacement_acc = _ax;
  _ax = -_ay;
  _ay = -replacement_acc;

  replacement_gyro = _gx;
  _gx = -_gy;
  _gy = -replacement_gyro;

  _my = -_my;
  _mz = -_mz;
}

void LSM9DS1::set_gyro_scale(int scale)
{
  uint8_t reg;
  reg = BITS_FS_G_MASK & ReadReg(spi_dev_imu_, LSM9DS1XG_CTRL_REG1_G);
  WriteReg(spi_dev_imu_, LSM9DS1XG_CTRL_REG1_G, reg | scale);
  switch (scale)
  {
    case BITS_FS_G_245DPS:
      gyro_scale = 0.00875;
      break;
    case BITS_FS_G_500DPS:
      gyro_scale = 0.0175;
      break;
    case BITS_FS_G_2000DPS:
      gyro_scale = 0.07;
      break;
  }
}

void LSM9DS1::set_acc_scale(int scale)
{
  uint8_t reg;
  reg = BITS_FS_XL_MASK & ReadReg(spi_dev_imu_, LSM9DS1XG_CTRL_REG6_XL);
  WriteReg(spi_dev_imu_, LSM9DS1XG_CTRL_REG6_XL, reg | scale);
  switch (scale)
  {
    case BITS_FS_XL_2G:
      acc_scale = 0.000061;
      break;
    case BITS_FS_XL_4G:
      acc_scale = 0.000122;
      break;
    case BITS_FS_XL_8G:
      acc_scale = 0.000244;
      break;
    case BITS_FS_XL_16G:
      acc_scale = 0.000732;
      break;
  }
}

void LSM9DS1::set_mag_scale(int scale)
{
  uint8_t reg;
  reg = BITS_FS_M_MASK & ReadReg(spi_dev_mag_, LSM9DS1M_CTRL_REG2_M);
  WriteReg(spi_dev_mag_, LSM9DS1M_CTRL_REG2_M, reg | scale);
  switch (scale)
  {
    case BITS_FS_M_4Gs:
      mag_scale = 0.00014;
      break;
    case BITS_FS_M_8Gs:
      mag_scale = 0.00029;
      break;
    case BITS_FS_M_12Gs:
      mag_scale = 0.00043;
      break;
    case BITS_FS_M_16Gs:
      mag_scale = 0.00058;
      break;
  }
}

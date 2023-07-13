#pragma once

#include <cinttypes>

struct NavPayload_STATUS
{
  uint8_t gpsFix;
  uint8_t flags;
};

struct NavPayload_POSLLH
{
  float lon;   // Longitude [deg]
  float lat;   // Latitude [deg]
  float hMSL;  // Height above mean sea level [m]
};

struct NavPayload_VELNED
{
  float velN;  // North velocity component [m/s]
  float velE;  // East velocity component [m/s]
  float velD;  // Down velocity component [m/s]
};

struct NavPayload_PVT
{
  float lon;   // Longitude [deg]
  float lat;   // Latitude [deg]
  float hMSL;  // Height above mean sea level [m]
  float velN;  // NED north veloity [m/s]
  float velE;  // NED east veloity [m/s]
  float velD;  // NED down veloity [m/s]
};

struct NavPayload_COV
{
  float posCovNN;  // Position covariance matrix value p_NN [m^2]
  float posCovNE;  // Position covariance matrix value p_NE [m^2]
  float posCovND;  // Position covariance matrix value p_ND [m^2]
  float posCovEE;  // Position covariance matrix value p_EE [m^2]
  float posCovED;  // Position covariance matrix value p_ED [m^2]
  float posCovDD;  // Position covariance matrix value p_DD [m^2]
  float velCovNN;  // Velocity covariance matrix value p_NN [m^2/s^2]
  float velCovNE;  // Velocity covariance matrix value p_NE [m^2/s^2]
  float velCovND;  // Velocity covariance matrix value p_ND [m^2/s^2]
  float velCovEE;  // Velocity covariance matrix value p_EE [m^2/s^2]
  float velCovED;  // Velocity covariance matrix value p_ED [m^2/s^2]
  float velCovDD;  // Velocity covariance matrix value p_DD [m^2/s^2]
};

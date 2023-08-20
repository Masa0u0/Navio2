#pragma once

#include <cinttypes>
#include <iostream>

struct NavPosllhPayload
{
  double lon;   // Longitude [deg]
  double lat;   // Latitude [deg]
  double hMSL;  // Height above mean sea level [m]

  friend std::ostream& operator<<(std::ostream& os, const NavPosllhPayload& arg);
};

struct NavStatusPayload
{
  uint8_t gpsFix;  // GPSfix Type, this value does not qualify a fix as valid and within the limits.
  uint8_t flags;   // Navigation Status Flags

  friend std::ostream& operator<<(std::ostream& os, const NavStatusPayload& arg);
};

struct NavDopPayload
{
  // TODO
};

struct NavPvtPayload
{
  uint16_t year;  // Year (UTC)
  uint8_t month;  // Month, range 1..12 (UTC)
  uint8_t day;    // Day of month, range 1..31 (UTC)
  uint8_t hour;   // Hour of day, range 0..23 (UTC)
  uint8_t min;    // Minute of hour, range 0..59 (UTC)
  uint8_t sec;    // Seconds of minute, range 0..60 (UTC)

  double lon;   // Longitude [deg]
  double lat;   // Latitude [deg]
  double hMSL;  // Height above mean sea level [m]
  double velN;  // NED north veloity [m/s]
  double velE;  // NED east veloity [m/s]
  double velD;  // NED down veloity [m/s]

  friend std::ostream& operator<<(std::ostream& os, const NavPvtPayload& arg);
};

struct NavVelnedPayload
{
  double velN;  // North velocity component [m/s]
  double velE;  // East velocity component [m/s]
  double velD;  // Down velocity component [m/s]

  friend std::ostream& operator<<(std::ostream& os, const NavVelnedPayload& arg);
};

struct NavTimegpsPayload
{
  // TODO
};

struct NavCovPayload
{
  double posCovNN;  // Position covariance matrix value p_NN [m^2]
  double posCovNE;  // Position covariance matrix value p_NE [m^2]
  double posCovND;  // Position covariance matrix value p_ND [m^2]
  double posCovEE;  // Position covariance matrix value p_EE [m^2]
  double posCovED;  // Position covariance matrix value p_ED [m^2]
  double posCovDD;  // Position covariance matrix value p_DD [m^2]
  double velCovNN;  // Velocity covariance matrix value v_NN [m^2/s^2]
  double velCovNE;  // Velocity covariance matrix value v_NE [m^2/s^2]
  double velCovND;  // Velocity covariance matrix value v_ND [m^2/s^2]
  double velCovEE;  // Velocity covariance matrix value v_EE [m^2/s^2]
  double velCovED;  // Velocity covariance matrix value v_ED [m^2/s^2]
  double velCovDD;  // Velocity covariance matrix value v_DD [m^2/s^2]

  friend std::ostream& operator<<(std::ostream& os, const NavCovPayload& arg);
};

struct AckNakPayload
{
  // TODO
};

struct AckAckPayload
{
  // TODO
};

struct MonHwPayload
{
  // TODO
};

struct MonHw2Payload
{
  // TODO
};

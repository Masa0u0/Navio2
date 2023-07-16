#include "./nav_payloads.hpp"

using namespace std;

ostream& operator<<(ostream& os, const NavPayload_POSLLH& arg)
{
  os << "Longitude: " << arg.lon << "[deg]" << endl;
  os << "Latitude: " << arg.lat << "[deg]" << endl;
  os << "Height above mean sea level: " << arg.hMSL << "[m]" << endl;
  return os;
}

ostream& operator<<(ostream& os, const NavPayload_STATUS& arg)
{
  os << "GPSfix Type: " << static_cast<int>(arg.gpsFix) << endl;
  os << "Navigation Status Flags: " << static_cast<int>(arg.flags) << endl;
  return os;
}

ostream& operator<<(ostream& os, const NavPayload_PVT& arg)
{
  os << "Longitude: " << arg.lon << "[deg]" << endl;
  os << "Latitude: " << arg.lat << "[deg]" << endl;
  os << "Height above mean sea level: " << arg.hMSL << "[m]" << endl;
  os << "NED north velocity: " << arg.velN << "[m/s]" << endl;
  os << "NED east velocity: " << arg.velE << "[m/s]" << endl;
  os << "NED down velocity: " << arg.velD << "[m/s]" << endl;
  return os;
}

ostream& operator<<(ostream& os, const NavPayload_VELNED& arg)
{
  os << "North velocity component: " << arg.velN << "[m/s]" << endl;
  os << "East velocity component: " << arg.velE << "[m/s]" << endl;
  os << "Down velocity component: " << arg.velD << "[m/s]" << endl;
  return os;
}

ostream& operator<<(ostream& os, const NavPayload_COV& arg)
{
  os << "Position covariance matrix value p_NN: " << arg.posCovNN << "[m^2]" << endl;
  os << "Position covariance matrix value p_NE: " << arg.posCovNE << "[m^2]" << endl;
  os << "Position covariance matrix value p_ND: " << arg.posCovND << "[m^2]" << endl;
  os << "Position covariance matrix value p_EE: " << arg.posCovEE << "[m^2]" << endl;
  os << "Position covariance matrix value p_ED: " << arg.posCovED << "[m^2]" << endl;
  os << "Position covariance matrix value p_DD: " << arg.posCovDD << "[m^2]" << endl;
  os << "Velocity covariance matrix value v_NN: " << arg.velCovNN << "[m^2/s^2]" << endl;
  os << "Velocity covariance matrix value v_NE: " << arg.velCovNE << "[m^2/s^2]" << endl;
  os << "Velocity covariance matrix value v_ND: " << arg.velCovND << "[m^2/s^2]" << endl;
  os << "Velocity covariance matrix value v_EE: " << arg.velCovEE << "[m^2/s^2]" << endl;
  os << "Velocity covariance matrix value v_ED: " << arg.velCovED << "[m^2/s^2]" << endl;
  os << "Velocity covariance matrix value v_DD: " << arg.velCovDD << "[m^2/s^2]" << endl;
  return os;
}

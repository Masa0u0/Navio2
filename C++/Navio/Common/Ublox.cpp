#include <stdexcept>
#include <cstring>

#include "Ublox.h"
#include "Util.h"

#define GPS_DEVICE "/dev/spidev0.0"

using namespace std;

UBXScanner::UBXScanner()
{
  reset();
}

uint8_t* UBXScanner::getMessage()
{
  return message_;
}

uint32_t UBXScanner::getMessageLength() const
{
  return message_length_;
}

uint32_t UBXScanner::getPosition() const
{
  return position_;
}

void UBXScanner::reset()
{
  message_length_ = 0;
  position_ = 0;
  state_ = Sync1;
}

int UBXScanner::update(uint8_t data)
{
  if (state_ != Done)
    message_[position_++] = data;

  switch (state_)
  {
    case Sync1:
      if (data == 0xb5)
        state_ = Sync2;
      else
        reset();
      break;

    case Sync2:
      if (data == 0x62)
        state_ = Class;
      else if (data == 0xb5)
        state_ = Sync1;
      else
        reset();
      break;

    case Class:
      state_ = ID;
      break;

    case ID:
      state_ = Length1;
      break;

    case Length1:
      payload_length_ = data;
      state_ = Length2;
      break;

    case Length2:
      payload_length_ += data << 8;
      state_ = Payload;
      break;

    case Payload:
      if (position_ == payload_length_ + 6)
        state_ = CK_A;
      if (position_ >= 1022)
        reset();
      break;

    case CK_A:
      state_ = CK_B;
      break;

    case CK_B:
      message_length_ = 6 + payload_length_ + 2;
      state_ = Done;
      break;

    case Done:
      break;

    default:
      break;
  }

  return state_;
}

UBXParser::UBXParser(UBXScanner* ubxsc) : scanner_(ubxsc), message_(ubxsc->getMessage())
{
}

void UBXParser::updateMessageData()
{
  length_ = scanner_->getMessageLength();
  position_ = scanner_->getPosition();
}

uint16_t UBXParser::calcId()
{
  const auto pos = position_ - length_;  // count the message start position
  const auto s = message_ + pos;

  // All UBX messages start with 2 sync chars: 0xb5 and 0x62
  if (*s != 0xb5)
    return 0;
  if (*(s + 1) != 0x62)
    return 0;

  // Count the checksum
  uint8_t CK_A = 0, CK_B = 0;
  for (uint32_t i = 2; i < (length_ - 2); i++)
  {
    CK_A += *(s + i);
    CK_B += CK_A;
  }
  if (CK_A != *(s + length_ - 2))
    return 0;
  if (CK_B != *(s + length_ - 1))
    return 0;

  // If we got everything right, then it's time to decide, what type of message this is
  // ID is a two-byte number with little endianness
  return latest_id_ = (*(s + 2)) << 8 | (*(s + 3));
}

int UBXParser::checkMessage()
{
  updateMessageData();  // get the length and end message coordinate from UBX scanner

  const auto pos = position_ - length_;  // count the message start position
  const auto s = message_ + pos;

  // All UBX messages start with 2 sync chars: 0xb5 and 0x62
  if (*(s) != 0xb5)
    return 0;
  if (*(s + 1) != 0x62)
    return 0;

  // Count the checksum
  uint8_t CK_A = 0, CK_B = 0;
  for (uint32_t i = 2; i < (length_ - 2); i++)
  {
    CK_A += *(s + i);
    CK_B += CK_A;
  }

  if (CK_A != *(s + length_ - 2))
    return 0;
  if (CK_B != *(s + length_ - 1))
    return 0;

  return 1;
}

uint8_t* UBXParser::getMessage() const
{
  return message_;
}

uint32_t UBXParser::getLength() const
{
  return length_;
}

uint32_t UBXParser::getPosition() const
{
  return position_;
}

uint16_t UBXParser::getLatestId() const
{
  return latest_id_;
}

Ublox::Ublox()
  : spi_dev_(GPS_DEVICE, kSpiSpeedHz), scanner_(new UBXScanner()), parser_(new UBXParser(scanner_))
{
}

Ublox::Ublox(UBXScanner* scan, UBXParser* pars)
  : spi_dev_(GPS_DEVICE, kSpiSpeedHz), scanner_(scan), parser_(pars)
{
}

int Ublox::enableNavMsg(message_t msg)
{
  CfgMsg cfg_msg;
  cfg_msg.msgClass = CLASS_NAV;
  cfg_msg.msgID = msg - (CLASS_NAV << 8);
  cfg_msg.rate = 1;

  return sendMessage(CLASS_CFG, ID_CFG_MSG, &cfg_msg, sizeof(CfgMsg));
}

int Ublox::disableNavMsg(message_t msg)
{
  CfgMsg cfg_msg;
  cfg_msg.msgClass = CLASS_NAV;
  cfg_msg.msgID = msg - (CLASS_NAV << 8);
  cfg_msg.rate = 0;  // rateを0にすると発行されなくなる

  return sendMessage(CLASS_CFG, ID_CFG_MSG, &cfg_msg, sizeof(CfgMsg));
}

void Ublox::enableAllNavMsgs()
{
  enableNavMsg(NAV_POSLLH);
  enableNavMsg(NAV_STATUS);
  enableNavMsg(NAV_PVT);
  enableNavMsg(NAV_VELNED);
  enableNavMsg(NAV_COV);
}

void Ublox::disableAllNavMsgs()
{
  disableNavMsg(NAV_POSLLH);
  disableNavMsg(NAV_STATUS);
  disableNavMsg(NAV_PVT);
  disableNavMsg(NAV_VELNED);
  disableNavMsg(NAV_COV);
}

int Ublox::configureSolutionRate(uint16_t meas_rate, uint16_t nav_rate, uint16_t time_ref)
{
  CfgRate cfg_rate;
  cfg_rate.measRate = meas_rate;
  cfg_rate.navRate = nav_rate;
  cfg_rate.timeRef = time_ref;

  return sendMessage(CLASS_CFG, ID_CFG_RATE, &cfg_rate, sizeof(CfgRate));
}

uint16_t Ublox::update()
{
  int status = -1;
  uint8_t to_gps_data = 0x00, from_gps_data = 0x00;

  while (status != UBXScanner::Done)
  {
    // From now on, we will send zeroes to the receiver, which it will ignore
    // However, we are simultaneously getting useful information from it
    spi_dev_.transfer(&to_gps_data, &from_gps_data, 1);

    // Scanner checks the message structure with every byte received
    status = scanner_->update(from_gps_data);
  }

  parser_->updateMessageData();
  scanner_->reset();
  return parser_->calcId();
}

void Ublox::decode(NavPayload_POSLLH& data) const
{
  if (parser_->getLatestId() != Ublox::NAV_POSLLH)
  {
    throw runtime_error("Message type mismatch.");
  }

  const auto msg = parser_->getMessage();
  const auto pos = parser_->getPosition() - parser_->getLength();
  const auto s = msg + pos;

  data.lon = ((*(s + 13) << 24) | (*(s + 12) << 16) | (*(s + 11) << 8) | (*(s + 10))) * 1e-7;
  data.lat = ((*(s + 17) << 24) | (*(s + 16) << 16) | (*(s + 15) << 8) | (*(s + 14))) * 1e-7;
  data.hMSL = ((*(s + 25) << 24) | (*(s + 24) << 16) | (*(s + 23) << 8) | (*(s + 22))) * 1e-3;
}

void Ublox::decode(NavPayload_STATUS& data) const
{
  if (parser_->getLatestId() != Ublox::NAV_STATUS)
  {
    throw runtime_error("Message type mismatch.");
  }

  const auto msg = parser_->getMessage();
  const auto pos = parser_->getPosition() - parser_->getLength();
  const auto s = msg + pos;

  data.gpsFix = *(s + 10);
  data.flags = *(s + 11);
}

void Ublox::decode(NavPayload_PVT& data) const
{
  if (parser_->getLatestId() != Ublox::NAV_PVT)
  {
    throw runtime_error("Message type mismatch.");
  }

  const auto msg = parser_->getMessage();
  const auto pos = parser_->getPosition() - parser_->getLength();
  const auto s = msg + pos;

  data.year = (*(s + 11) << 8) | (*(s + 10));
  data.month = *(s + 12);
  data.day = *(s + 13);
  data.hour = *(s + 14);
  data.min = *(s + 15);
  data.sec = *(s + 16);

  data.lon = ((*(s + 33) << 24) | (*(s + 32) << 16) | (*(s + 31) << 8) | (*(s + 30))) * 1e-7;
  data.lat = ((*(s + 37) << 24) | (*(s + 36) << 16) | (*(s + 35) << 8) | (*(s + 34))) * 1e-7;
  data.hMSL = ((*(s + 45) << 24) | (*(s + 44) << 16) | (*(s + 43) << 8) | (*(s + 42))) * 1e-3;
  data.velN = ((*(s + 57) << 24) | (*(s + 56) << 16) | (*(s + 55) << 8) | (*(s + 54))) * 1e-3;
  data.velE = ((*(s + 61) << 24) | (*(s + 60) << 16) | (*(s + 59) << 8) | (*(s + 58))) * 1e-3;
  data.velD = ((*(s + 65) << 24) | (*(s + 64) << 16) | (*(s + 63) << 8) | (*(s + 62))) * 1e-3;
}

void Ublox::decode(NavPayload_VELNED& data) const
{
  if (parser_->getLatestId() != Ublox::NAV_VELNED)
  {
    throw runtime_error("Message type mismatch.");
  }

  const auto msg = parser_->getMessage();
  const auto pos = parser_->getPosition() - parser_->getLength();
  const auto s = msg + pos;

  data.velN = ((*(s + 13) << 24) | (*(s + 12) << 16) | (*(s + 11) << 8) | (*(s + 10))) * 1e-2;
  data.velE = ((*(s + 17) << 24) | (*(s + 16) << 16) | (*(s + 15) << 8) | (*(s + 14))) * 1e-2;
  data.velD = ((*(s + 21) << 24) | (*(s + 20) << 16) | (*(s + 19) << 8) | (*(s + 18))) * 1e-2;
}

void Ublox::decode(NavPayload_COV& data) const
{
  if (parser_->getLatestId() != Ublox::NAV_COV)
  {
    throw runtime_error("Message type mismatch.");
  }

  const auto msg = parser_->getMessage();
  const auto pos = parser_->getPosition() - parser_->getLength();
  const auto s = msg + pos;

  data.posCovNN =
    decodeBinary32((*(s + 25) << 24) | (*(s + 24) << 16) | (*(s + 23) << 8) | (*(s + 22)));
  data.posCovNE =
    decodeBinary32((*(s + 29) << 24) | (*(s + 28) << 16) | (*(s + 27) << 8) | (*(s + 26)));
  data.posCovND =
    decodeBinary32((*(s + 33) << 24) | (*(s + 32) << 16) | (*(s + 31) << 8) | (*(s + 30)));
  data.posCovEE =
    decodeBinary32((*(s + 37) << 24) | (*(s + 36) << 16) | (*(s + 35) << 8) | (*(s + 34)));
  data.posCovED =
    decodeBinary32((*(s + 41) << 24) | (*(s + 40) << 16) | (*(s + 39) << 8) | (*(s + 38)));
  data.posCovDD =
    decodeBinary32((*(s + 45) << 24) | (*(s + 44) << 16) | (*(s + 43) << 8) | (*(s + 42)));
  data.velCovNN =
    decodeBinary32((*(s + 49) << 24) | (*(s + 48) << 16) | (*(s + 47) << 8) | (*(s + 46)));
  data.velCovNE =
    decodeBinary32((*(s + 53) << 24) | (*(s + 52) << 16) | (*(s + 51) << 8) | (*(s + 50)));
  data.velCovND =
    decodeBinary32((*(s + 57) << 24) | (*(s + 56) << 16) | (*(s + 55) << 8) | (*(s + 54)));
  data.velCovEE =
    decodeBinary32((*(s + 61) << 24) | (*(s + 60) << 16) | (*(s + 59) << 8) | (*(s + 58)));
  data.velCovED =
    decodeBinary32((*(s + 65) << 24) | (*(s + 64) << 16) | (*(s + 63) << 8) | (*(s + 62)));
  data.velCovDD =
    decodeBinary32((*(s + 69) << 24) | (*(s + 68) << 16) | (*(s + 67) << 8) | (*(s + 66)));
}

int Ublox::sendMessage(uint8_t msg_class, uint8_t msg_id, void* msg, uint16_t size)
{
  uint8_t buffer[kUbxBufferLength];

  UbxHeader header;
  header.preamble1 = PREAMBLE1;
  header.preamble2 = PREAMBLE2;
  header.msg_class = msg_class;
  header.msg_id = msg_id;
  header.length = size;

  auto offset = spliceMemory(buffer, &header, sizeof(UbxHeader));
  offset = spliceMemory(buffer, msg, size, offset);

  auto checksum = calculateCheckSum(buffer, offset);
  offset = spliceMemory(buffer, &checksum, sizeof(CheckSum), offset);

  return spi_dev_.transfer(buffer, nullptr, offset);
}

int Ublox::spliceMemory(uint8_t* dest, const void* const src, size_t size, int dest_offset)
{
  memmove(dest + dest_offset, src, size);
  return dest_offset + size;
}

Ublox::CheckSum Ublox::calculateCheckSum(uint8_t* message, size_t size) const
{
  CheckSum checksum;
  checksum.CK_A = checksum.CK_B = 0;

  for (int i = kPreambleOffset; i < size; i++)
  {
    checksum.CK_A += message[i];
    checksum.CK_B += checksum.CK_A;
  }
  return checksum;
}

#include <stdexcept>

#include "Ublox.h"
#include "Util.h"

#define PREAMBLE_OFFSET 2

using namespace std;

UBXScanner::UBXScanner()
{
  reset();
}

uint8_t* UBXScanner::getMessage()
{
  return message_;
}

uint32_t UBXScanner::getMessageLength()
{
  return message_length_;
}

uint32_t UBXScanner::getPosition()
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
    default:
      break;
  }

  return state_;
}

// class UBXParser

UBXParser::UBXParser(UBXScanner* ubxsc) : scanner_(ubxsc), message_(ubxsc->getMessage())
{
}

// This function updates message length and end position in the scanner's buffer

void UBXParser::updateMessageData()
{
  length_ = scanner_->getMessageLength();
  position_ = scanner_->getPosition();
}

// Function decodeMessage() returns 1 in case of a successful message verification.
// It looks through the buffer, checks 2 ubx protocol sync chars (0xb5 and 0x62),
// counts the checksum and if everything is alright, defines the message type by id.
// In this example we only decode two messages: Nav-Status and Nav-Posllh. It is possible to add
// more id cases

// datasheet:
// https://content.u-blox.com/sites/default/files/products/documents/u-blox8-M8_ReceiverDescrProtSpec_UBX-13003221.pdf

uint16_t UBXParser::calcId()
{
  const int pos = position_ - length_;  // count the message start position

  // All UBX messages start with 2 sync chars: 0xb5 and 0x62
  if (*(message_ + pos) != 0xb5)
    return 0;
  if (*(message_ + pos + 1) != 0x62)
    return 0;

  // Count the checksum
  uint8_t CK_A = 0, CK_B = 0;
  for (uint32_t i = 2; i < (length_ - 2); i++)
  {
    CK_A += *(message_ + pos + i);
    CK_B += CK_A;
  }
  if (CK_A != *(message_ + pos + length_ - 2))
    return 0;
  if (CK_B != *(message_ + pos + length_ - 1))
    return 0;

  // If we got everything right, then it's time to decide, what type of message this is
  return latest_id_ = (*(message_ + pos + 2)) << 8
                      | (*(message_ + pos + 3));  // ID is a two-byte number with little endianness
}

// Function checkMessage() returns 1 if the message, currently stored in the buffer is valid.
// Validity means, that the necessary sync chars are present and the checksum test is passed

int UBXParser::checkMessage()
{
  int flag = 1;
  int pos;
  uint8_t CK_A = 0, CK_B = 0;

  updateMessageData();        // get the length and end message coordinate from UBX scanner

  pos = position_ - length_;  // count the message start position

  // All UBX messages start with 2 sync chars: 0xb5 and 0x62

  if (*(message_ + pos) != 0xb5)
    flag = 0;
  if (*(message_ + pos + 1) != 0x62)
    flag = 0;

  // Count the checksum

  for (uint32_t i = 2; i < (length_ - 2); i++)
  {
    CK_A += *(message_ + pos + i);
    CK_B += CK_A;
  }

  if (CK_A != *(message_ + pos + length_ - 2))
    flag = 0;
  if (CK_B != *(message_ + pos + length_ - 1))
    flag = 0;

  return flag;
}

void UBXParser::decode(NavPayload_STATUS& data)
{
  if (latest_id_ != Ublox::NAV_STATUS)
  {
    throw runtime_error("Message type mismatch.");
  }

  int pos = position_ - length_;

  data.gpsFix = *(message_ + pos + 10);
  data.flags = *(message_ + pos + 11);
}

void UBXParser::decode(NavPayload_POSLLH& data)
{
  if (latest_id_ != Ublox::NAV_POSLLH)
  {
    throw runtime_error("Message type mismatch.");
  }

  int pos = position_ - length_;

  data.lon = ((*(message_ + pos + 13) << 24) | (*(message_ + pos + 12) << 16)
              | (*(message_ + pos + 11) << 8) | (*(message_ + pos + 10)))
             * 1e-7;
  data.lat = ((*(message_ + pos + 17) << 24) | (*(message_ + pos + 16) << 16)
              | (*(message_ + pos + 15) << 8) | (*(message_ + pos + 14)))
             * 1e-7;
  data.hMSL = ((*(message_ + pos + 25) << 24) | (*(message_ + pos + 24) << 16)
               | (*(message_ + pos + 23) << 8) | (*(message_ + pos + 22)))
              * 1e-3;
}

void UBXParser::decode(NavPayload_VELNED& data)
{
  if (latest_id_ != Ublox::NAV_VELNED)
  {
    throw runtime_error("Message type mismatch.");
  }

  int pos = position_ - length_;

  data.velN = ((*(message_ + pos + 13) << 24) | (*(message_ + pos + 12) << 16)
               | (*(message_ + pos + 11) << 8) | (*(message_ + pos + 10)))
              * 1e-2;
  data.velE = ((*(message_ + pos + 17) << 24) | (*(message_ + pos + 16) << 16)
               | (*(message_ + pos + 15) << 8) | (*(message_ + pos + 14)))
              * 1e-2;
  data.velD = ((*(message_ + pos + 21) << 24) | (*(message_ + pos + 20) << 16)
               | (*(message_ + pos + 19) << 8) | (*(message_ + pos + 18)))
              * 1e-2;
}

void UBXParser::decode(NavPayload_PVT& data)
{
  if (latest_id_ != Ublox::NAV_PVT)
  {
    throw runtime_error("Message type mismatch.");
  }

  int pos = position_ - length_;

  data.lon = ((*(message_ + pos + 33) << 24) | (*(message_ + pos + 32) << 16)
              | (*(message_ + pos + 31) << 8) | (*(message_ + pos + 30)))
             * 1e-7;
  data.lat = ((*(message_ + pos + 37) << 24) | (*(message_ + pos + 36) << 16)
              | (*(message_ + pos + 35) << 8) | (*(message_ + pos + 34)))
             * 1e-7;
  data.hMSL = ((*(message_ + pos + 45) << 24) | (*(message_ + pos + 44) << 16)
               | (*(message_ + pos + 43) << 8) | (*(message_ + pos + 42)))
              * 1e-3;
  data.velN = ((*(message_ + pos + 57) << 24) | (*(message_ + pos + 56) << 16)
               | (*(message_ + pos + 55) << 8) | (*(message_ + pos + 54)))
              * 1e-3;
  data.velE = ((*(message_ + pos + 61) << 24) | (*(message_ + pos + 60) << 16)
               | (*(message_ + pos + 59) << 8) | (*(message_ + pos + 58)))
              * 1e-3;
  data.velD = ((*(message_ + pos + 65) << 24) | (*(message_ + pos + 64) << 16)
               | (*(message_ + pos + 63) << 8) | (*(message_ + pos + 62)))
              * 1e-3;
}

void UBXParser::decode(NavPayload_COV& data)
{
  if (latest_id_ != Ublox::NAV_COV)
  {
    throw runtime_error("Message type mismatch.");
  }

  int pos = position_ - length_;

  data.posCovNN = decodeBinary32(
    (*(message_ + pos + 25) << 24) | (*(message_ + pos + 24) << 16) | (*(message_ + pos + 23) << 8)
    | (*(message_ + pos + 22)));
  data.posCovNE = decodeBinary32(
    (*(message_ + pos + 29) << 24) | (*(message_ + pos + 28) << 16) | (*(message_ + pos + 27) << 8)
    | (*(message_ + pos + 26)));
  data.posCovND = decodeBinary32(
    (*(message_ + pos + 33) << 24) | (*(message_ + pos + 32) << 16) | (*(message_ + pos + 31) << 8)
    | (*(message_ + pos + 30)));
  data.posCovEE = decodeBinary32(
    (*(message_ + pos + 37) << 24) | (*(message_ + pos + 36) << 16) | (*(message_ + pos + 35) << 8)
    | (*(message_ + pos + 34)));
  data.posCovED = decodeBinary32(
    (*(message_ + pos + 41) << 24) | (*(message_ + pos + 40) << 16) | (*(message_ + pos + 39) << 8)
    | (*(message_ + pos + 38)));
  data.posCovDD = decodeBinary32(
    (*(message_ + pos + 45) << 24) | (*(message_ + pos + 44) << 16) | (*(message_ + pos + 43) << 8)
    | (*(message_ + pos + 42)));
  data.velCovNN = decodeBinary32(
    (*(message_ + pos + 49) << 24) | (*(message_ + pos + 48) << 16) | (*(message_ + pos + 47) << 8)
    | (*(message_ + pos + 46)));
  data.velCovNE = decodeBinary32(
    (*(message_ + pos + 53) << 24) | (*(message_ + pos + 52) << 16) | (*(message_ + pos + 51) << 8)
    | (*(message_ + pos + 50)));
  data.velCovND = decodeBinary32(
    (*(message_ + pos + 57) << 24) | (*(message_ + pos + 56) << 16) | (*(message_ + pos + 55) << 8)
    | (*(message_ + pos + 54)));
  data.velCovEE = decodeBinary32(
    (*(message_ + pos + 61) << 24) | (*(message_ + pos + 60) << 16) | (*(message_ + pos + 59) << 8)
    | (*(message_ + pos + 58)));
  data.velCovED = decodeBinary32(
    (*(message_ + pos + 65) << 24) | (*(message_ + pos + 64) << 16) | (*(message_ + pos + 63) << 8)
    | (*(message_ + pos + 62)));
  data.velCovDD = decodeBinary32(
    (*(message_ + pos + 69) << 24) | (*(message_ + pos + 68) << 16) | (*(message_ + pos + 67) << 8)
    | (*(message_ + pos + 66)));
}

// class Ublox

Ublox::Ublox(string name)
  : spi_device_name_(name), scanner_(new UBXScanner()), parser_(new UBXParser(scanner_))
{
}

Ublox::Ublox(string name, UBXScanner* scan, UBXParser* pars)
  : spi_device_name_(name), scanner_(scan), parser_(pars)
{
}

int Ublox::enableNAV(message_t msg)
{
  constexpr size_t msg_size = 11;
  uint8_t tx[msg_size];  // UBX-CFG-MSG (p.216)

  // Header
  tx[0] = 0xb5;
  tx[1] = 0x62;

  // Class
  tx[2] = 0x06;

  // ID
  tx[3] = 0x01;

  // Length
  tx[4] = 0x03;  // 1の位
  tx[5] = 0x00;  // 16の位

  // Payload
  tx[6] = 0x01;               // msgClass
  tx[7] = msg - (0x01 << 8);  // msgID
  tx[8] = 0x01;               // rate

  // Checksum
  CheckSum ck = calculateCheckSum(tx, msg_size - 2);
  tx[9] = ck.CK_A;
  tx[10] = ck.CK_B;

  int length_ = (sizeof(tx) / sizeof(*tx));
  uint8_t rx[length_];

  return SPIdev::transfer(spi_device_name_.c_str(), tx, rx, length_, 200000);
}

int Ublox::disableNAV(message_t msg)
{
  constexpr size_t msg_size = 11;
  uint8_t tx[msg_size];  // UBX-CFG-MSG (p.216)

  // Header
  tx[0] = 0xb5;
  tx[1] = 0x62;

  // Class
  tx[2] = 0x06;

  // ID
  tx[3] = 0x01;

  // Length
  tx[4] = 0x03;  // 1の位
  tx[5] = 0x00;  // 16の位

  // Payload
  tx[6] = 0x01;               // msgClass
  tx[7] = msg - (0x01 << 8);  // msgID
  tx[8] = 0x00;               // rateを0Hzにすると発行されなくなる

  // Checksum
  CheckSum ck = calculateCheckSum(tx, msg_size - 2);
  tx[9] = ck.CK_A;
  tx[10] = ck.CK_B;

  int length_ = (sizeof(tx) / sizeof(*tx));
  uint8_t rx[length_];

  return SPIdev::transfer(spi_device_name_.c_str(), tx, rx, length_, 200000);
}

int Ublox::testConnection()
{
  int status;
  int count = 0;
  uint8_t to_gps_data = 0x00, from_gps_data = 0x00;

  // we do this, so that at least one ubx message is enabled

  while (count < UBX_BUFFER_LENGTH / 2)
  {
    // From now on, we will send zeroes to the receiver, which it will ignore
    // However, we are simultaneously getting useful information from it
    SPIdev::transfer(spi_device_name_.c_str(), &to_gps_data, &from_gps_data, 1, 200000);

    // Scanner checks the message structure with every byte received
    status = scanner_->update(from_gps_data);

    if (status == UBXScanner::Done)
    {
      // Once we have a full message we decode it and reset the scanner, making it look for another
      // message in the data stream, coming over SPI

      // If we find at least one valid message in the buffer, we consider connection to be
      // established
      if (parser_->checkMessage() == 1)
      {
        scanner_->reset();
        return 1;
      }

      scanner_->reset();
    }

    count++;
  }

  return 0;
}

int Ublox::configureSolutionRate(uint16_t meas_rate, uint16_t nav_rate, uint16_t timeref)
{
  CfgNavRate msg;
  msg.measure_rate = meas_rate;
  msg.nav_rate = nav_rate;
  msg.timeref = timeref;

  return sendMessage(CLASS_CFG, MSG_CFG_RATE, &msg, sizeof(CfgNavRate));
}

uint16_t Ublox::update()
{
  int status = -1;
  uint8_t to_gps_data = 0x00, from_gps_data = 0x00;

  while (status != UBXScanner::Done)
  {
    // From now on, we will send zeroes to the receiver, which it will ignore
    // However, we are simultaneously getting useful information from it
    SPIdev::transfer(spi_device_name_.c_str(), &to_gps_data, &from_gps_data, 1, 200000);

    // Scanner checks the message structure with every byte received
    status = scanner_->update(from_gps_data);
  }

  parser_->updateMessageData();
  scanner_->reset();
  return parser_->calcId();
}

void Ublox::decode(NavPayload_STATUS& data)
{
  parser_->decode(data);
}

void Ublox::decode(NavPayload_POSLLH& data)
{
  parser_->decode(data);
}

void Ublox::decode(NavPayload_VELNED& data)
{
  parser_->decode(data);
}

void Ublox::decode(NavPayload_PVT& data)
{
  parser_->decode(data);
}

void Ublox::decode(NavPayload_COV& data)
{
  parser_->decode(data);
}

int Ublox::sendMessage(uint8_t msg_class, uint8_t msg_id, void* msg, uint16_t size)
{
  uint8_t buffer[UBX_BUFFER_LENGTH];

  UbxHeader header;
  header.preamble1 = PREAMBLE1;
  header.preamble2 = PREAMBLE2;
  header.msg_class = msg_class;
  header.msg_id = msg_id;
  header.length = size;

  int offset = spliceMemory(buffer, &header, sizeof(UbxHeader));
  offset = spliceMemory(buffer, msg, size, offset);

  auto checksum = calculateCheckSum(buffer, offset);
  offset = spliceMemory(buffer, &checksum, sizeof(CheckSum), offset);

  return SPIdev::transfer(spi_device_name_.c_str(), buffer, nullptr, offset, 200000);
}

int Ublox::spliceMemory(uint8_t* dest, const void* const src, size_t size, int dest_offset)
{
  memmove(dest + dest_offset, src, size);
  return dest_offset + size;
}

Ublox::CheckSum Ublox::calculateCheckSum(uint8_t* message, size_t size)
{
  CheckSum checkSum;
  checkSum.CK_A = checkSum.CK_B = 0;

  for (int i = PREAMBLE_OFFSET; i < size; i++)
  {
    checkSum.CK_A += message[i];
    checkSum.CK_B += checkSum.CK_A;
  }
  return checkSum;
}

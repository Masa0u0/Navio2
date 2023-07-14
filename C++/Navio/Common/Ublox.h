#pragma once

#include <string>

#include "./SPIdev.h"
#include "./nav_payloads.hpp"

#define PACKED __attribute__((__packed__))

static constexpr uint32_t kUbxBufferLength = 1024;
static constexpr uint32_t kPreambleOffset = 2;
static constexpr uint32_t kSpiSpeedHz = 200000;
static constexpr uint32_t kConfigureMessageSize = 11;

class UBXScanner
{
public:
  enum State
  {
    Sync1,
    Sync2,
    Class,
    ID,
    Length1,
    Length2,
    Payload,
    CK_A,
    CK_B,
    Done,
  };

  explicit UBXScanner();

  uint8_t* getMessage();
  uint32_t getMessageLength() const;
  uint32_t getPosition() const;

  void reset();
  int update(uint8_t data);

private:
  uint8_t message_[kUbxBufferLength];  // Buffer for UBX message
  uint32_t message_length_;            // Length of the received message
  uint32_t position_;                  // Indicates current buffer offset
  uint32_t payload_length_;            // Length of current message payload
  State state_;                        // Current scanner state
};

class UBXParser
{
public:
  explicit UBXParser(UBXScanner* ubxsc);

  /* Updates message length and end position in the scanner's buffer. */
  void updateMessageData();
  uint16_t calcId();
  /**
   * @brief Returns 1 if the message, currently stored in the buffer is valid. Validity means, that
   * the necessary sync chars are present and the checksum test is passed.
   */
  int checkMessage();

  uint8_t* getMessage() const;
  uint32_t getLength() const;
  uint32_t getPosition() const;
  uint16_t getLatestId() const;

private:
  UBXScanner* scanner_;  // Pointer to the scanner, which finds the messages in the data stream

  uint8_t* message_;     // Pointer to the scanner's message buffer
  uint32_t length_;      // Current message length
  uint32_t position_;    // Current message end position
  uint16_t latest_id_;
};

/**
 * @brief Ublox handler.
 * datasheet:
 * https://content.u-blox.com/sites/default/files/products/documents/u-blox8-M8_ReceiverDescrProtSpec_UBX-13003221.pdf
 */
class Ublox
{
public:
  enum message_t
  {
    NAV_POSLLH = (0x01 << 8) + 0x02,
    NAV_STATUS = (0x01 << 8) + 0x03,
    NAV_PVT = (0x01 << 8) + 0x07,
    NAV_VELNED = (0x01 << 8) + 0x12,
    NAV_COV = (0x01 << 8) + 0x36,
  };

  explicit Ublox(std::string name = "/dev/spidev0.0");
  explicit Ublox(std::string name, UBXScanner* scan, UBXParser* pars);
  ~Ublox();

  int enableNAV(message_t msg);
  int disableNAV(message_t msg);

  /**
   * @brief Test connection with the receiver. Function testConnection() waits for a ubx protocol
   * message and checks it. If there's at least one correct message in the first 300 symbols the
   * test is passed.
   */
  int testConnection();

  /* 32.10.27.1 Navigation/measurement rate settings */
  int configureSolutionRate(uint16_t meas_rate, uint16_t nav_rate = 1, uint16_t timeref = 0);

  uint16_t update();

  void decode(NavPayload_POSLLH& data) const;
  void decode(NavPayload_STATUS& data) const;
  void decode(NavPayload_PVT& data) const;
  void decode(NavPayload_VELNED& data) const;
  void decode(NavPayload_COV& data) const;

private:
  enum ubx_protocol_bytes
  {
    PREAMBLE1 = 0xb5,
    PREAMBLE2 = 0x62,
    CLASS_CFG = 0x06,
    MSG_CFG_RATE = 0x08,
  };

  struct PACKED CfgNavRate
  {
    uint16_t measure_rate;
    uint16_t nav_rate;
    uint16_t timeref;
  };

  struct PACKED UbxHeader
  {
    uint8_t preamble1;
    uint8_t preamble2;
    uint8_t msg_class;
    uint8_t msg_id;
    uint16_t length;
  };

  struct PACKED CheckSum
  {
    uint8_t CK_A;
    uint8_t CK_B;
  };

  std::string spi_device_name_;
  UBXScanner* scanner_;
  UBXParser* parser_;

  int sendMessage(uint8_t msg_class, uint8_t msg_id, void* msg, uint16_t size);
  int spliceMemory(uint8_t* dest, const void* const src, size_t size, int dest_offset = 0);

  /* p.171, 32.4 UBX Checksum. */
  CheckSum calculateCheckSum(uint8_t* message, size_t size) const;
};

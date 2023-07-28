#pragma once

#include <string>

#include "./SPIdev.h"
#include "./nav_payloads.hpp"

#define PACKED __attribute__((__packed__))  // 構造体のメンバ変数がメモリ上で連続する

static constexpr uint32_t kUbxBufferLength = 1024;
static constexpr uint32_t kPreambleOffset = 2;
static constexpr uint32_t kSpiSpeedHz = 200000;  // Maximum frequency is 5.5MHz
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
private:
  enum ubx_protocol_bytes
  {
    PREAMBLE1 = 0xb5,
    PREAMBLE2 = 0x62,

    CLASS_CFG = 0x06,
    CLASS_NAV = 0x01,

    ID_CFG_MSG = 0x01,
    ID_CFG_RATE = 0x08,
    ID_CFG_NAV5 = 0x24,
    ID_CFG_GNSS = 0x3E,
    ID_NAV_POSLLH = 0x02,
    ID_NAV_STATUS = 0x03,
    ID_NAV_PVT = 0x07,
    ID_NAV_VELNED = 0x12,
    ID_NAV_COV = 0x36,
  };

public:
  // Class + ID
  enum message_t
  {
    NAV_POSLLH = (CLASS_NAV << 8) + ID_NAV_POSLLH,
    NAV_STATUS = (CLASS_NAV << 8) + ID_NAV_STATUS,
    NAV_PVT = (CLASS_NAV << 8) + ID_NAV_PVT,
    NAV_VELNED = (CLASS_NAV << 8) + ID_NAV_VELNED,
    NAV_COV = (CLASS_NAV << 8) + ID_NAV_COV,
  };

  enum dynamics_model
  {
    PORTABLE = 0,
    STATIONARY = 2,
    PEDESTRIAN = 3,
    AUTOMOTIVE = 4,
    SEA = 5,
    AIRBORNE_1G = 6,
    AIRBORNE_2G = 7,
    AIRBORNE_4G = 8,
    WRIST_WORN_WATCH = 9,
    MOTORBIKE = 10,
    ROBOTIC_LAWN_MOWER = 11,
    ELECTRIC_KICK_SCOOTER = 12,
  };

  explicit Ublox();
  explicit Ublox(UBXScanner* scan, UBXParser* pars);

  int enableNavMsg(message_t msg);
  int disableNavMsg(message_t msg);
  void enableAllNavMsgs();
  void disableAllNavMsgs();

  /* 32.10.27.1 Navigation/measurement rate settings */
  int configureSolutionRate(uint16_t meas_rate, uint16_t nav_rate = 1, uint16_t time_ref = 1);
  /* 32.10.19.1 Navigation engine settings */
  int configureDynamicsModel(dynamics_model dyn_model);

  uint16_t update();

  void decode(NavPayload_POSLLH& data) const;
  void decode(NavPayload_STATUS& data) const;
  void decode(NavPayload_PVT& data) const;
  void decode(NavPayload_VELNED& data) const;
  void decode(NavPayload_COV& data) const;

private:
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

  /* ===== Payload structures ===== */
  struct PACKED CfgMsg
  {
    uint8_t msgClass;
    uint8_t msgID;
    uint8_t rate;
  };

  struct PACKED CfgRate
  {
    uint16_t measRate;
    uint16_t navRate;
    uint16_t timeRef;
  };

  struct PACKED CfgNav5
  {
    uint16_t mask;
    uint8_t dynModel;
    uint8_t fixMode;
    int32_t fixedAlt;
    uint32_t fixedAltVar;
    int8_t minElev;
    uint8_t drLimit;
    uint16_t pDop;
    uint16_t tDop;
    uint16_t pAcc;
    uint16_t tAcc;
    uint8_t staticHoldThresh;
    uint8_t dgnssTimeout;
    uint8_t cnoThreshNumSVs;
    uint8_t cnoThresh;
    uint8_t reserved1[2];
    uint16_t staticHoldMaxDist;
    uint8_t utcStandard;
    uint8_t reserved2[5];
  };

  struct PACKED CfgGnss
  {
    // TODO
  };
  /* ==============================*/

  SPIdev spi_dev_;
  UBXScanner* scanner_;
  UBXParser* parser_;

  int sendMessage(uint8_t msg_class, uint8_t msg_id, void* msg, uint16_t size);
  int spliceMemory(uint8_t* dest, const void* const src, size_t size, int dest_offset = 0);

  /* p.171, 32.4 UBX Checksum. */
  CheckSum calculateCheckSum(uint8_t* message, size_t size) const;
};

/*
GPS driver dcode is placed under the BSD license.
Written by Egor Fedorov (egor.fedorov@emlid.com)
Copyright (c) 2014, Emlid Limited
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Emlid Limited nor the names of its contributors
      may be used to endorse or promote products derived from this software
      without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL EMLID LIMITED BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "Ublox.h"
#include "Util.h"

#define PREAMBLE_OFFSET 2
// class UBXScanner

using namespace std;

UBXScanner::UBXScanner()
{
    reset();
}

unsigned char* UBXScanner::getMessage()
{
    return message;
}

unsigned int UBXScanner::getMessageLength()
{
    return message_length;
}

unsigned int UBXScanner::getPosition()
{
    return position;
}

void UBXScanner::reset()
{
    message_length = 0;
    position = 0;
    state = Sync1;
}

int UBXScanner::update(unsigned char data)
{
    if (state != Done)
        message[position++] = data;

    switch (state)
    {
    case Sync1:
        if (data == 0xb5)
            state = Sync2;
        else
            reset();
        break;

    case Sync2:
        if (data == 0x62)
            state = Class;
        else
            if (data == 0xb5)
                state = Sync1;
            else
                reset();
        break;

    case Class:
        state = ID;
        break;

    case ID:
        state = Length1;
        break;

    case Length1:
        payload_length = data;
        state = Length2;
        break;

    case Length2:
        payload_length += data << 8;
        state = Payload;
        break;

    case Payload:
        if (position == payload_length + 6)
            state = CK_A;
        if (position >= 1022)
            reset();
        break;

    case CK_A:
        state = CK_B;
        break;

    case CK_B:
        message_length = 6 + payload_length + 2;
        state = Done;
        break;

    case Done:
    default:
        break;
    }

    return state;
}

// class UBXParser


UBXParser::UBXParser(UBXScanner* ubxsc) : scanner(ubxsc), message(ubxsc->getMessage())
{

}

// This function updates message length and end position in the scanner's buffer

void UBXParser::updateMessageData(){
    length = scanner->getMessageLength();
    position = scanner->getPosition();
}

// Function decodeMessage() returns 1 in case of a successful message verification.
// It looks through the buffer, checks 2 ubx protocol sync chars (0xb5 and 0x62),
// counts the checksum and if everything is alright, defines the message type by id.
// In this example we only decode two messages: Nav-Status and Nav-Posllh. It is possible to add more
// id cases

// datasheet: https://content.u-blox.com/sites/default/files/products/documents/u-blox8-M8_ReceiverDescrProtSpec_UBX-13003221.pdf

uint16_t UBXParser::calcId()
{
    const int pos = position-length; // count the message start position

    // All UBX messages start with 2 sync chars: 0xb5 and 0x62
    if (*(message+pos)!=0xb5) return 0;
    if (*(message+pos+1)!=0x62) return 0;

    // Count the checksum
    uint8_t CK_A=0, CK_B=0;
    for (unsigned int i=2;i<(length-2);i++){
        CK_A += *(message+pos+i);
        CK_B += CK_A;
    }
    if (CK_A != *(message+pos+length-2)) return 0;
    if (CK_B != *(message+pos+length-1)) return 0;

    // If we got everything right, then it's time to decide, what type of message this is
    return latest_id_ = (*(message+pos+2)) << 8 | (*(message+pos+3)); // ID is a two-byte number with little endianness
}

// Function checkMessage() returns 1 if the message, currently stored in the buffer is valid.
// Validity means, that the necessary sync chars are present and the checksum test is passed

int UBXParser::checkMessage()
{
    int flag=1;
    int pos;
    uint8_t CK_A=0, CK_B=0;

    updateMessageData(); // get the length and end message coordinate from UBX scanner

    pos = position-length; // count the message start position

    // All UBX messages start with 2 sync chars: 0xb5 and 0x62

    if (*(message+pos)!=0xb5) flag=0;
    if (*(message+pos+1)!=0x62) flag=0;

    // Count the checksum

    for (unsigned int i=2;i<(length-2);i++){
        CK_A += *(message+pos+i);
        CK_B += CK_A;
    }

    if (CK_A != *(message+pos+length-2)) flag=0;
    if (CK_B != *(message+pos+length-1)) flag=0;

    return flag;
}

void UBXParser::decode(NavPayload_STATUS& data)
{
    if (latest_id_ != Ublox::NAV_STATUS)
    {
        throw runtime_error("Message type mismatch.");
    }

    int pos = position - length;

    data.gpsFix = *(message+pos+10);
    data.flags = *(message+pos+11);
}

void UBXParser::decode(NavPayload_POSLLH& data)
{
    if (latest_id_ != Ublox::NAV_POSLLH)
    {
        throw runtime_error("Message type mismatch.");
    }

    int pos = position - length;

    data.lon = ((*(message+pos+13) << 24) | (*(message+pos+12) << 16) | (*(message+pos+11) << 8) | (*(message+pos+10))) * 1e-7;
    data.lat = ((*(message+pos+17) << 24) | (*(message+pos+16) << 16) | (*(message+pos+15) << 8) | (*(message+pos+14))) * 1e-7;
    data.hMSL = ((*(message+pos+25) << 24) | (*(message+pos+24) << 16) | (*(message+pos+23) << 8) | (*(message+pos+22))) * 1e-3;
}

void UBXParser::decode(NavPayload_VELNED& data)
{
    if (latest_id_ != Ublox::NAV_VELNED)
    {
        throw runtime_error("Message type mismatch.");
    }

    int pos = position - length;

    data.velN = ((*(message+pos+13) << 24) | (*(message+pos+12) << 16) | (*(message+pos+11) << 8) | (*(message+pos+10))) * 1e-2;
    data.velE = ((*(message+pos+17) << 24) | (*(message+pos+16) << 16) | (*(message+pos+15) << 8) | (*(message+pos+14))) * 1e-2;
    data.velD = ((*(message+pos+21) << 24) | (*(message+pos+20) << 16) | (*(message+pos+19) << 8) | (*(message+pos+18))) * 1e-2;
}

void UBXParser::decode(NavPayload_PVT& data)
{
    if (latest_id_ != Ublox::NAV_PVT)
    {
        throw runtime_error("Message type mismatch.");
    }

    int pos = position - length;

    data.lon = ((*(message+pos+33) << 24) | (*(message+pos+32) << 16) | (*(message+pos+31) << 8) | (*(message+pos+30))) * 1e-7;
    data.lat = ((*(message+pos+37) << 24) | (*(message+pos+36) << 16) | (*(message+pos+35) << 8) | (*(message+pos+34))) * 1e-7;
    data.hMSL = ((*(message+pos+45) << 24) | (*(message+pos+44) << 16) | (*(message+pos+43) << 8) | (*(message+pos+42))) * 1e-3;
    data.velN = ((*(message+pos+57) << 24) | (*(message+pos+56) << 16) | (*(message+pos+55) << 8) | (*(message+pos+54))) * 1e-3;
    data.velE = ((*(message+pos+61) << 24) | (*(message+pos+60) << 16) | (*(message+pos+59) << 8) | (*(message+pos+58))) * 1e-3;
    data.velD = ((*(message+pos+65) << 24) | (*(message+pos+64) << 16) | (*(message+pos+63) << 8) | (*(message+pos+62))) * 1e-3;
}

void UBXParser::decode(NavPayload_COV& data)
{
    if (latest_id_ != Ublox::NAV_COV)
    {
        throw runtime_error("Message type mismatch.");
    }

    int pos = position - length;

    data.posCovNN = decodeBinary32((*(message+pos+25) << 24) | (*(message+pos+24) << 16) | (*(message+pos+23) << 8) | (*(message+pos+22)));
    data.posCovNE = decodeBinary32((*(message+pos+29) << 24) | (*(message+pos+28) << 16) | (*(message+pos+27) << 8) | (*(message+pos+26)));
    data.posCovND = decodeBinary32((*(message+pos+33) << 24) | (*(message+pos+32) << 16) | (*(message+pos+31) << 8) | (*(message+pos+30)));
    data.posCovEE = decodeBinary32((*(message+pos+37) << 24) | (*(message+pos+36) << 16) | (*(message+pos+35) << 8) | (*(message+pos+34)));
    data.posCovED = decodeBinary32((*(message+pos+41) << 24) | (*(message+pos+40) << 16) | (*(message+pos+39) << 8) | (*(message+pos+38)));
    data.posCovDD = decodeBinary32((*(message+pos+45) << 24) | (*(message+pos+44) << 16) | (*(message+pos+43) << 8) | (*(message+pos+42)));
    data.velCovNN = decodeBinary32((*(message+pos+49) << 24) | (*(message+pos+48) << 16) | (*(message+pos+47) << 8) | (*(message+pos+46)));
    data.velCovNE = decodeBinary32((*(message+pos+53) << 24) | (*(message+pos+52) << 16) | (*(message+pos+51) << 8) | (*(message+pos+50)));
    data.velCovND = decodeBinary32((*(message+pos+57) << 24) | (*(message+pos+56) << 16) | (*(message+pos+55) << 8) | (*(message+pos+54)));
    data.velCovEE = decodeBinary32((*(message+pos+61) << 24) | (*(message+pos+60) << 16) | (*(message+pos+59) << 8) | (*(message+pos+58)));
    data.velCovED = decodeBinary32((*(message+pos+65) << 24) | (*(message+pos+64) << 16) | (*(message+pos+63) << 8) | (*(message+pos+62)));
    data.velCovDD = decodeBinary32((*(message+pos+69) << 24) | (*(message+pos+68) << 16) | (*(message+pos+67) << 8) | (*(message+pos+66)));
}

// class Ublox

Ublox::Ublox(std::string name) : spi_device_name(name), scanner(new UBXScanner()), parser(new UBXParser(scanner))
{
}

Ublox::Ublox(std::string name, UBXScanner* scan, UBXParser* pars) : spi_device_name(name), scanner(scan), parser(pars)
{
}

int Ublox::enableNAV(message_t msg)
{
    constexpr size_t msg_size = 11;
    unsigned char tx[msg_size];  // UBX-CFG-MSG (p.216)

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
    tx[6] = 0x01;  // msgClass
    tx[7] = msg - (0x01<<8);  // msgID
    tx[8] = 0x01;  // rate

    // Checksum
    CheckSum ck = _calculateCheckSum(tx, msg_size - 2);
    tx[9] = ck.CK_A;
    tx[10] = ck.CK_B;

    int length = (sizeof(tx)/sizeof(*tx));
    unsigned char rx[length];

    return SPIdev::transfer(spi_device_name.c_str(), tx, rx, length, 200000);
}

int Ublox::disableNAV(message_t msg)
{
    constexpr size_t msg_size = 11;
    unsigned char tx[msg_size];  // UBX-CFG-MSG (p.216)

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
    tx[6] = 0x01;  // msgClass
    tx[7] = msg - (0x01<<8);  // msgID
    tx[8] = 0x00;  // TODO: rateを0Hzにすると発行されなくなる？

    // Checksum
    CheckSum ck = _calculateCheckSum(tx, msg_size - 2);
    tx[9] = ck.CK_A;
    tx[10] = ck.CK_B;

    int length = (sizeof(tx)/sizeof(*tx));
    unsigned char rx[length];

    return SPIdev::transfer(spi_device_name.c_str(), tx, rx, length, 200000);
}

int Ublox::testConnection()
{
    int status;
    int count = 0;
    unsigned char to_gps_data = 0x00, from_gps_data = 0x00;

    // we do this, so that at least one ubx message is enabled

    while (count < UBX_BUFFER_LENGTH/2)
    {
        // From now on, we will send zeroes to the receiver, which it will ignore
        // However, we are simultaneously getting useful information from it
        SPIdev::transfer(spi_device_name.c_str(), &to_gps_data, &from_gps_data, 1, 200000);

        // Scanner checks the message structure with every byte received
        status = scanner->update(from_gps_data);

        if (status == UBXScanner::Done)
        {
            // Once we have a full message we decode it and reset the scanner, making it look for another message
            // in the data stream, coming over SPI

            // If we find at least one valid message in the buffer, we consider connection to be established
            if(parser->checkMessage()==1)
            {
                scanner->reset();
                return 1;
            }

            scanner->reset();
        }

        count++;
    }

    return 0;
}

int Ublox::configureSolutionRate(std::uint16_t meas_rate,
                           std::uint16_t nav_rate,
                           std::uint16_t timeref)
{
    CfgNavRate msg;
    msg.measure_rate = meas_rate;
    msg.nav_rate     = nav_rate;
    msg.timeref      = timeref;

    return _sendMessage(CLASS_CFG, MSG_CFG_RATE, &msg, sizeof(CfgNavRate));
}

uint16_t Ublox::update()
{
    int status = -1;
    unsigned char to_gps_data = 0x00, from_gps_data = 0x00;

    while (status != UBXScanner::Done)
    {
        // From now on, we will send zeroes to the receiver, which it will ignore
        // However, we are simultaneously getting useful information from it
        SPIdev::transfer(spi_device_name.c_str(), &to_gps_data, &from_gps_data, 1, 200000);

        // Scanner checks the message structure with every byte received
        status = scanner->update(from_gps_data);
    }

    parser->updateMessageData();
    scanner->reset();
    return parser->calcId();
}

void Ublox::decode(NavPayload_STATUS& data)
{
    parser->decode(data);
}

void Ublox::decode(NavPayload_POSLLH& data)
{
    parser->decode(data);
}

void Ublox::decode(NavPayload_VELNED& data)
{
    parser->decode(data);
}

void Ublox::decode(NavPayload_PVT& data)
{
    parser->decode(data);
}

void Ublox::decode(NavPayload_COV& data)
{
    parser->decode(data);
}


int Ublox::_sendMessage(std::uint8_t msg_class, std::uint8_t msg_id, void *msg, std::uint16_t size)
{
    unsigned char buffer[UBX_BUFFER_LENGTH];

    UbxHeader header;
    header.preamble1 = PREAMBLE1;
    header.preamble2 = PREAMBLE2;
    header.msg_class = msg_class;
    header.msg_id    = msg_id;
    header.length    = size;

    int offset = _spliceMemory(buffer, &header, sizeof(UbxHeader));
    offset = _spliceMemory(buffer, msg, size, offset);

    auto checksum = _calculateCheckSum(buffer, offset);
    offset = _spliceMemory(buffer, &checksum, sizeof(CheckSum), offset);

    return SPIdev::transfer(spi_device_name.c_str(), buffer, nullptr, offset, 200000);
}

int Ublox::_spliceMemory(unsigned char *dest, const void * const src, std::size_t size, int dest_offset)
{
    std::memmove(dest + dest_offset, src, size);
    return dest_offset + size;
}

Ublox::CheckSum Ublox::_calculateCheckSum(unsigned char *message, std::size_t size) {
    CheckSum checkSum;
    checkSum.CK_A = checkSum.CK_B = 0;

    for (int i = PREAMBLE_OFFSET; i < size; i++) {
        checkSum.CK_A += message[i];
        checkSum.CK_B += checkSum.CK_A;
    }
    return checkSum;
}

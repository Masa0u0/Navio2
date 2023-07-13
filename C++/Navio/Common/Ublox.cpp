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

int UBXParser::decodeMessage(std::vector<double>& data)
{
    int flag=1;
    int pos;
    uint16_t id;
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

    // If the sync chars, or the checksum is wrong, we should not be doing this anymore

    if (flag==0) return 0;

    // If we got everything right, then it's time to decide, what type of message this is

    id = (*(message+pos+2)) << 8 | (*(message+pos+3)); // ID is a two-byte number with little endianness

    flag = id; // will return message id, if we got the info decoded

    switch(id){
        case Ublox::NAV_POSLLH:
                // ID for Nav-Posllh messages is 0x0102 == 258
                // In this example we extract 7 variables - longitude, latitude,
                // height above ellipsoid and mean sea level, horizontal and vertical
                // accuracy estimate and iTOW - GPS Millisecond Time of Week

                // All the needed parameters are 4-byte numbers with little endianness.
                // We know the current message and we want to update the info in the data vector.
                // First we clear the old data:

                data.clear();

                // Second, we extract the needed data from the message buffer and save it to the vector.

                //iTOW
                data.push_back ((unsigned)((*(message+pos+9) << 24) | (*(message+pos+8) << 16) | (*(message+pos+7) << 8) | (*(message+pos+6))));
                //Longitude
                data.push_back ((*(message+pos+13) << 24) | (*(message+pos+12) << 16) | (*(message+pos+11) << 8) | (*(message+pos+10)));
                //Latitude
                data.push_back ((*(message+pos+17) << 24) | (*(message+pos+16) << 16) | (*(message+pos+15) << 8) | (*(message+pos+14)));
                //Height above Ellipsoid
                data.push_back ((*(message+pos+21) << 24) | (*(message+pos+20) << 16) | (*(message+pos+19) << 8) | (*(message+pos+18)));
                //Height above mean sea level
                data.push_back ((*(message+pos+25) << 24) | (*(message+pos+24) << 16) | (*(message+pos+23) << 8) | (*(message+pos+22)));
                //Horizontal Accuracy Estateimate
                data.push_back ((unsigned)((*(message+pos+29) << 24) | (*(message+pos+28) << 16) | (*(message+pos+27) << 8) | (*(message+pos+26))));
                //Vertical Accuracy Estateimate
                data.push_back ((unsigned)((*(message+pos+33) << 24) | (*(message+pos+32) << 16) | (*(message+pos+31) << 8) | (*(message+pos+30))));
                break;

        case Ublox::NAV_STATUS:
                // ID for Nav-Status messages is 0x0103 == 259
                // This message contains a lot of information, but the reason we use it the GPS fix status
                // This status is a one byte flag variable, with the offset of 10: first 6 bytes of the captured
                // message contain message header, id, payload length, next 4 are the first payload variable - iTOW.
                // We are not interested in it, so we just proceed to the fifth byte - gpsFix flag.
                // We are also interested in checking the gpsFixOk flag in the next byte/
                data.clear();
                // gpsFix
                data.push_back (*(message+pos+10));
                // flags, which contain gpsFixOk
                data.push_back (*(message+pos+11));

                break;
        
        case Ublox::NAV_PVT:
                // NAV-PVT, Class = 0x01, ID = 0x07

                data.clear();

                // lon
                data.push_back ((*(message+pos+33) << 24) | (*(message+pos+32) << 16) | (*(message+pos+31) << 8) | (*(message+pos+30)));
                // lat
                data.push_back ((*(message+pos+37) << 24) | (*(message+pos+36) << 16) | (*(message+pos+35) << 8) | (*(message+pos+34)));
                // height
                data.push_back ((*(message+pos+41) << 24) | (*(message+pos+40) << 16) | (*(message+pos+39) << 8) | (*(message+pos+38)));
                // velN
                data.push_back ((*(message+pos+57) << 24) | (*(message+pos+56) << 16) | (*(message+pos+55) << 8) | (*(message+pos+54)));
                // velE
                data.push_back ((*(message+pos+61) << 24) | (*(message+pos+60) << 16) | (*(message+pos+59) << 8) | (*(message+pos+58)));
                // velD
                data.push_back ((*(message+pos+65) << 24) | (*(message+pos+64) << 16) | (*(message+pos+63) << 8) | (*(message+pos+62)));

                break;
        
        case Ublox::NAV_COV:
                // NAV-COV, Class = 0x01, ID = 0x36

                data.clear();

                // posCovNN
                data.push_back (decodeBinary32((*(message+pos+25) << 24) | (*(message+pos+24) << 16) | (*(message+pos+23) << 8) | (*(message+pos+22))));
                // posCovNE
                data.push_back (decodeBinary32((*(message+pos+29) << 24) | (*(message+pos+28) << 16) | (*(message+pos+27) << 8) | (*(message+pos+26))));
                // posCovND
                data.push_back (decodeBinary32((*(message+pos+33) << 24) | (*(message+pos+32) << 16) | (*(message+pos+31) << 8) | (*(message+pos+30))));
                // posCovEE
                data.push_back (decodeBinary32((*(message+pos+37) << 24) | (*(message+pos+36) << 16) | (*(message+pos+35) << 8) | (*(message+pos+34))));
                // posCovED
                data.push_back (decodeBinary32((*(message+pos+41) << 24) | (*(message+pos+40) << 16) | (*(message+pos+39) << 8) | (*(message+pos+38))));
                // posCovDD
                data.push_back (decodeBinary32((*(message+pos+45) << 24) | (*(message+pos+44) << 16) | (*(message+pos+43) << 8) | (*(message+pos+42))));
                // velCovNN
                data.push_back (decodeBinary32((*(message+pos+49) << 24) | (*(message+pos+48) << 16) | (*(message+pos+47) << 8) | (*(message+pos+46))));
                // velCovNE
                data.push_back (decodeBinary32((*(message+pos+53) << 24) | (*(message+pos+52) << 16) | (*(message+pos+51) << 8) | (*(message+pos+50))));
                // velCovND
                data.push_back (decodeBinary32((*(message+pos+57) << 24) | (*(message+pos+56) << 16) | (*(message+pos+55) << 8) | (*(message+pos+54))));
                // velCovEE
                data.push_back (decodeBinary32((*(message+pos+61) << 24) | (*(message+pos+60) << 16) | (*(message+pos+59) << 8) | (*(message+pos+58))));
                // velCovED
                data.push_back (decodeBinary32((*(message+pos+65) << 24) | (*(message+pos+64) << 16) | (*(message+pos+63) << 8) | (*(message+pos+62))));
                // velCovDD
                data.push_back (decodeBinary32((*(message+pos+69) << 24) | (*(message+pos+68) << 16) | (*(message+pos+67) << 8) | (*(message+pos+66))));

                break;
        
        case Ublox::NAV_VELNED:
                // NAV-VELNED, Class = 0x01, ID = 0x12

                data.clear();

                // velN
                data.push_back ((*(message+pos+13) << 24) | (*(message+pos+12) << 16) | (*(message+pos+11) << 8) | (*(message+pos+10)));
                // velE
                data.push_back ((*(message+pos+17) << 24) | (*(message+pos+16) << 16) | (*(message+pos+15) << 8) | (*(message+pos+14)));
                // velD
                data.push_back ((*(message+pos+21) << 24) | (*(message+pos+20) << 16) | (*(message+pos+19) << 8) | (*(message+pos+18)));

                break;

        default:
                // In case we don't want to decode the received message
                flag = 0;

                break;
    }

    return flag;
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

// class Ublox

Ublox::Ublox(std::string name) : spi_device_name(name), scanner(new UBXScanner()), parser(new UBXParser(scanner))
{

}

Ublox::Ublox(std::string name, UBXScanner* scan, UBXParser* pars) : spi_device_name(name), scanner(scan), parser(pars)
{

}

int Ublox::enableNAV_POSLLH()
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
    tx[7] = 0x02;  // msgID
    tx[8] = 0x01;  // rate

    // Checksum
    CheckSum ck = _calculateCheckSum(tx, msg_size - 2);
    tx[9] = ck.CK_A;
    tx[10] = ck.CK_B;

    int length = (sizeof(tx)/sizeof(*tx));
    unsigned char rx[length];

    return SPIdev::transfer(spi_device_name.c_str(), tx, rx, length, 200000);
}

int Ublox::enableNAV_STATUS()
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
    tx[7] = 0x03;  // msgID
    tx[8] = 0x01;  // rate

    // Checksum
    CheckSum ck = _calculateCheckSum(tx, msg_size - 2);
    tx[9] = ck.CK_A;
    tx[10] = ck.CK_B;

    int length = (sizeof(tx)/sizeof(*tx));
    unsigned char rx[length];

    return SPIdev::transfer(spi_device_name.c_str(), tx, rx, length, 200000);
}

int Ublox::enableNAV_PVT()
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
    tx[7] = 0x07;  // msgID
    tx[8] = 0x01;  // rate

    // Checksum
    CheckSum ck = _calculateCheckSum(tx, msg_size - 2);
    tx[9] = ck.CK_A;
    tx[10] = ck.CK_B;

    int length = (sizeof(tx)/sizeof(*tx));
    unsigned char rx[length];

    return SPIdev::transfer(spi_device_name.c_str(), tx, rx, length, 200000);
}

int Ublox::enableNAV_COV()
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
    tx[7] = 0x36;  // msgID
    tx[8] = 0x01;  // rate

    // Checksum
    CheckSum ck = _calculateCheckSum(tx, msg_size - 2);
    tx[9] = ck.CK_A;
    tx[10] = ck.CK_B;

    int length = (sizeof(tx)/sizeof(*tx));
    unsigned char rx[length];

    return SPIdev::transfer(spi_device_name.c_str(), tx, rx, length, 200000);
}

int Ublox::enableNAV_VELNED()
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
    tx[7] = 0x12;  // msgID
    tx[8] = 0x01;  // rate

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

    if (enableNAV_POSLLH()<0)
    {
        std::cerr << "Could not configure ublox over SPI\n";
        return 0;
    }

    if (enableNAV_STATUS()<0)
    {
        std::cerr << "Could not configure ublox over SPI\n";
        return 0;
    }

    if (enableNAV_PVT()<0)
    {
        std::cerr << "Could not configure ublox over SPI\n";
        return 0;
    }

    if (enableNAV_COV()<0)
    {
        std::cerr << "Could not configure ublox over SPI\n";
        return 0;
    }

    if (enableNAV_VELNED()<0)
    {
        std::cerr << "Could not configure ublox over SPI\n";
        return 0;
    }

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

int Ublox::decodeMessages()
{
    int status;
    unsigned char to_gps_data = 0x00, from_gps_data = 0x00;
    std::vector<double> position_data;

    if (enableNAV_POSLLH()<0)
    {
        std::cerr << "Could not configure ublox over SPI\n";
    }

    if (enableNAV_STATUS()<0)
    {
        std::cerr << "Could not configure ublox over SPI\n";
    }

    if (enableNAV_PVT()<0)
    {
        std::cerr << "Could not configure ublox over SPI\n";
    }

    if (enableNAV_COV()<0)
    {
        std::cerr << "Could not configure ublox over SPI\n";
    }

    if (enableNAV_VELNED()<0)
    {
        std::cerr << "Could not configure ublox over SPI\n";
        return 0;
    }

    while (true)
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
            if (parser->decodeMessage(position_data) > 0)
            {
                // do something with the obtained data
                //
                // in case if NAV-POSLLH messages we can do this:
                // printf("decodeMessages(): \nCurrent location data:\n");
                // printf("GPS Millisecond Time of Week: %lf\n", position_data[0]/1000);
                // printf("Longitude: %lf\n", position_data[1]/10000000);
                // printf("Latitude: %lf\n", position_data[2]/10000000);
                // printf("Height above Ellipsoid: %.3lf m\n", pos_data[3]/1000);
                // printf("Height above mean sea level: %.3lf m\n", pos_data[4]/1000);
                // printf("Horizontal Accuracy Estateimate: %.3lf m\n", pos_data[5]/1000);
                // printf("Vertical Accuracy Estateimate: %.3lf m\n", pos_data[6]/1000);
                //
                // in case of NAV-STATUS messages we can do this:
                //
                // printf("Current GPS status:\n");
                // printf("gpsFixOk: %d\n", ((int)pos_data[1] & 0x01));
                //
                // printf("gps Fix status: ");
                // switch((int)pos_data[0]){
                //     case 0x00:
                //         printf("no fix\n");
                //         break;
                //
                //     case 0x01:
                //         printf("dead reckoning only\n");
                //         break;
                //
                //     case 0x02:
                //         printf("2D-fix\n");
                //         break;
                //
                //     case 0x03:
                //         printf("3D-fix\n");
                //         break;
                //
                //     case 0x04:
                //         printf("GPS + dead reckoning combined\n");
                //         break;
                //
                //     case 0x05:
                //         printf("Time only fix\n");
                //         break;
                //
                //     default:
                //         printf("Reserved value. Current state unknown\n");
                //         break;
                //
                // }
                //
                // printf("\n");

            }

            scanner->reset();
        }

        usleep(200);

    }

    return 0 ;
}

// int Ublox::decodeSingleMessage(message_t msg, std::vector<double>& position_data)
// {
//     switch(msg){
//         case NAV_POSLLH:
//             {
//                 uint16_t id = 0x0102;
//                 int status;
//                 int count = 0;
//                 unsigned char to_gps_data = 0x00, from_gps_data = 0x00;

//                 while (count < UBX_BUFFER_LENGTH/2)
//                 {
//                     // From now on, we will send zeroes to the receiver, which it will ignore
//                     // However, we are simultaneously getting useful information from it
//                     SPIdev::transfer(spi_device_name.c_str(), &to_gps_data, &from_gps_data, 1, 200000);

//                     // Scanner checks the message structure with every byte received
//                     status = scanner->update(from_gps_data);

//                     if (status == UBXScanner::Done)
//                     {
//                         // Once we have a full message we decode it and reset the scanner, making it look for another message
//                         // in the data stream, coming over SPI
//                         if(parser->decodeMessage(position_data) == id)
//                         {
//                             // Now let's do something with the extracted information
//                             // in case of NAV-POSLLH messages we can print the information like this:
//                             // printf("GPS Millisecond Time of Week: %lf\n", position_data[0]/1000);
//                             // printf("Longitude: %lf\n", position_data[1]/10000000);
//                             // printf("Latitude: %lf\n", position_data[2]/10000000);
//                             // printf("Height above Ellipsoid: %.3lf m\n", pos_data[3]/1000);
//                             // printf("Height above mean sea level: %.3lf m\n", pos_data[4]/1000);
//                             // printf("Horizontal Accuracy Estateimate: %.3lf m\n", pos_data[5]/1000);
//                             // printf("Vertical Accuracy Estateimate: %.3lf m\n", pos_data[6]/1000);


//                             // You can see ubx message structure in ublox reference manual

//                             scanner->reset();

//                             return 1;
//                         }

//                         scanner->reset();
//                     }

//                     count++;
//                 }

//                 return 0;
//             }

//         break;

//         case NAV_STATUS:
//             {
//                 uint16_t id = 0x0103;
//                 int status;
//                 int count = 0;
//                 unsigned char to_gps_data = 0x00, from_gps_data = 0x00;

//                 while (count < UBX_BUFFER_LENGTH/2)
//                 {
//                     // From now on, we will send zeroes to the receiver, which it will ignore
//                     // However, we are simultaneously getting useful information from it
//                     SPIdev::transfer(spi_device_name.c_str(), &to_gps_data, &from_gps_data, 1, 200000);

//                     // Scanner checks the message structure with every byte received
//                     status = scanner->update(from_gps_data);

//                     if (status == UBXScanner::Done)
//                     {
//                         // Once we have a full message we decode it and reset the scanner, making it look for another message
//                         // in the data stream, coming over SPI
//                         if(parser->decodeMessage(position_data) == id)
//                         {
//                             // Now let's do something with the extracted information
//                             // in case of NAV-STATUS messages we can do this:
//                             //
//                             // printf("Current GPS status:\n");
//                             // printf("gpsFixOk: %d\n", ((int)pos_data[1] & 0x01));
//                             //
//                             // printf("gps Fix status: ");
//                             // switch((int)pos_data[0]){
//                             //     case 0x00:
//                             //         printf("no fix\n");
//                             //         break;
//                             //
//                             //     case 0x01:
//                             //         printf("dead reckoning only\n");
//                             //         break;
//                             //
//                             //     case 0x02:
//                             //         printf("2D-fix\n");
//                             //         break;
//                             //
//                             //     case 0x03:
//                             //         printf("3D-fix\n");
//                             //         break;
//                             //
//                             //     case 0x04:
//                             //         printf("GPS + dead reckoning combined\n");
//                             //         break;
//                             //
//                             //     case 0x05:
//                             //         printf("Time only fix\n");
//                             //         break;
//                             //
//                             //     default:
//                             //         printf("Reserved value. Current state unknown\n");
//                             //         break;
//                             //
//                             // }
//                             //
//                             // printf("\n");

//                             // You can see ubx message structure in ublox reference manual

//                             scanner->reset();

//                             return 1;
//                         }

//                         scanner->reset();
//                     }

//                     count++;
//                 }

//                 return 0;
//             }

//         break;


//         // add your ubx message type here!

//         default:
//             return 0;

//         break;
//     }
// }

int Ublox::decodeSingleMessage(message_t msg, std::vector<double>& position_data)
{
    int status;
    int count = 0;
    unsigned char to_gps_data = 0x00, from_gps_data = 0x00;

    while (count < UBX_BUFFER_LENGTH/2)
    {
        SPIdev::transfer(spi_device_name.c_str(), &to_gps_data, &from_gps_data, 1, 200000);
        status = scanner->update(from_gps_data);
        if (status == UBXScanner::Done)
        {
            if (parser->decodeMessage(position_data) == msg)
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

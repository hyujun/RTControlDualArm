//
// Created by parkjunho on 2020-10-24.
//

#ifndef RTCONTROLDUALARM_PACKETHANDLER_H
#define RTCONTROLDUALARM_PACKETHANDLER_H

#include <cstdlib>
#include <cstring>

#define INDEX_TASK_TARGET 0x6090
#define Index_HandCommand_request 0x3333

struct packet_data{
    float index;
    float subindex;
    float _x;
    float _y;
    float _z;
    float _u;
    float _v;
    float _w;
};

union TCP_Packet{
    packet_data info;
    unsigned char data[sizeof(info)];
};

class PacketHandler{

public:
    PacketHandler(){}
    virtual ~PacketHandler(){}

    packet_data packetlibrary(char *uint8Data, uint8_t d_size)
    {
        memcpy(packet.data, uint8Data, d_size);
        return packet.info;
    }

    packet_data packetlibrary(TCP_Packet &_packet)
    {
        packet = _packet;
        return packet.info;
    }

private:
    TCP_Packet packet;
};

#endif //RTCONTROLDUALARM_PACKETHANDLER_H

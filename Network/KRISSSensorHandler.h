//
// Created by parkjunho on 2020-10-24.
//

#ifndef RTCONTROLDUALARM_KRISSSENSORHANDLER_H
#define RTCONTROLDUALARM_KRISSSENSORHANDLER_H


#include <strings.h>
#include <cstdlib>
#include <cstdint>

#define KRISS_PORT 6340
#define KRISS_IP_ADD 192.168.1.1

struct packet_classifier{
    uint8_t classifier[4];
};

union packet_ftdata{
    struct{
        float Fz[12];
        float Fx[12];
        float Fy[12];
    }info;
    uint8_t data[160];
};

union packet_kriss{
    struct{
        unsigned char header[3];
        packet_classifier classifier_info;
        packet_ftdata ftdata_info;
        unsigned char tail[3];
    }info;
    char data[170];
};



class SensorHandler{
public:
    SensorHandler(){}
    virtual ~SensorHandler(){}
    packet_ftdata packetlibrary(char *uint8Data, uint8_t d_size)
    {
        memcpy(mPacketKriss.data, uint8Data, d_size);
        return mPacketKriss.info.ftdata_info;
    }

    packet_ftdata packetlibrary(packet_kriss &_packet)
    {
        mPacketKriss = _packet;
        return mPacketKriss.info.ftdata_info;
    }

private:
    packet_kriss mPacketKriss;

};

#endif //RTCONTROLDUALARM_KRISSSENSORHANDLER_H

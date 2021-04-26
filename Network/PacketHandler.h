//
// Created by parkjunho on 2020-10-24.
//

#ifndef RTCONTROLDUALARM_PACKETHANDLER_H
#define RTCONTROLDUALARM_PACKETHANDLER_H

#include <cstdlib>
#include <cstring>

#define INDEX_TASK_TARGET 0x6090
#define Index_HandCommand_request 0x3333

struct packet_data_joint{
    unsigned char index1;
    unsigned char index2;
    unsigned char subindex;
    //float Target[16];
};

struct packet_data_task{
    unsigned char index1;
    unsigned char index2;
    unsigned char subindex;
    //float Target[12];
};

union TCP_Packet_Joint{
    packet_data_joint info;
    unsigned char data[sizeof(packet_data_joint)];
};

union TCP_Packet_Task{
    packet_data_task info;
    unsigned char data[sizeof(packet_data_task)];
};

class PacketHandler{

public:
    PacketHandler(){}
    virtual ~PacketHandler(){}


private:
    TCP_Packet_Joint packet_joint;
    TCP_Packet_Task packet_task;
};

#endif //RTCONTROLDUALARM_PACKETHANDLER_H

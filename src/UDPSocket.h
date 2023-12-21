//
// Created by adalberto-oliveira on 13/06/23.
//
/*
    Header file containing classess used to perform UDP communication.

    Author: Adalberto Oliveira <adalberto.oliveira@solinftec.com>
    Project: Solix AgBot
    Version: 1.0
    Date: 6-13-2023
*/
#ifndef CATKIN_WS_USPSOCKET_H
#define CATKIN_WS_USPSOCKET_H

#include <bits/stdc++.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <iostream>
#include <string>

class UDPSocket
{//Class begin

    /*
     * Class to handle communication issues.
     */

private:
    int sock;

    uint32_t
        PORT;

    struct sockaddr_in servAddr;

public:
    //Constructor
    UDPSocket();

    UDPSocket(uint32_t port);

    ~UDPSocket();


    // Workers
    int creatSocket();

    void setPort(uint32_t port);

    void setServer(uint32_t port);

    void send(std::string msg);


};


#endif //CATKIN_WS_USPSOCKET_H

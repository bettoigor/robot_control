//
// Created by adalberto-oliveira on 13/06/23.
//

/*
    File containing methods used to perform UDP communication.

    Author: Adalberto Oliveira <adalberto.oliveira@solinftec.com>
    Project: Solix AgBot
    Version: 1.0
    Date: 6-13-2-23
*/

#include "UDPSocket.h"


UDPSocket::UDPSocket()
{
    int is_valid = this->creatSocket();
    if (!is_valid)
    {
        std::cout << "\nSocket failure!";
        exit(EXIT_FAILURE);
    }
    else
    {
        std::cout << "\nSocket successfully created!\n";
    }

}


UDPSocket::UDPSocket(uint32_t port)
{
    int is_valid = this->creatSocket();
    if (!is_valid)
    {
        std::cout << "\nSocket creation failure!\n";
        exit(EXIT_FAILURE);
    }
    else
    {
        // Setting server information
        this->setServer(port);
        std::cout << "\nSocket created successfully! Server set successfully!\n";
    }

}

UDPSocket::~UDPSocket()
{

    //close(sock);

}

// Workers

int UDPSocket::creatSocket()
{
    sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0)
    {
        return false;
    }

    return true;

}

void UDPSocket::setPort(uint32_t port)
{

    PORT = port;

}

void UDPSocket::setServer(uint32_t port)
{
    this->setPort(port);
    servAddr.sin_family = AF_INET;
    servAddr.sin_port = htons(PORT);
    servAddr.sin_addr.s_addr = INADDR_ANY;
    std::cout << "Server parameters successfully configured!";
}


void UDPSocket::send(std::string msg)
{
    int ret = 0;
    std::cout << "\nMessage: " << msg.c_str() << "\nLength: " << msg.length() << std::endl;


    sendto(sock, msg.c_str(), msg.length()+1,MSG_CONFIRM,
           (const struct sockaddr *) &servAddr, sizeof(servAddr));

    if (ret < 0)
    {
        std::cout << "\nFail to send message\n";
    }
    std::cout << "\nMessage sent!\n";
}


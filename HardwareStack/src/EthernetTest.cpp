#include <iostream>
#include <chrono>
#include <string>
#include <cstring>
#include <memory>
#include <bitset>
#include <cstddef>

#include <arpa/inet.h> // htons, inet_addr
#include <netinet/in.h> // sockaddr_in
#include <sys/types.h> // uint16_t
#include <sys/socket.h> // socket, sendto
#include <unistd.h> // close

int main(int argc, char const *argv[])
{   
    std::string hostname{"10.0.0.6"};               // IP Adress of the MACHINE communicating with the teensy
    std::string destname{"10.0.0.7"};               // IP Address of the TEENSY communicating with the machine
    uint16_t port = 4333;                           // Port over which you want to communicate

    int sock = ::socket(AF_INET, SOCK_DGRAM, 0);    // AF_INET for IPV4, SOCK_DGRAM for UDP, 0 for IP (protocol value)

    sockaddr_in source, destination;
    source.sin_family = AF_INET;
    source.sin_port = htons(port);
    source.sin_addr.s_addr = inet_addr(hostname.c_str());

    destination.sin_family = AF_INET;
    destination.sin_port = htons(port);
    destination.sin_addr.s_addr = inet_addr(destname.c_str());
    socklen_t destinationAddrLen = sizeof(destination);

    int enabled = 1;
    setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &enabled, sizeof(enabled));

    int val = bind(sock, (struct sockaddr*)&source,
             sizeof(source));
    // std::cout << val << std::endl;
    // perror("bind");
    // printf("Error binding socket: %d\n", errno);

    char buffer[sizeof(float) *4];
    sockaddr_in senderAddr;
    senderAddr.sin_family = AF_INET;
    senderAddr.sin_port = htons(port);
    senderAddr.sin_addr.s_addr = inet_addr(hostname.c_str());
    socklen_t senderAddrLen = sizeof(senderAddr);

    //ssize_t is used for functions whose return value could either be a valid size, or a negative value to indicate an error
    ssize_t recvBytes = 0;
    ssize_t n_bytes = 0;

    /*
    TIMING
    auto t1 = std::chrono::high_resolution_clock::now();
    auto t2 = std::chrono::high_resolution_clock::now();
    long long microseconds = std::chrono::duration_cast<std::chrono::microseconds>(
        t2 - t1).count();
    */

    while(1) {

        float value[5] = {-1.0, -4.322,3434.2,-242,424.4};


        unsigned char line[sizeof(float)*5];
        memcpy(line, value, sizeof(float)*5);

        // n_bytes = ::sendto(sock, msg.c_str(), msg.length(), 0, reinterpret_cast<sockaddr*>(&destination), destinationAddrLen);
        n_bytes = ::sendto(sock, line, sizeof(float)*5, 0, reinterpret_cast<sockaddr*>(&destination), destinationAddrLen);

        //std::cout << n_bytes << " bytes sent" << std::endl;
        
        /*
        TIMING
        t2 = std::chrono::high_resolution_clock::now(); 
        microseconds = std::chrono::duration_cast<std::chrono::microseconds>(t2-t1).count();
        std::cout << microseconds<< std::endl;
        */
        recvBytes = ::recvfrom(sock, buffer, sizeof(buffer), 0, reinterpret_cast<sockaddr*>(&senderAddr), &senderAddrLen);
        std::cout << "Received: "  << std::endl;
        
        float data[4] = {0.0,0.0,0.0,0.0};
        std::memcpy(data, buffer, sizeof(data));

        for(int i = 0; i<sizeof(data)/sizeof(float); i++){
            std::cout<<data[i]<< " , ";
        }
        std::cout<<std::endl;
        
        /*
        TIMING
        t2 = std::chrono::high_resolution_clock::now(); 
        microseconds = std::chrono::duration_cast<std::chrono::microseconds>(t2-t1).count();
        std::cout << microseconds<< std::endl;
        t1 = t2;
        */
    }

    ::close(sock);              // close the socket


    return 0;
}

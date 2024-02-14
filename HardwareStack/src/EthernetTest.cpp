#include <iostream>
#include <chrono>
#include <string>

#include <arpa/inet.h> // htons, inet_addr
#include <netinet/in.h> // sockaddr_in
#include <sys/types.h> // uint16_t
#include <sys/socket.h> // socket, sendto
#include <unistd.h> // close

int main(int argc, char const *argv[])
{
    std::string hostname{"10.0.0.6"};
    std::string destname{"10.0.0.7"};
    uint16_t port = 4333;

    int sock = ::socket(AF_INET, SOCK_DGRAM, 0);

    sockaddr_in source, destination;
    source.sin_family = AF_INET;
    source.sin_port = htons(port);
    source.sin_addr.s_addr = inet_addr(hostname.c_str());

    destination.sin_family = AF_INET;
    destination.sin_port = htons(port);
    destination.sin_addr.s_addr = inet_addr(destname.c_str());

    int enabled = 1;
    setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &enabled, sizeof(enabled));

    int val = bind(sock, (struct sockaddr*)&source,
             sizeof(source));
    std::cout << val << std::endl;
    perror("bind");
    printf("Error binding socket: %d\n", errno);

    std::string msg = "So, if you'd like to simplify it down and make it more C-like, including removing all of the type-safe class stuff it does, here are 3 simple and very easy-to-use functions to get timestamps in milliseconds, microseconds, and nanoseconds...that only took me about 12 hrs to write*:";

   char buffer[1024];
    sockaddr_in senderAddr;
    senderAddr.sin_family = AF_INET;
    senderAddr.sin_port = htons(port);
    senderAddr.sin_addr.s_addr = inet_addr(hostname.c_str());
    socklen_t senderAddrLen = sizeof(senderAddr);

    ssize_t recvBytes = 0;
    // do {
        //recvBytes = ::recvfrom(sock, buffer, sizeof(buffer), 0, reinterpret_cast<sockaddr*>(&senderAddr), &senderAddrLen);
	//std::cout << recvBytes << std::endl;
    // } while(recvBytes==0);

    int n_bytes = 0;

    auto t1 = std::chrono::high_resolution_clock::now();
    auto t2 = std::chrono::high_resolution_clock::now();
    long long microseconds = std::chrono::duration_cast<std::chrono::microseconds>(
        t2 - t1).count();

    while(1) {
    n_bytes = ::sendto(sock, msg.c_str(), msg.length(), 0, reinterpret_cast<sockaddr*>(&destination), sizeof(destination));
    //std::cout << n_bytes << " bytes sent" << std::endl;
    t2 = std::chrono::high_resolution_clock::now(); 
    microseconds = std::chrono::duration_cast<std::chrono::microseconds>(t2-t1).count();
    std::cout << microseconds<< std::endl;
    recvBytes = ::recvfrom(sock, buffer, sizeof(buffer), 0, reinterpret_cast<sockaddr*>(&senderAddr), &senderAddrLen);
    std::cout << "Received: " << buffer << std::endl;
    t2 = std::chrono::high_resolution_clock::now(); 
    microseconds = std::chrono::duration_cast<std::chrono::microseconds>(t2-t1).count();
    std::cout << microseconds<< std::endl;
    t1 = t2;
    }

    ::close(sock);


    return 0;
}

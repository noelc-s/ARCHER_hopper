#include <iostream>
#include <string>

#include <arpa/inet.h> // htons, inet_addr
#include <netinet/in.h> // sockaddr_in
#include <sys/types.h> // uint16_t
#include <sys/socket.h> // socket, sendto
#include <unistd.h> // close

int main(int argc, char const *argv[])
{
    std::string hostname{"10.0.0.7"};
    uint16_t port = 4333;

    int sock = ::socket(AF_INET, SOCK_DGRAM, 0);

    sockaddr_in destination;
    destination.sin_family = AF_INET;
    destination.sin_port = htons(port);
    destination.sin_addr.s_addr = inet_addr(hostname.c_str());

    std::string msg = "Jane Doe";
    int n_bytes = ::sendto(sock, msg.c_str(), msg.length(), 0, reinterpret_cast<sockaddr*>(&destination), sizeof(destination));
    std::cout << n_bytes << " bytes sent" << std::endl;
    
   char buffer[1024];
    sockaddr_in senderAddr;
    socklen_t senderAddrLen = sizeof(senderAddr);

    ssize_t recvBytes = 0;
    // do {
        recvBytes = ::recvfrom(sock, buffer, sizeof(buffer), 0, reinterpret_cast<sockaddr*>(&senderAddr), &senderAddrLen);
        std::cout << recvBytes << std::endl;
    // } while(recvBytes==0);

    // ssize_t recvBytes = ::recvfrom(sock, buffer, sizeof(buffer), 0, reinterpret_cast<sockaddr*>(&senderAddr), &senderAddrLen);
    if (recvBytes > 0) {
        std::cout<<"YEP"<<std::endl;
        buffer[recvBytes] = '\0'; // Null-terminate the received data
        std::cout << "Received: " << buffer << std::endl;
    } else if (recvBytes == 0) {
        std::cerr << "Connection closed by peer\n";
    } else {
        std::cerr << "Error receiving data\n";
    }

    ::close(sock);


    return 0;
}
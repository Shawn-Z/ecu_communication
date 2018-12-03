#include "UDPServer.hpp"

namespace ecu_communication {

bool UDPServer::init(uint16_t port) {
    this->server_sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (this->server_sockfd < 0) {
        this->error = udp_server_error_type::get_server_sockfd_error;
        return false;
    }
    bzero(&this->server_ip, sizeof(this->server_ip));
    this->server_ip.sin_family = AF_INET;
    this->server_ip.sin_port = htons(port);
    this->server_ip.sin_addr.s_addr = INADDR_ANY;
    if ((bind(this->server_sockfd, (sockaddr *)(&this->server_ip), sizeof(this->server_ip))) < 0) {
        this->error = udp_server_error_type::bind_error;
        return false;
    }
    return true;
}

void UDPServer::process() {
    bzero(this->buffer, BUFFERSIZE);
    bzero(&this->client_ip, sizeof(this->client_ip));
    this->client_ip_len = sizeof(this->client_ip);
    //// todo test if need
    this->buffer[BUFFERSIZE] = 0;
    this->recv_len = recvfrom(this->server_sockfd, this->buffer, BUFFERSIZE, 0, (struct sockaddr *)(&this->client_ip), &this->client_ip_len);
}

const char* UDPServer::getClientIP() {
    //// todo need test
    return inet_ntoa(this->client_ip.sin_addr);
}

UDPServer::UDPServer() {
    this->error = udp_server_error_type::none;
    bzero(&this->server_ip, sizeof(this->server_ip));
    bzero(&this->client_ip, sizeof(this->client_ip));
}

intmax_t UDPServer::get_recv_len() {
    return this->recv_len;
}

}
#include "UDPServer.hpp"

namespace ecu_communication {

bool UDPServer::init(uint16_t port) {
    this->server_sockfd_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (this->server_sockfd_ < 0) {
        this->error = udp_server_error_type::get_server_sockfd_error;
        return false;
    }
    bzero(&this->server_ip_, sizeof(this->server_ip_));
    this->server_ip_.sin_family = AF_INET;
    this->server_ip_.sin_port = htons(port);
    this->server_ip_.sin_addr.s_addr = INADDR_ANY;
    if ((bind(this->server_sockfd_, (sockaddr *)(&this->server_ip_), sizeof(this->server_ip_))) < 0) {
        this->error = udp_server_error_type::bind_error;
        return false;
    }
    return true;
}

bool UDPServer::process(uint8_t *buffer, uintmax_t buffer_size) {
    bzero(buffer, buffer_size);
    bzero(&this->client_ip_, sizeof(this->client_ip_));
    this->client_ip_len_ = sizeof(this->client_ip_);
    buffer[buffer_size] = 0;
    this->recv_len_ = recvfrom(this->server_sockfd_, buffer, buffer_size, 0, (struct sockaddr *)(&this->client_ip_), &this->client_ip_len_);
    if (this->recv_len_ != buffer_size) {
        this->error = udp_server_error_type::recv_from_error;
        return false;
    }
    return true;
}

const char* UDPServer::getClientIP() {
    //// todo need test
    return inet_ntoa(this->client_ip_.sin_addr);
}

UDPServer::UDPServer() {
    this->error = udp_server_error_type::none;
    bzero(&this->server_ip_, sizeof(this->server_ip_));
    bzero(&this->client_ip_, sizeof(this->client_ip_));
}

intmax_t UDPServer::get_recv_len() {
    return this->recv_len_;
}

}
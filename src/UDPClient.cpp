#include "UDPClient.hpp"

namespace ecu_communication {

bool UDPClient::init(const char *ip_send_to, uint16_t port_send_to) {
    //// todo catch the error if init failure
    //// todo check ip_send_to anf port_send_to first, the same for server
    this->client_sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (this->client_sockfd_ < 0) {
        this->error = udp_client_error_type::get_client_sockfd_error;
        return false;
    }
    bzero(&this->server_ip_, sizeof(this->server_ip_));
    this->server_ip_.sin_family = AF_INET;
    this->server_ip_.sin_port = htons(port_send_to);
    this->server_ip_.sin_addr.s_addr = inet_addr(ip_send_to);
    return true;
}

bool UDPClient::process(uint8_t *buffer, intmax_t buffer_size) {
    this->send_len_ = sendto(this->client_sockfd_, buffer, buffer_size, 0, (struct sockaddr *)(&this->server_ip_), sizeof(this->server_ip_));
    if (this->send_len_ != buffer_size) {
        this->error = udp_client_error_type::send_error;
        return false;
    }
    return true;
}

UDPClient::UDPClient() {
    this->error = udp_client_error_type::none;
    bzero(&this->server_ip_, sizeof(this->server_ip_));
}

intmax_t UDPClient::get_send_len() {
    return this->send_len_;
}

}
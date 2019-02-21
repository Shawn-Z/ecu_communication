#include "UDPCommunication.hpp"

namespace ecu_communication {

bool UDPCommunication::init() {
    this->error = udp_server_error_type::none;
    bzero(this->buffer, BUFFERSIZE);

    bzero(&this->local_addr, sizeof(this->local_addr));
    bzero(&this->remote_addr, sizeof(this->remote_addr));
    bzero(&this->client_addr, sizeof(this->client_addr));
    this->remote_addr_len = sizeof(this->remote_addr);
    this->client_addr_len = sizeof(this->client_addr);

    this->local_addr.sin_family = AF_INET;
    this->local_addr.sin_port = htons(this->params.local_port);
    this->local_addr.sin_addr.s_addr = inet_addr(this->params.local_ip.c_str());

    this->remote_addr.sin_family = AF_INET;
    this->remote_addr.sin_port = htons(this->params.remote_port);
    this->remote_addr.sin_addr.s_addr = inet_addr(this->params.remote_ip.c_str());

    this->sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (this->sockfd < 0) {
        this->error = udp_server_error_type::get_sockfd_error;
        return false;
    }
    if ((bind(this->sockfd, (sockaddr *)(&this->local_addr), sizeof(this->local_addr))) < 0) {
        this->error = udp_server_error_type::bind_error;
        return false;
    }
    return true;
}

intmax_t UDPCommunication::recv() {
    bzero(this->buffer, BUFFERSIZE);
    bzero(&this->client_addr, sizeof(this->client_addr));
    //// todo test if need
    this->buffer[BUFFERSIZE] = 0;
    this->recv_len = recvfrom(this->sockfd, this->buffer, BUFFERSIZE, 0, (struct sockaddr *)&this->client_addr, &this->client_addr_len);
    if (this->recv_len < 1) {
        this->error = udp_server_error_type::recv_error;
    }
    return this->recv_len;
}



const char* UDPCommunication::getClientIP() {
    //// todo need test
    return inet_ntoa(this->remote_addr.sin_addr);
}

UDPCommunication::UDPCommunication() {
    this->params.local_ip = "0.0.0.0";
    this->params.local_port = 0;
    this->params.remote_ip = "0.0.0.0";
    this->params.remote_port = 0;
}

intmax_t UDPCommunication::get_recv_len() {
    return this->recv_len;
}

bool UDPCommunication::sendToClient(uint8_t *buffer, size_t send_len) {
    this->send_len = sendto(this->sockfd, buffer, send_len, 0, (struct sockaddr *)(&this->client_addr), this->client_addr_len);
    if (this->send_len != send_len) {
        this->error = udp_server_error_type::send_error;
        return false;
    }
    return true;
}

bool UDPCommunication::sendToRemote(uint8_t *buffer, size_t send_len) {
    this->send_len = sendto(this->sockfd, buffer, send_len, 0, (struct sockaddr *)(&this->remote_addr), this->remote_addr_len);
    if (this->send_len != send_len) {
        this->error = udp_server_error_type::send_error;
        return false;
    }
    return true;
}

intmax_t UDPCommunication::get_send_len() {
    return this->send_len;
}

}
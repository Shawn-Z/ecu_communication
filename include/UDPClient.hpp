#ifndef ECU_COMMUNICATION_UDPCLIENT_HPP
#define ECU_COMMUNICATION_UDPCLIENT_HPP

#include <stdlib.h>
#include <sys/socket.h>
#include <string.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

enum class udp_client_error_type {
    none = 0,
    get_client_sockfd_error = 1,
    send_error = 2
};

namespace ecu_communication {

class UDPClient {
public:
    udp_client_error_type error;
    UDPClient();
    bool init(const char *ip_send_to, uint16_t port_send_to);
    bool process(uint8_t *buffer, intmax_t buffer_size);
    intmax_t get_send_len();

private:
    int client_sockfd_;
    sockaddr_in server_ip_;
    intmax_t send_len_;

};

}

#endif
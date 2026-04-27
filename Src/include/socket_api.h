/**
 * @file socket_api.h
 * @brief 使用socket进行通信的API 
 * @version 0.1
 * @date 2024-08-10
 * @copyright Copyright (c) 2024
 * 
 */


#pragma once
#include <iostream>
#include <cstring>
#include <vector>
#if defined(_WIN32)
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <errno.h>
typedef int SOCKET;
#define INVALID_SOCKET (-1)
#define SOCKET_ERROR   (-1)
#define closesocket(s) ::close(s)
#define WSAGetLastError() (errno)
typedef int WSADATA;
#define WSAStartup(a, b) (0)
#define WSACleanup()     ((void)0)
#define MAKEWORD(a, b)   (0)
#endif

namespace lzd_tools{
    class Socket {
    public:
        Socket(const std::string& ip, int port);
        ~Socket();

        bool connectToServer();
        bool SendDoubleArray(const std::vector<double>& data);

        /**
         * @brief 
         * @param data 用于接收数据的vector
         * @param expectedSize 期望接收的数据大小，即data的项数
         */
        bool ReceiveDoubleArray(std::vector<double>& data, size_t expectedSize);
        bool ReceiveIntArray(std::vector<int>& data, size_t expectedSize);

        bool SendJson(std::string json);
        bool ReceiveJson(std::string& json);

    private:
        SOCKET sock;
        struct sockaddr_in server_addr;

        bool initializeWinsock();
        void cleanupWinsock();
    };
}



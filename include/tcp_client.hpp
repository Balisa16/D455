#ifndef TCP_CLIENT_HPP
#define TCP_CLIENT_HPP

#include <iostream>
#include <boost/asio.hpp>
#include <fstream>
#include <sstream>
#include <string>

namespace EMIRO{
    class TCP{
    private:
        std::string filename;
        boost::asio::io_service io_service;
        boost::asio::ip::tcp::socket socket;
        std::string address;
        int port;
    public:
        TCP();
        void connection(std::string address = "127.0.0.1", int port = 8888);
        void send(std::string filename);
        ~TCP();
    };
}

#endif
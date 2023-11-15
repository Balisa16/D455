#include <tcp_client.hpp>

namespace EMIRO{
    TCP::TCP() :
        socket(io_service)
    {
        address = "127.0.0.1";
        port = 8888;
    }

    void TCP::connection(std::string address, int port)
    {
        this->address = address;
        this->port = port;
    }

    void TCP::send(std::string filename)
    {
        socket.connect( boost::asio::ip::tcp::endpoint( boost::asio::ip::address::from_string(address), port ));
        
        std::cout << "Send : " << filename << '\n';
        std::ifstream file(filename, std::ios::binary);
        std::string buff_string;

        if (!file.is_open()) {
            std::cerr << "Error opening file\n";
            return;
        }

        while (std::getline(file, buff_string)) {
            buff_string += '\n';
            boost::system::error_code ec;
            const char* buf_char = buff_string.c_str();
            size_t length = buff_string.length();
            socket.write_some(boost::asio::buffer(buf_char, length), ec);
        }

        socket.close();
        file.close();
        std::cout << "Send file successfully";
        for (int i = 0; i < 40; i++) std::cout << " ";
        std::cout << '\n';
        
    }

    TCP::~TCP()
    {

    }
}
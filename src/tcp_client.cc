#include <tcp_client.hpp>

namespace EMIRO{
    TCP::TCP() :
        socket(io_service)
    {

    }

    void TCP::connection(std::string address, int port)
    {
        this->address = address;
        this->port = port;
        socket.connect(boost::asio::ip::tcp::endpoint( boost::asio::ip::address::from_string(address), port ));
    }

    void TCP::send(std::string filename)
    {
        // socket.connect(boost::asio::ip::tcp::endpoint( boost::asio::ip::address::from_string(address), port ));
        std::ifstream file(filename, std::ios::binary);
        std::string buff_string;

        if (!file.is_open()) {
            std::cerr << "Error opening file\n";
            return;
        }

        uint64_t line_cnt = 0;
        while (std::getline(file, buff_string)) {
            boost::system::error_code ec;
            socket.write_some(boost::asio::buffer(buff_string.c_str(), sizeof(buff_string.c_str())), ec);
            line_cnt++;
        }
        file.close();
        std::cout << "Sending " << line_cnt << " line.\n";
    }

    TCP::~TCP()
    {

    }
}
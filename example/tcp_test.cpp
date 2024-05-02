#include <tcpserver.hpp>

int main()
{
    TCPServer server;
    Odometry odometry;
    while (true)
    {
        odometry = server.get_odometry();
        std::cout << odometry << std::endl;
        // server.send_data("Ok");
    }

    return 0;
}
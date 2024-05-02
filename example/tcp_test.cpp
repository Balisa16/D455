#include <tcpserver.hpp>

int main()
{
    TCPServer server;
    Odometry odometry;
    while (true)
    {
        odometry = server.get_odometry();
        std::cout << odometry << std::endl;
    }

    return 0;
}
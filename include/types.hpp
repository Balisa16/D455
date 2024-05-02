#pragma once
#include <iostream>

struct Position
{
    float x, y, z;
    Position() : x(0.0f), y(0.0f), z(0.0f) {}
    Position(float x, float y, float z) : x(x), y(y), z(z) {}
    friend std::ostream &operator<<(std::ostream &os, const Position &position)
    {
        os << "x: " << position.x << " y: " << position.y << " z: " << position.z;
        return os;
    }
};

struct Quaternion
{
    float w, x, y, z;
    Quaternion() : w(0.0f), x(0.0f), y(0.0f), z(0.0f) {}
    Quaternion(float w, float x, float y, float z) : w(w), x(x), y(y), z(z) {}
    friend std::ostream &operator<<(std::ostream &os, const Quaternion &quaternion)
    {
        os << "w: " << quaternion.w << " x: " << quaternion.x << " y: " << quaternion.y << " z: " << quaternion.z;
        return os;
    }
};

struct Odometry
{
    Position position;
    Quaternion orientation;
    Odometry() : position(0.0f, 0.0f, 0.0f), orientation(0.0f, 0.0f, 0.0f, 0.0f) {}
    Odometry(Position position, Quaternion orientation) : position(position), orientation(orientation) {}
    friend std::ostream &operator<<(std::ostream &os, const Odometry &odometry)
    {
        os << "Position: " << odometry.position.x << " " << odometry.position.y << " " << odometry.position.z << "\nOrientation: " << odometry.orientation.w << " " << odometry.orientation.x << " " << odometry.orientation.y << " " << odometry.orientation.z;
        return os;
    }
};
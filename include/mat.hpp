#include <cmath>

enum class TStatus{
	None,
	Init,
	Available,
	Exit
};

typedef struct{
	float x, y, z;
}Position;

typedef struct{
	float w, x, y, z;
}Quaternion;

namespace EMIRO
{
	Quaternion euler_to_quaternion(float roll, float pitch, float yaw)
	{
		float roll2 = roll/2.0f;
		float pitch2 = pitch/2.0f;
		float yaw2 = yaw/2.0f;
	    float qx = sin(roll2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2);
	    float qy = cos(roll2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2);
	    float qz = cos(roll2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2);
	    float qw = cos(roll2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2);
	    return {qx, qy, qz, qw};
	}
}
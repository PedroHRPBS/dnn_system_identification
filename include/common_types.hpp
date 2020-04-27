#pragma once
#include <stdint.h>

enum class block_id {PID_X=0, PID_Y=1, PID_Z=2, PID_ROLL=3, PID_PITCH=4, 
					PID_YAW=5, PID_YAW_RATE=6, REF_Y=7, REF_Z=8, REF_ROLL=9, REF_PITCH=10, 
					REF_YAW=11, PID_PITCH_RATE = 12, REF_X = 13,
					MRFT_X=14, MRFT_Y=15, MRFT_Z=16, MRFT_ROLL=17, MRFT_PITCH=18, 
					MRFT_YAW=19, MRFT_YAW_RATE=20, REF_YAW_RATE=21, NULL_ID=999};

enum class control_system {roll=3, pitch=4, yaw=5, x=0, y=1, z=2, yaw_rate = 6};

enum Dimension3D {X,Y,Z};
enum class flight_command{TAKEOFF=0, LAND=1, NULL_TYPE=999};
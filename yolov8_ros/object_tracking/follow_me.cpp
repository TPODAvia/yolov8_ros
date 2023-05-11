/***************************************************************************************************************************
*
* Author: bingo
* Email: bingobin.lw@gmail.com
* Time: 2020.03.10
* Description: Implement px4 quadrotor QR code tracking
***************************************************************************************************************************/
#include "tracking_quadrotor.h"
using namespace std;
using namespace Eigen;
PX4Tracking::PX4Tracking(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private):
  nh_(nh),
  nh_private_(nh_private) {
  Initialize();
  cmdloop_timer_ = nh_.createTimer(ros::Duration(0.1), &PX4Tracking::CmdLoopCallback, this); //The period is 0.1s
  //Subscribe to the position directly in front of the aircraft relative to the QR code
  ar_pose_sub_ = nh_private_.subscribe("/ar_pose_marker", 1, &PX4Tracking::ArPoseCallback, this,ros::TransportHints().tcpNoDelay());

  position_sub_ = nh_private_.subscribe("/mavros/local_position/pose", 1, &PX4Tracking::Px4PosCallback,this,ros::TransportHints().tcpNoDelay());

  state_sub_ = nh_private_.subscribe("/mavros/state", 1, &PX4Tracking::Px4StateCallback,this,ros::TransportHints().tcpNoDelay());
  // 【Service】Modify system mode
  set_mode_client_ = nh_private_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

}

PX4Tracking::~PX4Tracking() {
  //Destructor
}

/**
* @name       S_SETPOINT_VEL PX4Tracking::TRACKINGPidProcess(Eigen::Vector3d &currentPos,Eigen::Vector3d &expectPos)

* @brief      pid control program, control the UAV in the body coordinate system
*             
* @param[in]  &currentPos The position of the current aircraft relative to the AR code
*             
* @param[in]  &expectPos Expected position expectPos[0]: the distance in the front and back directions relative to the AR code; expectPos[1]: the distance in the left and right directions relative to the AR code; expectPos[2]: the distance in the up and down direction relative to the AR code
* @param[out] &The desired speed in y, z, and the desired speed in the yaw direction.
*
* @param[out] 
**/
Eigen::Vector3d PX4Tracking::TrackingPidProcess(Eigen::Vector3d &currentPos,Eigen::Vector3d &expectPos)
{
  Eigen::Vector3d s_PidOut;

	/*PID control in the front and rear directions, speed control in the y direction under the output machine system*/
	s_PidItemY.difference = currentPos[2] - expectPos[0];
	s_PidItemY.intergral += s_PidItemY.difference;
	if(s_PidItemY.intergral >= 100)		
		s_PidItemY.intergral = 100;
	else if(s_PidItemY.intergral <= -100) 
		s_PidItemY.intergral = -100;
	s_PidItemY.differential =  s_PidItemY.difference  - s_PidItemY.tempDiffer;
    s_PidItemY.tempDiffer = s_PidItemY.difference;
	cout << "s_PidItemY.tempDiffer: " << s_PidItemY.tempDiffer << endl;
	cout << "s_PidItemY.differential: " << s_PidItemY.differential << endl;
	s_PidOut[0] = s_PidY.p*s_PidItemY.difference + s_PidY.d*s_PidItemY.differential + s_PidY.i*s_PidItemY.intergral;

	/*PID control in left and right direction, output speed control in yaw direction*/
	s_PidItemYaw.difference = expectPos[1] - currentPos[0];
	s_PidItemYaw.intergral += s_PidItemYaw.difference;
	if(s_PidItemYaw.intergral >= 100)		
		s_PidItemYaw.intergral = 100;
	else if(s_PidItemYaw.intergral <= -100) 
		s_PidItemYaw.intergral = -100;
	s_PidItemYaw.differential =  s_PidItemYaw.difference  - s_PidItemYaw.tempDiffer;
    s_PidItemYaw.tempDiffer = s_PidItemYaw.difference;
	s_PidOut[1] = s_PidYaw.p*s_PidItemYaw.difference + s_PidYaw.d*s_PidItemYaw.differential + s_PidYaw.i*s_PidItemYaw.intergral;

	/*PID control in left and right direction, output speed control in z direction*/
	s_PidItemZ.difference = expectPos[2] - currentPos[1];
	s_PidItemZ.intergral += s_PidItemZ.difference;
	if(s_PidItemZ.intergral >= 100)		
		s_PidItemZ.intergral = 100;
	else if(s_PidItemZ.intergral <= -100) 
		s_PidItemZ.intergral = -100;
	s_PidItemZ.differential =  s_PidItemZ.difference  - s_PidItemZ.tempDiffer;
    s_PidItemZ.tempDiffer = s_PidItemZ.difference;
	s_PidOut[2] = s_PidZ.p*s_PidItemZ.difference + s_PidZ.d*s_PidItemZ.differential + s_PidZ.i*s_PidItemZ.intergral;

	return s_PidOut;
}
void PX4Tracking::CmdLoopCallback(const ros::TimerEvent& event)
{
  TrackingStateUpdate();
}


/**
* @name       void PX4Tracking::TrackingStateUpdate()
* @brief      State machine update function
*             
* @param[in]  none
*             
* @param[in]  none
* @param[out] 
*
* @param[out] 
**/
void PX4Tracking::TrackingStateUpdate()
{


	desire_vel_ = TrackingPidProcess(ar_pose_,desire_pose_);
	cout << "desire_vel_[0]:  "<< desire_vel_[0] <<endl;
	cout << "desire_vel_[1]:  "<< desire_vel_[1] <<endl;
	cout << "desire_vel_[2]:  "<< desire_vel_[2] <<endl;
	cout << "desire_vel_[3]:  "<< desire_vel_[3] <<endl;
	cout << "markers_yaw_: "  << markers_yaw_ << endl;
	cout << "ar_pose_[0]:  "<<  ar_pose_[0] << endl;
	cout << "ar_pose_[1]:  "<<  ar_pose_[1] << endl;
	cout << "ar_pose_[2]:  "<<  ar_pose_[2] << endl;
	cout << "desire_pose_[0]:  "<<  desire_pose_[0] << endl;
	cout << "desire_pose_[1]:  "<<  desire_pose_[1] << endl;
	cout << "desire_pose_[2]:  "<<  desire_pose_[2] << endl;
	cout << "detect_state : " << detect_state << endl;
	switch(TrackingState)
	{
		case WAITING:
			if(px4_state_.mode != "OFFBOARD")//Wait for offboard mode
			{
				temp_pos_drone[0] = px4_pose_[0];
				temp_pos_drone[1] = px4_pose_[1];
				temp_pos_drone[2] = px4_pose_[2];
				OffboardControl_.send_pos_setpoint(temp_pos_drone, 0);
			}
			else
			{
				temp_pos_drone[0] = px4_pose_[0];
				temp_pos_drone[1] = px4_pose_[1];
				temp_pos_drone[2] = px4_pose_[2];
				TrackingState = CHECKING;
				cout << "CHECKING" <<endl;
			}
				//cout << "WAITING" <<endl;
			break;
		case CHECKING:
			if(px4_pose_[0] == 0 && px4_pose_[1] == 0) 			//Execute landing mode without position information
			{
				cout << "Check error, make sure have local location" <<endl;
				mode_cmd_.request.custom_mode = "AUTO.LAND";
				set_mode_client_.call(mode_cmd_);
				TrackingState = WAITING;	
			}
			else
			{
				TrackingState = PREPARE;
				cout << "PREPARE" <<endl;
			}
			
			break;
		case PREPARE:											//Take off to the specified altitude
			posxyz_target[0] = temp_pos_drone[0];
			posxyz_target[1] = temp_pos_drone[1];
			posxyz_target[2] = search_alt_;
			if((px4_pose_[2]<=search_alt_+0.1) && (px4_pose_[2]>=search_alt_-0.1))
			{
				TrackingState = SEARCH;
			}
			else if(detect_state == true)
			{
			//	TrackingState = SEARCH;
			}
			OffboardControl_.send_pos_setpoint(posxyz_target, 0);					
			if(px4_state_.mode != "OFFBOARD")				//If you switch to onboard in the middle of preparation, skip to WAITING
			{
				TrackingState = WAITING;
			}

			break;
		case SEARCH:
			if(detect_state == true)
			{
				TrackingState = TRACKING;
			  cout << "TRACKING" <<endl;
			}	
			else//Here the drone is not actively searching for targets
			{
				OffboardControl_.send_pos_setpoint(posxyz_target, 0);
			}
			if(px4_state_.mode != "OFFBOARD")				//If you switch to onboard during SEARCH, skip to WAITING
			{
				TrackingState = WAITING;
			}
     // cout << "SEARCH" <<endl;
			break;
		case TRACKING:
			{
				if(detect_state == true)
				{
					desire_vel_ = TrackingPidProcess(ar_pose_,desire_pose_);

					//cout << "search_" <<endl;
				}
			  else
				{
					desire_vel_[0] = 0;
					desire_vel_[1] = 0;
					desire_vel_[2] = 0;
				}
				if(ar_pose_[2] <= 0.3)
				{
					TrackingState = TRACKOVER;
					cout << "TRACKOVER" <<endl;
				}
				if(px4_state_.mode != "OFFBOARD")			//If you switch to onboard in the middle of TRACKING, skip to WAITING
				{
					TrackingState = WAITING;
				}
				desire_yzVel_[0] = desire_vel_[0];
				desire_yzVel_[1] = desire_vel_[2];
				desire_yawVel_ = desire_vel_[1];

				OffboardControl_.send_body_velyz_setpoint(desire_yzVel_,desire_yawVel_);
			}

			break;
		case TRACKOVER:
			{
				mode_cmd_.request.custom_mode = "AUTO.LAND";
        set_mode_client_.call(mode_cmd_);
				TrackingState = WAITING;
			}

			break;

		default:
			cout << "error" <<endl;
	}	

}

/*Receive the position of the landing board relative to the aircraft and the yaw angle*/
void PX4Tracking::ArPoseCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &msg)
{
	detect_state = false;
	for(auto &item : msg->markers)
	{
		if(item.id == markers_id_)
		{
			detect_state = true;
      ar_pose_[0] = item.pose.pose.position.x;
      ar_pose_[1] = item.pose.pose.position.y;
      ar_pose_[2] = item.pose.pose.position.z;

//			cout << "ar_pose_[0]:"  << ar_pose_[0] << endl;
//			cout << "ar_pose_[1]:"  << ar_pose_[1] << endl;
//			cout << "ar_pose_[2]:"  << ar_pose_[2] << endl;
//			cout << "markers_yaw_: "  << markers_yaw_ << endl;
		}
	}
//	cout << "detect_state :" << detect_state << endl;
}

/*Receive the current aircraft position from the flight controller*/                  
void PX4Tracking::Px4PosCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    // Read the Drone Position from the Mavros Package [Frame: ENU]
    Eigen::Vector3d pos_drone_fcu_enu(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);

    px4_pose_ = pos_drone_fcu_enu;
}
/*Receive current aircraft status from flight controller*/
void PX4Tracking::Px4StateCallback(const mavros_msgs::State::ConstPtr& msg)
{
	px4_state_ = *msg;
}

/*initialization*/
void PX4Tracking::Initialize()
{
  //Read the search altitude of the aircraft in offboard mode
  nh_private_.param<float>("search_alt_", search_alt_, 2);

  nh_private_.param<float>("markers_id_", markers_id_, 4.0);

  nh_private_.param<float>("PidY_p", s_PidY.p, 0.6);
  nh_private_.param<float>("PidY_d", s_PidY.d, 0.01);
  nh_private_.param<float>("PidY_i", s_PidY.i, 0);
  nh_private_.param<float>("PidZ_p", s_PidZ.p, 0.6);
  nh_private_.param<float>("PidZ_d", s_PidZ.d, 0.01);
  nh_private_.param<float>("PidZ_i", s_PidZ.i, 0);
  nh_private_.param<float>("PidYaw_p", s_PidYaw.p, 0.4);
  nh_private_.param<float>("PidYaw_d", s_PidYaw.d, 0.01);
  nh_private_.param<float>("PidYaw_i", s_PidYaw.i, 0);

  //The desired position of the aircraft relative to the AR code
	float desire_pose_x,desire_pose_y,desire_pose_z;
  nh_private_.param<float>("desire_pose_x", desire_pose_x, 6.5);
  nh_private_.param<float>("desire_pose_y", desire_pose_y, 0);
  nh_private_.param<float>("desire_pose_z", desire_pose_z, 0);
  desire_pose_[0] = desire_pose_x;
  desire_pose_[1] = desire_pose_y;
  desire_pose_[2] = desire_pose_z;

  detect_state = false;
  desire_vel_[0] = 0;
  desire_vel_[1] = 0;
  desire_vel_[2] = 0;
  desire_yzVel_[0]  = 0;
  desire_yzVel_[1]  = 0;
  s_PidItemY.tempDiffer = 0;
  s_PidItemYaw.tempDiffer = 0;
  s_PidItemZ.tempDiffer = 0;
  s_PidItemY.intergral = 0;
  s_PidItemYaw.intergral = 0;
  s_PidItemZ.intergral = 0;
	cout << "search_alt_ = " << search_alt_ << endl;
	cout << "markers_id_ = " << markers_id_ << endl;
	cout << "PidY_p = " << s_PidY.p << endl;
	cout << "PidY_d = " << s_PidY.d << endl;
	cout << "PidY_i = " << s_PidY.i << endl;
	cout << "PidZ_p = " << s_PidZ.p << endl;
	cout << "PidZ_d = " << s_PidZ.d << endl;
	cout << "PidZ_i = " << s_PidZ.i << endl;
	cout << "PidYaw_p = " << s_PidYaw.p << endl;
	cout << "PidYaw_d = " << s_PidYaw.d << endl;
	cout << "PidYaw_i = " << s_PidYaw.i << endl;
	cout << "desire_pose_x = " << desire_pose_[0] << endl;
	cout << "desire_pose_y = " << desire_pose_[1] << endl;
	cout << "desire_pose_z = " << desire_pose_[2] << endl;

}
int main(int argc, char** argv) {
  ros::init(argc,argv,"tracking_quadrotor");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  PX4Tracking PX4Tracking(nh, nh_private);

  ros::spin();
  return 0;
}
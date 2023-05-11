#include <ros/ros.h>
#include <iostream>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt32.h>
#include "offboard_control.h"
#include "px4_control_cfg.h"
#include <tf/transform_datatypes.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
using namespace std;
using namespace Eigen;
class PX4Tracking {
 public:
    /**
     * default constructor
     */
    PX4Tracking(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    /**
     * destructor
     */
    ~PX4Tracking();
    void Initialize();
   OffboardControl OffboardControl_;
 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Timer cmdloop_timer_;
  void CmdLoopCallback(const ros::TimerEvent& event);
  void TrackingStateUpdate();
  void ArPoseCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &msg);
  void Px4PosCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void Px4StateCallback(const mavros_msgs::State::ConstPtr& msg);
  Eigen::Vector3d TrackingPidProcess(Eigen::Vector3d &currentPos,Eigen::Vector3d &expectPos);
  Eigen::Vector3d temp_pos_drone;
  Eigen::Vector3d posxyz_target; //Desired aircraft position in space
  Eigen::Vector3d ar_pose_;      //AR code relative to aircraft position
  Eigen::Vector3d px4_pose_;     //Receive the aircraft position from the flight controller
  Eigen::Vector3d desire_pose_;  //The desired position of the aircraft relative to the AR code
  mavros_msgs::State px4_state_; //The state of the aircraft
  mavros_msgs::SetMode mode_cmd_;
  float search_alt_;
  float markers_id_;             //The AR code to be detected, the default is 4
  bool detect_state;             //Whether the AR code flag is detected
  Eigen::Vector3d desire_vel_;
	Eigen::Vector3d desire_yzVel_;
	float desire_yawVel_;
  S_PID s_PidY,s_PidZ,s_PidYaw;
  S_PID_ITEM s_PidItemY;
  S_PID_ITEM s_PidItemZ;
  S_PID_ITEM s_PidItemYaw;
  enum Drone
 {
  WAITING,		//Wait for offboard mode
  CHECKING,		//Check aircraft status
  PREPARE,		//Take off to the specified altitude
  SEARCH,		//Search
  TRACKING,	    //AR code detected, start tracking
  TRACKOVER,	//Finish	
};
Drone TrackingState = WAITING;//Initial state WAITING

  ros::Subscriber ar_pose_sub_;
  ros::Subscriber position_sub_;
  ros::Subscriber state_sub_;
  ros::ServiceClient set_mode_client_;
};
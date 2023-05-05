#ifndef KEYBOARD_CONTROL_NODE_H
#define KEYBOARD_CONTROL_NODE_H
#include <stdio.h>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseArray.h>


#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#define KEYBOARD_Q			16
#define KEYBOARD_A			30
#define KEYBOARD_Z			44
#define KEYBOARD_J			36
#define KEYBOARD_L			38
#define KEYBOARD_I			23
#define KEYBOARD_K			37
#define KEYBOARD_COMMA		51
#define KEYBOARD_U			22
#define KEYBOARD_O			24
#define KEYBOARD_H			35
#define KEYBOARD_N			49
#define KEYBOARD_Y			21
#define KEYBOARD_T			20
#define KEYBOARD_G			34
#define KEYBOARD_B			48

namespace Xh_Cai_tool{
    class KeyboardControl
    {
        public:
            KeyboardControl();
            ~KeyboardControl();

            void run();
            
        private:
           
            bool init();
            void twistCallback(const ros::TimerEvent&);
            void state_cb(const mavros_msgs::State::ConstPtr& msg);   
            void parseKeyboard();
            void uavLocalPoseReceived(const geometry_msgs::PoseStampedConstPtr& msg);
            void updateControlCommandStats();
         
            ros::NodeHandle nh_;
            
            
            int fd_;
            struct input_event ev_;
            std::string port_name_;

            ros::Publisher vel_sp_pub;
            ros::Timer vel_sp_pub_timer;

            ros::Publisher pos_sp_pub;
            ros::Subscriber state_sub;
            ros::Subscriber commandSubscriber;
            ros::Subscriber uavLocalPoseSubscriber;
            

            geometry_msgs::TwistStamped vs;
            geometry_msgs::TwistStamped vs_body_axis;
            geometry_msgs::PoseStamped ps;
            geometry_msgs::PoseStamped uavCurrentLocalPose;
            geometry_msgs::PoseStamped uavLockPose;
            geometry_msgs::PoseStamped uavNextPose;

            double uavRollENU, uavPitchENU, uavYawENU;
            bool isSendNextPose;
            bool isSendNextVelocity;
            bool isSendNextPoseInitial;
            bool isPositionReached;

            int rosInfoTimeDealy;

            double rate_;

            mavros_msgs::SetMode offb_set_mode;
            mavros_msgs::State current_state;
            mavros_msgs::CommandBool arm_cmd;

            ros::ServiceClient arming_client;
            ros::ServiceClient set_mode_client;

            
            
            

    };
}
#endif // KEYBOARD_CONTROL_NODE_H
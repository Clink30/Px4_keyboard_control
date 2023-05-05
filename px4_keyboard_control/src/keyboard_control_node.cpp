#include <fcntl.h>
#include <dirent.h>
#include <linux/input.h>
#include <sys/stat.h>

#include <boost/bind.hpp>
#include <boost/thread.hpp>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#include "px4_keyboard_control/keyboard_control_node.h"

namespace Xh_Cai_tool{
KeyboardControl::KeyboardControl(){
    isSendNextPose = false;
    isSendNextVelocity = false;
    isSendNextPoseInitial = false;

    vs.twist.linear.x = 0.0;
    vs.twist.linear.y = 0.0;
    vs.twist.linear.z = 0.0; 
    vs.twist.angular.x = 0.0;
    vs.twist.angular.y = 0.0;
    vs.twist.angular.z = 0.0;

    vs_body_axis.twist.linear.x = 0.0;
    vs_body_axis.twist.linear.y = 0.0;
    vs_body_axis.twist.linear.z = 0.0; 
    vs_body_axis.twist.angular.x = 0.0;
    vs_body_axis.twist.angular.y = 0.0;
    vs_body_axis.twist.angular.z = 0.0;

    rosInfoTimeDealy = 0;
    
    rate_ = 20;
}

KeyboardControl::~KeyboardControl(){

}

void KeyboardControl::state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void KeyboardControl::updateControlCommandStats()
{
    std::string controlCommandStatus = "";
    if (isSendNextVelocity)
        controlCommandStatus  += "[isSendNextVelocity = true]  ";
    else
        controlCommandStatus  += "[isSendNextVelocity = false]  "; 

    if (isSendNextPose)
        controlCommandStatus  += "[isSendNextPose = true]  ";
    else
        controlCommandStatus  += "[isSendNextPose = false]  ";  

    if (isSendNextPoseInitial)
        controlCommandStatus  += "[isSendNextPoseInitial = true]  ";
    else
        controlCommandStatus  += "[isSendNextPoseInitial = false]  ";
    ROS_INFO_STREAM(controlCommandStatus); 

    std::string connectionStatus = "";
    if (current_state.connected)
        connectionStatus += "[Connected] ";
    else
        connectionStatus += "[Disconnected] ";

    if (current_state.armed)
        connectionStatus += "[Armed] ";
    else
        connectionStatus += "[Disarmed] ";

    if (current_state.mode == "OFFBOARD")
        connectionStatus += "[OFFBOARD] ";
    else
        connectionStatus += "[NOT OFFBOARD] ";
    ROS_INFO_STREAM(connectionStatus);

    //ROS_INFO("current yaw = %0.3f\n", uavYawENU*180/3.1415926);
    //ROS_INFO("[vbx, vby, vbaz] = [%0.5f, %0.5f, %0.5f]\n", vs.twist.linear.x, vs.twist.linear.y, vs.twist.linear.z);
    //printf("[vroll, vpitch, vyaw = [%0.5f, %0.5f, %0.5f]\n", vs.twist.angular.x, vs.twist.angular.y, vs.twist.angular.z);
      
    ROS_INFO("[vx, vy, vz] = [%0.5f, %0.5f, %0.5f]\n", vs_body_axis.twist.linear.x, vs_body_axis.twist.linear.y, vs_body_axis.twist.linear.z);
    //printf("[vroll, vpitch, vyaw = [%0.5f, %0.5f, %0.5f]\n", vs_body_axis.twist.angular.x, vs_body_axis.twist.angular.y, vs_body_axis.twist.angular.z);
}

void KeyboardControl::parseKeyboard(){
    while(true){
        read(fd_, &ev_, sizeof(struct input_event));
        if(ev_.type == EV_KEY){
            ROS_DEBUG_STREAM("INFO: [key]: " << ev_.code << ", [value]: " << ev_.value);
            switch (ev_.code){
                case KEYBOARD_Q:
                {
                    ROS_WARN_STREAM("up");
                    vs.twist.linear.z += 0.1;
                    isSendNextPose = false;      
                    isSendNextVelocity = true;
                    //updateControlCommandStats();
                    break;
                }
                case KEYBOARD_A:
                {
                    ROS_WARN_STREAM("z is zero");
                    vs.twist.linear.z = 0.0;
                    isSendNextPose = false;      
                    isSendNextVelocity = true;
                    //updateControlCommandStats();
                    break;
                }
                case KEYBOARD_Z:
                {
                    ROS_WARN_STREAM("down");
                    vs.twist.linear.z -= 0.1;
                    isSendNextPose = false;      
                    isSendNextVelocity = true; 
                    //updateControlCommandStats();
                    break;
                }
                case KEYBOARD_J:
                {
                    ROS_WARN_STREAM("left");
                    vs.twist.linear.y -= 0.1; 
                    isSendNextPose = false;      
                    isSendNextVelocity = true;
                    //updateControlCommandStats();             
                    break;
                }
                case KEYBOARD_L:
                {
                    ROS_WARN_STREAM("right");
                    vs.twist.linear.y += 0.1;
                    isSendNextPose = false;      
                    isSendNextVelocity = true; 
                    //updateControlCommandStats();           
                    break;
                }
                case KEYBOARD_I:
                {
                    ROS_WARN_STREAM("forward");
                    vs.twist.linear.x += 0.1;
                    isSendNextPose = false;      
                    isSendNextVelocity = true; 
                    //updateControlCommandStats();
                    break;
                }
                case KEYBOARD_K:
                {
                    ROS_WARN_STREAM("except z is zero");
                    vs.twist.linear.x = 0.0;
                    vs.twist.linear.y = 0.0;
                    vs.twist.angular.x = 0.0;
                    vs.twist.angular.y = 0.0;
                    vs.twist.angular.z = 0.0;
                    isSendNextPose = false;      
                    isSendNextVelocity = true; 
                    //updateControlCommandStats();            
                    break;
                }
                case KEYBOARD_COMMA:
                {
                    ROS_WARN_STREAM("backward");
                    vs.twist.linear.x -= 0.1;
                    isSendNextPose = false;      
                    isSendNextVelocity = true; 
                    //updateControlCommandStats();            
                    break;
                }
                case KEYBOARD_U:
                {
                    ROS_WARN_STREAM("turn clockwise");
                    vs.twist.angular.z += 0.1;
                    isSendNextPose = false;      
                    isSendNextVelocity = true; 
                    //updateControlCommandStats();
                    break;
                }
                case KEYBOARD_O:
                {
                    ROS_WARN_STREAM("turn anti-clockwise");
                    vs.twist.angular.z -= 0.1;
                    isSendNextPose = false;      
                    isSendNextVelocity = true;
                    //updateControlCommandStats();
                    break;
                }
                case KEYBOARD_H:
                {
                    ROS_WARN_STREAM("set all to zero");
                    vs.twist.linear.x = 0.0;
                    vs.twist.linear.y = 0.0;
                    vs.twist.linear.z = 0.0; 
                    vs.twist.angular.x = 0.0;
                    vs.twist.angular.y = 0.0;
                    vs.twist.angular.z = 0.0;
                    isSendNextPose = false;      
                    isSendNextVelocity = true; 
                    //updateControlCommandStats();
                    break;
                }
                case KEYBOARD_N:
                {
                    ROS_INFO_STREAM("get current lock position");
                    uavLockPose.pose.position.x = uavCurrentLocalPose.pose.position.x;
                    uavLockPose.pose.position.y = uavCurrentLocalPose.pose.position.y;
                    uavLockPose.pose.position.z = uavCurrentLocalPose.pose.position.z;
                    ROS_INFO("current lock position: Pos:[%0.3f, %0.3f, %0.3f]", uavLockPose.pose.position.x, uavLockPose.pose.position.y, uavLockPose.pose.position.z);
                    float body_next_pos_x = 0;
                    float body_next_pos_y = -3.0;
                    float body_next_pos_z = 3.0;
                    float body_vector_length = sqrt((body_next_pos_x)*(body_next_pos_x)+(body_next_pos_y)*(body_next_pos_y)+(body_next_pos_z)*(body_next_pos_z));
                    float body_xy_length = sqrt((body_next_pos_x)*(body_next_pos_x) + (body_next_pos_y)*(body_next_pos_y));
      
                    float delta_x = body_next_pos_x * cos(uavYawENU) - body_next_pos_y * sin(uavYawENU);
                    float delta_y = body_next_pos_x * sin(uavYawENU) + body_next_pos_y * cos(uavYawENU);
                    
                    uavNextPose.pose.position.x = uavLockPose.pose.position.x + delta_x;// + 3.0;
                    uavNextPose.pose.position.y = uavLockPose.pose.position.y + delta_y;// - 3.0;
                    uavNextPose.pose.position.z = uavLockPose.pose.position.z + body_next_pos_z;//+ 3.0;
                    uavNextPose.pose.orientation.x = uavCurrentLocalPose.pose.orientation.x;
                    uavNextPose.pose.orientation.y = uavCurrentLocalPose.pose.orientation.y;
                    uavNextPose.pose.orientation.z = uavCurrentLocalPose.pose.orientation.z;
                    uavNextPose.pose.orientation.w = uavCurrentLocalPose.pose.orientation.w;

                    ROS_INFO("next position: Pos:[%0.3f, %0.3f, %0.3f]", uavNextPose.pose.position.x, uavNextPose.pose.position.y, uavNextPose.pose.position.z);
                    
                    isSendNextPoseInitial = true;
                    isSendNextPose = false;
                    //updateControlCommandStats();
                    break;
                }
                case KEYBOARD_Y:
                {
                    ROS_WARN_STREAM("change to control pose");
                    isSendNextPose = true;      
                    isSendNextVelocity = false; 
                    break;
                }
                case KEYBOARD_T:
                {
                    ROS_WARN_STREAM("T");
                    offb_set_mode.request.custom_mode = "OFFBOARD";
                    set_mode_client.call(offb_set_mode);
                    printf("%d",offb_set_mode.response.mode_sent);
                    if (offb_set_mode.response.mode_sent)
                        ROS_WARN_STREAM("--------Offboard enabled--------");
                    break;
                }
                case KEYBOARD_G:
                {
                    ROS_WARN_STREAM("G");
                    arm_cmd.request.value = true;
                    arming_client.call(arm_cmd);
                    if (arm_cmd.response.success)
                        ROS_WARN_STREAM("--------Vehicle armed-----------");
                    break;
                }
                case KEYBOARD_B:
                {
                    ROS_WARN_STREAM("B");
                    arm_cmd.request.value = false;
                    arming_client.call(arm_cmd);
                    if (arm_cmd.response.success)
                        ROS_WARN_STREAM("Vehicle disarmed");
                    break;
                }
                default:
                {
                    break;
                }

            }
        }
    }
}

void KeyboardControl::uavLocalPoseReceived(const geometry_msgs::PoseStampedConstPtr& msg){
    //uavPose = *msg;
    uavCurrentLocalPose.pose.position.x = msg->pose.position.x;
    uavCurrentLocalPose.pose.position.y = msg->pose.position.y;
    uavCurrentLocalPose.pose.position.z = msg->pose.position.z;
    uavCurrentLocalPose.pose.orientation.x = msg->pose.orientation.x;
    uavCurrentLocalPose.pose.orientation.y = msg->pose.orientation.y;
    uavCurrentLocalPose.pose.orientation.z = msg->pose.orientation.z;
    uavCurrentLocalPose.pose.orientation.w = msg->pose.orientation.w;

    // Using ROS tf to get RPY angle from Quaternion
    tf::Quaternion quat;
    tf::quaternionMsgToTF(uavCurrentLocalPose.pose.orientation, quat);  
    tf::Matrix3x3(quat).getRPY(uavRollENU, uavPitchENU, uavYawENU);

    rosInfoTimeDealy++;
    if (rosInfoTimeDealy >=15)
    {
        //ROS_INFO("local_position/local: Pos:[%0.3f, %0.3f, %0.3f], RPY:[%0.3f, %0.3f, %0.3f]", uavCurrentLocalPose.pose.position.x, uavCurrentLocalPose.pose.position.y, uavCurrentLocalPose.pose.position.z,uavRollENU*180/3.1415926, uavPitchENU*180/3.1415926, uavYawENU*180/3.1415926);
        //updateControlCommandStats();    
        rosInfoTimeDealy = 0;
    }

    if( isSendNextPose )
    {
        double distance = sqrt((uavCurrentLocalPose.pose.position.x - uavNextPose.pose.position.x) * (uavCurrentLocalPose.pose.position.x - uavNextPose.pose.position.x)  +
        (uavCurrentLocalPose.pose.position.y - uavNextPose.pose.position.y) * (uavCurrentLocalPose.pose.position.y - uavNextPose.pose.position.y) + (uavCurrentLocalPose.pose.position.z - uavNextPose.pose.position.z) * (uavCurrentLocalPose.pose.position.z - uavNextPose.pose.position.z));
        double threshold = 0.2;
        if (distance < threshold)
        {
            ROS_INFO("Reached!!!");
            isSendNextPose = false;
            isSendNextVelocity = true;
            vs.twist.linear.x = 0.0;
            vs.twist.linear.y = 0.0;
            vs.twist.linear.z = 0.0;
            vs.twist.angular.x = 0.0;
            vs.twist.angular.y = 0.0;
            vs.twist.angular.z = 0.0;
        }
        else
        {
            double deltaX = uavCurrentLocalPose.pose.position.x - uavNextPose.pose.position.x;
            double deltaY = uavCurrentLocalPose.pose.position.y - uavNextPose.pose.position.y;
            double deltaZ = uavCurrentLocalPose.pose.position.y - uavNextPose.pose.position.z;
            ROS_INFO("Delta Pos:[%0.3f, %0.3f, %0.3f], Delta Distance:[%0.3f]", deltaX, deltaY, deltaZ, distance);
        }
    }
}

void KeyboardControl::twistCallback(const ros::TimerEvent &){
     if(isSendNextVelocity == true){
        vs_body_axis.header.stamp = ros::Time::now();
        vs_body_axis.twist.linear.x = vs.twist.linear.x * cos(uavYawENU) + vs.twist.linear.y * sin(uavYawENU);
        vs_body_axis.twist.linear.y = vs.twist.linear.x * sin(uavYawENU) - vs.twist.linear.y * cos(uavYawENU);
        vs_body_axis.twist.linear.z = vs.twist.linear.z;
        vs_body_axis.twist.angular.x = vs.twist.angular.x;
        vs_body_axis.twist.angular.y = vs.twist.angular.y;
        vs_body_axis.twist.angular.z = vs.twist.angular.z;   

        vel_sp_pub.publish(vs_body_axis);
     }
     if ((isSendNextPose == true) && (isSendNextPoseInitial == true))
     {
        ps.pose = uavNextPose.pose;
        ps.header.stamp = ros::Time::now();
        pos_sp_pub.publish(ps);
     }     

}

bool KeyboardControl::init(){
    const char path[] = "/dev/input/by-path";
    DIR *dev_dir = opendir(path);
    struct dirent *entry;
    if (dev_dir == NULL){
        return false;
    }    
    while ((entry = readdir(dev_dir)) != NULL){
      std::string dir_str = entry->d_name;
      if (dir_str.find("event-kbd") < dir_str.length()){
        port_name_ = std::string(path) + "/" + dir_str;
        ROS_INFO_STREAM("INFO: The keyboard port is :" << port_name_);
        break;
      }
    }
    closedir(dev_dir);

    if (port_name_ != ""){
      fd_ = open(port_name_.c_str(), O_RDONLY);
    if (fd_ < 0){
      ROS_ERROR_STREAM("ERROR: Can't Open The Port :" << port_name_);
      return false;
    }else{
      ROS_INFO_STREAM("INFO: Open The Port :" << port_name_);
      return true;
    }
  }else{
    return false;
  }

}


void KeyboardControl::run(){
    ros::Rate loop_rate(20.0);
    if(init()){
        ROS_INFO("1");
        vel_sp_pub = nh_.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);
        pos_sp_pub = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local",10);
        state_sub = nh_.subscribe<mavros_msgs::State>("mavros/state", 10, &KeyboardControl::state_cb,this);
     
        vel_sp_pub_timer = nh_.createTimer(ros::Duration(1.0/50.0),&KeyboardControl::twistCallback,this);
        
        boost::thread parse_thread(boost::bind(&KeyboardControl::parseKeyboard, this));

        uavLocalPoseSubscriber = nh_.subscribe("/mavros/local_position/pose", 1000, &KeyboardControl::uavLocalPoseReceived,this);
        
        arming_client = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
        set_mode_client = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

        ros::spin();
    }
}

}

int main(int argc,char *argv[]){
    ros::init(argc,argv,"px4_keyboard_control");
    Xh_Cai_tool::KeyboardControl keyboard_control;
    keyboard_control.run();
    return 0;
}


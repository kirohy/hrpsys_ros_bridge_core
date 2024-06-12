#ifndef HRPSYSSEQSTATEROSBRIDGE2_H
#define HRPSYSSEQSTATEROSBRIDGE2_H

#include "HrpsysSeqStateROSBridge2Impl.h"

#include <rtm/CorbaNaming.h>

#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <geometry_msgs/WrenchStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int32.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <sensor_msgs/Imu.h>

#ifdef USE_PR2_CONTROLLERS_MSGS
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
#else
#include <control_msgs/JointTrajectoryControllerState.h>
#endif

#include <seq_state_ros_bridge/ContactStatesStamped.h>
#include <seq_state_ros_bridge/MotorStates.h>
#include <seq_state_ros_bridge/SetSensorTransformation.h>

#include <mutex>

extern const char *hrpsysseqstaterosbridge2impl_spec[];

class HrpsysSeqStateROSBridge2 : public HrpsysSeqStateROSBridge2Impl {
  public:
    HrpsysSeqStateROSBridge2(RTC::Manager *manager);
    ~HrpsysSeqStateROSBridge2();

    RTC::ReturnCode_t onInitialize();
    RTC::ReturnCode_t onFinalize();
    RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

    void onJointTrajectory(trajectory_msgs::JointTrajectory trajectory);
#ifdef USE_PR2_CONTROLLERS_MSGS
    void onJointTrajectoryActionGoal();
    void onJointTrajectoryActionPreempt();
#endif
    void onFollowJointTrajectoryActionGoal();
    void onFollowJointTrajectoryActionPreempt();
    void onTrajectoryCommandCB(const trajectory_msgs::JointTrajectoryConstPtr &msg);
    bool sendMsg(dynamic_reconfigure::Reconfigure::Request &req, dynamic_reconfigure::Reconfigure::Response &res);
    bool setSensorTransformation(seq_state_ros_bridge::SetSensorTransformation::Request &req, seq_state_ros_bridge::SetSensorTransformation::Response &res);

  private:
    ros::NodeHandle nh;
    ros::Publisher joint_state_pub, joint_controller_state_pub, mot_states_pub, diagnostics_pub, clock_pub, zmp_pub, ref_cp_pub, act_cp_pub, odom_pub, imu_pub,
        em_mode_pub, ref_contact_states_pub, act_contact_states_pub;
    ros::Subscriber trajectory_command_sub;
    std::vector<ros::Publisher> fsensor_pub, cop_pub;
#ifdef USE_PR2_CONTROLLERS_MSGS
    actionlib::SimpleActionServer<pr2_controllers_msgs::JointTrajectoryAction> joint_trajectory_server;
#endif
    actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> follow_joint_trajectory_server;
    ros::ServiceServer sendmsg_srv;
    ros::ServiceServer set_sensor_transformation_srv;
    bool interpolationp, use_sim_time, use_hrpsys_time;
    bool publish_sensor_transforms;
    tf::TransformBroadcaster br;

    std::mutex m_mutex;
    coil::TimeMeasure tm;
    sensor_msgs::JointState prev_joint_state;

    std::string nameserver;
    std::string rootlink_name;

    ros::Subscriber clock_sub;

    nav_msgs::Odometry prev_odom;
    bool prev_odom_acquired;
    cnoid::Vector3 prev_rpy;
    void clock_cb(const rosgraph_msgs::ClockPtr &str){};

    bool follow_action_initialized;
    ros::Time traj_start_tm;

    boost::mutex tf_mutex;
    double tf_rate;
    ros::Timer periodic_update_timer;
    std::vector<geometry_msgs::TransformStamped> tf_transforms;
    void periodicTimerCallback(const ros::TimerEvent &event);

    // odometry relatives
    void updateOdometry(const cnoid::Vector3 &trans, const cnoid::Matrix3 &R, const ros::Time &stamp);

    // imu relatives
    ros::Time last_updated_imu_tf_stamp;
    void updateImu(tf::Transform &base, bool is_base_valid, const ros::Time &stamp);

    // sensor relatives
    ros::Time last_updated_sensor_tf_stamp;
    void updateSensorTransform(const ros::Time &stamp);
    std::map<std::string, geometry_msgs::Transform> sensor_transformations;
    boost::mutex sensor_transformation_mutex;
};

extern "C" {
DLL_EXPORT void HrpsysSeqStateROSBridge2Init(RTC::Manager *manager);
};

#endif // HRPSYSSEQSTATEROSBRIDGE2_H

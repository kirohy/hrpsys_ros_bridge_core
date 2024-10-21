#ifndef HRPSYSJOINTTRAJECTORYBRIDGE2_H
#define HRPSYSJOINTTRAJECTORYBRIDGE2_H

#include <rtm/CorbaNaming.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <rtm/Manager.h>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>

#include <cnoid/Body>
#include <cnoid/BodyLoader>
#include <cnoid/Link>

#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>

#ifdef USE_PR2_CONTROLLERS_MSGS
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
#else
#include <control_msgs/JointTrajectoryControllerState.h>
#endif

#include <sequence_player/idl/SequencePlayer2ServiceStub.h>

class HrpsysJointTrajectoryBridge2 : public RTC::DataFlowComponentBase {
  public:
    HrpsysJointTrajectoryBridge2(RTC::Manager *manager);
    ~HrpsysJointTrajectoryBridge2();

    // The initialize action (on CREATED->ALIVE transition)
    // formaer rtc_init_entry()
    RTC::ReturnCode_t onInitialize();
    RTC::ReturnCode_t onFinalize();
    RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);
    RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);

    class jointTrajectoryActionObj {
      protected:
        HrpsysJointTrajectoryBridge2 *parent;

        ros::Publisher joint_controller_state_pub;

#ifdef USE_PR2_CONTROLLERS_MSGS
        boost::shared_ptr<actionlib::SimpleActionServer<pr2_controllers_msgs::JointTrajectoryAction>>
            joint_trajectory_server;
#endif
        std::shared_ptr<actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>>
            follow_joint_trajectory_server;
        ros::Subscriber trajectory_command_sub;

        std::string controller_name;
        std::string groupname;
        std::vector<std::string> joint_list;
        bool interpolationp;
        ros::Time traj_start_tm;

      public:
        typedef std::shared_ptr<jointTrajectoryActionObj> Ptr;
        jointTrajectoryActionObj(HrpsysJointTrajectoryBridge2 *ptr, std::string &cname, std::string &gname,
                                 std::vector<std::string> &jlist);
        ~jointTrajectoryActionObj();

        void onJointTrajectory(trajectory_msgs::JointTrajectory trajectory);
#ifdef USE_PR2_CONTROLLERS_MSGS
        void onJointTrajectoryActionGoal();
        void onJointTrajectoryActionPreempt();
#endif
        void onFollowJointTrajectoryActionGoal();
        void onFollowJointTrajectoryActionPreempt();
        void onTrajectoryCommandCB(const trajectory_msgs::JointTrajectoryConstPtr &msg);

        void proc();
        void restart();
    };

  protected:
    RTC::CorbaPort m_SequencePlayerServicePort;
    RTC::CorbaConsumer<sequence_player::SequencePlayer2Service> m_service0;


  protected:
    cnoid::BodyPtr body;

    ros::NodeHandle nh;
    std::vector<jointTrajectoryActionObj::Ptr> trajectory_actions;
    std::mutex m_mutex;
    std::string nameserver;

  private:
};

extern "C" {
DLL_EXPORT void HrpsysJointTrajectoryBridge2Init(RTC::Manager *manager);
};

#endif // HRPSYSJOINTTRAJECTORYBRIDGE_H

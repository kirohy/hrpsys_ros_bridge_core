#include <joint_trajectory_ros_bridge/HrpsysJointTrajectoryBridge2.h>
#include <rtm/idl/RTC.hh>

// clang-format off
static const char *hrpsysjointtrajectorybridge2_spec[] = {
    "implementation_id", "HrpsysJointTrajectoryBridge2",
    "type_name",         "HrpsysJointTrajectoryBridge2",
    "description",       "hrpsys setJointAngle - ros joint trajectory bridge",
    "version",           "1.0",
    "vendor",            "JSK",
    "category",          "example",
    "activity_type",     "SPORADIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    ""};
// clang-format on

HrpsysJointTrajectoryBridge2::HrpsysJointTrajectoryBridge2(RTC::Manager *manager)
    : RTC::DataFlowComponentBase(manager), m_SequencePlayerServicePort("SequencePlayer2Service") {}

HrpsysJointTrajectoryBridge2::~HrpsysJointTrajectoryBridge2() {}

RTC::ReturnCode_t HrpsysJointTrajectoryBridge2::onInitialize() {
    m_SequencePlayerServicePort.registerConsumer("service0", "SequencePlayer2Service", m_service0);

    addPort(m_SequencePlayerServicePort);

    RTC::Properties &prop = getProperties();

    cnoid::BodyLoader body_loader;
    std::string body_filename;
    if (prop.hasKey("model"))
        body_filename = std::string(prop["model"]);
    else
        body_filename = std::string(this->m_pManager->getConfig()["model"]);
    if (body_filename.find("file://") == 0) { body_filename.erase(0, strlen("file://")); }
    body = body_loader.load(body_filename);
    if (!body) {
        ROS_ERROR_STREAM("failed to load model [" << body_filename << "]");
        return RTC::RTC_ERROR;
    } else {
        ROS_INFO_STREAM("successed to load model [" << body_filename << "]");
    }

    body->calcForwardKinematics();

    ROS_INFO_STREAM("[HrpsysJointTrajectoryBridge] @Initilize name : " << getInstanceName() << " done");

    return RTC::RTC_OK;
}

RTC::ReturnCode_t HrpsysJointTrajectoryBridge2::onActivated(RTC::UniqueId ec_id) {
    // ROS_INFO("ON_ACTIVATED");
    std::string config_name;
    config_name = nh.resolveName("controller_configuration");
    if (nh.hasParam(config_name)) {
        XmlRpc::XmlRpcValue param_val;
        nh.getParam(config_name, param_val);
        if (param_val.getType() == XmlRpc::XmlRpcValue::TypeArray) {
            for (int i = 0; i < param_val.size(); i++) {
                if (param_val[i].getType() == XmlRpc::XmlRpcValue::TypeStruct) {
                    XmlRpc::XmlRpcValue gval = param_val[i]["group_name"];
                    XmlRpc::XmlRpcValue cval = param_val[i]["controller_name"];
                    XmlRpc::XmlRpcValue lval = param_val[i]["joint_list"];

                    std::string gname = gval;
                    std::string cname = cval;
                    std::vector<std::string> jlst;
                    if (lval.getType() == XmlRpc::XmlRpcValue::TypeArray) {
                        for (int s = 0; s < lval.size(); s++) {
                            jlst.push_back(lval[s]);
                        }
                    }
                    if (gname.length() == 0 && cname.length() > 0) { gname = cname; }
                    if (gname.length() > 0 && cname.length() == 0) { cname = gname; }
                    if (gname.length() > 0 && cname.length() > 0 && jlst.size() > 0) {
                        std::stringstream ss;
                        for (size_t s = 0; s < jlst.size(); s++) {
                            ss << " " << jlst[s];
                        }
                        ROS_INFO_STREAM("ADD_GROUP: " << gname << " (" << cname << ")");
                        ROS_INFO_STREAM("    JOINT:" << ss.str());
                        jointTrajectoryActionObj::Ptr tmp =
                            std::make_shared<jointTrajectoryActionObj>(this, cval, gval, jlst);
                        trajectory_actions.push_back(tmp);
                    }
                }
            }
        } else {
            ROS_WARN_STREAM("param: " << config_name << ", configuration is not an array.");
        }
    } else {
        ROS_WARN_STREAM("param: " << config_name << ", param does not exist.");
    }

    return RTC::RTC_OK;
}

RTC::ReturnCode_t HrpsysJointTrajectoryBridge2::onFinalize() {
    // delete objs
    for (size_t i = 0; i < trajectory_actions.size(); i++) {
        trajectory_actions[i].reset();
    }

    return RTC::RTC_OK;
}

RTC::ReturnCode_t HrpsysJointTrajectoryBridge2::onExecute(RTC::UniqueId ec_id) {
    ros::spinOnce();
    for (size_t i = 0; i < trajectory_actions.size(); i++) {
        trajectory_actions[i]->proc();
    }

    return RTC::RTC_OK;
}

HrpsysJointTrajectoryBridge2::jointTrajectoryActionObj::jointTrajectoryActionObj(HrpsysJointTrajectoryBridge2 *ptr,
                                                                                 std::string &cname, std::string &gname,
                                                                                 std::vector<std::string> &jlist) {
    parent          = ptr;
    controller_name = cname;
    groupname       = gname;
    joint_list      = jlist;

#ifdef USE_PR2_CONTROLLERS_MSGS
    joint_trajectory_server.reset(new actionlib::SimpleActionServer<pr2_controllers_msgs::JointTrajectoryAction>(
        parent->nh, controller_name + "/joint_trajectory_action", false));
#endif
    follow_joint_trajectory_server.reset(new actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>(
        parent->nh, controller_name + "/follow_joint_trajectory_action", false));
    trajectory_command_sub =
        parent->nh.subscribe(controller_name + "/command", 1,
                             &HrpsysJointTrajectoryBridge2::jointTrajectoryActionObj::onTrajectoryCommandCB, this);


#ifdef USE_PR2_CONTROLLERS_MSGS
    joint_trajectory_server->registerGoalCallback(
        std::bind(&HrpsysJointTrajectoryBridge2::jointTrajectoryActionObj::onJointTrajectoryActionGoal, this));
    joint_trajectory_server->registerPreemptCallback(
        std::bind(&HrpsysJointTrajectoryBridge2::jointTrajectoryActionObj::onJointTrajectoryActionPreempt, this));
#endif
    follow_joint_trajectory_server->registerGoalCallback(
        std::bind(&HrpsysJointTrajectoryBridge2::jointTrajectoryActionObj::onFollowJointTrajectoryActionGoal, this));
    follow_joint_trajectory_server->registerPreemptCallback(
        std::bind(&HrpsysJointTrajectoryBridge2::jointTrajectoryActionObj::onFollowJointTrajectoryActionPreempt, this));

#ifdef USE_PR2_CONTROLLERS_MSGS
    joint_controller_state_pub =
        parent->nh.advertise<pr2_controllers_msgs::JointTrajectoryControllerState>(controller_name + "/state", 1);
#else
    joint_controller_state_pub =
        parent->nh.advertise<control_msgs::JointTrajectoryControllerState>(controller_name + "/state", 1);
#endif

    if (groupname.length() > 0) {
        sequence_player::SequencePlayer2Service::StrSequence jnames;
        jnames.length(joint_list.size());
        for (size_t i = 0; i < joint_list.size(); i++) {
            jnames[i] = joint_list[i].c_str();
        }
        try {
            parent->m_service0->addJointGroup(groupname.c_str(), jnames);
        } catch (CORBA::SystemException &ex) {
            std::cerr << "[HrpsysJointTrajectoryBridge] addJointGroup(" << groupname << "), CORBA::SystemException "
                      << ex._name() << std::endl;
            sleep(1);
        } catch (...) {
            std::cerr << "[HrpsysJointTrajectoryBridge] addJointGroup(" << groupname << "), failed to addJointGroup["
                      << groupname.c_str() << "]" << std::endl;
            ;
            sleep(1);
        }
    }

    interpolationp = false;
#ifdef USE_PR2_CONTROLLERS_MSGS
    joint_trajectory_server->start();
#endif
    follow_joint_trajectory_server->start();
}

HrpsysJointTrajectoryBridge2::jointTrajectoryActionObj::~jointTrajectoryActionObj() {
    ROS_INFO_STREAM("[" << parent->getInstanceName() << "] @~jointTrajectoryActionObj (" << this->groupname);
#ifdef USE_PR2_CONTROLLERS_MSGS
    if (joint_trajectory_server->isActive()) { joint_trajectory_server->setPreempted(); }
#endif

    if (follow_joint_trajectory_server->isActive()) { follow_joint_trajectory_server->setPreempted(); }

#ifdef USE_PR2_CONTROLLERS_MSGS
    joint_trajectory_server->shutdown();
#endif
    follow_joint_trajectory_server->shutdown();
}

void HrpsysJointTrajectoryBridge2::jointTrajectoryActionObj::proc() {
    // finish interpolation
#ifdef USE_PR2_CONTROLLERS_MSGS
    if (joint_trajectory_server->isActive() && interpolationp == true && parent->m_service0->isEmpty() == true) {
        pr2_controllers_msgs::JointTrajectoryResult result;
        joint_trajectory_server->setSucceeded(result);
        interpolationp = false;
        ROS_INFO_STREAM("[" << parent->getInstanceName() << "] @proc joint_trajectory_server->setSucceeded()");
    }
#endif
    if (follow_joint_trajectory_server->isActive() && interpolationp == true && parent->m_service0->isEmpty() == true) {
        control_msgs::FollowJointTrajectoryResult result;
        result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
        follow_joint_trajectory_server->setSucceeded(result);
        interpolationp = false;
        ROS_INFO_STREAM("[" << parent->getInstanceName() << "] @proc follow_joint_trajectory_server->setSucceeded()");
    }

    ros::Time tm_on_execute = ros::Time::now();

    // FIXME: need to set actual informatoin, currently we set dummy information
    trajectory_msgs::JointTrajectoryPoint commanded_joint_trajectory_point, error_joint_trajectory_point;
    commanded_joint_trajectory_point.time_from_start = tm_on_execute - traj_start_tm;
    commanded_joint_trajectory_point.positions.resize(joint_list.size());
    commanded_joint_trajectory_point.velocities.resize(joint_list.size());
    commanded_joint_trajectory_point.accelerations.resize(joint_list.size());
    commanded_joint_trajectory_point.effort.resize(joint_list.size());
    for (unsigned int j = 0; j < joint_list.size(); j++) {
        commanded_joint_trajectory_point.positions[j]     = parent->body->link(joint_list[j])->q();
        commanded_joint_trajectory_point.velocities[j]    = parent->body->link(joint_list[j])->dq();
        commanded_joint_trajectory_point.accelerations[j] = parent->body->link(joint_list[j])->ddq();
        commanded_joint_trajectory_point.effort[j]        = parent->body->link(joint_list[j])->u();
    }
    error_joint_trajectory_point.time_from_start = tm_on_execute - traj_start_tm;
    error_joint_trajectory_point.positions.resize(joint_list.size());
    error_joint_trajectory_point.velocities.resize(joint_list.size());
    error_joint_trajectory_point.accelerations.resize(joint_list.size());
    error_joint_trajectory_point.effort.resize(joint_list.size());

#ifdef USE_PR2_CONTROLLERS_MSGS
    if (joint_trajectory_server->isActive()) {
        pr2_controllers_msgs::JointTrajectoryFeedback joint_trajectory_feedback;
        joint_trajectory_server->publishFeedback(joint_trajectory_feedback);
    }
#endif

    if (follow_joint_trajectory_server->isActive()) {
        control_msgs::FollowJointTrajectoryFeedback follow_joint_trajectory_feedback;
        follow_joint_trajectory_feedback.header.stamp = tm_on_execute;
        follow_joint_trajectory_feedback.joint_names  = joint_list;

        follow_joint_trajectory_feedback.desired = commanded_joint_trajectory_point;
        follow_joint_trajectory_feedback.actual  = commanded_joint_trajectory_point;
        follow_joint_trajectory_feedback.error   = error_joint_trajectory_point;

        follow_joint_trajectory_server->publishFeedback(follow_joint_trajectory_feedback);
    }

#ifdef USE_PR2_CONTROLLERS_MSGS
    pr2_controllers_msgs::JointTrajectoryControllerState joint_controller_state;
#else
    control_msgs::JointTrajectoryControllerState joint_controller_state;
#endif
    joint_controller_state.joint_names = joint_list;

    joint_controller_state.desired = commanded_joint_trajectory_point;
    joint_controller_state.actual  = commanded_joint_trajectory_point;
    joint_controller_state.error   = error_joint_trajectory_point;

    joint_controller_state_pub.publish(joint_controller_state);
}

void HrpsysJointTrajectoryBridge2::jointTrajectoryActionObj::restart() {
    parent->m_service0->removeJointGroup(groupname.c_str());
    usleep(100000);
    if (groupname.length() > 0) {
        sequence_player::SequencePlayer2Service::StrSequence jnames;
        jnames.length(joint_list.size());
        for (size_t i = 0; i < joint_list.size(); i++) {
            jnames[i] = joint_list[i].c_str();
        }
        try {
            parent->m_service0->addJointGroup(groupname.c_str(), jnames);
        } catch (CORBA::SystemException &ex) {
            std::cerr << "[HrpsysJointTrajectoryBridge] addJointGroup(" << groupname << "), CORBA::SystemException "
                      << ex._name() << std::endl;
            sleep(1);
        } catch (...) {
            std::cerr << "[HrpsysJointTrajectoryBridge] addJointGroup(" << groupname << "), failed to addJointGroup["
                      << groupname.c_str() << "]" << std::endl;
            ;
        }
    }
}

void HrpsysJointTrajectoryBridge2::jointTrajectoryActionObj::onJointTrajectory(
    trajectory_msgs::JointTrajectory trajectory) {
    parent->m_mutex.lock();

    ROS_INFO_STREAM("[" << parent->getInstanceName() << "] @onJointTrajectoryAction (" << this->groupname << ")");
    // TODO: check size and joint names

    sequence_player::dSequenceSequence angles;
    sequence_player::dSequence duration;

    angles.length(trajectory.points.size());
    duration.length(trajectory.points.size());

    std::vector<std::string> joint_names = trajectory.joint_names;
    if (joint_names.size() < joint_list.size()) {
        ROS_ERROR_STREAM("[" << parent->getInstanceName() << "] @onJointTrajectoryAction / Error : "
                             << "required joint_names.size() = " << joint_names.size()
                             << " < joint_list.size() = " << joint_list.size());
        parent->m_mutex.unlock();
        return;
    }
    for (unsigned int i = 0; i < joint_list.size(); i++) {
        if (count(joint_names.begin(), joint_names.end(), joint_list[i]) != 1) {
            ROS_ERROR_STREAM("[" << parent->getInstanceName() << "] @onJointTrajectoryAction / Error : "
                                 << "joint : " << joint_list[i] << " did not exist in the required trajectory.");
            parent->m_mutex.unlock();
            return;
        }
    }

    ROS_INFO_STREAM("[" << parent->getInstanceName() << "] @onJointTrajectoryAction (" << this->groupname
                        << ") : trajectory.points.size() " << trajectory.points.size());
    for (unsigned int i = 0; i < trajectory.points.size(); i++) {
        angles[i].length(joint_names.size());

        trajectory_msgs::JointTrajectoryPoint point = trajectory.points[i];
        for (unsigned int j = 0; j < joint_names.size(); j++) {
            cnoid::Link *l = parent->body->link(joint_names[j]);
            if (l)
                l->q() = point.positions[j];
            else
                ROS_WARN_STREAM_ONCE("[" << parent->getInstanceName() << "] @onJointTrajectoryAction ("
                                         << this->groupname << ") : joint named " << joint_names[j]
                                         << " is not found. Skipping ...");
        }

        parent->body->calcForwardKinematics();

        std::stringstream ss;
        for (unsigned int j = 0; j < joint_list.size(); j++) {
            angles[i][j] = parent->body->link(joint_list[j])->q();
            ss << " " << point.positions[j];
        }
        ROS_INFO_STREAM("[" << parent->getInstanceName() << "] i:" << i << " : time_from_start "
                            << trajectory.points[i].time_from_start.toSec());

        if (i > 0) {
            duration[i] =
                trajectory.points[i].time_from_start.toSec() - trajectory.points[i - 1].time_from_start.toSec();
        } else { // if i == 0
            duration[i] = trajectory.points[i].time_from_start.toSec();
            if (std::abs(duration[i]) < 0.001)
                duration[i] = 0.001; // set magic delta ... https://github.com/start-jsk/rtmros_common/issues/1036
        }
        ROS_INFO_STREAM("[" << parent->getInstanceName() << "] [" << ss.str() << "] " << duration[i]);
    }

    parent->m_mutex.unlock();
    if (duration.length() == 1) {
        if (groupname.length() > 0) { // group
            ROS_INFO_STREAM("[" << parent->getInstanceName() << "] setJointAnglesOfGroup: " << groupname);
            parent->m_service0->setJointAnglesOfGroup(groupname.c_str(), angles[0], duration[0]);
        } else { // fullbody
            ROS_INFO_STREAM("[" << parent->getInstanceName() << "] setJointAngles");
            parent->m_service0->setJointAngles(angles[0], duration[0]);
        }
    } else {
        if (groupname.length() > 0) { // group
            ROS_INFO_STREAM("[" << parent->getInstanceName() << "] setJointAnglesSequenceOfGroup: " << groupname);
            parent->m_service0->setJointAnglesSequenceOfGroup(groupname.c_str(), angles, duration);
        } else { // fullbody
            ROS_INFO_STREAM("[" << parent->getInstanceName() << "] setJointAnglesSequence");
            parent->m_service0->setJointAnglesSequence(angles, duration);
        }
    }
    traj_start_tm = ros::Time::now();

    interpolationp = true;
}

#ifdef USE_PR2_CONTROLLERS_MSGS
void HrpsysJointTrajectoryBridge2::jointTrajectoryActionObj::onJointTrajectoryActionGoal() {
    ROS_INFO_STREAM("[" << parent->getInstanceName() << "] @onJointTrajectoryActionGoal");
    pr2_controllers_msgs::JointTrajectoryGoalConstPtr goal = joint_trajectory_server->acceptNewGoal();
    onJointTrajectory(goal->trajectory);
}
#endif

void HrpsysJointTrajectoryBridge2::jointTrajectoryActionObj::onFollowJointTrajectoryActionGoal() {
    ROS_INFO_STREAM("[" << parent->getInstanceName() << "] @onFollowJointTrajectoryActionGoal()");
    control_msgs::FollowJointTrajectoryGoalConstPtr goal = follow_joint_trajectory_server->acceptNewGoal();
    onJointTrajectory(goal->trajectory);
}

#ifdef USE_PR2_CONTROLLERS_MSGS
void HrpsysJointTrajectoryBridge2::jointTrajectoryActionObj::onJointTrajectoryActionPreempt() {
    ROS_INFO_STREAM("[" << parent->getInstanceName() << "] @onJointTrajectoryActionPreempt()");
    joint_trajectory_server->setPreempted();
    if (!joint_trajectory_server
             ->isNewGoalAvailable()) { // Cancel request comes from client, so motion should be stopped immediately,
        // while motion should be changed smoothly when new goal comes
        if (groupname.length() > 0) { // group
            ROS_INFO_STREAM("[" << parent->getInstanceName() << "] clearJointAnglesOfGroup: " << groupname);
            parent->m_service0->clearJointAnglesOfGroup(groupname.c_str());
        } else { // fullbody
            ROS_INFO_STREAM("[" << parent->getInstanceName() << "] clearJointAngles");
            parent->m_service0->clearJointAngles();
        }
    }
}
#endif

void HrpsysJointTrajectoryBridge2::jointTrajectoryActionObj::onFollowJointTrajectoryActionPreempt() {
    ROS_INFO_STREAM("[" << parent->getInstanceName() << "] @onFollowJointTrajectoryActionPreempt()");
    follow_joint_trajectory_server->setPreempted();

    if (!follow_joint_trajectory_server
             ->isNewGoalAvailable()) { // Cancel request comes from client, so motion should be stopped immediately,
        // while motion should be changed smoothly when new goal comes
        if (groupname.length() > 0) { // group
            ROS_INFO_STREAM("[" << parent->getInstanceName() << "] clearJointAnglesOfGroup: " << groupname);
            parent->m_service0->clearJointAnglesOfGroup(groupname.c_str());
        } else { // fullbody
            ROS_INFO_STREAM("[" << parent->getInstanceName() << "] clearJointAngles");
            parent->m_service0->clearJointAngles();
        }
    }
}

void HrpsysJointTrajectoryBridge2::jointTrajectoryActionObj::onTrajectoryCommandCB(
    const trajectory_msgs::JointTrajectoryConstPtr &msg) {
    ROS_INFO_STREAM("[" << parent->getInstanceName() << "] @onTrajectoryCommandCB()");
    onJointTrajectory(*msg);
}

extern "C" {

void HrpsysJointTrajectoryBridge2Init(RTC::Manager *manager) {
    RTC::Properties profile(hrpsysjointtrajectorybridge2_spec);
    manager->registerFactory(profile, RTC::Create<HrpsysJointTrajectoryBridge2>,
                             RTC::Delete<HrpsysJointTrajectoryBridge2>);
}
};

#include <seq_state_ros_bridge/HrpsysSeqStateROSBridge2Impl.h>

static const char *hrpsysseqstaterosbridge2impl_spec[] = {"implementation_id", "HrpsysSeqStateROSBridge2Impl", "type_name", "HrpsysSeqStateROSBridge2Impl",
                                                          "description", "hrpsys seq state - ros bridge", "version", "1.0", "vendor", "JSK", "category",
                                                          "example", "activity_type", "SPORADIC", "kind", "DataFlowComponent", "max_instance", "10", "language",
                                                          "C++", "lang_type", "compile",
                                                          // Configuration variables
                                                          ""};

HrpsysSeqStateROSBridge2Impl::HrpsysSeqStateROSBridge2Impl(RTC::Manager *manager)
    : RTC::DataFlowComponentBase(manager), m_rsangleIn("rsangle", m_rsangle), m_mcangleIn("mcangle", m_mcangle), m_baseTformIn("baseTform", m_baseTform),
      m_baseRpyIn("baseRpy", m_baseRpy), m_rsvelIn("rsvel", m_rsvel), m_rstorqueIn("rstorque", m_rstorque), m_servoStateIn("servoState", m_servoState),
      m_rszmpIn("rszmp", m_rszmp), m_rsrefCPIn("rsrefCapturePoint", m_rsrefCP), m_rsactCPIn("rsactCapturePoint", m_rsactCP),
      m_rsCOPInfoIn("rsCOPInfo", m_rsCOPInfo), m_emergencyModeIn("emergencyMode", m_emergencyMode),
      m_refContactStatesIn("refContactStates", m_refContactStates), m_actContactStatesIn("actContactStates", m_actContactStates),
      m_controlSwingSupportTimeIn("controlSwingSupportTime", m_controlSwingSupportTime), m_mctorqueOut("mctorque", m_mctorque),
      m_SequencePlayerServicePort("SequencePlayer2Service") {}

HrpsysSeqStateROSBridge2Impl::~HrpsysSeqStateROSBridge2Impl() {}

RTC::ReturnCode_t HrpsysSeqStateROSBridge2Impl::onInitialize() {
    // Set InPort buffers
    addInPort("rsangle", m_rsangleIn);
    addInPort("mcangle", m_mcangleIn);
    addInPort("baseTform", m_baseTformIn);
    addInPort("baseRpy", m_baseRpyIn);
    addInPort("rsvel", m_rsvelIn);
    addInPort("rstorque", m_rstorqueIn);
    addInPort("rszmp", m_rszmpIn);
    addInPort("rsrefCapturePoint", m_rsrefCPIn);
    addInPort("rsactCapturePoint", m_rsactCPIn);
    addInPort("servoState", m_servoStateIn);
    addInPort("rsCOPInfo", m_rsCOPInfoIn);
    addInPort("emergencyMode", m_emergencyModeIn);
    addInPort("refContactStates", m_refContactStatesIn);
    addInPort("actContactStates", m_actContactStatesIn);
    addInPort("controlSwingSupportTime", m_controlSwingSupportTimeIn);

    // Set OutPort buffer
    addOutPort("mctorque", m_mctorqueOut);

    // Set service provider to Ports

    // Set service consumers to Ports
    m_SequencePlayerServicePort.registerConsumer("service0", "SequencePlayer2Service", m_service0);

    // Set CORBA Service Ports
    addPort(m_SequencePlayerServicePort);

    RTC::Properties &prop = getProperties();

    if (prop.hasKey("dt")) {
        dt = std::stod(std::string(prop["dt"]));
    } else {
        double rate = std::stod(std::string(this->m_pManager->getConfig()["exec_cxt.periodic.rate"]));
        if (rate > 0.0) {
            dt = 1.0 / rate;
        } else {
            RTC_WARN_STREAM("dt is invalid");
            return RTC::RTC_ERROR;
        }
    }
    RTC_INFO_STREAM("dt = " << dt);

    cnoid::BodyLoader body_loader;
    std::string body_filename;
    if (prop.hasKey("model"))
        body_filename = std::string(prop["model"]);
    else
        body_filename = std::string(this->m_pManager->getConfig()["model"]);
    if (body_filename.find("file://") == 0) { body_filename.erase(0, strlen("file://")); }
    body = body_loader.load(body_filename);
    if (!body) {
        RTC_WARN_STREAM("failed to load model [" << body_filename << "]");
        return RTC::RTC_ERROR;
    } else {
        RTC_INFO_STREAM("successed to load model [" << body_filename << "]");
    }

    // Force Sensor Settings
    // coil::vstring virtual_force_sensor =
    // coil::split(prop["virtual_force_sensor"], ","); int npforce =
    // body->numSensors(hrp::Sensor::FORCE); int nvforce =
    // virtual_force_sensor.size()/10; int nforce  = npforce + nvforce;
    cnoid::DeviceList<cnoid::ForceSensor> force_sensors(body->devices());
    int nforce = force_sensors.size();
    m_rsforce.resize(nforce);
    m_rsforceIn.resize(nforce);
    m_offforce.resize(nforce);
    m_offforceIn.resize(nforce);
    m_mcforce.resize(nforce);
    m_mcforceIn.resize(nforce);
    for (unsigned int i = 0; i < nforce; i++) {
        cnoid::ForceSensorPtr s = force_sensors[i];
        // force and moment
        m_rsforceIn[i] = std::make_unique<RTC::InPort<RTC::TimedDoubleSeq>>(s->name().c_str(), m_rsforce[i]);
        m_rsforce[i].data.length(6);
        registerInPort(s->name().c_str(), *m_rsforceIn[i]);
        m_rsforceName.push_back(s->name());
        // off force and moment
        m_offforceIn[i] = std::make_unique<RTC::InPort<RTC::TimedDoubleSeq>>(std::string("off_" + s->name()).c_str(), m_offforce[i]);
        m_offforce[i].data.length(6);
        registerInPort(s->name().c_str(), *m_offforceIn[i]);
        m_offforceName.push_back(std::string("off_" + s->name()));
        // ref force and moment
        m_mcforceIn[i] = std::make_unique<RTC::InPort<RTC::TimedDoubleSeq>>(std::string("ref_" + s->name()).c_str(), m_mcforce[i]);
        m_mcforce[i].data.length(6);
        registerInPort(std::string("ref_" + s->name()).c_str(), *m_mcforceIn[i]);
        m_mcforceName.push_back(std::string("ref_" + s->name()));
        RTC_INFO_STREAM(i << " physical force sensor : " << s->name());

        SensorInfo si;
        cnoid::Vector3 localp = s->p_local();
        si.transform.setOrigin(tf::Vector3(localp(0), localp(1), localp(2)));
        cnoid::Vector3 rpy = cnoid::rpyFromRot(s->R_local());
        si.transform.setRotation(tf::createQuaternionFromRPY(rpy(0), rpy(1), rpy(2)));
        si.link_name           = s->link()->name();
        si.type_name           = s->typeName();
        sensor_info[s->name()] = si;
    }

    // Sensor Settings
    // for (int j = 0 ; j < body->numSensorTypes(); j++) {
    //   for (int i = 0 ; i < body->numSensors(j); i++) {
    //     hrp::Sensor* sensor = body->sensor(j, i);
    //     if (! sensor ) {
    //       std::cerr << "ERROR : Unknown sensor (type : " << j << ", id : " << i
    //       << ")" << std::endl; std::cerr << "ERROR : Please make sure that each
    //       sensor type start from 0 sensorId" << std::endl; std::cerr << "ERROR
    //       : THIS WILL CAUSE SEVERE PROBLEM, PLEASE FIX YOUR MODEL FILE " <<
    //       std::endl; continue;
    //     }
    //     SensorInfo si;
    //     // localPos is parent.
    //     https://github.com/start-jsk/rtmros_common/pull/925
    //     https://github.com/start-jsk/rtmros_common/pull/1114 hrp::Vector3
    //     localp = sensor->link->Rs.inverse() * sensor->localPos;
    //     si.transform.setOrigin( tf::Vector3(localp(0), localp(1), localp(2)) );
    //     hrp::Vector3 rpy;
    //     if ( hrp::Sensor::VISION == sensor->type )
    //       // Rotate sensor->localR 180[deg] because OpenHRP3 camera -Z axis
    //       equals to ROS camera Z axis
    //       // http://www.openrtp.jp/openhrp3/jp/create_model.html
    //       // localR is parent.
    //       https://github.com/start-jsk/rtmros_common/pull/925
    //       https://github.com/start-jsk/rtmros_common/pull/1114 rpy =
    //       hrp::rpyFromRot(sensor->link->Rs.inverse() * sensor->localR *
    //       hrp::rodrigues(hrp::Vector3(1,0,0), M_PI));
    //     else if ( hrp::Sensor::RANGE == sensor->type )
    //       {
    //         // OpenHRP3 range sensor, front direction is -Z axis, and detected
    //         plane is XZ plane
    //         // ROS LaserScan, front direction is X axis, and detected plane is
    //         XY plane
    //         // http://www.openrtp.jp/openhrp3/jp/create_model.html
    //         hrp::Matrix33 m;
    //         m << 0, -1, 0, 0, 0, 1, -1, 0, 0;
    //         // localR is parent.
    //         https://github.com/start-jsk/rtmros_common/pull/925
    //         https://github.com/start-jsk/rtmros_common/pull/1114 rpy =
    //         hrp::rpyFromRot(sensor->link->Rs.inverse() * sensor->localR * m);
    //       }
    //     else
    //     {
    //       // localR is parent.
    //       https://github.com/start-jsk/rtmros_common/pull/925
    //       https://github.com/start-jsk/rtmros_common/pull/1114 rpy =
    //       hrp::rpyFromRot(sensor->link->Rs.inverse() * sensor->localR);
    //     }
    //     si.transform.setRotation( tf::createQuaternionFromRPY(rpy(0), rpy(1),
    //     rpy(2)) ); OpenHRP::LinkInfoSequence_var links = bodyinfo->links(); for
    //     ( int k = 0; k < links->length(); k++ ) {
    //       OpenHRP::SensorInfoSequence& sensors = links[k].sensors;
    //       for ( int l = 0; l < sensors.length(); l++ ) {
    //         if ( std::string(sensors[l].name) == std::string(sensor->name) ) {
    //           si.link_name = links[k].segments[0].name;
    //           si.type_name = sensors[l].type;
    //           sensor_info[sensor->name] = si;
    //         }
    //       }
    //     }
    //   }
    // }

    // Virtual Force Sensor Settings
    // for(unsigned int j = 0, i = npforce; j < nvforce; j++, i++ ){
    //   std::string name = virtual_force_sensor[j*10+0];
    //   std::string base = virtual_force_sensor[j*10+1];
    //   std::string target = virtual_force_sensor[j*10+2];
    //   hrp::dvector tr(7);
    //   for (int k = 0; k < 7; k++ ) {
    //       coil::stringTo(tr[k], virtual_force_sensor[j*10+3+k].c_str());
    //   }
    //   // virtual force and moment
    //   m_rsforceIn[i] = new InPort<TimedDoubleSeq>(name.c_str(), m_rsforce[i]);
    //   m_rsforce[i].data.length(6);
    //   registerInPort(name.c_str(), *m_rsforceIn[i]);
    //   m_rsforceName.push_back(name);
    //   // off force and moment
    //   m_offforceIn[i] = new InPort<TimedDoubleSeq>(std::string("off_" +
    //   name).c_str(), m_offforce[i]); m_offforce[i].data.length(6);
    //   registerInPort(name.c_str(), *m_offforceIn[i]);
    //   m_offforceName.push_back(std::string("off_" + name));
    //   // reference virtual force and moment
    //   m_mcforceIn[i] = new
    //   InPort<TimedDoubleSeq>(std::string("ref_"+name).c_str(), m_mcforce[i]);
    //   m_mcforce[i].data.length(6);
    //   registerInPort(std::string("ref_"+name).c_str(), *m_mcforceIn[i]);
    //   m_mcforceName.push_back(std::string("ref_"+name).c_str());

    //   if ( ! body->link(base) ) {
    //     std::cerr << "ERROR : unknown link : " << base << std::endl;
    //   }
    //   if ( ! body->link(target) ) {
    //     std::cerr << "ERROR : unknown link : " << target << std::endl;
    //   }

    //   SensorInfo si;
    //   si.transform.setOrigin( tf::Vector3(tr[0], tr[1], tr[2]) );
    //   Eigen::Quaternion<double> qtn(Eigen::AngleAxis<double>(tr[6],
    //   hrp::Vector3(tr[3],tr[4],tr[5]))); si.transform.setRotation(
    //   tf::Quaternion(qtn.x(), qtn.y(), qtn.z(), qtn.w()) );
    //   OpenHRP::LinkInfoSequence_var links = bodyinfo->links();
    //   for ( int k = 0; k < links->length(); k++ ) {
    //     if ( std::string(links[k].name) == target ) {
    //       si.link_name = links[k].segments[0].name;
    //       si.type_name = "Force";
    //     }
    //   }
    //   sensor_info[name] = si;

    //   std::cerr << i << " virtual force sensor : " << name << ": "  << base <<
    //   "," << target << std::endl;
    // }

    cnoid::DeviceList<cnoid::AccelerationSensor> acc_sensors(body->devices());
    int nacc = acc_sensors.size();
    m_gsensor.resize(nacc);
    m_gsensorIn.resize(nacc);
    m_gsensorName.resize(nacc);
    for (unsigned int i = 0; i < nacc; i++) {
        cnoid::AccelerationSensorPtr s = acc_sensors[i];
        m_gsensorIn[i]                 = std::make_unique<RTC::InPort<RTC::TimedAcceleration3D>>(s->name().c_str(), m_gsensor[i]);
        m_gsensorName[i]               = s->name().c_str();
        registerInPort(s->name().c_str(), *m_gsensorIn[i]);
        RTC_INFO_STREAM(i << " acceleration sensor : " << s->name().c_str());

        SensorInfo si;
        cnoid::Vector3 localp = s->p_local();
        si.transform.setOrigin(tf::Vector3(localp(0), localp(1), localp(2)));
        cnoid::Vector3 rpy = cnoid::rpyFromRot(s->R_local());
        si.transform.setRotation(tf::createQuaternionFromRPY(rpy(0), rpy(1), rpy(2)));
        si.link_name           = s->link()->name();
        si.type_name           = s->typeName();
        sensor_info[s->name()] = si;
    }

    cnoid::DeviceList<cnoid::RateGyroSensor> gyro_sensors(body->devices());
    int ngyro = gyro_sensors.size();
    m_gyrometer.resize(ngyro);
    m_gyrometerIn.resize(ngyro);
    m_gyrometerName.resize(ngyro);
    for (unsigned int i = 0; i < ngyro; i++) {
        cnoid::RateGyroSensorPtr s = gyro_sensors[i];
        m_gyrometerIn[i]           = std::make_unique<RTC::InPort<RTC::TimedAngularVelocity3D>>(s->name().c_str(), m_gyrometer[i]);
        m_gyrometerName[i]         = s->name().c_str();
        registerInPort(s->name().c_str(), *m_gyrometerIn[i]);
        RTC_INFO_STREAM(i << " rate sensor : " << s->name().c_str());

        SensorInfo si;
        cnoid::Vector3 localp = s->p_local();
        si.transform.setOrigin(tf::Vector3(localp(0), localp(1), localp(2)));
        cnoid::Vector3 rpy = cnoid::rpyFromRot(s->R_local());
        si.transform.setRotation(tf::createQuaternionFromRPY(rpy(0), rpy(1), rpy(2)));
        si.link_name           = s->link()->name();
        si.type_name           = s->typeName();
        sensor_info[s->name()] = si;
    }

    cnoid::DeviceList<cnoid::VisionSensor> vision_sensors(body->devices());
    for (unsigned int i = 0; i < vision_sensors.size(); i++) {
        cnoid::VisionSensorPtr s = vision_sensors[i];
        RTC_INFO_STREAM(i << " vision sensor : " << s->name().c_str());

        SensorInfo si;
        cnoid::Vector3 localp = s->p_local();
        si.transform.setOrigin(tf::Vector3(localp(0), localp(1), localp(2)));
        cnoid::Vector3 rpy = cnoid::rpyFromRot(s->R_local());
        si.transform.setRotation(tf::createQuaternionFromRPY(rpy(0), rpy(1), rpy(2)));
        si.link_name           = s->link()->name();
        si.type_name           = s->typeName();
        sensor_info[s->name()] = si;
    }

    cnoid::DeviceList<cnoid::RangeSensor> range_sensors(body->devices());
    for (unsigned int i = 0; i < range_sensors.size(); i++) {
        cnoid::RangeSensorPtr s = range_sensors[i];
        RTC_INFO_STREAM(i << " range sensor : " << s->name().c_str());

        SensorInfo si;
        cnoid::Vector3 localp = s->p_local();
        si.transform.setOrigin(tf::Vector3(localp(0), localp(1), localp(2)));
        cnoid::Vector3 rpy = cnoid::rpyFromRot(s->R_local());
        si.transform.setRotation(tf::createQuaternionFromRPY(rpy(0), rpy(1), rpy(2)));
        si.link_name           = s->link()->name();
        si.type_name           = s->typeName();
        sensor_info[s->name()] = si;
    }

    // initialize basePos, baseRpy
    {
        cnoid::Matrix3 R   = body->rootLink()->R();
        cnoid::Vector3 rpy = cnoid::rpyFromRot(R);

        m_baseRpy.data.r = rpy[0];
        m_baseRpy.data.p = rpy[1];
        m_baseRpy.data.y = rpy[2];
    }

    // End effector setting from conf file
    // rleg,TARGET_LINK,BASE_LINK,x,y,z,rx,ry,rz,rth #<=pos + rot (axis+angle)
    // coil::vstring end_effectors_str = coil::split(prop["end_effectors"], ",");
    // if (end_effectors_str.size() > 0) {
    //   size_t prop_num = 10;
    //   size_t num = end_effectors_str.size()/prop_num;
    //   for (size_t i = 0; i < num; i++) {
    //     // Parse end-effector information from conf
    //     std::string ee_name, ee_target, ee_base;
    //     coil::stringTo(ee_name, end_effectors_str[i*prop_num].c_str());
    //     coil::stringTo(ee_target, end_effectors_str[i*prop_num+1].c_str());
    //     coil::stringTo(ee_base, end_effectors_str[i*prop_num+2].c_str());
    //     hrp::Vector3 eep;
    //     for (size_t j = 0; j < 3; j++) {
    //       coil::stringTo(eep(j), end_effectors_str[i*prop_num+3+j].c_str());
    //     }
    //     std::cerr << "[" << m_profile.instance_name << "] End Effector [" <<
    //     ee_name << "]" << ee_target << " " << ee_base << std::endl;
    //     // Find pair between end-effector information and force sensor name
    //     bool is_sensor_exists = false;
    //     std::string sensor_name;
    //     for (size_t ii = 0; ii < m_mcforceName.size(); ii++) {
    //       std::string tmpname = m_mcforceName[ii];
    //       tmpname.erase(0,4);
    //       hrp::ForceSensor* sensor = body->sensor<hrp::ForceSensor>(tmpname);
    //       std::string sensor_link_name;
    //       if ( sensor ) {
    //         // real force sensor
    //         sensor_link_name = sensor->link->name;
    //       } else if (sensor_info.find(tmpname) !=  sensor_info.end()) {
    //         // virtual force sensor
    //         sensor_link_name = sensor_info[tmpname].link_name;
    //         sensor_link_name = sensor_link_name.substr(0,
    //         sensor_link_name.size()-5); // such that LLEG_JOINT0_LINK ->
    //         LLEG_JOINT0
    //       } else {
    //         std::cerr << "[" << m_profile.instance_name << "]   unknown force
    //         param" << std::endl; continue;
    //       }
    //       hrp::Link* alink = body->link(ee_target);
    //       while (alink != NULL && alink->name != ee_base && !is_sensor_exists)
    //       {
    //         if ( alink->name == sensor_link_name ) {
    //           is_sensor_exists = true;
    //           sensor_name = tmpname;
    //         }
    //         alink = alink->parent;
    //       }
    //     }
    //     if (!is_sensor_exists) {
    //       std::cerr << "[" << m_profile.instance_name << "]   No force sensors
    //       found [" << ee_target << "]" << std::endl; continue;
    //     }
    //     // Set cop_link_info
    //     COPLinkInfo ci;
    //     ci.link_name = sensor_info[sensor_name].link_name; // Link name for tf
    //     frame ci.cop_offset_z = eep(2);
    //     cop_link_info.insert(std::pair<std::string, COPLinkInfo>(sensor_name,
    //     ci)); std::cerr << "[" << m_profile.instance_name << "]   target = " <<
    //     ci.link_name << ", sensor_name = " << sensor_name << std::endl;
    //     std::cerr << "[" << m_profile.instance_name << "]   localPos = " <<
    //     eep.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "",
    //     "", "    [", "]")) << "[m]" << std::endl;
    //   }
    // }

    std::string endEffectors = std::string(prop["endEffectors"]);
    std::stringstream ss_endEffectors(endEffectors);
    std::string buf;
    while (std::getline(ss_endEffectors, buf, ',')) {
        std::string name;
        std::string parentLink;
        std::string baseLink;
        cnoid::Vector3 localp;
        cnoid::Vector3 localaxis;
        double localangle;

        // name, parentLink, (not used), x, y, z, theta, ax, ay, az
        name = buf;
        if (!std::getline(ss_endEffectors, buf, ',')) break;
        parentLink = buf;
        if (!std::getline(ss_endEffectors, buf, ',')) break;
        baseLink = buf;
        if (!std::getline(ss_endEffectors, buf, ',')) break;
        localp[0] = std::stod(buf);
        if (!std::getline(ss_endEffectors, buf, ',')) break;
        localp[1] = std::stod(buf);
        if (!std::getline(ss_endEffectors, buf, ',')) break;
        localp[2] = std::stod(buf);
        if (!std::getline(ss_endEffectors, buf, ',')) break;
        localaxis[0] = std::stod(buf);
        if (!std::getline(ss_endEffectors, buf, ',')) break;
        localaxis[1] = std::stod(buf);
        if (!std::getline(ss_endEffectors, buf, ',')) break;
        localaxis[2] = std::stod(buf);
        if (!std::getline(ss_endEffectors, buf, ',')) break;
        localangle = std::stod(buf);
        RTC_INFO_STREAM(" End Effector [" << name << "]" << parentLink << " " << baseLink);

        // check validity
        name.erase(std::remove(name.begin(), name.end(), ' '),
                   name.end()); // remove whitespace
        parentLink.erase(std::remove(parentLink.begin(), parentLink.end(), ' '),
                         parentLink.end()); // remove whitespace
        if (!body->link(parentLink)) {
            RTC_WARN_STREAM(" link [" << parentLink << "]"
                                      << " is not found for " << name);
            return RTC::RTC_ERROR;
        }

        cnoid::Matrix3 localR;
        if (localaxis.norm() == 0)
            localR = cnoid::Matrix3::Identity();
        else
            localR = Eigen::AngleAxisd(localangle, localaxis.normalized()).toRotationMatrix();
        cnoid::Isometry3 localT;
        localT.translation() = localp;
        localT.linear()      = localR;

        bool is_sensor_exists = false;
        std::string sensor_name;
        for (size_t ii = 0; ii < m_mcforceName.size(); ii++) {
            std::string tmpname = m_mcforceName[ii];
            tmpname.erase(0, 4);
            cnoid::ForceSensorPtr sensor = body->findDevice<cnoid::ForceSensor>(tmpname);
            std::string sensor_link_name;
            if (sensor) {
                // real force sensor
                sensor_link_name = sensor->link()->name();
                // } else if (sensor_info.find(tmpname) !=  sensor_info.end()) {
                //   // virtual force sensor
                //   sensor_link_name = sensor_info[tmpname].link_name();
                //   sensor_link_name = sensor_link_name.substr(0,
                //   sensor_link_name.size()-5); // such that LLEG_JOINT0_LINK ->
                //   LLEG_JOINT0
            } else {
                RTC_INFO_STREAM("  unknown force param");
                continue;
            }
            cnoid::LinkPtr alink = body->link(parentLink);
            while (alink != nullptr && alink->name() != baseLink && !is_sensor_exists) {
                if (alink->name() == sensor_link_name) {
                    is_sensor_exists = true;
                    sensor_name      = tmpname;
                }
                alink = alink->parent();
            }
        }
        if (!is_sensor_exists) {
            RTC_INFO_STREAM("  No force sensors found [" << parentLink << "]");
            continue;
        }
        COPLinkInfo ci;
        ci.link_name    = sensor_info[sensor_name].link_name; // Link name for tf frame
        ci.cop_offset_z = localp(2);
        cop_link_info.insert(std::pair<std::string, COPLinkInfo>(sensor_name, ci));
        RTC_INFO_STREAM("  target = " << ci.link_name << ", sensor_name = " << sensor_name);
        RTC_INFO_STREAM("  localPos = " << localp.transpose() << "[m]");
    }

    return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t HrpsysSeqStateROSBridgeImpl::onFinalize()
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t HrpsysSeqStateROSBridgeImpl::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t HrpsysSeqStateROSBridgeImpl::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t HrpsysSeqStateROSBridge2Impl::onActivated(RTC::UniqueId ec_id) {
    // hrpsys_version = GetHrpsysVersion(m_SequencePlayerServicePort);

    return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t HrpsysSeqStateROSBridgeImpl::onDeactivated(RTC::UniqueId
ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t HrpsysSeqStateROSBridgeImpl::onExecute(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t HrpsysSeqStateROSBridgeImpl::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t HrpsysSeqStateROSBridgeImpl::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t HrpsysSeqStateROSBridgeImpl::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t HrpsysSeqStateROSBridgeImpl::onStateUpdate(RTC::UniqueId
ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t HrpsysSeqStateROSBridgeImpl::onRateChanged(RTC::UniqueId
ec_id)
{
  return RTC::RTC_OK;
}
*/

extern "C" {

void HrpsysSeqStateROSBridge2ImplInit(RTC::Manager *manager) {
    coil::Properties profile(hrpsysseqstaterosbridge2impl_spec);
    manager->registerFactory(profile, RTC::Create<HrpsysSeqStateROSBridge2Impl>, RTC::Delete<HrpsysSeqStateROSBridge2Impl>);
}
};

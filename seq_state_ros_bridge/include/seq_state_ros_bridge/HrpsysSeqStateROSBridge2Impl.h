#ifndef HRPSYSSEQSTATEROSBRIDGE2IMPL_H
#define HRPSYSSEQSTATEROSBRIDGE2IMPL_H

#include <rtm/CorbaNaming.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <rtm/Manager.h>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>

#include <cnoid/AccelerationSensor>
#include <cnoid/Body>
#include <cnoid/BodyLoader>
#include <cnoid/EigenUtil>
#include <cnoid/ForceSensor>
#include <cnoid/Link>
#include <cnoid/RangeSensor>
#include <cnoid/RateGyroSensor>
#include <cnoid/VisionSensor>

#include <memory>
#include <tf/transform_broadcaster.h>

#include <hrpsys/idl/HRPDataTypes.hh>
#include <sequence_player/idl/SequencePlayer2ServiceStub.h>

#define RTC_INFO_STREAM(var) std::cout << "[" << m_profile.instance_name << "] " << var << std::endl;
#define RTC_WARN_STREAM(var) std::cerr << "\x1b[31m[" << m_profile.instance_name << "] " << var << "\x1b[39m" << std::endl;

class HrpsysSeqStateROSBridge2Impl : public RTC::DataFlowComponentBase {
  public:
    HrpsysSeqStateROSBridge2Impl(RTC::Manager *manager);
    ~HrpsysSeqStateROSBridge2Impl();

    virtual RTC::ReturnCode_t onInitialize();
    virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);

  protected:
    RTC::TimedDoubleSeq m_rsangle;
    RTC::InPort<RTC::TimedDoubleSeq> m_rsangleIn;
    RTC::TimedDoubleSeq m_mcangle;
    RTC::InPort<RTC::TimedDoubleSeq> m_mcangleIn;
    // Actual force/moment ports
    std::vector<RTC::TimedDoubleSeq> m_rsforce;
    std::vector<std::unique_ptr<RTC::InPort<RTC::TimedDoubleSeq>>> m_rsforceIn;
    std::vector<std::string> m_rsforceName;
    // Offset-removed force/moment ports
    std::vector<RTC::TimedDoubleSeq> m_offforce;
    std::vector<std::unique_ptr<RTC::InPort<RTC::TimedDoubleSeq>>> m_offforceIn;
    std::vector<std::string> m_offforceName;
    // Reference force/moment ports
    std::vector<RTC::TimedDoubleSeq> m_mcforce;
    std::vector<std::unique_ptr<RTC::InPort<RTC::TimedDoubleSeq>>> m_mcforceIn;
    std::vector<std::string> m_mcforceName;
    // Gyro and acceleration sensors
    std::vector<RTC::TimedAcceleration3D> m_gsensor;
    std::vector<std::unique_ptr<RTC::InPort<RTC::TimedAcceleration3D>>> m_gsensorIn;
    std::vector<std::string> m_gsensorName;
    std::vector<RTC::TimedAngularVelocity3D> m_gyrometer;
    std::vector<std::unique_ptr<RTC::InPort<RTC::TimedAngularVelocity3D>>> m_gyrometerIn;
    std::vector<std::string> m_gyrometerName;

    // for odom
    RTC::TimedDoubleSeq m_baseTform;
    RTC::InPort<RTC::TimedDoubleSeq> m_baseTformIn;

    // for imu topic
    RTC::TimedOrientation3D m_baseRpy;
    RTC::InPort<RTC::TimedOrientation3D> m_baseRpyIn;

    RTC::TimedDoubleSeq m_rsvel;
    RTC::InPort<RTC::TimedDoubleSeq> m_rsvelIn;
    RTC::TimedDoubleSeq m_rstorque;
    RTC::InPort<RTC::TimedDoubleSeq> m_rstorqueIn;
    OpenHRP::TimedLongSeqSeq m_servoState;
    RTC::InPort<OpenHRP::TimedLongSeqSeq> m_servoStateIn;
    RTC::TimedPoint3D m_rszmp;
    RTC::InPort<RTC::TimedPoint3D> m_rszmpIn;
    RTC::TimedPoint3D m_rsrefCP;
    RTC::InPort<RTC::TimedPoint3D> m_rsrefCPIn;
    RTC::TimedPoint3D m_rsactCP;
    RTC::InPort<RTC::TimedPoint3D> m_rsactCPIn;
    RTC::TimedDoubleSeq m_rsCOPInfo;
    RTC::InPort<RTC::TimedDoubleSeq> m_rsCOPInfoIn;
    RTC::TimedLong m_emergencyMode;
    RTC::InPort<RTC::TimedLong> m_emergencyModeIn;
    RTC::TimedBooleanSeq m_refContactStates, m_actContactStates;
    RTC::InPort<RTC::TimedBooleanSeq> m_refContactStatesIn, m_actContactStatesIn;
    RTC::TimedDoubleSeq m_controlSwingSupportTime;
    RTC::InPort<RTC::TimedDoubleSeq> m_controlSwingSupportTimeIn;

    RTC::TimedDoubleSeq m_mctorque;
    RTC::OutPort<RTC::TimedDoubleSeq> m_mctorqueOut;

    RTC::CorbaPort m_SequencePlayerServicePort;
    RTC::CorbaConsumer<OpenHRP::SequencePlayer2Service> m_service0;

  protected:
    cnoid::BodyPtr body;

    typedef struct {
        std::string link_name, type_name;
        tf::Transform transform;
    } SensorInfo;
    std::map<std::string, SensorInfo> sensor_info;

    typedef struct {
        double cop_offset_z;
        std::string link_name;
    } COPLinkInfo;
    std::map<std::string, COPLinkInfo> cop_link_info;

    double dt;

  private:
};

extern "C" {
DLL_EXPORT void HrpsysSeqStateROSBridge2ImplInit(RTC::Manager *manager);
};

#endif // HRPSYSSEQSTATEROSBRIDGE2IMPL_H

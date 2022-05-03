
//  Copyright (c) 2003-2021 Xsens Technologies B.V. or subsidiaries worldwide.
//  All rights reserved.
//
//  Redistribution and use in source and binary forms, with or without modification,
//  are permitted provided that the following conditions are met:
//
//  1.	Redistributions of source code must retain the above copyright notice,
//  	this list of conditions, and the following disclaimer.
//
//  2.	Redistributions in binary form must reproduce the above copyright notice,
//  	this list of conditions, and the following disclaimer in the documentation
//  	and/or other materials provided with the distribution.
//
//  3.	Neither the names of the copyright holders nor the names of their contributors
//  	may be used to endorse or promote products derived from this software without
//  	specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
//  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
//  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
//  THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
//  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
//  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
//  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.THE LAWS OF THE NETHERLANDS
//  SHALL BE EXCLUSIVELY APPLICABLE AND ANY DISPUTES SHALL BE FINALLY SETTLED UNDER THE RULES
//  OF ARBITRATION OF THE INTERNATIONAL CHAMBER OF COMMERCE IN THE HAGUE BY ONE OR MORE
//  ARBITRATORS APPOINTED IN ACCORDANCE WITH SAID RULES.
//

#include "xdainterface.h"

#include "iostream"
#include "vector"
#include "fstream"
#include "iterator"
#include "string"

#include <xscontroller/xsscanner.h>
#include <xscontroller/xscontrol_def.h>
#include <xscontroller/xsdevice_def.h>

#include "messagepublishers/packetcallback.h"
#include "messagepublishers/accelerationpublisher.h"
#include "messagepublishers/angularvelocitypublisher.h"
#include "messagepublishers/freeaccelerationpublisher.h"
#include "messagepublishers/gnsspublisher.h"
#include "messagepublishers/imupublisher.h"
#include "messagepublishers/magneticfieldpublisher.h"
#include "messagepublishers/orientationincrementspublisher.h"
#include "messagepublishers/orientationpublisher.h"
#include "messagepublishers/pressurepublisher.h"
#include "messagepublishers/temperaturepublisher.h"
#include "messagepublishers/timereferencepublisher.h"
#include "messagepublishers/transformpublisher.h"
#include "messagepublishers/twistpublisher.h"
#include "messagepublishers/velocityincrementpublisher.h"
#include "messagepublishers/positionllapublisher.h"
#include "messagepublishers/velocitypublisher.h"
#include "messagepublishers/eulerpublisher.h"
#include "messagepublishers/rawmagneticfieldpublisher.h"
#include "messagepublishers/rawaccelerationpublisher.h"
#include "messagepublishers/rawangularvelocitypublisher.h"


XdaInterface::XdaInterface()
    : m_device(nullptr), m_pnh("~"), m_nh()
{
    ROS_INFO("Creating XsControl object...");
    m_control = XsControl::construct();
    assert(m_control != 0);
}

XdaInterface::~XdaInterface()
{
    ROS_INFO("Cleaning up ...");
    close();
    m_control->destruct();

    m_set_lla_server.shutdown();
    m_stop_recording_server.shutdown();
    m_data_record_server.shutdown();
}

void XdaInterface::spinFor(std::chrono::milliseconds timeout)
{
    RosXsDataPacket rosPacket = m_xdaCallback.next(timeout);

    if (!rosPacket.second.empty())
    {
        for (auto &cb : m_callbacks)
        {
            cb->operator()(rosPacket.second, rosPacket.first);
        }
    }
}

void XdaInterface::registerPublishers()
{
    bool should_publish;

    if (m_pnh.getParam("pub_imu", should_publish) && should_publish)
    {
        registerCallback(new ImuPublisher(m_nh));
    }
    if (m_pnh.getParam("pub_quaternion", should_publish) && should_publish)
    {
        registerCallback(new OrientationPublisher(m_nh));
    }
    if (m_pnh.getParam("pub_acceleration", should_publish) && should_publish)
    {
        registerCallback(new AccelerationPublisher(m_nh));
    }
    if (m_pnh.getParam("pub_angular_velocity", should_publish) && should_publish)
    {
        registerCallback(new AngularVelocityPublisher(m_nh));
    }
    if (m_pnh.getParam("pub_mag", should_publish) && should_publish)
    {
        registerCallback(new MagneticFieldPublisher(m_nh));
    }
    if (m_pnh.getParam("pub_dq", should_publish) && should_publish)
    {
        registerCallback(new OrientationIncrementsPublisher(m_nh));
    }
    if (m_pnh.getParam("pub_dv", should_publish) && should_publish)
    {
        registerCallback(new VelocityIncrementPublisher(m_nh));
    }
    if (m_pnh.getParam("pub_sampletime", should_publish) && should_publish)
    {
        registerCallback(new TimeReferencePublisher(m_nh));
    }
    if (m_pnh.getParam("pub_temperature", should_publish) && should_publish)
    {
        registerCallback(new TemperaturePublisher(m_nh));
    }
    if (m_pnh.getParam("pub_pressure", should_publish) && should_publish)
    {
        registerCallback(new PressurePublisher(m_nh));
    }
    if (m_pnh.getParam("pub_gnss", should_publish) && should_publish)
    {
        registerCallback(new GnssPublisher(m_nh));
    }
    if (m_pnh.getParam("pub_twist", should_publish) && should_publish)
    {
        registerCallback(new TwistPublisher(m_nh));
    }
    if (m_pnh.getParam("pub_free_acceleration", should_publish) && should_publish)
    {
        registerCallback(new FreeAccelerationPublisher(m_nh));
    }
    if (m_pnh.getParam("pub_transform", should_publish) && should_publish)
    {
        registerCallback(new TransformPublisher(m_nh));
    }
    if (m_pnh.getParam("pub_positionLLA", should_publish) && should_publish)
    {
        registerCallback(new PositionLLAPublisher(m_nh));
    }
    if (m_pnh.getParam("pub_velocity", should_publish) && should_publish)
    {
        registerCallback(new VelocityPublisher(m_nh));
    }
    if (m_pnh.getParam("pub_euler", should_publish) && should_publish)
    {
        registerCallback(new EulerPublisher(m_nh));
    }
    if (m_pnh.getParam("pub_raw_mag", should_publish) && should_publish)
    {
        registerCallback(new RawMagneticFieldPublisher(m_nh));
    }
    if (m_pnh.getParam("pub_raw_accel", should_publish) && should_publish)
    {
        registerCallback(new RawAccelerationPublisher(m_nh));
    }
    if (m_pnh.getParam("pub_raw_gyro", should_publish) && should_publish)
    {
        registerCallback(new RawAngularVelocityPublisher(m_nh));
    }
}

void XdaInterface::registerServices() {

    m_data_record_server = m_pnh.advertiseService(
        "data_record", &XdaInterface::dataRecordCallack, this);

    m_stop_recording_server = m_pnh.advertiseService(
        "stop_recoding", &XdaInterface::stopRecordingCallback, this);

    m_set_lla_server = m_pnh.advertiseService(
        "set_lla", &XdaInterface::setLlaCallback, this);

    m_calibrate_server = m_pnh.advertiseService(
        "apply_calibration", &XdaInterface::sendRawCallback, this
    );

}

bool XdaInterface::connectDevice()
{
    // Read baudrate parameter if set
    XsBaudRate baudrate = XBR_Invalid;
    if (m_pnh.hasParam("baudrate"))
    {
        int baudrateParam = 0;
        m_pnh.getParam("baudrate", baudrateParam);
        ROS_INFO("Found baudrate parameter: %d", baudrateParam);
        baudrate = XsBaud::numericToRate(baudrateParam);
    }
    // Read device ID parameter
    bool checkDeviceID = false;
    std::string deviceId;
    if (m_pnh.hasParam("device_id"))
    {
        m_pnh.getParam("device_id", deviceId);
        checkDeviceID = true;
        ROS_INFO("Found device ID parameter: %s.",deviceId.c_str());

    }
    // Read port parameter if set
    XsPortInfo mtPort;
    if (m_pnh.hasParam("port"))
    {
        std::string portName;
        m_pnh.getParam("port", portName);
        ROS_INFO("Found port name parameter: %s", portName.c_str());
        mtPort = XsPortInfo(portName, baudrate);
        ROS_INFO("Scanning port %s ...", portName.c_str());
        if (!XsScanner::scanPort(mtPort, baudrate))
            return handleError("No MTi device found. Verify port and baudrate.");
        if (checkDeviceID && mtPort.deviceId().toString().c_str() != deviceId)
            return handleError("No MTi device found with matching device ID.");

    }
    else
    {
        ROS_INFO("Scanning for devices...");
        XsPortInfoArray portInfoArray = XsScanner::scanPorts(baudrate);

        for (auto const &portInfo : portInfoArray)
        {
            if (portInfo.deviceId().isMti() || portInfo.deviceId().isMtig())
            {
                if (checkDeviceID)
                {
                    if (portInfo.deviceId().toString().c_str() == deviceId)
                    {
                        mtPort = portInfo;
                        break;
                    }
                }
                else
                {
                    mtPort = portInfo;
                    break;
                }
            }
        }
    }

    if (mtPort.empty())
        return handleError("No MTi device found.");

    ROS_INFO("Found a device with ID: %s @ port: %s, baudrate: %d", mtPort.deviceId().toString().toStdString().c_str(), mtPort.portName().toStdString().c_str(), XsBaud::rateToNumeric(mtPort.baudrate()));

    ROS_INFO("Opening port %s ...", mtPort.portName().toStdString().c_str());
    if (!m_control->openPort(mtPort))
        return handleError("Could not open port");

    m_device = m_control->device(mtPort.deviceId());
    assert(m_device != 0);

    ROS_INFO("Device: %s, with ID: %s opened.", m_device->productCode().toStdString().c_str(), m_device->deviceId().toString().c_str());

    m_device->addCallbackHandler(&m_xdaCallback);

    return true;
}

bool XdaInterface::prepare()
{
    assert(m_device != 0);

    if (!m_device->gotoConfig())
        return handleError("Could not go to config");

    // read EMTS and device config stored in .mtb file header.
    if (!m_device->readEmtsAndDeviceConfiguration())
        return handleError("Could not read device configuration");

    XsOutputConfigurationArray configArray;

    m_device->deviceConfiguration();


    int value;

    m_pnh.param<int>("quaternion_freq", value, 100);
    configArray.push_back(XsOutputConfiguration(XDI_Quaternion, value));

    m_pnh.param<int>("magnetic_field_freq", value, 20);
    configArray.push_back(XsOutputConfiguration(XDI_MagneticField, value));

    m_pnh.param<int>("acceleration_freq", value, 100);
    configArray.push_back(XsOutputConfiguration(XDI_Acceleration, value));

    m_pnh.param<int>("rate_of_turn_freq", value, 100);
    configArray.push_back(XsOutputConfiguration(XDI_RateOfTurn, value));

    m_pnh.param<int>("euler_angles_freq", value, 100);
    configArray.push_back(XsOutputConfiguration(XDI_EulerAngles, value));

    if(m_pnh.getParam("raw_data_freq", value)) {
        configArray.push_back(XsOutputConfiguration(XDI_RawAccGyrMagTemp, value));
    }

    ROS_INFO("Configuring ...");
    if(!m_device->setOutputConfiguration(configArray)) {
        return handleError("Could not configure");
    }


    std::string filter_profile;
    m_pnh.param<std::string>("onboard_filter_profile", filter_profile, "Robust/NorthReference");
    m_device->setOnboardFilterProfile(filter_profile);

    bool icc;
    m_pnh.param<bool>("inrun_compass_calibration", icc, true);
    if(icc) {
        m_device->setDeviceOptionFlags(XDOF_EnableInrunCompassCalibration,
                                       XDOF_None);
    }

    ROS_INFO("Measuring ...");
    if (!m_device->gotoMeasurement())
        return handleError("Could not put device into measurement mode");

    std::string logFile;
    if (m_pnh.getParam("log_file", logFile))
    {
        if (m_device->createLogFile(logFile) != XRV_OK)
            return handleError("Failed to create a log file! (" + logFile + ")");
        else
            ROS_INFO("Created a log file: %s", logFile.c_str());

        ROS_INFO("Recording to %s ...", logFile.c_str());
        if (!m_device->startRecording())
            return handleError("Could not start recording");
    }

    return true;
}

void XdaInterface::close()
{
    if (m_device != nullptr)
    {
        m_device->stopRecording();
        m_device->closeLogFile();
        m_device->removeCallbackHandler(&m_xdaCallback);
    }
    m_control->closePort(m_port);
}

void XdaInterface::registerCallback(PacketCallback *cb)
{
    m_callbacks.push_back(cb);
}

bool XdaInterface::handleError(std::string error)
{
    ROS_ERROR("%s", error.c_str());
    close();
    return false;
}

bool XdaInterface::setLlaCallback(xsens_mti_driver::SetLLA::Request &req,
                                  xsens_mti_driver::SetLLA::Response &resp) {

    m_device->gotoConfig();

    XsVector3 f;
    f[0] = req.latitude;
    f[1] = req.longitude;
    f[2] = req.altitude;

    m_device->setInitialPositionLLA(f);

    m_device->gotoMeasurement();

    return true;
}

bool XdaInterface::dataRecordCallack(
    xsens_mti_driver::DataRecord::Request &req,
    xsens_mti_driver::DataRecord::Response &resp) {

    if(req.data_file.empty()) {
        ROS_WARN("data record request received with empty file name!");
        return false;
    }

    if (m_device->createLogFile(req.data_file) != XRV_OK) {
        return handleError(
            "Failed to create a log file! (" + req.data_file + ")");
    } else {
        ROS_INFO("Created a log file: %s", req.data_file.c_str());
    }

    ROS_INFO("Recording to %s ...", req.data_file.c_str());
    if (!m_device->startRecording()) {
        return handleError("Could not start recording");
    }

    return true;
}

bool XdaInterface::stopRecordingCallback(std_srvs::Trigger::Request &req,
                                         std_srvs::Trigger::Response &resp) {

    if (!m_device->stopRecording()) {
        return handleError("Could not stop recording");
    }

    if(!m_device->closeLogFile()) {
        return handleError(
            "Failed to close the log file!");
    } else {
        ROS_INFO("Recording has stopped");
    }

    resp.success = true;

    return true;
}

bool XdaInterface::sendRawCallback(xsens_mti_driver::SendCalibration::Request& req,
                                   xsens_mti_driver::SendCalibration::Response& resp) {

    XsMessage msg;

    // open the file:
    std::ifstream file(req.calibration_file, std::ios::binary);

    file.unsetf(std::ios::skipws);

    std::streampos fileSize;

    file.seekg(0, std::ios::end);
    fileSize = file.tellg();
    file.seekg(0, std::ios::beg);

    // reserve capacity
    std::vector<uint8_t> vec;
    vec.reserve(fileSize);

    // read the data:
    vec.insert(vec.begin(),
               std::istream_iterator<uint8_t>(file),
               std::istream_iterator<uint8_t>());

    for(int i = 0 ; i < vec.size() ; i++) {
        if(i % 16 == 0) {
            ROS_INFO("\n");
        }
        ROS_INFO("%02x ", vec[i]);
    }   ROS_INFO("\n");

    msg.loadFromString(vec.data(), vec.size());

    if(!m_device->gotoConfig()) {
        return handleError("Can't goto config mode");
    }


    if(!m_device->sendRawMessage(msg)) {
        return handleError("Can't send the message");
    }

    if(!m_device->gotoMeasurement()) {
        m_device->reset();
        return handleError("Can't goto measurement mode");
    }


    return true;
}
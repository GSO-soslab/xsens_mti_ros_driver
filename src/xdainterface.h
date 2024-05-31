
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

#ifndef XDAINTERFACE_H
#define XDAINTERFACE_H

#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"

#include "xsens_mti_driver/SetLLA.h"
#include "xsens_mti_driver/DataRecord.h"
#include "xsens_mti_driver/SendCalibration.h"

#include "std_srvs/Trigger.h"

#include "xdacallback.h"
#include "xstypes/xsportinfo.h"

#include "chrono"

struct XsControl;
struct XsDevice;


class PacketCallback;

class XdaInterface
{
public:
    XdaInterface();
    ~XdaInterface();

    void spinFor(std::chrono::milliseconds timeout);
    void registerPublishers();
    void registerSubscribers();
    void registerServices();

    bool connectDevice();
    bool prepare();
    void close();


private:
    void registerCallback(PacketCallback *cb);
    bool handleError(std::string error);

    ros::NodeHandle m_pnh;

    ros::NodeHandle m_nh;

    XsControl *m_control;
    XsDevice *m_device;
    XsPortInfo m_port;
    XdaCallback m_xdaCallback;
    std::list<PacketCallback *> m_callbacks;

    ros::ServiceServer m_set_lla_server;

    ros::ServiceServer m_data_record_server;

    ros::ServiceServer m_stop_recording_server;

    ros::ServiceServer m_calibrate_server;
    
    ros::Subscriber m_gps_fix_sub;

    bool use_fix_topic_to_set_lla;

    bool setLlaCallback(xsens_mti_driver::SetLLA::Request& req,
                        xsens_mti_driver::SetLLA::Response& resp);

    bool dataRecordCallack(xsens_mti_driver::DataRecord::Request& req,
                           xsens_mti_driver::DataRecord::Response& resp);

    bool stopRecordingCallback(std_srvs::Trigger::Request &req,
                               std_srvs::Trigger::Response& resp);

    bool sendRawCallback(xsens_mti_driver::SendCalibration::Request& req,
                         xsens_mti_driver::SendCalibration::Response& resp);

    void gps_callback(const sensor_msgs::NavSatFix &msg);
};

#endif

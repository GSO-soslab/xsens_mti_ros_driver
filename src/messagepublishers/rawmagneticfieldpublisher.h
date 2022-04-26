
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

#ifndef RAWMAGNETICFIELDPUBLISHER_H
#define RAWMAGNETICFIELDPUBLISHER_H

#include "packetcallback.h"
#include "sensor_msgs/MagneticField.h"

struct RawMagneticFieldPublisher : public PacketCallback
{
    ros::Publisher pub;
    std::string frame_id = DEFAULT_FRAME_ID;

    double magnetic_field_variance[3];

    static void variance_from_stddev_param(std::string param, double *variance_out)
    {
        std::vector<double> stddev;
        if (ros::param::get(param, stddev))
        {
            if (stddev.size() == 3)
            {
                auto squared = [](double x) { return x * x; };
                std::transform(stddev.begin(), stddev.end(), variance_out, squared);
            }
            else
            {
                ROS_WARN("Wrong size of param: %s, must be of size 3", param.c_str());
            }
        }
        else
        {
            memset(variance_out, 0, 3 * sizeof(double));
        }
    }

    RawMagneticFieldPublisher(ros::NodeHandle &node)
    {
        int pub_queue_size = 5;
        ros::param::get("~publisher_queue_size", pub_queue_size);
        pub = node.advertise<sensor_msgs::MagneticField>("imu/raw/mag", pub_queue_size);
        variance_from_stddev_param("~magnetic_field_stddev", magnetic_field_variance);
        ros::param::get("~frame_id", frame_id);
    }

    void operator()(const XsDataPacket &packet, ros::Time timestamp)
    {
        if (packet.containsRawMagneticField())
        {
            // TODO: Use sensor_msgs::MagneticField
            // Problem: Sensor gives normalized magnetic field vector with unknown units
            sensor_msgs::MagneticField msg;

            msg.header.stamp = timestamp;
            msg.header.frame_id = frame_id;

            XsVector mag = packet.rawMagneticFieldConverted();

            msg.magnetic_field.x = mag[0];
            msg.magnetic_field.y = mag[1];
            msg.magnetic_field.z = mag[2];

            msg.magnetic_field_covariance[0] = magnetic_field_variance[0];
            msg.magnetic_field_covariance[4] = magnetic_field_variance[1];
            msg.magnetic_field_covariance[8] = magnetic_field_variance[2];

            pub.publish(msg);
        }
    }
};

#endif

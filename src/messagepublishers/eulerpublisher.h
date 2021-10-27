// author: Emir Cem Gezer, emircem@uri.edu

#ifndef EULERPUBLISHER_H
#define EULERPUBLISHER_H

#include "packetcallback.h"
#include <geometry_msgs/Vector3Stamped.h>

struct EulerPublisher : public PacketCallback
{
    ros::Publisher pub_degrees, pub_radians;
    std::string frame_id = DEFAULT_FRAME_ID;

    EulerPublisher(ros::NodeHandle &node)
    {
        int pub_queue_size = 5;
        ros::param::get("~publisher_queue_size", pub_queue_size);
        pub_degrees = node.advertise<geometry_msgs::Vector3Stamped>("imu/euler/degrees", pub_queue_size);
        pub_radians = node.advertise<geometry_msgs::Vector3Stamped>("imu/euler/radians", pub_queue_size);
        ros::param::get("~frame_id", frame_id);
    }

    void operator()(const XsDataPacket &packet, ros::Time timestamp)
    {
        if (packet.containsOrientation())
        {
            geometry_msgs::Vector3Stamped msg;

            msg.header.stamp = timestamp;
            msg.header.frame_id = frame_id;

            XsEuler euler = packet.orientationEuler();

            msg.vector.x = euler.x();
            msg.vector.y = euler.y();
            msg.vector.z = euler.z();

            pub_degrees.publish(msg);

            msg.vector.x = euler.x() * M_PI / 180;
            msg.vector.y = euler.y() * M_PI / 180;
            msg.vector.z = euler.z() * M_PI / 180;

            pub_radians.publish(msg);
        }
    }
};

#endif

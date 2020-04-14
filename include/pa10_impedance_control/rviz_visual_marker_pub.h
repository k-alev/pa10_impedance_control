#pragma once

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <fcl2rct/utils_fcl.h>

#include <ros/package.h>


#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <fcl2rct/utils_fcl.h>

#include <ros/package.h>


class VisualMarkerPub
{
public:
    VisualMarkerPub(ros::NodeHandle n)
    {
        marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
        line_list.ns = "xml_map";
        line_list.action = visualization_msgs::Marker::ADD;

        line_list.id = 2;

        line_list.type = visualization_msgs::Marker::LINE_LIST;

        line_list.scale.x = 0.01;

        line_list.color.r = 1.0;
        line_list.color.a = 1.0;
    }
    void visualMarkerBroadcaster(std::vector<std::vector<fcl::DistanceData>>& data, const std::string &frame_name)
    {
        line_list.header.frame_id = frame_name;

        for (int i = 0; i < data.size(); ++i)
        {
            for (std::vector<fcl::DistanceData>::iterator it = data[i].begin(); it != data[i].end(); ++it)
            {   
                p1.x = (it)->distVctWF.second.data[0];
                p1.y = (it)->distVctWF.second.data[1];
                p1.z = (it)->distVctWF.second.data[2];

                p2.x = (it)->distVctWF.first.data[0] + (it)->distVctWF.second.data[0];
                p2.y = (it)->distVctWF.first.data[1] + (it)->distVctWF.second.data[1];
                p2.z = (it)->distVctWF.first.data[2] + (it)->distVctWF.second.data[2];

                line_list.points.push_back(p1);
                line_list.points.push_back(p2);
            }
        }

        marker_pub.publish(line_list);
        line_list.points.clear();
    }
    private:
    ros::Publisher marker_pub;
    visualization_msgs::Marker line_list;
    geometry_msgs::Point p1, p2;
};
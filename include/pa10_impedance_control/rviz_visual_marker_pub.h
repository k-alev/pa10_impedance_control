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
        points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
        points.ns = line_strip.ns = line_list.ns = "xml_map";
        points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
        points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;

        points.id = 0;
        line_strip.id = 1;
        line_list.id = 2;

        points.type = visualization_msgs::Marker::POINTS;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        line_list.type = visualization_msgs::Marker::LINE_LIST;

        // POINTS markers use x and y scale for width/height respectively
        points.scale.x = 0.2;
        points.scale.y = 0.2;

        // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
        line_strip.scale.x = 0.1;
        line_list.scale.x = 0.01;

        // Points are green
        points.color.g = 1.0f;
        points.color.a = 1.0;

        // Line strip is blue
        line_strip.color.b = 1.0;
        line_strip.color.a = 1.0;

        // Line list is red
        line_list.color.r = 1.0;
        line_list.color.a = 1.0;
    }
    void visualMarkerBroadcaster(std::vector<std::vector<fcl::DistanceData>>& data, const std::string &frame_name)
    {
        points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = frame_name;

        for (int i = 0; i < data.size(); ++i)
        {
            for (std::vector<fcl::DistanceData>::iterator it = data[i].begin(); it != data[i].end(); ++it)
            {
                // std::cout << "other_mngr" << i << ": " << (it)->result.min_distance << " == " << (it)->distVctWF.first.length() << std::endl;
                // std::cout << "other_mngr_1pts" << i << ": " << (it)->result.nearest_points[0] << std::endl;
                // std::cout << "other_mngr_2pts" << i << ": " << (it)->result.nearest_points[1] << std::endl;
                // std::cout << "first vector" << i << ": " << (it)->distVctWF.second << std::endl;
                
                p1.x = (it)->distVctWF.second.data[0];
                p1.y = (it)->distVctWF.second.data[1];
                p1.z = (it)->distVctWF.second.data[2];

                p2.x = (it)->distVctWF.first.data[0] + (it)->distVctWF.second.data[0];
                p2.y = (it)->distVctWF.first.data[1] + (it)->distVctWF.second.data[1];
                p2.z = (it)->distVctWF.first.data[2] + (it)->distVctWF.second.data[2];

                points.points.push_back(p1);
                line_strip.points.push_back(p1);

                // The line list needs two points for each line
                line_list.points.push_back(p1);
                // p.z += 1.0;
                line_list.points.push_back(p2);
            }
        }
        // marker_pub.publish(points);
        // marker_pub.publish(line_strip);
        marker_pub.publish(line_list);
        line_list.points.clear();
    }
    private:
    ros::Publisher marker_pub;
    visualization_msgs::Marker points, line_strip, line_list;
    geometry_msgs::Point p1, p2;
};
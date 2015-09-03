/*********************************************************************************************//**
* @file filter.h
*
* Image filter for sharping edges and removing reflection
*
* Copyright (c)
* Jan Bacik
* Smart Robotic Systems
* www.smartroboticsys.eu
* March 2015
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
*
* Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/
/*
 * Copyright (C) September 2015 (Raul Acuna)
 *
 * - Heavily changed the original code for using it with camera calibration file
 *   and directly with the ROS package ar_sys
*/

#ifndef IMG_FILTER_NODE_H
#define IMG_FILTER_NODE_H

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <opencv2/opencv.hpp>

class ImgFilterNode
{
public:
    ImgFilterNode();
    ~ImgFilterNode();

    ros::NodeHandle nh;
    image_transport::ImageTransport image_transport;

    void ImageCallback(const sensor_msgs::ImageConstPtr &input_image);
    void CameraInfoCallback(const sensor_msgs::CameraInfo &cam_info);

private:   
    ros::Publisher cam_info_pub;
    image_transport::CameraPublisher pub_filtered;

    ros::Subscriber cam_info_sub;
    image_transport::Subscriber subVideo;

    sensor_msgs::CameraInfo cam_info_msg;
    camera_info_manager::CameraInfoManager *cinfo;

    int m_treshold;

    cv_bridge::CvImagePtr m_cv_ptr;
    cv::Mat m_I;
    cv::Mat m_I_filtered;
};

#endif // IMG_FILTER_NODE

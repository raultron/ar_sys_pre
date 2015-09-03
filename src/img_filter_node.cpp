/*********************************************************************************************//**
* @file filter.cpp
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

#include "ar_sys_prep/img_filter_node.h"

ImgFilterNode::ImgFilterNode(): image_transport(nh)
{
    subVideo = image_transport.subscribe("/ardrone/bottom/image_raw",1,&ImgFilterNode::ImageCallback,this);
    cam_info_sub = nh.subscribe("/ardrone/bottom/camera_info", 1, &ImgFilterNode::CameraInfoCallback, this);

    pub_filtered = image_transport.advertiseCamera("/ardrone/bottom/filtered/image_raw",1);

    // Limiting value of threshold
    int treshold = 210;
    nh.getParam("treshold",treshold);

    if(treshold > 255)
        m_treshold = 0;
    else
        m_treshold = treshold;
}


ImgFilterNode::~ImgFilterNode()
{
}


void ImgFilterNode::ImageCallback(const sensor_msgs::ImageConstPtr &input_image)
{
    m_cv_ptr = cv_bridge::toCvCopy(input_image, sensor_msgs::image_encodings::RGB8);

    // OpenCV to MAT structure
    m_I = m_cv_ptr->image;

    // Gaussian Blur
    cv::GaussianBlur(m_I, m_I_filtered, cv::Size(3,3), 0, 0, cv::BORDER_DEFAULT);

    // Weights
    //cv::addWeighted(I, 2.5, I_filtered, -1.5, 0, I_filtered);

    /// Convert the image to grayscale
    cv::cvtColor( m_I_filtered, m_I_filtered, CV_RGB2GRAY );

    // Equalize histogram
    cv::equalizeHist(m_I_filtered,m_I_filtered);

    // Treshold    
    cv::threshold(m_I_filtered,m_I_filtered,m_treshold,0,3);

    cv::cvtColor(m_I_filtered,m_I_filtered,CV_GRAY2RGB);

    // Creating new filtered image_raw and publishing
    cv_bridge::CvImagePtr in_msg;
    in_msg = m_cv_ptr;
    cv_bridge::CvImage out_msg;

    // Same timestamp and tf frame as input image
    out_msg.header = in_msg->header;

    // Format
    out_msg.encoding = sensor_msgs::image_encodings::RGB8;
    out_msg.image = m_I_filtered;

    // To publish same camera calibration information as input image
    cam_info_msg.header.stamp = out_msg.header.stamp;
    cam_info_msg.header.frame_id = "/ardrone/bottom/filtered";

    // Publish filtered image
    pub_filtered.publish(*out_msg.toImageMsg(),cam_info_msg);
}

void ImgFilterNode::CameraInfoCallback(const sensor_msgs::CameraInfo &cam_info){
    // Receive input image calibration information
    cam_info_msg = cam_info;
}

#include <iostream>
#include <stdio.h>
#include <string.h>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "ros/ros.h"
#include "std_msgs/UInt16MultiArray.h"
#include "std_msgs/MultiArrayDimension.h"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "sensor_msgs/image_encodings.h"

#include "LeptonIntercept.h"


#define SCALED_IMAGE
#define RESIZE


void scaleImageU8(uint16_t*, uint8_t*, int, int);

int main(int argc, char **argv)
{       
    ros::init(argc, argv, "lepton_capture");
    
    ros::NodeHandle n;
    image_transport::ImageTransport it_(n);
    image_transport::Publisher image_pub = it_.advertise("/lepton_output", 10);
    
    cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
    cv_ptr -> encoding = "mono16";
    
#ifdef SCALED_IMAGE
    image_transport::ImageTransport it2_(n);
    image_transport::Publisher image_scaled_pub = it2_.advertise("/lepton_scaled", 10);
    
    cv_bridge::CvImagePtr cv_ptr_scaled(new cv_bridge::CvImage);
    cv_ptr_scaled -> encoding = "mono8";
#endif

	
	LeptonIntercept cameraLepton = LeptonIntercept();
    cameraLepton.setSpiSpeedMHz(20);  
    cameraLepton.connect();
    if( !cameraLepton.isConnected() )
        ROS_INFO("[Error] Failed connect SPI");
    
    int image_columns = cameraLepton.getImageColumns();
    int image_rows    = cameraLepton.getImageRows();
    uint16_t *image_pointer; //here can be use cameraLepton.getImagePointer();
    
    cv::Mat raw_image(image_rows, image_columns, CV_16U);
    
#ifdef SCALED_IMAGE
    uint8_t scaled_image_pointer[image_columns*image_rows];
    cv::Mat scaled_image(image_rows, image_columns, CV_8U);
    #ifdef RESIZE
    cv::Mat resized_scaled_image;
    #endif
    
#endif
    
    ros::Rate loop_rate(8);
    
    while (ros::ok())
    {
        
		//its bad name function (refresh stored vector in LeptonIntercept)
		cameraLepton.getFrame();
        image_pointer = cameraLepton.getImagePointer();

        //copy to cv::Mat raw_image
        std::memcpy(raw_image.data, image_pointer, image_rows*image_columns*sizeof(uint16_t));
        
        cv_ptr -> header.stamp = ros::Time::now();
        cv_ptr -> header.frame_id = "/lepton_output";
        cv_ptr -> image = raw_image;
        
        image_pub.publish(cv_ptr->toImageMsg());            
        ROS_INFO("Raw image Send!");	
        
#ifdef SCALED_IMAGE

        scaleImageU8(image_pointer, scaled_image_pointer, image_columns, image_rows);
        std::memcpy(scaled_image.data, scaled_image_pointer, image_rows*image_columns*sizeof(uint8_t));
        
    #ifdef RESIZE
        cv::resize(scaled_image, resized_scaled_image, cv::Size(scaled_image.cols*5, scaled_image.rows*5));
    #endif
        
        cv_ptr_scaled -> header.stamp = ros::Time::now();
        cv_ptr_scaled -> header.frame_id = "/lepton_scaled";
    #ifdef RESIZE
        cv_ptr_scaled -> image = resized_scaled_image;
    #else
        cv_ptr_scaled -> image = scaled_image;
    #endif

        image_scaled_pub.publish(cv_ptr_scaled->toImageMsg());            
        ROS_INFO("Scaled image Send!");
        
#endif

  
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

#ifdef SCALED_IMAGE
void scaleImageU8(uint16_t* imgU16, uint8_t* imgU8, int column, int row){
    int min = imgU16[0];
    int max = imgU16[0];
    
    for(int i=1; i<column*row; ++i){
        if(imgU16[i] < min)
            min = imgU16[i];
        if(imgU16[i] > max)
            max = imgU16[i];
    }
    
    float diff = max - min;
    
    for(int i=0; i<column*row; ++i){
        imgU8[i] = uint8_t(((imgU16[i]-min)/diff)*255);
    }
}
#endif

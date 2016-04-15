#pragma once


//#include <stdafx.h>
#ifdef LINUX
#include <unistd.h>
#endif

#include "/usr/include/flycapture/FlyCapture2.h"


#include <iostream>
#include <unistd.h>
#include <sstream>

#define _POSIX_C_SOURCE 200809L

#include <sys/time.h>
#include <math.h>
#include <stdio.h>
#include <time.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
//#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/image_encodings.h>

#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>



#include "cam_grid/multi_cam.hpp"

#include "INIReader.h"
// Software trigger the camera instead of using an external hardware trigger
//
//#define SOFTWARE_TRIGGER_CAMERA

using namespace FlyCapture2;
using namespace std;

	
int main(int argc, char** argv)
{
	int i;
	char trigger[] = "External";  // Software or External
	char pixelFormat[] = "BGR";   //BGR, RGB8, RGB16, RAW8, MONO8, MONO16, 444YUV8
	//int *numberCamera;
	
	INIReader reader("/home/rrc/catkin_ws/src/cam_grid/test.ini");

	string config = reader.Get("rig", "config", "");
	std::string delimiter = ",";

	size_t pos = 0;
	std::vector<std::string> token;
	while ((pos = config.find(delimiter)) != std::string::npos) {
		token.push_back(config.substr(0, pos));
		config.erase(0, pos + delimiter.length());
	}
	token.push_back(config);
	
	string master = reader.Get("rig", "master", "left");
	int master_id = 0;
	
	int ids[token.size()];
	for(int i =0; i < token.size(); i++){
		ids[i] = reader.GetInteger("rig", token[i], 0);
		if(!token[i].compare(master))
			master_id = i;
	}
	
	int W = reader.GetInteger("props", "width", 640);
	int H = reader.GetInteger("props", "height", 640);
	int oW = reader.GetInteger("props", "offset_width", 0);
	int oH = reader.GetInteger("props", "offset_height", 0);

	
	MultiCam grid1;

	if(grid1.instantiateRig(ids, token.size(), master_id))
		return 0;
	if(grid1.triggerRig(trigger))
		return 0;
	if(grid1.imagePropRig(pixelFormat, W, H, oW, oH)){
		cout<<"Format & config error";
		return 0;
	}
	if(grid1.startRigCapture(trigger))
		return 0;

	ros::init(argc, argv, "image_publisher");
    
    ros::NodeHandle nh;
    
    std::vector<int> cams;
    nh.getParam("cams",cams);

	std::vector<std::string> camera_info_url;
	nh.getParam("camera_info_url",camera_info_url);
    
	image_transport::ImageTransport it(nh);
	image_transport::CameraPublisher* pub;
    
	camera_info_manager::CameraInfoManager **cinfo;
    
    for(int i = 0; i < grid1.numCameras; i++)
    {
		std::stringstream ss;
		ss<<"/rrc_camera/"<<token[i]<<"/image_raw";
		*(pub+i) = it.advertiseCamera(ss.str(), 1);
		ss.str("");
		ss<<i;
		*(cinfo+i) = new camera_info_manager::CameraInfoManager(nh, ss.str(), camera_info_url[i]);
	}
	
	int update_counter = 0;
	while(nh.ok())
	{
		if(update_counter>10){
			grid1.updateProps(master_id);
			update_counter = 0;
		}
		update_counter++;
		for(int i = 0; i < grid1.numCameras; i++)
		{

			Image image;
			
			grid1.imageGrab(trigger, pixelFormat, image, i);
					
			std::string imageEncoding = sensor_msgs::image_encodings::MONO8;
			BayerTileFormat bayer_format = image.GetBayerTileFormat();
			 switch(bayer_format)
			        {
				        case RGGB:
							imageEncoding = sensor_msgs::image_encodings::BAYER_RGGB8;
							break;
				        case GRBG:
							imageEncoding = sensor_msgs::image_encodings::BAYER_GRBG8;
							break;
				        case GBRG:
							imageEncoding = sensor_msgs::image_encodings::BAYER_GBRG8;
							break;
				        case BGGR:
							imageEncoding = sensor_msgs::image_encodings::BAYER_BGGR8;
							break;
						default:
							imageEncoding = sensor_msgs::image_encodings::MONO8;
			}
			ros::Time begin = ros::Time::now();
			TimeStamp embeddedTime = image.GetTimeStamp();
			sensor_msgs::Image oimage;
			oimage.header.stamp.sec = begin.toSec();
			oimage.header.stamp.nsec = 1000 * begin.toSec() * 1000 *1000;
			fillImage(oimage, imageEncoding, image.GetRows(), image.GetCols(), image.GetStride(), image.GetData());
			
			sensor_msgs::CameraInfoPtr ci(new sensor_msgs::CameraInfo(cinfo[i]->getCameraInfo()));
			ci->header.frame_id = oimage.header.frame_id;
			ci->header.stamp = oimage.header.stamp;
			pub[i].publish(oimage,*ci);
		}
	}
	return 1;

}

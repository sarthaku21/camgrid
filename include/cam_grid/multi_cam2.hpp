//=============================================================================
// Copyright © 2008 Point Grey Research, Inc. All Rights Reserved.
//
// This software is the confidential and proprietary information of Point
// Grey Research, Inc. ("Confidential Information").  You shall not
// disclose such Confidential Information and shall use it only in
// accordance with the terms of the license agreement you entered into
// with PGR.
//
// PGR MAKES NO REPRESENTATIONS OR WARRANTIES ABOUT THE SUITABILITY OF THE
// SOFTWARE, EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
// PURPOSE, OR NON-INFRINGEMENT. PGR SHALL NOT BE LIABLE FOR ANY DAMAGES
// SUFFERED BY LICENSEE AS A RESULT OF USING, MODIFYING OR DISTRIBUTING
// THIS SOFTWARE OR ITS DERIVATIVES.
//=============================================================================
//=============================================================================
// $Id: AsyncTriggerEx.cpp,v 1.21 2010-07-22 22:51:51 soowei Exp $
//=============================================================================

//#include <stdafx.h>
#ifdef LINUX
#include <unistd.h>
#endif

#include "/usr/include/flycapture/FlyCapture2.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

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
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/image_encodings.h>

#include <sensor_msgs/PointCloud.h>
#include <stereo_image_proc/DisparityConfig.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_geometry/pinhole_camera_model.h>

// Software trigger the camera instead of using an external hardware trigger
//
//#define SOFTWARE_TRIGGER_CAMERA

using namespace FlyCapture2;
using namespace std;
using namespace cv;


class MultiCam{

	public:
	long long int x;
	long long int y;
	long long int counter;
	double max_frame_rate;
	double min_frame_rate;
	double avg_frame_rate;
	unsigned int numCameras;
	Error error;
	TriggerMode triggerMode;													//
	BusManager busMgr;
	cv::Mat** frame;


	Camera ** ppCameras;
	CameraInfo tempcamInfo;
	int camera_found;
	int camera_test;
	int *cam_ID;
	int *cam_Order;
	int argc;
	char **argv;
	//int flag = 0;
	Image image, image1, image2;
	//int j = 1;
	//sensor_msgs::ImagePtr msg1, msg2;
	//char path[500] = "/home/rrc/image_transport_ws/src/stereo_camera/src/images1/";
	Property prop;
	float k_shutterVal;
	//Error error;
		
	MultiCam()
	{
		x=0;
		y=0;
		counter = 0;
		max_frame_rate = 0.0;
		min_frame_rate = 100.0;
		avg_frame_rate = 0.0;
		counter = 0;
		max_frame_rate = 0.0;
		min_frame_rate = 100.0;
		avg_frame_rate = 0.0;
		camera_found = 0;
		camera_test = 0;
	}


	void current_timestamp()
	{
		counter++;
		struct timeval te; 
		gettimeofday(&te, NULL); // get current time
		long long milliseconds = te.tv_sec*1000LL + te.tv_usec/1000; // caculate milliseconds
		y=milliseconds;
		double frame_rate = 1.0/(0.001*(y-x));
		if(frame_rate > max_frame_rate) max_frame_rate = frame_rate;
		avg_frame_rate = (avg_frame_rate * (counter-1) + frame_rate)/counter;
		printf("milliseconds: %.3lf, max : %.3lf, avg: %.3lf\n", frame_rate, max_frame_rate, avg_frame_rate);
		x=y;
		//return milliseconds;
	}
	//

	void PrintBuildInfo()
	{
		FC2Version fc2Version;
		Utilities::GetLibraryVersion( &fc2Version );

		ostringstream version;
		version << "FlyCapture2 library version: " << fc2Version.major << "." << fc2Version.minor << "." << fc2Version.type << "." << fc2Version.build;
		cout << version.str() <<endl;  

		ostringstream timeStamp;
		timeStamp << "Application build date: " << __DATE__ << " " << __TIME__;
		cout << timeStamp.str() << endl << endl;  
	}

	void PrintCameraInfo( CameraInfo* pCamInfo )
	{
		cout << endl;
		cout << "*** CAMERA INFORMATION ***" << endl;
		cout << "Serial number -" << pCamInfo->serialNumber << endl;
		cout << "Camera model - " << pCamInfo->modelName << endl;
		cout << "Camera vendor - " << pCamInfo->vendorName << endl;
		cout << "Sensor - " << pCamInfo->sensorInfo << endl;
		cout << "Resolution - " << pCamInfo->sensorResolution << endl;
		cout << "Firmware version - " << pCamInfo->firmwareVersion << endl;
		cout << "Firmware build time - " << pCamInfo->firmwareBuildTime << endl << endl;

	}

	void PrintFormat7Capabilities( Format7Info fmt7Info )
	{
		cout << "Max image pixels: (" << fmt7Info.maxWidth << ", " << fmt7Info.maxHeight << ")" << endl;
		cout << "Image Unit size: (" << fmt7Info.imageHStepSize << ", " << fmt7Info.imageVStepSize << ")" << endl;
		cout << "Offset Unit size: (" << fmt7Info.offsetHStepSize << ", " << fmt7Info.offsetVStepSize << ")" << endl;
		cout << "Pixel format bitfield: 0x" << fmt7Info.pixelFormatBitField << endl;

	}

	void PrintFormat7Specifications( Format7ImageSettings fmt7ImageSettings )
	{
		cout << "Image pixels: (" << fmt7ImageSettings.width << ", " << fmt7ImageSettings.height << ")" << endl;
		// cout << "Image Unit size: (" << fmt7ImageSettings.imageHStepSize << ", " << fmt7ImageSettings.imageVStepSize << ")" << endl;
		cout << "Offset: (" << fmt7ImageSettings.offsetX << ", " << fmt7ImageSettings.offsetY << ")" << endl;
		// cout << "Pixel format bitfield: 0x" << fmt7ImageSettings.pixelFormatBitField << endl;
	}

	void PrintError( Error error )
	{
		error.PrintErrorTrace();
	}

	bool CheckSoftwareTriggerPresence( Camera* pCam )
	{
		const unsigned int k_triggerInq = 0x530;

		Error error;
		unsigned int regVal = 0;

		error = pCam->ReadRegister( k_triggerInq, &regVal );

		if (error != PGRERROR_OK)
		{
			PrintError( error );
			return false;
		}

		if( ( regVal & 0x10000 ) != 0x10000 )
		{
			return false;
		}

		return true;
	}

	bool PollForTriggerReady( Camera* pCam )
	{
		const unsigned int k_softwareTrigger = 0x62C;
		Error error;
		unsigned int regVal = 0;

		do 
		{
			error = pCam->ReadRegister( k_softwareTrigger, &regVal );
			if (error != PGRERROR_OK)
			{
				PrintError( error );
				return false;
			}

		} while ( (regVal >> 31) != 0 );

		return true;
	}

	bool FireSoftwareTrigger( Camera* pCam )
	{
		const unsigned int k_softwareTrigger = 0x62C;
		const unsigned int k_fireVal = 0x80000000;
		Error error;    

		error = pCam->WriteRegister( k_softwareTrigger, k_fireVal );
		if (error != PGRERROR_OK)
		{
			PrintError( error );
			return false;
		}

		return true;
	}

	int instantiateRig(int argc, char** argv)
	{
		ros::init(argc, argv, "image_publisher");
	 	ros::NodeHandle nh;
	    image_transport::ImageTransport it(nh);
	    image_transport::Publisher pub1 = it.advertise("/rrc_camera/left/image_raw", 1);
	    image_transport::Publisher pub2 = it.advertise("/rrc_camera/right/image_raw", 1);

		PrintBuildInfo();

		image_geometry::PinholeCameraModel model_l;
	    image_geometry::PinholeCameraModel model_r; 
	    //*****************************************Camera Parameters************************************//    


		
		error = busMgr.GetNumOfCameras(&numCameras);
		ppCameras = new Camera*[numCameras];
		cam_ID = new int[numCameras];
		cam_Order = new int[numCameras];

	//14234066 && tempcamInfo.serialNumber!=14234097
	/////////////////////////////////////////////////////////////
		// read cam info file to initalise numCam, cam_ID, 

		if (error != PGRERROR_OK)
		{
			PrintError( error );
			return -1;
		}

		cout << "Number of cameras 	detected: " << numCameras << endl; 

		if ( numCameras < 1)																// changed to 2 
		{
			cout << "Insufficient number of cameras... exiting" << endl; 
			return -1;
		}
		
	/////////////////////////////
	// Instantiating Cameras   //
	/////////////////////////////

		while(camera_found != numCameras)
		{
			
			ppCameras[camera_found] = new Camera();
			PGRGuid guid1,guid2,guid;
			error = busMgr.GetCameraFromIndex(camera_test, &guid1);
			if (error != PGRERROR_OK)
			{
				PrintError( error );
				return -1;
			}
			error = ppCameras[camera_test]->Connect(&guid1);
			if (error != PGRERROR_OK)
			{
				PrintError( error );
				return -1;
			}
			error = ppCameras[camera_found]->GetCameraInfo(&tempcamInfo);
			cout<<tempcamInfo.serialNumber<<endl;
			

			int camNotFound  = 1;
			for(int i = 0; i<numCameras; i++)
	    		camNotFound= camNotFound* int (tempcamInfo.serialNumber!=cam_ID[i]);

	  		if(!camNotFound)
	    	{
	    		camera_test++;
	    		continue;
	    	}
			//if(tempcamInfo.serialNumber==14234066||tempcamInfo.serialNumber==14234097)
	    	else
	    	{
	    		camera_found++;
	    		camera_test++;
	    	}
		}

		// error = ppCameras[0]->GetCameraInfo(&tempcamInfo);
  		//   if(tempcamInfo.serialNumber!=14234066)
  		//   {
  		//   	swap(ppCameras[0],ppCameras[1]);
  		//   }

		///////////////////////////////////////////////
		//                  Powering up              //
		///////////////////////////////////////////////

		// Power on the camera
		const unsigned int k_cameraPower = 0x610;
		const unsigned int k_powerVal = 0x80000000;

		for(int i = 0; i<numCameras;i++)
		{
			error  = ppCameras[i]->WriteRegister( k_cameraPower, k_powerVal );
			if (error != PGRERROR_OK)
			{
				PrintError( error );
				return -1;
			}

			const unsigned int millisecondsToSleep = 100;
			unsigned int regVal = 0;
			unsigned int retries = 10;

			// Wait for camera to complete power-up
			do 
			{
				#if defined(WIN32) || defined(WIN64)
					Sleep(millisecondsToSleep);    
				#else
					usleep(millisecondsToSleep * 1000);
				#endif
			
				error = ppCameras[i]->ReadRegister(k_cameraPower, &regVal);
				if (error == PGRERROR_TIMEOUT)
				{
					// ignore timeout errors, camera may not be responding to
					// register reads during power-up
				}
				else if (error != PGRERROR_OK)
				{
					PrintError( error );
					return -1;
				}

				retries--;
			} while ((regVal & k_powerVal) == 0 && retries > 0);

			// Check for timeout errors after retrying
			if (error == PGRERROR_TIMEOUT)
			{
				PrintError( error );
				return -1;
			}
			regVal = 0,retries = 10;
		}

		/////////////////////////////////////////////////////////
		// 	          Get the camera information               //
		/////////////////////////////////////////////////////////

		CameraInfo camInfo;
		for(int i = 0; i<numCameras;i++)
		{
			error = ppCameras[i]->GetCameraInfo(&camInfo);
			if (error != PGRERROR_OK)
			{
				PrintError( error );
				return -1;
			}
			PrintCameraInfo(&camInfo);   
			
		}


	}

		//////////////////////////////////////////////////////////////
		//                Settting rig trigger                      //
		//////////////////////////////////////////////////////////////

	int triggerRig(char *trigger)
	{
		for(int i = 0; i<numCameras;i++)
		{

			// #ifndef SOFTWARE_TRIGGER_CAMERA
			// // Check for external trigger support
			// TriggerModeInfo triggerModeInfo;
			// error = ppCameras[i]->GetTriggerModeInfo( &triggerModeInfo );
			// if (error != PGRERROR_OK)
			// {
			// 	PrintError( error );
			// 	return -1;
			// }

			// if ( triggerModeInfo.present != true )
			// {
			// 	cout << "Camera does not support external trigger! Exiting..." << endl; 
			// 	return -1;
			// }
			// #endif

			// Get current trigger settings
			//TriggerMode triggerMode;													//
			error = ppCameras[i]->GetTriggerMode( &triggerMode );
			if (error != PGRERROR_OK)
			{
				PrintError( error );
				return -1;
			}

			// Set camera to trigger mode 0
			triggerMode.onOff = true;
			triggerMode.mode = 14;
			triggerMode.parameter = 0;	



			if(strcmp(trigger,"Software")==0)
			//			#ifdef SOFTWARE_TRIGGER_CAMERA
				// A source of 7 means software trigger
				triggerMode.source = 7;
			else
				// Triggering the camera externally using source 0.
				triggerMode.source = 0;
			//#endif

			error = ppCameras[i]->SetTriggerMode( &triggerMode );
			if (error != PGRERROR_OK)
			{
				PrintError( error );
				return -1;
			}

			// Poll to ensure camera is ready
			bool retVal = PollForTriggerReady( ppCameras[i] );
			if( !retVal )
			{	
				cout << endl;
				cout << "Error polling for trigger ready!" << endl; 
				return -1;
			}
			//cout<<"Trigger mode set"<<endl;
		}
	}


		/////////////////////////////////////////////////////
		//            Setting Image properties             //
		/////////////////////////////////////////////////////

	int imagePropRig(char *pixelFormat)
	{


		const Mode k_fmt7Mode = MODE_1;


		const PixelFormat k_fmt7PixFmt  = PIXEL_FORMAT_RAW8;

		// if ( strcmp(pixelFormat,"444YUV8") == 0 )
		// 	 {k_fmt7PixFmt = PIXEL_FORMAT_444YUV8;cout<<"1";}
		// else if ( strcmp(pixelFormat,"RGB8") == 0 )
		// 	 {k_fmt7PixFmt = PIXEL_FORMAT_RGB8;cout<<"2";}
		// else if ( strcmp(pixelFormat,"RGB16") == 0 )
		// 	 {k_fmt7PixFmt = PIXEL_FORMAT_RGB16;cout<<"3";}
		// else if ( strcmp(pixelFormat,"RAW8") == 0 )
		// 	 {k_fmt7PixFmt = PIXEL_FORMAT_RAW8;cout<<"4";}
		// else if ( strcmp(pixelFormat,"MONO8") == 0 )
		// 	 {k_fmt7PixFmt = PIXEL_FORMAT_MONO8;cout<<"5";}
		// else if ( strcmp(pixelFormat,"MONO16") == 0 )
		// 	 {k_fmt7PixFmt = PIXEL_FORMAT_MONO16;cout<<"6";}
		// else
		// 	 {k_fmt7PixFmt = PIXEL_FORMAT_BGR;cout<<"7";}

		//const PixelFormat k_fmt7PixFmt = k_fmt7PixFmtemp;

		cout<< "\n"<<k_fmt7PixFmt<<endl;getchar();
		//const PixelFormat k_fmt7PixFmt = PIXEL_FORMAT_RAW8;

		Format7Info fmt7Info;
		bool supported;

		frame = new cv::Mat*[numCameras];

     	for(int i = 0; i<numCameras;i++)
		{
			frame[i] = new cv::Mat;
			fmt7Info.mode = k_fmt7Mode;
			error =  ppCameras[i]->GetFormat7Info( &fmt7Info, &supported );
			if (error != PGRERROR_OK)
			{
				PrintError( error );
				return -1;
			}

			PrintFormat7Capabilities( fmt7Info );

			if ( (k_fmt7PixFmt & fmt7Info.pixelFormatBitField) == 0 )
			{
				// Pixel format not supported!
				cout << "Pixel format is not supported" << endl; 
				return -1;
			}

			Format7ImageSettings fmt7ImageSettings;
			fmt7ImageSettings.mode = k_fmt7Mode;
			fmt7ImageSettings.offsetX = 2;
			fmt7ImageSettings.offsetY = 2;
			fmt7ImageSettings.width = 640;
			fmt7ImageSettings.height = 480;
			fmt7ImageSettings.pixelFormat = k_fmt7PixFmt;

			PrintFormat7Specifications(fmt7ImageSettings);

			Property prop;
			prop.type = SHUTTER;
			error = ppCameras[i]->GetProperty( &prop );
			if (error != PGRERROR_OK)
			{
				PrintError( error );
				return -1;
			}

			//prop.autoManualMode = false;
			//prop.absControl = true;

			float k_shutterVal = prop.absValue;



			//prop.absValue = k_shutterVal;

			error = ppCameras[i]->SetProperty( &prop );
			if (error != PGRERROR_OK)
			{
				PrintError( error );
				return -1;
			}

			cout << "Shutter time set to " << setprecision(2) << k_shutterVal << "ms" << endl;

			prop.type = GAIN;
			error = ppCameras[i]->GetProperty( &prop );
			if (error != PGRERROR_OK)
			{
				PrintError( error );
				return -1;
			}

			prop.autoManualMode = false;
			prop.absControl = true;


			const float k_GainVal = 3.57;
			prop.absValue = k_GainVal;

			error = ppCameras[i]->SetProperty( &prop );
			if (error != PGRERROR_OK)
			{
				PrintError( error );
				return -1;
			}

			cout << "Gain set to " << setprecision(2) << k_GainVal << "dB" << endl;



			prop.type = WHITE_BALANCE;
			error = ppCameras[i]->GetProperty( &prop );
			if (error != PGRERROR_OK)
			{
				PrintError( error );
				return -1;
			}

			prop.autoManualMode = false;
			prop.absControl = false;
			const float k_RedVal = 600 , k_BlueVal=750;

			prop.valueA = k_RedVal;
			prop.valueB = k_BlueVal;

			error = ppCameras[i]->SetProperty( &prop );
			if (error != PGRERROR_OK)
			{
				PrintError( error );
				return -1;
			}


			cout << "WHITE_BALANCE set to " <<  k_RedVal <<" "<< k_BlueVal << endl;

			bool valid;
			Format7PacketInfo fmt7PacketInfo;

			// Validate the settings to make sure that they are valid
			error =  ppCameras[i]->ValidateFormat7Settings(
					&fmt7ImageSettings,
					&valid,
					&fmt7PacketInfo );
			if (error != PGRERROR_OK)
			{
				PrintError( error );
				return -1;
			}

			if ( !valid )
			{
				// Settings are not valid
				cout << "Format7 settings are not valid" << endl; 
				return -1;
			}
			
			// Set the settings to the camera
			error =  ppCameras[i]->SetFormat7Configuration(
					&fmt7ImageSettings,
					fmt7PacketInfo.recommendedBytesPerPacket );
			if (error != PGRERROR_OK)
			{
				PrintError( error );
				return -1;
			}
			
			// Get the camera configuration
			FC2Config config;
			error =  ppCameras[i]->GetConfiguration( &config );
			if (error != PGRERROR_OK)
			{
				PrintError( error );
				return -1;
			}

			// Set the grab timeout to 5 seconds
			config.grabTimeout = 5000;

			// Set the camera configuration
			error =  ppCameras[i]->SetConfiguration( &config );
			if (error != PGRERROR_OK)
			{
				PrintError( error );
				return -1;
			} 

		}
	}
	//////////////////////////////////////////////////////////
	///////////////////// Camera is ready, start capturing images

	int startRigCapture(char *trigger)
	{
		for(int i = 0; i<numCameras;i++)
		{	
		
			error =  ppCameras[i]->StartCapture();

			if (error != PGRERROR_OK)
			{
				PrintError( error );
				return -1;
			}   

			if(strcmp(trigger,"Software")==0)
			//#ifdef SOFTWARE_TRIGGER_CAMERA
				if (!CheckSoftwareTriggerPresence(  ppCameras[i] ))
				{
					cout << "SOFT_ASYNC_TRIGGER not implemented on this camera!  Stopping application" << endl ; 
					return -1;
				}
			else	
				cout << "Trigger the camera by sending a trigger pulse to GPIO" << triggerMode.source << endl; 

			//#endif
		}
		//}  // end of for ; no of cameras


		cout<<"\n\t\t\t\t CAMERA INITIALIZATION COMPLETE \n";
	}




	int imageGrab(char *trigger, char *pixelFormat)
	{
		// ros::NodeHandle nh;
	 	// image_transport::ImageTransport it(nh);
	    // image_transport::Publisher pub1 = it.advertise("/rrc_camera/left/image_raw", 1);
	    // image_transport::Publisher pub2 = it.advertise("/rrc_camera/right/image_raw", 1);


		// namedWindow("cam1",WINDOW_AUTOSIZE); 			//
		// namedWindow("cam2",WINDOW_AUTOSIZE); 			//

		//   for ( int imageCount=0; imageCount < k_numImages; imageCount++ )
		// while(nh.ok())
		// {
			////////////////////////////////////////////////////////
			////////////////////////////////////////////////////
			////////////////////////////////////////////////////
			prop.type = SHUTTER;
			//for ( unsigned int i = 0; i < numCameras; i++)
			//{
				error = ppCameras[0]->GetProperty( &prop );
				if (error != PGRERROR_OK)
				{
					PrintError( error );
					return -1;
				}

				k_shutterVal = prop.absValue;

				
				
				prop.type = SHUTTER;
				
				error = ppCameras[1]->GetProperty( &prop );
				if (error != PGRERROR_OK)
				{
					PrintError( error );
					return -1;
				}

				prop.autoManualMode = false;
				prop.absControl = true;

				prop.absValue = k_shutterVal;

				error = ppCameras[1]->GetProperty( &prop );
				if (error != PGRERROR_OK)
				{
					PrintError( error );
					return -1;
				}

				

				// error = ppCameras[1]->SetProperty( &prop );
				// if (error != PGRERROR_OK)
				// {
				// 	PrintError( error );
				// 	return -1;
				// }
				//}
			cout <<  k_shutterVal << endl;

			for ( unsigned int i = 0; i < numCameras; i++)
			{																				//

				if(strcmp(trigger,"Software")==0)
				{
				//#ifdef SOFTWARE_TRIGGER_CAMERA
					cout<<" Software Trigger\n";
					// Check that the trigger is ready
					PollForTriggerReady(  ppCameras[i]);
					bool retVal = FireSoftwareTrigger(  ppCameras[i] );
					if ( !retVal )
					{
						cout << endl;
						cout << "Error firing software trigger" << endl; 
						return -1;        
					}
				}

				//#endif
				// Grab image        
				
				error = ppCameras[i]->RetrieveBuffer( &image );

				if (error != PGRERROR_OK)
				{
					PrintError( error );
					break;
				}

				if(i == 0)
				{
					image1 = image;
				}
				else
				{
					image2 = image;
				}

			}
		
			Image rgbImage1,rgbImage2;
			
			if ( strcmp(pixelFormat,"444YUV8") == 0 )
			 	{
			 		image1.Convert( FlyCapture2::PIXEL_FORMAT_444YUV8, &rgbImage1 );
					image2.Convert( FlyCapture2::PIXEL_FORMAT_444YUV8, &rgbImage2 );
				}

			else if ( strcmp(pixelFormat,"RGB8") == 0 )
			 	{
			 		image1.Convert( FlyCapture2::PIXEL_FORMAT_RGB8, &rgbImage1 );
					image2.Convert( FlyCapture2::PIXEL_FORMAT_RGB8, &rgbImage2 );
				}

			else if ( strcmp(pixelFormat,"RGB16") == 0 )
			 	{
			 		image1.Convert( FlyCapture2::PIXEL_FORMAT_RGB16, &rgbImage1 );
					image2.Convert( FlyCapture2::PIXEL_FORMAT_RGB16, &rgbImage2 );
				}
			else if ( strcmp(pixelFormat,"RAW8") == 0 )
			 	{
			 		image1.Convert( FlyCapture2::PIXEL_FORMAT_RAW8, &rgbImage1 );
					image2.Convert( FlyCapture2::PIXEL_FORMAT_RAW8, &rgbImage2 );
			 	}
			else if ( strcmp(pixelFormat,"MONO8") == 0 )
			 	{
			 		image1.Convert( FlyCapture2::PIXEL_FORMAT_MONO8, &rgbImage1 );
					image2.Convert( FlyCapture2::PIXEL_FORMAT_MONO8, &rgbImage2 );
				}
			else if ( strcmp(pixelFormat,"MONO16") == 0 )
			 	{
			 		image1.Convert( FlyCapture2::PIXEL_FORMAT_MONO16, &rgbImage1 );
					image2.Convert( FlyCapture2::PIXEL_FORMAT_MONO16, &rgbImage2 );
				}
			else
			 	{
			 		image1.Convert( FlyCapture2::PIXEL_FORMAT_BGR, &rgbImage1 );
					image2.Convert( FlyCapture2::PIXEL_FORMAT_BGR, &rgbImage2 );
				}


			// image1.Convert( FlyCapture2::PIXEL_FORMAT_BGR, &rgbImage1 );
			// image2.Convert( FlyCapture2::PIXEL_FORMAT_BGR, &rgbImage2 );
			
			// convert to OpenCV Mat
			unsigned int rowBytes = (double)rgbImage1.GetReceivedDataSize()/(double)rgbImage1.GetRows();
			*frame[0] = Mat(rgbImage1.GetRows(), rgbImage1.GetCols(), CV_8UC3, rgbImage1.GetData(),rowBytes);
			*frame[1] = Mat(rgbImage2.GetRows(), rgbImage2.GetCols(), CV_8UC3, rgbImage2.GetData(),rowBytes);
			// unsigned int rowBytes = (double)image1.GetReceivedDataSize()/(double)image1.GetRows();
			// Mat imageMat1 = Mat(image1.GetRows(), image1.GetCols(), CV_8UC3, image1.GetData(),rowBytes);
			// Mat imageMat2 = Mat(image2.GetRows(), image2.GetCols(), CV_8UC3, image2.GetData(),rowBytes);
			current_timestamp();

			namedWindow("cam1",WINDOW_AUTOSIZE); 			//
			namedWindow("cam2",WINDOW_AUTOSIZE); 			//

			//namedWindow( "Left window", WINDOW_NORMAL );// Create a window for display.
			imshow( "cam1", *frame[0]);                   // Show our image inside it.

			//namedWindow( "Right window", WINDOW_NORMAL );// Create a window for display.
			imshow( "cam2", *frame[1]);                   // Show our image inside it.

			 waitKey(3);

			// //namedWindow( "Left window", WINDOW_NORMAL );// Create a window for display.
	  		//   	imshow( "cam1", imageMat1 );                   // Show our image inside it.

			// //namedWindow( "Right window", WINDOW_NORMAL );// Create a window for display.
	  		//   	imshow( "cam2", imageMat2 );                   // Show our image inside it.

	  		//   	waitKey(3); 

			// 	if ((char)waitKey(1) == 27) 								// 10 to 3 ; for escape
			// 	{
			// 		cout << "user exit" << endl;
			// 		flag=1;
			// 		break; 
			// 	}
			// }

			//TriggerMode triggerMode;
			// Turn trigger mode off.
	}

	~MultiCam()
	{

		for ( unsigned int i = 0; i < numCameras; i++)
		{

			triggerMode.onOff = false;    
			error = ppCameras[i]->SetTriggerMode( &triggerMode );
			if (error != PGRERROR_OK)
			{
				PrintError( error );
				//return -1;
			}
			cout << endl;
			cout << "Finished grabbing images" << endl; 
			destroyAllWindows();
			// Stop capturing images
			error = ppCameras[i]->StopCapture();
			if (error != PGRERROR_OK)
			{
				PrintError( error );
				//return -1;
			}      

			// Turn off trigger mode
			triggerMode.onOff = false;
			error = ppCameras[i]->SetTriggerMode( &triggerMode );
			if (error != PGRERROR_OK)
			{
				PrintError( error );
				//return -1;
			}    

			// Disconnect the camera
			error = ppCameras[i]->Disconnect();
			if (error != PGRERROR_OK)
			{
				PrintError( error );
				//return -1;
			}
		}			

		cout << "Done! Press Enter to exit..." << endl; 
		cin.ignore();
		//return 0;
	}
};

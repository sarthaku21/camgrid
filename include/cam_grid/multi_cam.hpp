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

#include <iostream>
#include <unistd.h>
#include <sstream>

#define _POSIX_C_SOURCE 200809L

#include <sys/time.h>
#include <math.h>
#include <stdio.h>
#include <time.h>

#include "settings.h"
//#include <sensor_msgs/CameraInfo.h>
//#include <image_geometry/pinhole_camera_model.h>

// Software trigger the camera instead of using an external hardware trigger
//
//#define SOFTWARE_TRIGGER_CAMERA

using namespace FlyCapture2;
using namespace std;


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
	//cv::Mat** frame;
	int master_id;

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
		master_id = 0;
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

	int instantiateRig(int* ids, int camCount, int mid)
	{
		
		PrintBuildInfo();

		//image_geometry::PinholeCameraModel model_l;
		//image_geometry::PinholeCameraModel model_r; 
		//*****************************************Camera Parameters************************************//    


		master_id = mid;
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
			PGRGuid guid;
			error = busMgr.GetCameraFromIndex(camera_test, &guid);
			if (error != PGRERROR_OK)
			{
				PrintError( error );
				return -1;
			}
			error = ppCameras[camera_test]->Connect(&guid);
			if (error != PGRERROR_OK)
			{
				PrintError( error );
				return -1;
			}
			error = ppCameras[camera_found]->GetCameraInfo(&tempcamInfo);
			cout<<tempcamInfo.serialNumber<<"   cam_ID"<<endl;
			

			int camNotFound  = 1;
			//for(int i = 0; i<numCameras; i++)
	    	//	camNotFound= camNotFound* int (tempcamInfo.serialNumber!=cam_ID[i]);

	  		if(!camNotFound)
	    	{
	    		camera_test++;
	    		continue;
	    	}
	    	else
	    	{
	    		camera_found++;
	    		camera_test++;
	    	}
		}
		
		for(int i = 0; i < camCount; i++){
			for(int j = 0; i < numCameras; j++){
				ppCameras[camera_found]->GetCameraInfo(&tempcamInfo);
				if(ids[i] == tempcamInfo.serialNumber)
					swap(ppCameras[i], ppCameras[j]);
			}
		}
		
		for(int i = camCount; i < numCameras; i++){
			diconsingleCam(ppCameras[i]);
		}
				
		numCameras = camCount;

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

		return 0;
	
	}

	int setProps2auto(int master){
			Property prop;
			prop.type = SHUTTER;
			prop.autoManualMode = true;
			error = ppCameras[master]->SetProperty( &prop );
			if (error != PGRERROR_OK){
				PrintError( error );
				return -1;
			}
						
			prop.type = GAIN;
			prop.autoManualMode = true;
			error = ppCameras[master]->SetProperty( &prop );
			if (error != PGRERROR_OK){
				PrintError( error );
				return -1;
			}
	
			prop.type = WHITE_BALANCE;
			prop.autoManualMode = true;
			error = ppCameras[master]->SetProperty( &prop );
			if (error != PGRERROR_OK){
				PrintError( error );
				return -1;
			}
	}

	int updateProps(int master){
		for ( unsigned int i = 0; i < numCameras; i++){
			Property prop;
			prop.type = SHUTTER;
			if(i == master)
				continue;
			error = ppCameras[master]->GetProperty( &prop );
			if (error != PGRERROR_OK){
				PrintError( error );
				return -1;
			}
			error = ppCameras[i]->SetProperty( &prop );
			if (error != PGRERROR_OK){
				PrintError( error );
				return -1;
			}
		}
		for ( unsigned int i = 0; i < numCameras; i++){
			Property prop;
			prop.type = GAIN;
			if(i == master)
				continue;
			error = ppCameras[master]->GetProperty( &prop );
			if (error != PGRERROR_OK){
				PrintError( error );
				return -1;
			}
			error = ppCameras[i]->SetProperty( &prop );
			if (error != PGRERROR_OK){
				PrintError( error );
				return -1;
			}
		}
		for ( unsigned int i = 0; i < numCameras; i++){
			Property prop;
			prop.type = WHITE_BALANCE;
			if(i == master)
				continue;
			error = ppCameras[master]->GetProperty( &prop );
			if (error != PGRERROR_OK){
				PrintError( error );
				return -1;
			}
			error = ppCameras[i]->SetProperty( &prop );
			if (error != PGRERROR_OK){
				PrintError( error );
				return -1;
			}
		}
	}
		

		//////////////////////////////////////////////////////////////
		//                Settting rig trigger                      //
		//////////////////////////////////////////////////////////////

	int triggerRig(char *trigger)
	{
		for(int i = 0; i<numCameras;i++)
		{

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
		return 0;
	
	}


		/////////////////////////////////////////////////////
		//            Setting Image properties             //
		/////////////////////////////////////////////////////

	int imagePropRig(char *pixelFormat, int W, int H, int oW, int oH)
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

     	for(int i = 0; i<numCameras;i++)
		{
			
			
			setProps2auto(master_id);
			updateProps(master_id);
			
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
			fmt7ImageSettings.offsetX = oW;
			fmt7ImageSettings.offsetY = oH;
			fmt7ImageSettings.width = W;
			fmt7ImageSettings.height = H;
			fmt7ImageSettings.pixelFormat = k_fmt7PixFmt;

			PrintFormat7Specifications(fmt7ImageSettings);
/*
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
*/
			bool valid;
			Format7PacketInfo fmt7PacketInfo;

			// Validate the settings to make sure that they are valid
			error =  ppCameras[i]->ValidateFormat7Settings(
					&fmt7ImageSettings,
					&valid,
					&fmt7PacketInfo );
			if (error != PGRERROR_OK){
				PrintError( error );
				return -1;
			}

			if ( !valid ){
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
		return 0;
	}
	//////////////////////////////////////////////////////////
	///////////////////// Camera is ready, start capturing images

	int startRigCapture(char *trigger){
		for(int i = 0; i<numCameras;i++){	
			error =  ppCameras[i]->StartCapture();
			if (error != PGRERROR_OK){
				PrintError( error );
				return -1;
			}   

			if(strcmp(trigger,"Software")==0){
				cout << "Trigger " << triggerMode.source << endl; 

			//#ifdef SOFTWARE_TRIGGER_CAMERA
				if (!CheckSoftwareTriggerPresence(  ppCameras[i] )){
					cout << "SOFT_ASYNC_TRIGGER not implemented on this camera!  Stopping application" << endl ; 
					return -1;
				}
			}
			else	
				cout << "Trigger the camera by sending a trigger pulse to GPIO" << triggerMode.source << endl; 

			//#endif
		}
		//}  // end of for ; no of cameras

		cout<<"\n\t\t\t\t CAMERA INITIALIZATION COMPLETE \n";
		return 0;
	}



	int imageGrab(char *trigger, char *pixelFormat, Image &in_image, int cam_no){
		if(strcmp(trigger,"Software")==0)
		{
			//#ifdef SOFTWARE_TRIGGER_CAMERA
			cout<<" Software Trigger\n";
			// Check that the trigger is ready
			PollForTriggerReady(  ppCameras[cam_no]);
			bool retVal = FireSoftwareTrigger(  ppCameras[cam_no] );
			if ( !retVal )
			{
				cout << endl;
				cout << "Error firing software trigger" << endl; 
				return -1;        
			}
		}

		error = ppCameras[cam_no]->RetrieveBuffer( &image );
		if (error != PGRERROR_OK)
			PrintError( error );
		in_image = image;
	}

	void diconsingleCam(Camera* Cam){
		// Disconnect the camera
		error = Cam->Disconnect();
		if (error != PGRERROR_OK)
		{
			PrintError( error );
			//return -1;
		}
	}
	
	~MultiCam()
	{

		for ( unsigned int i = 0; i < numCameras; i++)
		{
			cout << endl;
			cout << "Finished grabbing images" << endl; 
			// Stop capturing images
			error = ppCameras[i]->StopCapture();
			if (error != PGRERROR_OK)
			{
				PrintError( error );
			}      

			// Disconnect the camera
			error = ppCameras[i]->Disconnect();
			if (error != PGRERROR_OK)
			{
				PrintError( error );
			}
		}			

		cout << "Done! Press Enter to exit..." << endl; 
		cin.ignore();
		//return 0;
	}
};

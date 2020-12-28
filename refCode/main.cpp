/*****************************************************************************
*                                                                            *
*  OpenNI 2.x Alpha                                                          *
*  Copyright (C) 2012 PrimeSense Ltd.                                        *
*                                                                            *
*  This file is part of OpenNI.                                              *
*                                                                            *
*  Licensed under the Apache License, Version 2.0 (the "License");           *
*  you may not use this file except in compliance with the License.          *
*  You may obtain a copy of the License at                                   *
*                                                                            *
*      http://www.apache.org/licenses/LICENSE-2.0                            *
*                                                                            *
*  Unless required by applicable law or agreed to in writing, software       *
*  distributed under the License is distributed on an "AS IS" BASIS,         *
*  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  *
*  See the License for the specific language governing permissions and       *
*  limitations under the License.                                            *
*                                                                            *
*****************************************************************************/
#include <OpenNI.h>
//#include "Viewer.h"
#include "AXonLink.h"
int main(int argc, char** argv)
{
    openni::Status rc = openni::STATUS_OK;

    openni::Device device;
    openni::VideoStream depth, color, ir;
	int nResolutionColor = 0;
	int nResolutionDepth = 0;
	int lastResolutionX = 0;
	int lastResolutionY = 0;
    const char* deviceURI = openni::ANY_DEVICE;
    if (argc > 1)
    {
        deviceURI = argv[1];
    }

    rc = openni::OpenNI::initialize();

    printf("After initialization:\n%s\n", openni::OpenNI::getExtendedError());

    rc = device.open(deviceURI);
    if (rc != openni::STATUS_OK)
    {
        printf("SimpleViewer: Device open failed:\n%s\n", openni::OpenNI::getExtendedError());
        openni::OpenNI::shutdown();
        return 1;
    }
	const openni::SensorInfo* info = device.getSensorInfo(openni::SENSOR_COLOR);
	if(info)
	{
		for(int i = 0; i < info->getSupportedVideoModes().getSize(); i++)
		{
			printf("Color info video %d %dx%d FPS %d f %d\n", i,
			info->getSupportedVideoModes()[i].getResolutionX(),
			info->getSupportedVideoModes()[i].getResolutionY(),
			info->getSupportedVideoModes()[i].getFps(),
			info->getSupportedVideoModes()[i].getPixelFormat());
			if((info->getSupportedVideoModes()[i].getResolutionX() != lastResolutionX)||(info->getSupportedVideoModes()[i].getResolutionY() != lastResolutionY))
			{
				nResolutionColor++;
				lastResolutionX = info->getSupportedVideoModes()[i].getResolutionX();
				lastResolutionY = info->getSupportedVideoModes()[i].getResolutionY();
			}
		}
	}
	lastResolutionX = 0;
	lastResolutionY = 0;
	const openni::SensorInfo* depthinfo = device.getSensorInfo(openni::SENSOR_DEPTH);
	if(depthinfo)
	{
		for(int i = 0; i < depthinfo->getSupportedVideoModes().getSize(); i++)
		{
			printf("Depth info video %d %dx%d FPS %d f %d\n", i,
				depthinfo->getSupportedVideoModes()[i].getResolutionX(),
				depthinfo->getSupportedVideoModes()[i].getResolutionY(),
				depthinfo->getSupportedVideoModes()[i].getFps(),
				depthinfo->getSupportedVideoModes()[i].getPixelFormat());
			if((depthinfo->getSupportedVideoModes()[i].getResolutionX() != lastResolutionX)||(depthinfo->getSupportedVideoModes()[i].getResolutionY() != lastResolutionY))
			{
				nResolutionDepth++;
				lastResolutionX = depthinfo->getSupportedVideoModes()[i].getResolutionX();
				lastResolutionY = depthinfo->getSupportedVideoModes()[i].getResolutionY();
			}
		}
	}
    rc = depth.create(device, openni::SENSOR_DEPTH);
    if (rc == openni::STATUS_OK)
    {
        rc = depth.start();
        if (rc != openni::STATUS_OK)
        {
            printf("SimpleViewer: Couldn't start depth stream:\n%s\n", openni::OpenNI::getExtendedError());
            depth.destroy();
        }
    }
    else
    {
        printf("SimpleViewer: Couldn't find depth stream:\n%s\n", openni::OpenNI::getExtendedError());
    }

    rc = color.create(device, openni::SENSOR_COLOR);
    if (rc == openni::STATUS_OK)
    {
    	openni::VideoMode vm;
		vm = color.getVideoMode();
		vm.setResolution(1280, 960);
		color.setVideoMode(vm);
        rc = color.start();
        if (rc != openni::STATUS_OK)
        {
            printf("SimpleViewer: Couldn't start color stream:\n%s\n", openni::OpenNI::getExtendedError());
            color.destroy();
        }
    }
    else
    {
        printf("SimpleViewer: Couldn't find color stream:\n%s\n", openni::OpenNI::getExtendedError());
    }
	AXonLinkCamParam camParam;
	int dataSize = sizeof(AXonLinkCamParam);
	device.getProperty(AXONLINK_DEVICE_PROPERTY_GET_CAMERA_PARAMETERS, &camParam, &dataSize);
	for(int i =0 ; i<nResolutionColor;i++)
	{
		printf("astColorParam x =%d\n",camParam.astColorParam[i].ResolutionX);
		printf("astColorParam y =%d\n",camParam.astColorParam[i].ResolutionY);
		printf("astColorParam fx =%.5f\n",camParam.astColorParam[i].fx);
		printf("astColorParam fy =%.5f\n",camParam.astColorParam[i].fy);
		printf("astColorParam cx =%.5f\n",camParam.astColorParam[i].cx);
		printf("astColorParam cy =%.5f\n",camParam.astColorParam[i].cy);
		printf("astColorParam k1 =%.5f\n",camParam.astColorParam[i].k1);
		printf("astColorParam k2 =%.5f\n",camParam.astColorParam[i].k2);
		printf("astColorParam p1 =%.5f\n",camParam.astColorParam[i].p1);
		printf("astColorParam p2 =%.5f\n",camParam.astColorParam[i].p2);
		printf("astColorParam k3 =%.5f\n",camParam.astColorParam[i].k3);
		printf("astColorParam k4 =%.5f\n",camParam.astColorParam[i].k4);
		printf("astColorParam k5 =%.5f\n",camParam.astColorParam[i].k5);
		printf("astColorParam k6 =%.5f\n",camParam.astColorParam[i].k6);
	}
	for(int i =0 ; i<nResolutionDepth;i++)
	{
		printf("astDepthParam x =%d\n",camParam.astDepthParam[i].ResolutionX);
		printf("astDepthParam y =%d\n",camParam.astDepthParam[i].ResolutionY);
		printf("astDepthParam fx =%.5f\n",camParam.astDepthParam[i].fx);
		printf("astDepthParam fy =%.5f\n",camParam.astDepthParam[i].fy);
		printf("astDepthParam cx =%.5f\n",camParam.astDepthParam[i].cx);
		printf("astDepthParam cy =%.5f\n",camParam.astDepthParam[i].cy);
		printf("astDepthParam k1 =%.5f\n",camParam.astDepthParam[i].k1);
		printf("astDepthParam k2 =%.5f\n",camParam.astDepthParam[i].k2);
		printf("astDepthParam p1 =%.5f\n",camParam.astDepthParam[i].p1);
		printf("astDepthParam p2 =%.5f\n",camParam.astDepthParam[i].p2);
		printf("astDepthParam k3 =%.5f\n",camParam.astDepthParam[i].k3);
		printf("astDepthParam k4 =%.5f\n",camParam.astDepthParam[i].k4);
		printf("astDepthParam k5 =%.5f\n",camParam.astDepthParam[i].k5);
		printf("astDepthParam k6 =%.5f\n",camParam.astDepthParam[i].k6);
	}
	printf("R = %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f \n",camParam.stExtParam.R_Param[0],camParam.stExtParam.R_Param[1],camParam.stExtParam.R_Param[2],camParam.stExtParam.R_Param[3],camParam.stExtParam.R_Param[4],camParam.stExtParam.R_Param[5],camParam.stExtParam.R_Param[6],camParam.stExtParam.R_Param[7],camParam.stExtParam.R_Param[8]);
	printf("T = %.5f %.5f %.5f \n",camParam.stExtParam.T_Param[0],camParam.stExtParam.T_Param[1],camParam.stExtParam.T_Param[2]);
    // rc = ir.create(device, openni::SENSOR_IR);
    // if (rc == openni::STATUS_OK)
    // {
        // rc = ir.start();
        // if (rc != openni::STATUS_OK)
        // {
            // printf("SimpleViewer: Couldn't start color stream:\n%s\n", openni::OpenNI::getExtendedError());
            // ir.destroy();
        // }
    // }
    // else
    // {
        // printf("SimpleViewer: Couldn't find color stream:\n%s\n", openni::OpenNI::getExtendedError());
    // }
	// AXonLinkGetExposureLevel value;
	// int nSize = sizeof(value);
	// ir.getProperty(AXONLINK_STREAM_PROPERTY_EXPOSURE_LEVEL, &value,&nSize);
	// printf("Get level:custId=%d,max=%d,current=%d\n",value.customID, value.maxLevel, value.curLevel);

    // if (!depth.isValid() || !color.isValid() || !ir.isValid())
    // {
        // printf("SimpleViewer: No valid streams. Exiting\n");
        // openni::OpenNI::shutdown();
        // return 2;
    // }
	// //rc = device.setDepthColorSyncEnabled(true);
	// if(rc !=openni::STATUS_OK )
	// {
		// printf("start sync failed1\n");
        // openni::OpenNI::shutdown();
        // return 4;
	// }
    // SampleViewer sampleViewer("ALL Viewer", device, depth, color, ir);

    // rc = sampleViewer.init(argc, argv);
    // if (rc != openni::STATUS_OK)
    // {
        // openni::OpenNI::shutdown();
        // return 3;
    // }
    // sampleViewer.run();
}

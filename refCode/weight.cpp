#include <iostream>
#include <stdio.h>
#include <iomanip> //I/O流控制流头文件
#include <time.h>
#include <signal.h>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <ctime>
#include <json/json.h>
#include <pthread.h>
#include <json/json.h>
#include <stdlib.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <assert.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fstream>
#include <OpenNI.h>
//#include <opencv2/opencv.hpp>
#include <vector>
//#include <io.h>
#include <string>
#include <sstream>
#include <pbs/OniSampleUtilities.h>
#include <pbs/Pig_Body_Size.h>
#include <pcl/io/png_io.h>
#include "sql_ifly.h"
#include "ini/INIReader.h"
#include <sstream>
#include "AXonLink.h"

#define READ_FIFO_PATH "/etc/weight/read.pipe"
#define WRITE_FIFO_PATH "/etc/weight/write.pipe"
#define SAMPLE_READ_WAIT_TIMEOUT 2000

using namespace std;
using namespace cv;

//using namespace cv;
using namespace openni;
//openni::Device mDevice;
Device mDevice;
openni::VideoStream mColorStream;
openni::VideoStream mDepthStream;

int makeDir(const char *path)
{
	int beginCmpPath;
	int endCmpPath;
	int fullPathLen;
	int pathLen = strlen(path);
	char currentPath[128] = {0};
	char fullPath[128] = {0};

	printf("path = %s\n", path);
	if ('/' != path[0])
	{
		getcwd(currentPath, sizeof(currentPath));
		strcat(currentPath, "/");
		printf("currentPath = %s\n", currentPath);
		beginCmpPath = strlen(currentPath);
		strcat(currentPath, path);
		if (path[pathLen] != '/')
		{
			strcat(currentPath, "/");
		}
		endCmpPath = strlen(currentPath);
	}
	else
	{
		int pathLen = strlen(path);
		strcpy(currentPath, path);
		if (path[pathLen] != '/')
		{
			strcat(currentPath, "/");
		}
		beginCmpPath = 1;
		endCmpPath = strlen(currentPath);
	}

	for (int i = beginCmpPath; i < endCmpPath; i++)
	{
		if ('/' == currentPath[i])
		{
			currentPath[i] = '\0';
			if (access(currentPath, NULL) != 0)
			{
				if (mkdir(currentPath, 0755) == -1)
				{
					printf("currentPath = %s\n", currentPath);
					perror("mkdir error %s\n");
					return -1;
				}
			}
			currentPath[i] = '/';
		}
	}
	return 0;
}

enum
{
	Processor_cl,
	Processor_gl,
	Processor_cpu
};

struct Config
{
	string CollectFormats;
	bool DepthColorSyncEnabled;
	string RGBFilePath;
	string DepthFilePath;
	string DepthShowFilePath;
	string PointCloudFilePath;
	string FileNameFormat;
	double WeightRangeMin;
	double WeightRangeMax;
	long TimeRangeMin;
	long TimeRangeMax;
	long CollectInterval;
	string SerialPort;
	long Baund;
	string DataBase;
	void init(INIReader reader)
	{
		this->CollectFormats = reader.GetString("DEFAULT", "CollectFormats", "rgb,depth,pointCloud");
		this->DepthColorSyncEnabled = reader.GetBoolean("DEFAULT", "DepthColorSyncEnabled", true);
		this->RGBFilePath = reader.GetString("DEFAULT", "RGBFilePath", "./rgb");
		makeDir(this->RGBFilePath.c_str());
		this->DepthFilePath = reader.GetString("DEFAULT", "DepthFilePath", "./dept");
		makeDir(this->DepthFilePath.c_str());
		this->PointCloudFilePath = reader.GetString("DEFAULT", "PointCloudFilePath", "./point");
		makeDir(this->PointCloudFilePath.c_str());
		this->DepthShowFilePath = reader.GetString("DEFAULT", "DepthShowFilePath", "./depthshow");
		makeDir(this->DepthShowFilePath.c_str());
		this->FileNameFormat = reader.GetString("DEFAULT", "FileNameFormat", "%Y%m%d_%H%M%S");
		this->WeightRangeMin = reader.GetReal("DEFAULT", "WeightRangeMin", 0.0);
		this->WeightRangeMax = reader.GetReal("DEFAULT", "WeightRangeMax", 100.0);
		this->CollectInterval = reader.GetInteger("DEFAULT", "CollectInterval", 100);
		this->TimeRangeMin = reader.GetInteger("DEFAULT", "TimeRangeMin", 5);
		this->TimeRangeMax = reader.GetInteger("DEFAULT", "TimeRangeMax", 18);
		this->SerialPort = reader.GetString("DEFAULT", "SerialPort", "/dev/ttyS3");
		this->Baund = reader.GetInteger("DEFAULT", "Baund", 9600);
		this->DataBase = reader.GetString("DEFAULT", "DataBase", "./weight.db");
	}
	string str()
	{
		ostringstream output;
		output << "CollectFormats:" << this->CollectFormats << ";\n"
			   << "DepthColorSyncEnabled:" << this->DepthColorSyncEnabled << ";\n"
			   << "RGBFilePath:" << this->RGBFilePath << ";\n"
			   << "DepthFilePath:" << this->DepthFilePath + ";\n"
			   << "DepthFilePath:" << this->DepthFilePath + ";\n"
			   << "DepthShowFilePath:" << this->DepthShowFilePath << ";\n"
			   << "FileNameFormat:" << this->FileNameFormat << ";\n"
			   << "WeightRangeMin:" << this->WeightRangeMin << ";\n"
			   << "WeightRangeMax:" << this->WeightRangeMax << ";\n"
			   << "TimeRangeMin:" << this->TimeRangeMin << ";\n"
			   << "TimeRangeMax:" << this->TimeRangeMax << ";\n"
			   << "CollectInterval:" << this->CollectInterval << ";\n"
			   << "SerialPort:" << this->SerialPort << ";\n"
			   << "DataBase:" << this->DataBase << ";\n"
			   << "Baund:" << this->Baund << ";\n";
		return string(output.str());
	}
} config;

int speed_arr[] = {
	B38400,
	B19200,
	B9600,
	B4800,
	B2400,
	B1200,
	B300,
	B38400,
	B19200,
	B9600,
	B4800,
	B2400,
	B1200,
	B300,
};
int name_arr[] = {
	38400,
	19200,
	9600,
	4800,
	2400,
	1200,
	300,
	38400,
	19200,
	9600,
	4800,
	2400,
	1200,
	300,
};

bool protonect_shutdown = false; // Whether the running application should shut down.
const int MIN_WEIGHT = 0;
const int MAX_WEIGHT = 100;
const int WEIGH_WAIT_TIME = 3;
const int WEIGH_DIS = 2;
bool isWeighted = false;
float pig_weight = 0;
char *weight_file_name = "/data/pic/weight_records.txt";
string FILE_DIR = "/data/pic/records/";
string TIME_FILE = "2019-7-1";
string strWeightFile = "weight_records.txt";
bool isGenWeightData = false;
int genWeightDataLimit = 3;
string batchNumber = "";

/* sqlite3 */
char sql_create_weight[1024] = "CREATE TABLE IF NOT EXISTS 'weight_records' ('id' INTEGER PRIMARY KEY AUTOINCREMENT, 'weight' REAL NOT NULL, 'time' TEXT NOT NULL, 'batchnumber' TEXT NOT NULL,'rgbfilename' TEXT NOT NULL, 'depthfilename' TEXT NOT NULL, 'pointfilename' TEXT NOT NULL)";

char sql_create_index[200] = "CREATE INDEX 'weightId' ON 'weight_records' ('id')";

vector<string> split(const string &str, const string &pattern)
{
	vector<string> res;
	if (str == "")
		return res;

	string strs = str + pattern;
	size_t pos = strs.find(pattern);

	while (pos != strs.npos)
	{
		string temp = strs.substr(0, pos);
		temp.erase(std::remove(temp.begin(), temp.end(), '\n'), temp.end());
		res.push_back(temp);
		strs = strs.substr(pos + 1, strs.size());
		pos = strs.find(pattern);
	}

	return res;
}

int readPipe(void)
{
	int pipe_fd;
	/* Create the FIFO if it does not exist */
	mkfifo(READ_FIFO_PATH, 0666 | S_IFIFO);
	pipe_fd = open(READ_FIFO_PATH, O_RDONLY);

	while (1)
	{
		char readbuf[64] = {0};
		int readbytes = read(pipe_fd, readbuf, 64);
		if (readbytes <= 0)
			continue;

		printf("readbuf:%s,readbytes:%d.\n", readbuf, readbytes);
		vector<string> res = split(string(readbuf), ",");
		std::cout << "buffer:"
				  << "genWeightDataLimit=" << res[0] << ",batchNumber=" << res[1] << std::endl;
		genWeightDataLimit = atoi(res[0].c_str());
		batchNumber = res[1];
	}
	close(pipe_fd);
	return (0);
}

int writePipe(string buffer)
{
	int pipe_fd;
	/* Create the FIFO if it does not exist */
	mkfifo(WRITE_FIFO_PATH, 0666 | S_IFIFO);
	pipe_fd = open(WRITE_FIFO_PATH, O_WRONLY);

	write(pipe_fd, buffer.c_str(), buffer.size());

	close(pipe_fd);
	return (0);
}

int open_db(void)
{
	create_database(config.DataBase);

	//打开数据库
	if (create_table(sql_create_weight) != 0)
	{
		printf("warning sql_create_weight, pigBw already exist\n");
	}

	if (create_index(sql_create_index) != 0)
	{
		printf("warning sql_create_index index, weight index already exist\n");
	}

	return 0;
}

time_t stringToDatetime(const std::string &time_string)
{
	struct tm tm1;
	time_t time1;
	int i = sscanf(time_string.c_str(), "%d-%d-%d-%d-%d-%d",
				   &(tm1.tm_year),
				   &(tm1.tm_mon),
				   &(tm1.tm_mday),
				   &(tm1.tm_hour),
				   &(tm1.tm_min),
				   &(tm1.tm_sec),
				   &(tm1.tm_wday),
				   &(tm1.tm_yday));

	tm1.tm_year -= 1900;
	tm1.tm_mon--;
	tm1.tm_isdst = -1;
	time1 = mktime(&tm1);

	return time1;
}

bool loadConfig(string &config_file)
{
	//load config
	INIReader reader(config_file);
	if (reader.ParseError() < 0)
	{
		std::cout << "Can't load config file:" << config_file << std::endl;
		return false;
	}
	config.init(reader);
	std::cout << "Config loaded from:" << config_file << std::endl
			  << "***********************************\n"
			  << config.str()
			  << "***********************************\n";
	return true;
}

bool init(AXonLinkCamParam &camParam)
{
	/* 打开数据库，并创建表 */
	open_db();

	makeDir(FILE_DIR.c_str());

	// Initial OpenNI
	Status rc = OpenNI::initialize();
	if (rc != STATUS_OK)
	{
		printf("Initialize failed\n%s\n", OpenNI::getExtendedError());
		return false;
	}

	//Device mDevice;
	rc = mDevice.open(ANY_DEVICE);

	if (rc != STATUS_OK)
	{
		printf("Couldn't open device\n%s\n", OpenNI::getExtendedError());
		return false;
	}

	OniVersion drver;
	int nsize = sizeof(drver);
	int dataSize = sizeof(AXonLinkCamParam);
	mDevice.setDepthColorSyncEnabled(config.DepthColorSyncEnabled);
	mDevice.getProperty(ONI_DEVICE_PROPERTY_DRIVER_VERSION, &drver, &nsize);
	mDevice.getProperty(AXONLINK_DEVICE_PROPERTY_GET_CAMERA_PARAMETERS, &camParam, &dataSize);

	printf("AXon driver version V%d.%d.%d.%d\n", drver.major, drver.minor, drver.maintenance, drver.build);
	return true;
}

int printCamParam()
{
	openni::Status rc = openni::STATUS_OK;

	openni::Device device;
	openni::VideoStream depth, color, ir;
	int nResolutionColor = 0;
	int nResolutionDepth = 0;
	int lastResolutionX = 0;
	int lastResolutionY = 0;

	const char *deviceURI = openni::ANY_DEVICE;

	rc = openni::OpenNI::initialize();

	printf("After initialization:\n%s\n", openni::OpenNI::getExtendedError());

	rc = device.open(deviceURI);
	if (rc != openni::STATUS_OK)
	{
		printf("SimpleViewer: Device open failed:\n%s\n", openni::OpenNI::getExtendedError());
		openni::OpenNI::shutdown();
		return 1;
	}

	const openni::SensorInfo *info = device.getSensorInfo(openni::SENSOR_COLOR);
	if (info)
	{
		for (int i = 0; i < info->getSupportedVideoModes().getSize(); i++)
		{
			printf("Color info video %d %dx%d FPS %d f %d\n", i,
				   info->getSupportedVideoModes()[i].getResolutionX(),
				   info->getSupportedVideoModes()[i].getResolutionY(),
				   info->getSupportedVideoModes()[i].getFps(),
				   info->getSupportedVideoModes()[i].getPixelFormat());
			if ((info->getSupportedVideoModes()[i].getResolutionX() != lastResolutionX) || (info->getSupportedVideoModes()[i].getResolutionY() != lastResolutionY))
			{
				nResolutionColor++;
				lastResolutionX = info->getSupportedVideoModes()[i].getResolutionX();
				lastResolutionY = info->getSupportedVideoModes()[i].getResolutionY();
			}
		}
	}
	lastResolutionX = 0;
	lastResolutionY = 0;
	const openni::SensorInfo *depthinfo = device.getSensorInfo(openni::SENSOR_DEPTH);
	if (depthinfo)
	{
		for (int i = 0; i < depthinfo->getSupportedVideoModes().getSize(); i++)
		{
			printf("Depth info video %d %dx%d FPS %d f %d\n", i,
				   depthinfo->getSupportedVideoModes()[i].getResolutionX(),
				   depthinfo->getSupportedVideoModes()[i].getResolutionY(),
				   depthinfo->getSupportedVideoModes()[i].getFps(),
				   depthinfo->getSupportedVideoModes()[i].getPixelFormat());
			if ((depthinfo->getSupportedVideoModes()[i].getResolutionX() != lastResolutionX) || (depthinfo->getSupportedVideoModes()[i].getResolutionY() != lastResolutionY))
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
	for (int i = 0; i < nResolutionColor; i++)
	{
		printf("astColorParam x =%d\n", camParam.astColorParam[i].ResolutionX);
		printf("astColorParam y =%d\n", camParam.astColorParam[i].ResolutionY);
		printf("astColorParam fx =%.5f\n", camParam.astColorParam[i].fx);
		printf("astColorParam fy =%.5f\n", camParam.astColorParam[i].fy);
		printf("astColorParam cx =%.5f\n", camParam.astColorParam[i].cx);
		printf("astColorParam cy =%.5f\n", camParam.astColorParam[i].cy);
		printf("astColorParam k1 =%.5f\n", camParam.astColorParam[i].k1);
		printf("astColorParam k2 =%.5f\n", camParam.astColorParam[i].k2);
		printf("astColorParam p1 =%.5f\n", camParam.astColorParam[i].p1);
		printf("astColorParam p2 =%.5f\n", camParam.astColorParam[i].p2);
		printf("astColorParam k3 =%.5f\n", camParam.astColorParam[i].k3);
		printf("astColorParam k4 =%.5f\n", camParam.astColorParam[i].k4);
		printf("astColorParam k5 =%.5f\n", camParam.astColorParam[i].k5);
		printf("astColorParam k6 =%.5f\n", camParam.astColorParam[i].k6);
	}
	for (int i = 0; i < nResolutionDepth; i++)
	{
		printf("astDepthParam x =%d\n", camParam.astDepthParam[i].ResolutionX);
		printf("astDepthParam y =%d\n", camParam.astDepthParam[i].ResolutionY);
		printf("astDepthParam fx =%.5f\n", camParam.astDepthParam[i].fx);
		printf("astDepthParam fy =%.5f\n", camParam.astDepthParam[i].fy);
		printf("astDepthParam cx =%.5f\n", camParam.astDepthParam[i].cx);
		printf("astDepthParam cy =%.5f\n", camParam.astDepthParam[i].cy);
		printf("astDepthParam k1 =%.5f\n", camParam.astDepthParam[i].k1);
		printf("astDepthParam k2 =%.5f\n", camParam.astDepthParam[i].k2);
		printf("astDepthParam p1 =%.5f\n", camParam.astDepthParam[i].p1);
		printf("astDepthParam p2 =%.5f\n", camParam.astDepthParam[i].p2);
		printf("astDepthParam k3 =%.5f\n", camParam.astDepthParam[i].k3);
		printf("astDepthParam k4 =%.5f\n", camParam.astDepthParam[i].k4);
		printf("astDepthParam k5 =%.5f\n", camParam.astDepthParam[i].k5);
		printf("astDepthParam k6 =%.5f\n", camParam.astDepthParam[i].k6);
	}
	printf("R = %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f \n", camParam.stExtParam.R_Param[0], camParam.stExtParam.R_Param[1], camParam.stExtParam.R_Param[2], camParam.stExtParam.R_Param[3], camParam.stExtParam.R_Param[4], camParam.stExtParam.R_Param[5], camParam.stExtParam.R_Param[6], camParam.stExtParam.R_Param[7], camParam.stExtParam.R_Param[8]);
	printf("T = %.5f %.5f %.5f \n", camParam.stExtParam.T_Param[0], camParam.stExtParam.T_Param[1], camParam.stExtParam.T_Param[2]);
}

bool createColorStream()
{
	if (mDevice.hasSensor(openni::SENSOR_COLOR))
	{
		if (mColorStream.create(mDevice, openni::SENSOR_COLOR) == openni::STATUS_OK)
		{
			// set video mode
			openni::VideoMode mMode;
			mMode.setResolution(640, 480);
			mMode.setFps(30);
			mMode.setPixelFormat(openni::PIXEL_FORMAT_RGB888);

			if (mColorStream.setVideoMode(mMode) != openni::STATUS_OK)
			{
				std::cout << "Can't apply VideoMode: " << openni::OpenNI::getExtendedError() << std::endl;
				return false;
			}
		}
		else
		{
			std::cerr << "Can't create color stream on device: " << openni::OpenNI::getExtendedError() << std::endl;
			return false;
		}
		// start color stream
		mColorStream.start();
		return true;
	}
	return false;
}

bool createDepthStream()
{
	if (mDevice.hasSensor(openni::SENSOR_DEPTH))
	{
		if (mDepthStream.create(mDevice, openni::SENSOR_DEPTH) == openni::STATUS_OK)
		{
			// set video mode
			openni::VideoMode mMode;
			mMode.setResolution(640, 480);
			mMode.setFps(30);
			mMode.setPixelFormat(openni::PIXEL_FORMAT_DEPTH_1_MM);

			if (mDepthStream.setVideoMode(mMode) != openni::STATUS_OK)
			{
				std::cout << "Can't apply VideoMode to depth stream: " << openni::OpenNI::getExtendedError() << std::endl;
				return false;
			}
		}
		else
		{
			std::cerr << "Can't create depth stream on device: " << openni::OpenNI::getExtendedError() << std::endl;
			return false;
		}
		// start depth stream
		mDepthStream.start();
		// image registration
		if (mDevice.isImageRegistrationModeSupported(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR))
			mDevice.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
		else
			std::cerr << "Don't support registration" << std::endl;
		return true;
	}
	else
	{
		std::cerr << "ERROR: This device does not have depth sensor" << std::endl;
		return false;
	}
}

//openni图像流转化成点云
bool getCloudXYZCoordinate(PointCloud::Ptr cloud_XYZ)
{
	openni::VideoFrameRef mDepthFrame;

	if (mDepthStream.readFrame(&mDepthFrame) == openni::STATUS_OK)
	{
		float fx, fy, fz;
		int i = 0;
		//以米为单位
		double fScale = 0.001;
		openni::DepthPixel *pDepthArray = (openni::DepthPixel *)mDepthFrame.getData();
		for (int y = 0; y < mDepthFrame.getHeight(); y++)
		{
			for (int x = 0; x < mDepthFrame.getWidth(); x++)
			{
				int idx = x + y * mDepthFrame.getWidth();
				const openni::DepthPixel rDepth = pDepthArray[idx];
				openni::CoordinateConverter::convertDepthToWorld(mDepthStream, x, y, rDepth, &fx, &fy, &fz);
				fx = -fx;
				fy = -fy;
				cloud_XYZ->points[i].x = fx * fScale;
				cloud_XYZ->points[i].y = -fy * fScale;
				cloud_XYZ->points[i].z = -fz * fScale;
				i++;
			}
		}
		return true;
	}
	else
	{
		std::cout << "getCloudXYZCoordinate: fail to read frame from depth stream" << std::endl;
		return false;
	}
}

bool getCloudXYZRGBCoordinate(PointCloud::Ptr cloud_XYZRGB, AXonLinkCamParam &camParam, string file_depth_name, string file_depth_show_name)
{

	int changedStreamDummy;
	openni::VideoStream *pStream = &mDepthStream;
	if (OpenNI::waitForAnyStream(&pStream, 1, &changedStreamDummy, SAMPLE_READ_WAIT_TIMEOUT) != openni::STATUS_OK)
	{
		printf("Wait failed! (timeout is %d ms)\n%s\n", SAMPLE_READ_WAIT_TIMEOUT, OpenNI::getExtendedError());
		return false;
	}

	openni::VideoFrameRef colorFrame;
	mColorStream.readFrame(&colorFrame);
	openni::RGB888Pixel *pColor = (openni::RGB888Pixel *)colorFrame.getData();
	openni::VideoFrameRef mDepthFrame;

	if (mDepthStream.readFrame(&mDepthFrame) == openni::STATUS_OK)
	{
		//float fx, fy, fz;
		float px, py, pz;
		int i = 0;
		//以米为单位
		double fScale = 0.001;
		openni::DepthPixel *pDepthArray = (openni::DepthPixel *)mDepthFrame.getData();

		//save depth
		if (config.CollectFormats.find("depth") != string::npos)
		{
			cv::Mat image(480, 640, CV_16UC1, pDepthArray);
			vector<int> compression_params;
			compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
			compression_params.push_back(0);
			cv::imwrite(file_depth_name, image, compression_params);
			std::cout << "getCloudXYZCoordinate: save depth" << std::endl;

			//save depth show
			normalize(image, image, 0, 256 * 256, NORM_MINMAX);
			cv::imwrite(file_depth_show_name, image);
			std::cout << "getCloudXYZCoordinate: save depth show" << std::endl;
		}

		for (int y = 0; y < mDepthFrame.getHeight(); y++)
		{
			for (int x = 0; x < mDepthFrame.getWidth(); x++)
			{
				int idx = x + y * mDepthFrame.getWidth();
				const openni::DepthPixel rDepth = pDepthArray[idx];

				/*
				openni::CoordinateConverter::convertDepthToWorld(mDepthStream, x, y, rDepth, &fx, &fy, &fz);
				fx = -fx;
				fy = -fy;
				cloud_XYZRGB->points[i].x = fx * fScale;
				cloud_XYZRGB->points[i].y = fy * fScale;
				cloud_XYZRGB->points[i].z = fz * fScale;
				cloud_XYZRGB->points[i].r = pColor[i].r;
				cloud_XYZRGB->points[i].g = pColor[i].g;
				cloud_XYZRGB->points[i].b = pColor[i].b;
				*/

				pz = (float)rDepth;
				px = (x - camParam.astDepthParam->cx) * pz / camParam.astDepthParam->fx;
				py = (camParam.astDepthParam->cy - y) * pz / camParam.astDepthParam->fy;
				cloud_XYZRGB->points[i].x = px * fScale;
				cloud_XYZRGB->points[i].y = py * fScale;
				cloud_XYZRGB->points[i].z = pz * fScale;
				cloud_XYZRGB->points[i].r = pColor[i].r;
				cloud_XYZRGB->points[i].g = pColor[i].g;
				cloud_XYZRGB->points[i].b = pColor[i].b;

				i++;
			}
		}
		std::cout << "getCloudXYZCoordinate: end" << std::endl;
		return true;
	}
	else
	{
		std::cout << "getCloudXYZCoordinate: fail to read frame from depth stream" << std::endl;
		return false;
	}
}

std::string createJson(float weight, string time, string imgbase64)
{
	std::string jsonStr;
	Json::Value root, redate, predetec, compart;
	Json::StreamWriterBuilder writerBuilder;
	std::ostringstream os;
	weight = (int)(weight * 100 + 0.5) / 100.0;
	root["weight"] = weight;
	redate["__type"] = "Date";
	redate["iso"] = time;
	root["recorddate"] = redate;

	predetec["__type"] = "Pointer";
	predetec["className"] = "PredictWeightDetec";
	predetec["objectId"] = "mBiUU3c3dC";
	predetec["objectId"] = "mBiUU3c3dC";
	root["predictweightdetec"] = predetec;

	compart["__type"] = "Pointer";
	compart["className"] = "Compartment";
	compart["objectId"] = "J3pbGY58Ey";
	root["compartment"] = compart;

	root["pignum"] = 1;
	root["resultimg"] = imgbase64;
	std::unique_ptr<Json::StreamWriter> jsonWriter(writerBuilder.newStreamWriter());
	jsonWriter->write(root, &os);
	jsonStr = os.str();

	//std::cout << "Json:\n" << jsonStr << std::endl;
	return jsonStr;
}

void sigint_handler(int s)
{
	protonect_shutdown = true;
}
string int2string(int value)
{
	stringstream ss;
	ss << value;
	return ss.str();
}
string float2string(float value)
{
	stringstream ss;
	ss << value;
	return ss.str();
}
int set_interface_attribs(int fd, int speed)
{
	struct termios tty;

	if (tcgetattr(fd, &tty) < 0)
	{
		printf("Error from tcgetattr: %s\n", strerror(errno));
		return -1;
	}
	for (int i = 0; i < sizeof(speed_arr) / sizeof(int); i++)
	{
		if (speed == name_arr[i])
		{
			printf("speed=%d,name_arr=%d,speed_arr=%d\n", speed, name_arr[i], speed_arr[i]);
			cfsetospeed(&tty, (speed_t)speed_arr[i]);
			cfsetispeed(&tty, (speed_t)speed_arr[i]);

			tty.c_cflag |= (CLOCAL | CREAD); /* ignore modem controls */
			tty.c_cflag &= ~CSIZE;
			tty.c_cflag |= CS8;		 /* 8-bit characters */
			tty.c_cflag &= ~PARENB;	 /* no parity bit */
			tty.c_cflag &= ~CSTOPB;	 /* only need 1 stop bit */
			tty.c_cflag &= ~CRTSCTS; /* no hardware flowcontrol */

			/* setup for non-canonical mode */
			tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
			tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
			tty.c_oflag &= ~OPOST;

			/* fetch bytes as they become available */
			tty.c_cc[VMIN] = 1;
			tty.c_cc[VTIME] = 1;

			if (tcsetattr(fd, TCSANOW, &tty) != 0)
			{
				//printf("Error from tcsetattr: %s\n", strerror(errno));
				std::cout << "Error from tcsetattr" << std::endl;
				return -1;
			}
			break;
		}
	}

	return 0;
}

void set_mincount(int fd, int mcount)
{
	struct termios tty;

	if (tcgetattr(fd, &tty) < 0)
	{
		printf("Error tcgetattr: %s\n", strerror(errno));
		return;
	}

	tty.c_cc[VMIN] = mcount ? 1 : 0;
	tty.c_cc[VTIME] = 5; /* half second timer */

	if (tcsetattr(fd, TCSANOW, &tty) < 0)
		printf("Error tcsetattr: %s\n", strerror(errno));
}

//地磅重量数据
void *get_weight(void *arg)
{
	//连接串口
	int fd, nread, wlen;

	fd = open(config.SerialPort.data(), O_RDWR | O_NOCTTY | O_SYNC); //O_NONBLOCK | O_NDELAY
	if (fd == -1)
	{
		std::cout << "weight dev is not connected,SerialPort=" << config.SerialPort << std::endl;
	}
	else
	{
		std::cout << "weight dev is connected,SerialPort=" << config.SerialPort << std::endl;
	}

	set_interface_attribs(fd, config.Baund);

	wlen = write(fd, "Hello!\n", 7);
	std::cout << "write weighing device:" << wlen << std::endl;
	if (wlen != 7)
	{
		printf("Error from write: %d, %d\n", wlen, errno);
	}
	tcdrain(fd); /* delay for output */
	float lastweight = 0.0f;
	float tempweight = 0.0f;
	int weightdis = WEIGH_DIS;
	//地磅数据
	string weight_record;
	//临时数据，控制输出地磅数据频率
	string temp_record = "";
	string strFileName = "";

	while (!protonect_shutdown)
	{
		if (isGenWeightData)
			continue;

		pig_weight = tempweight;

		if (tempweight < config.WeightRangeMin || tempweight > config.WeightRangeMax)
		{
			lastweight = tempweight;
		}

		//写入重量数据
		time_t t = std::time(0);
		struct tm *now = std::localtime(&t);
		string time_record_file = int2string(now->tm_year + 1900) +
								  '-' + int2string(now->tm_mon + 1) +
								  '-' + int2string(now->tm_mday) +
								  '-' + int2string(now->tm_hour);
		bool bExit = time_record_file.compare(TIME_FILE);
		if (bExit == 0)
		{
			;
		}
		else
		{
			TIME_FILE = time_record_file;
			strFileName = FILE_DIR + time_record_file + strWeightFile;
		}

		ofstream weightfile(strFileName.c_str(), ios::out | ios::app);
		if (temp_record == "")
			temp_record = weight_record;
		if (!weightfile)
		{
			cout << strFileName << endl;
			cout << "Error opening file!" << endl;
		}
		else
		{

			weight_record = int2string(now->tm_year + 1900) +
							'-' + int2string(now->tm_mon + 1) +
							'-' + int2string(now->tm_mday) +
							'-' + int2string(now->tm_hour) +
							'-' + int2string(now->tm_min) +
							'-' + int2string(now->tm_sec) +
							':' + float2string(tempweight);

			if (weight_record != temp_record)
				weightfile << weight_record << std::endl;
			temp_record = weight_record;
		}
		weightfile.close();

		weightdis = fabs(tempweight - lastweight);
		lastweight = tempweight;

		if (weightdis <= WEIGH_DIS && now->tm_hour >= config.TimeRangeMin && now->tm_hour <= config.TimeRangeMax)
		{
			isWeighted = true;
		}
		else
		{
			isWeighted = false;
			continue;
		}
		int key = waitKey(1);
		protonect_shutdown = protonect_shutdown || (key > 0 && ((key & 0xFF) == 27)); // shutdown on escape
	}
	std::cout << "exit get_weight\n";
	return NULL;
}

void *get_img_bw(void *arg)
{
	//openni初始化、打开摄像头
	AXonLinkCamParam camParam;
	if (!init(camParam))
	{
		std::cout << "Fail to init ..." << std::endl;
		exit(-1);
	}
	//openni创建图像流
	if (createColorStream() && createDepthStream())
		std::cout << "displayPointCloud: create color stream and depth stream ..." << std::endl;
	else
	{
		std::cout << "displayPointCloud: can not create color stream and depth stream ..." << std::endl;
		exit(-1);
	}
	//创建pcl云
	std::cout << "displayPointCloud: create cloud..." << std::endl;
	PointCloud::Ptr cloud(new PointCloud());
	cloud->width = 640;
	cloud->height = 480;
	cloud->points.resize(cloud->width * cloud->height);
	//pcl可视化
	//std::cout << "displayPointCloud: create pcl viewer ..." << std::endl;
	//pcl::visualization::PCLVisualizer::Ptr m_pViewer(new pcl::visualization::PCLVisualizer("Viewer"));
	//pcl::visualization::CloudViewer m_pViewer ("Viewer");  //创建一个显示点云的窗口
	//m_pViewer->setCameraPosition(0, 0, -2, 0,-1, 0, 0);
	//m_pViewer->addCoordinateSystem(0.3);
	int frames_count = 0;
	int last_min = 0;
	float last_min_weight = 0;

	while (!protonect_shutdown)
	{
		char buffer[80] = {0};
		time_t t = std::time(0);
		struct tm *now = std::localtime(&t);
		string file_pcd_name, file_path, file_time, img_name, file_img_name, file_depth_name, file_depth_show_name;

		if (frames_count > INT_MAX)
		{
			frames_count = 0;
		}
		frames_count += 1;
		strftime(buffer, 80, config.FileNameFormat.c_str(), now);
		file_time = string(buffer) + '_' + int2string(frames_count) + '_' + float2string(pig_weight);
		file_pcd_name = config.PointCloudFilePath + "/" + file_time + ".pcd";
		file_img_name = config.RGBFilePath + "/" + file_time + ".jpg";
		file_depth_name = config.DepthFilePath + "/" + file_time + ".png";
		file_depth_show_name = config.DepthShowFilePath + "/" + file_time + ".png";

		PigBody_Size pgby_size;
		pgby_size.weight_gd = pig_weight;
		pgby_size.pcd_file_name = file_time + ".pcd";

		if (isGenWeightData)
		{
			if (genWeightDataLimit <= 0)
				continue;

			bool ret = getCloudXYZRGBCoordinate(cloud, camParam, file_depth_name, file_depth_show_name);
			if (ret)
			{
				std::cout << "getCloudXYZRGBCoordinate:file_depth_name=" << file_depth_name << "frames_count=" << frames_count << std::endl;

				std::cout << "begin save file:" << file_time << std::endl;
				if (config.CollectFormats.find("point") != string::npos)
				{
					pcl::io::savePCDFileBinary(file_pcd_name, *cloud);
				}

				if (config.CollectFormats.find("rgb") != string::npos)
				{
					pcl::io::savePNGFile(file_img_name, *cloud, "rgb");
				}
				std::cout << "end save file:" << file_time << std::endl;
				memset(buffer, 0, 80);
				strftime(buffer, 80, "%Y-%m-%d %H:%M:%S", now);
				table_insert_weight(string(buffer), pig_weight, file_time + ".jpg", file_time + ".pcd", file_time + ".png", batchNumber);
			}

			genWeightDataLimit--;
		}
		else
		{
			if (frames_count % config.CollectInterval == 0 && isWeighted && pig_weight > config.WeightRangeMin && pig_weight < config.WeightRangeMax)
			{
				std::cout << "save file:" << file_time << std::endl;
				isWeighted = false;
				getCloudXYZRGBCoordinate(cloud, camParam, file_depth_name, file_depth_show_name);
				if (now->tm_min == 59)
				{
					last_min = 0;
				}
				if (now->tm_min - 1 > last_min && pig_weight == last_min_weight)
				{
					if (last_min == 0)
					{
						last_min = now->tm_min - 1;
					}
					continue;
				}

				frames_count = 0;
				last_min = now->tm_min - 1;
				last_min_weight = pig_weight;
				if (config.CollectFormats.find("point") != string::npos)
				{
					pcl::io::savePCDFileBinary(file_pcd_name, *cloud);
				}

				if (config.CollectFormats.find("rgb") != string::npos)
				{
					pcl::io::savePNGFile(file_img_name, *cloud, "rgb");
				}
				batchNumber = string(buffer);
				memset(buffer, 0, 80);
				strftime(buffer, 80, "%Y-%m-%d %H:%M:%S", now);
				table_insert_weight(string(buffer), pig_weight, file_time + ".jpg", file_time + ".pcd", file_time + ".png", batchNumber);
			}
		}

		int key = waitKey(1);
		protonect_shutdown = protonect_shutdown || (key > 0 && ((key & 0xFF) == 27)); // shutdown on escape
	}
	std::cout << "exit get_img_bw begin\n";
	mColorStream.destroy();
	mDepthStream.destroy();
	mDevice.close();
	openni::OpenNI::shutdown();
	std::cout << "exit get_img_bw end\n";
}

int main(int argc, char **argv)
{
	int res, t;
	pthread_t img, weigh;
	if (argc < 3)
	{
		cout << "for example:./weight config_file start_type[collect|camParam|test limit batch_number]" << endl;
		exit(0);
	}

	string config_file(argv[1]);
	string start_type(argv[2]);

	if (start_type.compare("collect") == 0 || start_type.compare("test") == 0)
	{
		if (start_type.compare("test") == 0)
		{
			isGenWeightData = true;
			genWeightDataLimit = atoi(argv[3]);
			if (argc == 5)
			{
				batchNumber = string(argv[4]);
			}
		}
		loadConfig(config_file);
		cout << "Create pthread img!" << endl;
		res = pthread_create(&img, NULL, get_img_bw, NULL);
		if (res != 0)
		{
			cout << "Create pthread error!" << endl;
			return -1;
		}
		cout << "Create pthread img success!" << endl;

		cout << "Create pthread weigh!" << endl;
		res = pthread_create(&weigh, NULL, get_weight, NULL);
		if (res != 0)
		{
			cout << "Create pthread error!" << endl;
			return -1;
		}
		cout << "Create pthread weigh success!" << endl;

		readPipe();
		pthread_join(weigh, NULL);
		cout << "Join pthread weigh!" << endl;
		pthread_join(img, NULL);
		cout << "Join pthread img!" << endl;
		exit(0);
	}
	else if (start_type.compare("camParam") == 0)
	{
		printCamParam();
	}

	return 0;
}

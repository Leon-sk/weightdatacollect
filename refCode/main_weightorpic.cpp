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
using namespace std;
using namespace cv;

//using namespace cv;
using namespace openni;
//openni::Device mDevice;
Device mDevice;
openni::VideoStream mColorStream;
openni::VideoStream mDepthStream;

enum
{
	Processor_cl,
	Processor_gl,
	Processor_cpu
};

bool protonect_shutdown = false; // Whether the running application should shut down.
const int MIN_WEIGHT = 0;
const int MAX_WEIGHT = 100;
const int WEIGH_WAIT_TIME = 3;
const int WEIGH_DIS = 2;
bool isWeighted = false;
float pig_weight = 0;
//char* WEIGHT_FILE_NAME = "/home/gkj8/image/weight_records.txt";
char *weight_file_name = "/mnt/tf/weight_records.txt";
string FILE_DIR = "/mnt/tf/records/";
string TIME_FILE = "2019-7-1";
string strWeightFile = "weight_records.txt";

/* sqlite3 */
char sql_create_pigBw[1024] = "CREATE TABLE 'pigBw' ('id' INTEGER NOT NULL PRIMARY KEY, 'pcd_name' TEXT NOT NULL, 'time' DATETIME NOT NULL, 'imgbase_name' TEXT NOT NULL, 'area' INTEGER NOT NULL, 'len_body' INTEGER NOT NULL, 'waist_width' INTEGER NOT NULL, 'shoulder_width' INTEGER NOT NULL, 'hip_width' INTEGER NOT NULL, 'scale_box' INTEGER NOT NULL, 'symmetry_dis' INTEGER NOT NULL, 'weight_gd' INTEGER NOT NULL, 'w_estimate' INTEGER NOT NULL, 'wucha' INTEGER NOT NULL, 'box_w' INTEGER NOT NULL, 'box_h' INTEGER NOT NULL, 'box_d' INTEGER NOT NULL, 'symmetry' INTEGER NOT NULL, 'flag' INTEGER NOT NULL)";

char sql_create_index[200] = "CREATE INDEX 'pigBwId' ON 'pigBw' ('id')";

int open_db(void)
{
	create_database();

	//打开数据库
	if (create_table(sql_create_pigBw) != 0)
	{
		printf("warning sql_create_pigBw, pigBw already exist\n");
	}

	if (create_index(sql_create_index) != 0)
	{
		printf("warning sql_create_pigBw index, pigBw index already exist\n");
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
bool init()
{
	/* 打开数据库，并创建表 */
	open_db();

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
	int nsize;
	nsize = sizeof(drver);
	mDevice.getProperty(ONI_DEVICE_PROPERTY_DRIVER_VERSION, &drver, &nsize);
	printf("AXon driver version V%d.%d.%d.%d\n", drver.major, drver.minor, drver.maintenance, drver.build);
	return true;
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
bool getCloudXYZRGBCoordinate(PointCloud::Ptr cloud_XYZRGB)
{
	openni::VideoFrameRef colorFrame;
	mColorStream.readFrame(&colorFrame);
	openni::RGB888Pixel *pColor = (openni::RGB888Pixel *)colorFrame.getData();
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
				cloud_XYZRGB->points[i].x = fx * fScale;
				cloud_XYZRGB->points[i].y = fy * fScale;
				cloud_XYZRGB->points[i].z = fz * fScale;
				cloud_XYZRGB->points[i].r = pColor[i].r;
				cloud_XYZRGB->points[i].g = pColor[i].g;
				cloud_XYZRGB->points[i].b = pColor[i].b;
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
	//std::cout << "enter success" << std::endl;

	if (tcgetattr(fd, &tty) < 0)
	{
		printf("Error from tcgetattr: %s\n", strerror(errno));
		return -1;
	}

	cfsetospeed(&tty, (speed_t)speed);
	cfsetispeed(&tty, (speed_t)speed);

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
	char *dev = "/dev/ttyS3";
	//char *buf;
	fd = open(dev, O_RDWR | O_NOCTTY | O_SYNC); //O_NONBLOCK | O_NDELAY
	if (fd == -1)
	{
		std::cout << "dev is not connected!" << std::endl;
	}
	set_interface_attribs(fd, B9600);
	wlen = write(fd, "Hello!\n", 7);
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
	do
	{
		unsigned char buf[14];
		int rdlen;
		rdlen = read(fd, buf, sizeof(buf) - 1);
		if (rdlen > 0)
		{
			;
		}
		else if (rdlen < 0)
		{
			printf("Error from read: %d: %s\n", rdlen, strerror(errno));
		}

		string temp((char *)buf);
		string temp1(temp, 2, 7);
		tempweight = atof(temp1.c_str());
		pig_weight = tempweight;
		if (tempweight < MIN_WEIGHT || tempweight > MAX_WEIGHT)
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

		if (weightdis <= WEIGH_DIS) //== 0)
		{
			isWeighted = true;
		}
		else
		{
			isWeighted = false;
			continue;
		}
		/* repeat read to get full message */
	} while (1);
	return NULL;
}

void *get_img_bw(void *arg)
{
	//openni初始化、打开摄像头
	if (!init())
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
	std::cout << "displayPointCloud: ..." << std::endl;
	PointCloud::Ptr cloud(new PointCloud());
	cloud->width = 640;
	cloud->height = 480;
	cloud->points.resize(cloud->width * cloud->height);
	//pcl可视化
	std::cout << "displayPointCloud: create color stream and depth stream ..." << std::endl;
	//pcl::visualization::PCLVisualizer::Ptr m_pViewer(new pcl::visualization::PCLVisualizer("Viewer"));
	std::cout << "displayPointCloud: depth stream ...111" << std::endl;
	//pcl::visualization::CloudViewer m_pViewer ("Viewer");  //创建一个显示点云的窗口
	std::cout << "displayPointCloud: depth stream ...222" << std::endl;
	//m_pViewer->setCameraPosition(0, 0, -2, 0,-1, 0, 0);
	//m_pViewer->addCoordinateSystem(0.3);
	std::cout << "displayPointCloud: depth stream ..." << std::endl;
	int frames_count = 0;
	int last_min = 0;
	float last_min_weight = 0;
	ofstream oFile;
	//float weight_gd;
	//打开要保存数据的csv文件
	oFile.open("/mnt/tf/pig_bw_0.csv", ios::out | ios::trunc);
	//oFile<< "pcd_name"<<","<<"area" << "," << "w_gd" << "," << "w_et" <<","<<"error"<< ","<<"center_x"<<","<<"center_y"<<","<<"center_z"<<endl;
	oFile << "pcd_name"
		  << ","
		  << "area"
		  << ","
		  << "len_body"
		  << ","
		  << "waist_width"
		  << ","
		  << "shoulder_width"
		  << ","
		  << "hip_width"
		  << ","
		  << "w_gd"
		  << ","
		  << "w_estimate"
		  << ","
		  << "error"
		  << ","
		  << "box_w"
		  << ","
		  << "box_h"
		  << ","
		  << "box_d"
		  << ","
		  << "box_w/h"
		  << ","
		  << "symmetry"
		  << ","
		  << "flag" << endl;
	PigBodySize pbs;
	cv::Mat img;
	while (!protonect_shutdown)
	{
		time_t t = std::time(0);
		struct tm *now = std::localtime(&t);
		string file_pcd_name, file_path, file_time, img_name, file_img_name;

		frames_count = frames_count + 1;
		//std::cout << " create color stream and depth stream ..." << std::endl;
		file_path = "/mnt/tf/Axon_data/";
		file_time = int2string(now->tm_year + 1900) +
					'-' + int2string(now->tm_mon + 1) +
					'-' + int2string(now->tm_mday) +
					'-' + int2string(now->tm_hour) +
					'-' + int2string(now->tm_min) +
					'-' + int2string(now->tm_sec) +
					'-' + float2string(pig_weight);
		file_pcd_name = file_path + file_time + ".pcd";
		file_img_name = file_path + file_time + ".jpg";
		string time_now = int2string(now->tm_year + 1900) +
						  '-' + int2string(now->tm_mon + 1) +
						  '-' + int2string(now->tm_mday) +
						  ' ' + int2string(now->tm_hour) +
						  ':' + int2string(now->tm_min) +
						  ':' + int2string(now->tm_sec);
		PigBody_Size pgby_size;
		pgby_size.weight_gd = pig_weight;
		pgby_size.pcd_file_name = file_time + ".pcd";
		if (frames_count % 100 == 0 && isWeighted && pig_weight > MIN_WEIGHT && pig_weight < MAX_WEIGHT) //默认frames_count%100
		{
			isWeighted = false;
			getCloudXYZRGBCoordinate(cloud);
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
			last_min = now->tm_min - 1;
			last_min_weight = pig_weight;
			//cout << "save" << endl;
			pcl::io::savePCDFileASCII(file_pcd_name, *cloud); //测试用，安装好后注释掉。采集时也用
			pcl::io::savePNGFile(file_img_name, *cloud, "rgb");
		}

		int key = waitKey(1);
		protonect_shutdown = protonect_shutdown || (key > 0 && ((key & 0xFF) == 27)); // shutdown on escape
	}
	mColorStream.destroy();
	mDepthStream.destroy();
	mDevice.close();
	openni::OpenNI::shutdown();
}
int main()
{
	int res, t;
	pthread_t img, weigh;
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
	pthread_join(img, NULL);
	cout << "Join pthread img!" << endl;
	pthread_join(weigh, NULL);
	cout << "Join pthread weigh!" << endl;
	pthread_exit(NULL);
	return 0;
}

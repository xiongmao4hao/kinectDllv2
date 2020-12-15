// pch.cpp: 与预编译标头对应的源文件
#include "pch.h"

// 当使用预编译的头时，需要使用此源文件，编译才能成功。

#include "kinect_record.h"
#include "kinectDLL.h"

using namespace cv;
using namespace std;

Kinct kinct;
cv::Mat cmatData;

Kinct::Kinct()
{
	reKinct();
	cout << "Kinct cause done" << endl;
}

Kinct::~Kinct()
{
	del();
	cout << "Kinct delete done" << endl;
}


int Kinct::onePicture(const int i, k4a_capture_t sensor_capture, float(&fAngelUsing)[ANGLE_NUM], k4abt_tracker_t& tracker, uint64_t& timeStamp)
{
	//为了保存单独一个相机的关节角度建立的数组
	float joints_Angel[ANGLE_NUM];
	for (int i = 0; i < ANGLE_NUM; i++) joints_Angel[i] = NULL;
	//主要功能实现所用变量
	k4a_float3_t tf_target_color;
	//捕获并写入人体骨架
	k4a_wait_result_t queue_capture_result = \
		k4abt_tracker_enqueue_capture(tracker, sensor_capture, K4A_WAIT_INFINITE);//异步提取骨骼信息
	if (queue_capture_result == K4A_WAIT_RESULT_TIMEOUT)// && queue_capture_result1 == K4A_WAIT_RESULT_TIMEOUT)
	{
		// It should never hit timeout when K4A_WAIT_INFINITE is set.
		cout << "Error! Add capture to tracker process queue timeout!\n" << endl;
	}
	else if (queue_capture_result == K4A_WAIT_RESULT_FAILED)// && queue_capture_result1 == K4A_WAIT_RESULT_FAILED)
	{
		cout << "Error! Add capture to tracker process queue failed!\n" << endl;
	}
	else
	{
		k4abt_frame_t body_frame = NULL;
		k4a_wait_result_t pop_frame_result = \
			k4abt_tracker_pop_result(tracker, &body_frame, K4A_WAIT_INFINITE);
		timeStamp = k4abt_frame_get_device_timestamp_usec(body_frame);//获取时间戳
		// 骨骼点数据清除和计算人数
		uint32_t numBodies = k4abt_frame_get_num_bodies(body_frame);


		if (pop_frame_result == K4A_WAIT_RESULT_SUCCEEDED)
		{
			//Get the number of detecied human bodies
			//size_t num_bodies = k4abt_frame_get_num_bodies(body_frame0);
			k4a_result_t get_body_skeleton = \
				k4abt_frame_get_body_skeleton(body_frame, 0, &skeleton);

			if (get_body_skeleton == K4A_RESULT_SUCCEEDED)
			{
				cout << "have joints\n" << endl;
				////***************求角度*******************
				JointsPositionToAngel(skeleton, &joints_Angel);//必须传入地址&，joints_Angel虽然值相同但是数据类型有问题
				//保存关节点数据融合结果
				/*std::unique_lock<std::mutex> locker_r(Flagangle1to1);*/
				for (int i = 0; i < ANGLE_NUM; i++) fAmgleALL[i] = joints_Angel[i];
				//for (int i = 0; i < ANGLE_NUM; i++) fAngelUsing[i] += joints_Angel[i];//进行结果的累加
				//if (iFlagAngle1to1 != 0 || i == iMasterNum && iFlagMaster < 2)//在拍照记录抵消（即主设备和从设备都拍完一张，flagangle1to1 == 0）时进行关节角平均（else操作）。前提是这不是主设备。
				//{
				//	if (i == iMasterNum)
				//	{
				//		iFlagAngle1to1 += uintNum - 1;
				//		iFlagMaster += 1;//计数master个数，如果超过2个代表从设备未捕捉到
				//	}
				//	else
				//		iFlagAngle1to1 -= 1;
				//	locker_r.unlock();
				//}
				//else
				//{
				//	locker_r.unlock();
				//	if (iFlagMaster == 1 || uintNum == 1)//master个数，如果超过2个代表从设备未捕捉到（但是单摄像机时不一样）
				//	{
				//		for (int i = 0; i < ANGLE_NUM; i++) fAmgleALL[i] = fAngelUsing[i] / uintNum;
				//		bOnePicture = true;
				//		for (int i = 0; i < ANGLE_NUM; i++) fAngelUsing[i] = 0;
				//	}
				//	else
				//	{
				//		//以下两句在下一个主设备拍到前都会进入这个else
				//		iFlagMaster = 0;
				//		iFlagAngle1to1 = 0;
				//		//以下一句保证初始化累加数组
				//		for (int i = 0; i < ANGLE_NUM; i++) fAngelUsing[i] = 0;
				//	}
				//}
				////****************************************
				//for (int i = 0; i < BODU_POINT_NUM; i++)
				//{
				//	// write the raw cordinates into the txt file
				//	/*tf_source_depth.xyz.x = skeleton.joints[i].position.xyz.x;
				//	tf_source_depth.xyz.y = skeleton.joints[i].position.xyz.y;
				//	tf_source_depth.xyz.z = skeleton.joints[i].position.xyz.z;*/
				//	tf_target_color.xyz.x = skeleton.joints[i].position.xyz.x;
				//	tf_target_color.xyz.y = skeleton.joints[i].position.xyz.y;
				//	tf_target_color.xyz.z = skeleton.joints[i].position.xyz.z;

				//}
			}
			else if (get_body_skeleton == K4A_RESULT_FAILED)
			{
				cout << "Get body skeleton failed!!\n" << endl;
			}
			uint32_t id = k4abt_frame_get_body_id(body_frame, 1);
			k4abt_frame_release(body_frame);
		}
		else if (pop_frame_result == K4A_WAIT_RESULT_TIMEOUT)
		{
			//  It should never hit timeout when K4A_WAIT_INFINITE is set.
			cout << "Error! Pop body frame result timeout!\n" << endl;
			return 1;
		}
		else
		{
			cout << "Pop body frame result failed!\n" << endl;
			return 1;
		}
	}
	bOnePicture = false;
	cout << "have recorded\n" <<endl;
	return 0;
}

void Kinct::cap(k4a_device_t& dev, cv::Mat& colorFrame, const int i,k4a_calibration_t &sensorCalibration,\
	float(&fAngelUsing)[ANGLE_NUM], uint64_t& timeStamp)  //普通的函数，用来执行线程
{
	k4a_image_t colorImage;\
	uint8_t* colorTextureBuffer;
	k4a_capture_t sensor_capture;//捕获用变量
	bool capTidFlag = false;
	thread capTid;

	

	//关节点追踪变量tracker的建立
	k4abt_tracker_t tracker = NULL;
	k4abt_tracker_configuration_t tracker_config = K4ABT_TRACKER_CONFIG_DEFAULT;
	VERIFY(k4abt_tracker_create(&sensorCalibration, tracker_config, &tracker), "Body tracker initialization failed!");
	while (1)
	{
		if (k4a_device_get_capture(dev, &sensor_capture, K4A_WAIT_INFINITE) == K4A_WAIT_RESULT_SUCCEEDED)
		{
			std::unique_lock<std::mutex> locker(onePictureFlag[0]);
			colorImage = k4a_capture_get_color_image(sensor_capture);//从捕获中获取图像
			if (bOnePicture)
			{
				capTidFlag = true;
				capTid = thread(&Kinct::onePicture, this, i, sensor_capture, ref(fAngelUsing), ref(tracker), ref(timeStamp));
			}
			colorTextureBuffer = k4a_image_get_buffer(colorImage);

			//depthFrame = cv::Mat(depthImage.get_height_pixels(), depthImage.get_width_pixels(), CV_8UC4, depthTextureBuffer.data());
			colorFrame = cv::Mat(1, k4a_image_get_height_pixels(colorImage) * k4a_image_get_width_pixels(colorImage), CV_8UC1, colorTextureBuffer);
			colorFrame = imdecode(colorFrame, IMREAD_COLOR);
			k4a_image_release(colorImage);
			cvtColor(colorFrame, colorFrame, COLOR_BGRA2BGR);

			if (colorFrame.data == NULL)
			{
				cout << "colorframe imdecode erro" << endl;
			}

			//imshow("Kinect color frame" + std::to_string(i), colorFrame);
			//waitKey(1);//窗口的要等待时间，当显示图片时，窗口不用实时更新，所以imshow之前不加waitKey也是可以的，但若显示实时的视频，就必须加waitKey
			if (capTidFlag)
			{
				capTid.join();
				capTidFlag = false;
			}
			locker.unlock();
			k4a_capture_release(sensor_capture);
		}
		else if (k4a_device_get_capture(dev, &sensor_capture, K4A_WAIT_INFINITE) == K4A_WAIT_RESULT_FAILED)
			VERIFY(k4a_device_get_capture(dev, &sensor_capture, 10), "Capture failed");
		if (bDel)
		{
			k4a_device_stop_cameras(dev);//停止流
			k4abt_tracker_shutdown(tracker);//关闭捕捉
			k4abt_tracker_destroy(tracker);
			k4a_device_close(dev);
			break;
		}
	}
}

int Kinct::init()
{
	//相机启动
	uintNum = k4a::device::get_installed_count();
	cout << uintNum << endl;
	if (uintNum == 0)
	{
		cout << "no azure kinect dk devices detected!" << endl;
		return 1;
	}
	//初始化为NULL但是会造成程序在k4a_record_close时报错，于是在后续的if中解决了此问题
	dev = new k4a_device_t[uintNum];
	k4a_device_configuration_t* config = new k4a_device_configuration_t[uintNum];
	sensorCalibration = new k4a_calibration_t[uintNum];

	for (uint8_t deviceIndex = 0; deviceIndex < uintNum; deviceIndex++)
	{
		if (K4A_RESULT_SUCCEEDED != k4a_device_open(deviceIndex, &dev[deviceIndex]))
		{
			printf("%d: Failed to open device\n", deviceIndex);
			return 1;
		}
		config[deviceIndex] = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
		config[deviceIndex].camera_fps = K4A_FRAMES_PER_SECOND_30;
		config[deviceIndex].depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
		config[deviceIndex].color_format = K4A_IMAGE_FORMAT_COLOR_MJPG;
		config[deviceIndex].color_resolution = K4A_COLOR_RESOLUTION_720P;
		config[deviceIndex].synchronized_images_only = true;
		bool sync_in, sync_out;
		VERIFY(k4a_device_get_sync_jack(dev[deviceIndex], &sync_in, &sync_out), "get sync jack failed");
		if (sync_in == true)
		{
			cout << "subordinate device detected!" << endl;
			config[deviceIndex].wired_sync_mode = K4A_WIRED_SYNC_MODE_SUBORDINATE;
		}
		else if (sync_out == true)
		{
			cout << "master device detected!" << endl;
			iMasterNum = (int)deviceIndex;
			config[deviceIndex].wired_sync_mode = K4A_WIRED_SYNC_MODE_MASTER;
		}
		else
		{
			cout << "standalone device detected!" << endl;
			iMasterNum = 0;
			config[deviceIndex].wired_sync_mode = K4A_WIRED_SYNC_MODE_STANDALONE;
		}

		cout << "started opening k4a device..." << endl;
		VERIFY(k4a_device_start_cameras(dev[deviceIndex], &config[deviceIndex]), "Start K4A cameras failed!");//启动
		//校准设备
		VERIFY(k4a_device_get_calibration(dev[deviceIndex], config[deviceIndex].depth_mode, config[deviceIndex].color_resolution, &sensorCalibration[deviceIndex]),
			"Get depth camera calibration failed!")
		VERIFY(k4a_device_set_color_control(dev[deviceIndex], K4A_COLOR_CONTROL_BRIGHTNESS, K4A_COLOR_CONTROL_MODE_MANUAL, 150), "color brightness control failed");//手动设置曝光
		cout << "finished opening k4a device!\n" << endl;
	}

	//初始化成功标志
	bInitFlag = true;
	return 0;
}

int Kinct::del()
{
	if (bInitFlag)
	{
		bDel = true;//通知关闭进程
		//已有线程等待停止
		for (uint i = 0; i < uintNum; i++)
		{
			tids[i].join();
		}
	}
	reKinct();//重置Kinct
	return 0;
}

void Kinct::reKinct()
{
	//释放内存,new后最好做，虽然进程结束后都会回收
	delete[] tids;
	delete[] colorFrame;
	delete[] sensorCalibration;
	delete[] dev;
	delete[] timeStamp;
	//进程关闭flag重置
	bOnePicture = false;
	bDel = false;
	bInitFlag = false;
	uintNum = 0;
	tids = nullptr;
	dev = nullptr;
	sensorCalibration = nullptr;
	colorFrame = nullptr;
	onePictureFlag = nullptr;
	timeStamp = nullptr;
}

int Kinct::capThread()
{
	//创建的可变变量
	colorFrame = new cv::Mat[uintNum];
	tids = new thread[uintNum];
	onePictureFlag = new std::mutex[uintNum];
	timeStamp = new uint64_t[uintNum];
	//拍摄相关标志初始化
	iFlagAngle1to1 = 0;
	iFlagMaster = 0;










	for (int i = 0; i < ANGLE_NUM; i++) fAngelUsing[i] = NULL;
	//创建线程
	for (uint i = 0; i < uintNum; ++i)
	{
		tids[i] = thread(&Kinct::cap, this, ref(dev[i]), ref(colorFrame[i]), i, ref(sensorCalibration[i]), ref(fAngelUsing), ref(timeStamp[i]));
	}
	return 0;
}


int Kinct::getAngle(float* const fAngle)
{
	for (int i = 0; i < 18; i++)
	{
		fAngle[i] = fAmgleALL[i];
	}
	return 0;
}


int Kinct::getJoint(float* const fJoint)
{
	fJoint[0] = timeStamp[0];
	for (int i = 0; i < 32; i++)
	{
		fJoint[i*3 + 1] = skeleton.joints[i].position.xyz.x;//传递的为向量的数组指针
		fJoint[i*3 + 2] = skeleton.joints[i].position.xyz.y;//传递的为向量的数组指针
		fJoint[i*3 + 3] = skeleton.joints[i].position.xyz.z;//传递的为向量的数组指针
	}
	return 0;
}


int Kinct::getCmat(cv::Mat& oneMat)
{
	oneMat = colorFrame[0].clone();
	return 0;
}

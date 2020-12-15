#include "kinect_record.h"
#include "kinectDLL.h"

using namespace cv;
using namespace std;

int Observer::static_number_ = 0;

kinectSubject::kinectSubject() {
    std::cout << "hi, I was the kinectSubject.\n";
	reKinct();
	cout << "Kinct cause done" << endl;
  }

kinectSubject::~kinectSubject() {
    std::cout << "Goodbye, I was the kinectSubject.\n";
    del();
	cout << "Kinct delete done" << endl;
  }

int kinectSubject::recordStart()
{
	recordStop();
	VERIFY(init(), "kinect inint failed!");//初始化,启动相机
	return 0;
}

int kinectSubject::recordStop()
{
	VERIFY(del(), "kinect del failed!");//关闭相机捕获
	return 0;
}

int kinectSubject::init()
{
	//相机启动
	uintNum_ = k4a::device::get_installed_count();
	cout << uintNum_ << endl;
	if (uintNum_ == 0)
	{
		cout << "no azure kinect dk devices detected!" << endl;
		return 1;
	}
	//初始化为NULL但是会造成程序在k4a_record_close时报错，于是在后续的if中解决了此问题
	dev_ = new k4a_device_t[uintNum_];
	mtx_ = new mutex[uintNum_];
	k4a_device_configuration_t* config = new k4a_device_configuration_t[uintNum_];
	sensorCalibration_ = new k4a_calibration_t[uintNum_];

	for (uint8_t deviceIndex = 0; deviceIndex < uintNum_; deviceIndex++)
	{
		if (K4A_RESULT_SUCCEEDED != k4a_device_open(deviceIndex, &dev_[deviceIndex]))
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
		VERIFY(k4a_device_get_sync_jack(dev_[deviceIndex], &sync_in, &sync_out), "get sync jack failed");
		if (sync_in == true)
		{
			cout << "subordinate device detected!" << endl;
			config[deviceIndex].wired_sync_mode = K4A_WIRED_SYNC_MODE_SUBORDINATE;
		}
		else if (sync_out == true)
		{
			cout << "master device detected!" << endl;
			iMasterNum_ = (int)deviceIndex;
			config[deviceIndex].wired_sync_mode = K4A_WIRED_SYNC_MODE_MASTER;
		}
		else
		{
			cout << "standalone device detected!" << endl;
			iMasterNum_ = 0;
			config[deviceIndex].wired_sync_mode = K4A_WIRED_SYNC_MODE_STANDALONE;
		}

		cout << "started opening k4a device..." << endl;
		VERIFY(k4a_device_start_cameras(dev_[deviceIndex], &config[deviceIndex]), "Start K4A cameras failed!");//启动
		//校准设备
		VERIFY(k4a_device_get_calibration(dev_[deviceIndex], config[deviceIndex].depth_mode, config[deviceIndex].color_resolution, &sensorCalibration_[deviceIndex]),
			"Get depth camera calibration failed!")
		VERIFY(k4a_device_set_color_control(dev_[deviceIndex], K4A_COLOR_CONTROL_BRIGHTNESS, K4A_COLOR_CONTROL_MODE_MANUAL, 150), "color brightness control failed");//手动设置曝光
		cout << "finished opening k4a device!\n" << endl;
	}

	//初始化成功标志
	bInitFlag_ = true;
	return 0;
}

int kinectSubject::del()
{
	if (bInitFlag_)
	{
		bDel_ = true;//通知关闭进程
		//已有线程等待停止
		for (uint i = 0; i < uintNum_; i++)
		{
			tids_[i].join();
		}
	}
	reKinct();//重置Kinct
	return 0;
}

int kinectSubject::reKinct()
{
	uintNum_ = iMasterNum_ = 0;
	//释放内存,new后最好做，虽然进程结束后都会回收
	delete[] tids_;
	delete[] sensorCalibration_;
	delete[] dev_;
	//进程关闭flag重置
	bDel_ = false;
	bInitFlag_ = false;
	tids_ = nullptr;
	dev_ = nullptr;
	sensorCalibration_ = nullptr;
	return 0;
}

void kinectSubject::cap(k4a_device_t& dev, const int i, const k4a_calibration_t& sensorCalibration)  //普通的函数，用来执行线程
{
	k4a_image_t colorImage;
	uint8_t* colorTextureBuffer;
	oneElement element;

	//注意要在外部调用库时依据kinect数目创建
	std::list<IObserver*>::iterator iterator = list_observer_.begin();
	if (uintNum_ != list_observer_.size())
	{
		printf("no matching observe：\n");
		HowManyObserver();
		cout << "while " << i << "kinect" << endl;
		exit(1);
	}
	else {
		for (int j = i; j > 0; --j) {
			++iterator;
		}
	}
	//关节点追踪变量tracker的建立
	k4abt_tracker_t tracker = NULL;
	k4abt_tracker_configuration_t tracker_config = K4ABT_TRACKER_CONFIG_DEFAULT;
	VERIFY(k4abt_tracker_create(&sensorCalibration, tracker_config, &tracker), "Body tracker initialization failed!");
	while (1)
	{
		if (k4a_device_get_capture(dev, &element.sensor_capture, K4A_WAIT_INFINITE) == K4A_WAIT_RESULT_SUCCEEDED)
		{
			colorImage = k4a_capture_get_color_image(element.sensor_capture);//从捕获中获取图像
			colorTextureBuffer = k4a_image_get_buffer(colorImage);

			//depthFrame = cv::Mat(depthImage.get_height_pixels(), depthImage.get_width_pixels(), CV_8UC4, depthTextureBuffer.data());
			std::unique_lock<std::mutex> locker(mtx_[i]);
			element.colorFrame = cv::Mat(1, k4a_image_get_height_pixels(colorImage) * k4a_image_get_width_pixels(colorImage), CV_8UC1, colorTextureBuffer);
			element.colorFrame = imdecode(element.colorFrame, IMREAD_COLOR);
			k4a_image_release(colorImage);
			cvtColor(element.colorFrame, element.colorFrame, COLOR_BGRA2BGR);
			locker.unlock();
			if (element.colorFrame.data == NULL)
			{
				cout << "colorframe imdecode erro" << endl;
			}
			onePicture(tracker, &element, iterator);
			//imshow("Kinect color frame" + std::to_string(i), colorFrame);
			//waitKey(1);//窗口的要等待时间，当显示图片时，窗口不用实时更新，所以imshow之前不加waitKey也是可以的，但若显示实时的视频，就必须加waitKey
			k4a_capture_release(element.sensor_capture);
		}
		else if (k4a_device_get_capture(dev, &element.sensor_capture, K4A_WAIT_INFINITE) == K4A_WAIT_RESULT_FAILED)
			VERIFY(k4a_device_get_capture(dev, &element.sensor_capture, 10), "Capture failed");
		if (bDel_)
		{
			k4a_device_stop_cameras(dev);//停止流
			k4abt_tracker_shutdown(tracker);//关闭捕捉
			k4abt_tracker_destroy(tracker);
			k4a_device_close(dev);
			break;
		}
	}
}

int kinectSubject::capThread()
{
	tids_ = new thread[uintNum_];
	for (uint i = 0; i < uintNum_; ++i)
	{
		tids_[i] = thread(&kinectSubject::cap, this, ref(dev_[i]), i, ref(sensorCalibration_[i]));
	}
	return 0;
}

int kinectSubject::onePicture(k4abt_tracker_t& tracker, \
	oneElement* const element, std::list<IObserver*>::iterator iterator)
{
	//捕获并写入人体骨架
	k4a_wait_result_t queue_capture_result = \
		k4abt_tracker_enqueue_capture(tracker, element->sensor_capture, K4A_WAIT_INFINITE);//异步提取骨骼信息
	if (queue_capture_result == K4A_WAIT_RESULT_TIMEOUT)
	{
		// It should never hit timeout when K4A_WAIT_INFINITE is set.
		cout << "Error! Add capture to tracker process queue timeout!\n" << endl;
		(*iterator)->OnlyShowMat(element->colorFrame);
	}
	else if (queue_capture_result == K4A_WAIT_RESULT_FAILED)
	{
		cout << "Error! Add capture to tracker process queue failed!\n" << endl;
		(*iterator)->OnlyShowMat(element->colorFrame);
	}
	else
	{
		
		k4a_wait_result_t pop_frame_result = \
			k4abt_tracker_pop_result(tracker, &element->body_frame, K4A_WAIT_INFINITE);
		element->timeStamp = k4abt_frame_get_device_timestamp_usec(element->body_frame);//获取时间戳
		// 骨骼点数据清除和计算人数
		uint32_t numBodies = k4abt_frame_get_num_bodies(element->body_frame);
		if (pop_frame_result == K4A_WAIT_RESULT_SUCCEEDED)
		{
			//Get the number of detecied human bodies
			//size_t num_bodies = k4abt_frame_get_num_bodies(body_frame0);
			k4a_result_t get_body_skeleton = \
				k4abt_frame_get_body_skeleton(element->body_frame, 0, &element->skeleton);

			if (get_body_skeleton == K4A_RESULT_SUCCEEDED)
			{
				cout << "have joints\n" << endl;
				/***************求角度*******************/
				JointsPositionToAngel(element->skeleton, &element->joints_Angel);//必须传入地址&，joints_Angel虽然值相同但是数据类型有问题
			}
			else if (get_body_skeleton == K4A_RESULT_FAILED)
			{
				cout << "Get body skeleton failed!!\n" << endl;
			}
			//获取kinect的人体ID
			/*uint32_t id = k4abt_frame_get_body_id(element->body_frame, 1);*/
			k4abt_frame_release(element->body_frame);
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
		(*iterator)->Update(element);
	}
	return 0;
}
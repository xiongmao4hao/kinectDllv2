// pch.cpp: 与预编译标头对应的源文件
#include "pch.h"

// 当使用预编译的头时，需要使用此源文件，编译才能成功。

#include "kinect_record.h"
#include "kinectDLL.h"

extern Kinct kinct;
extern cv::Mat cmatData;

using namespace cv;
using namespace std;


int recordStart()
{
	recordStop();
	VERIFY(kinct.init(), "kinect inint failed!");//初始化,启动相机
	kinct.capThread();//创建捕获线程
	return 0;
}


int recordStop()
{
	VERIFY(kinct.del(), "kinect del failed!");//关闭相机捕获
	return 0;
}


uchar* recordY(int& rows, int& cols, int& channels, SkeletonFrame& skeletonFrame)
{
	kinct.bOnePicture = true;
	uchar* frameDataPtr;
	
	while (1)
	{
		if (!kinct.bOnePicture)
		{
			std::unique_lock<std::mutex> locker(kinct.onePictureFlag[0]);
			VERIFY(kinct.getAngle(skeletonFrame.skeletonAngle), " angle fail....");
			VERIFY(kinct.getJoint(skeletonFrame.skeletonPoint), " joint fail....");

			kinct.getCmat(cmatData);
			locker.unlock();
			
			frameDataPtr = cmatData.data;
			channels = cmatData.channels();
			cols = cmatData.cols;
			rows = cmatData.rows;
			break;
		}
	}
	return frameDataPtr;
}



// pch.cpp: ��Ԥ�����ͷ��Ӧ��Դ�ļ�
#include "pch.h"

// ��ʹ��Ԥ�����ͷʱ����Ҫʹ�ô�Դ�ļ���������ܳɹ���

#include "kinect_record.h"
#include "kinectDLL.h"

extern Kinct kinct;
extern cv::Mat cmatData;

using namespace cv;
using namespace std;


int recordStart()
{
	recordStop();
	VERIFY(kinct.init(), "kinect inint failed!");//��ʼ��,�������
	kinct.capThread();//���������߳�
	return 0;
}


int recordStop()
{
	VERIFY(kinct.del(), "kinect del failed!");//�ر��������
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



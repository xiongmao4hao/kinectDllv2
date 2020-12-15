
#pragma once

#pragma warning(disable : 4996)

#include <stdlib.h>

#include <k4a/k4a.hpp>
#include "windows_thread.h"
#include "windows_time.h"
#include "kinect_cv_dk.h"
#include "kinect_angle.h"

#include <array>
#include <iostream>
#include <map>
#include <vector>

#include <cmath>
#include <time.h>

#include <math.h>

#define LOGFILE "D:\\logForKinect.txt"

#define KEY_DOWN(VK_NONAME) ((GetAsyncKeyState(VK_NONAME) & 0x8001) ? 1:0)//�Զ�GetAsyncKeyState��������һ�ε�������������ѱ���������λ0��Ϊ1��������Ϊ0�����Ŀǰ���ڰ���״̬����λ15��Ϊ1����̧����Ϊ0������е���
//����ı�����ʾ����
#define VERIFY(result, error)                                                                            \
    if(result != K4A_RESULT_SUCCEEDED)                                                                   \
    {                                                                                                    \
        printf("%s \n - (File: %s, Function: %s, Line: %d)\n", error, __FILE__, __FUNCTION__, __LINE__); \
		FILE* logErro;\
		errno_t err = fopen_s(&logErro, LOGFILE, "a");													 \
		fprintf(logErro,error);																			 \
		fprintf(logErro,"\n");																			 \
		fclose(logErro);																				 \
        exit(1);                                                                                         \
    }   
#define PRINTFORLOG(logFile, log)\
	printf("%s",log);\
	fprintf(logFile,log);\
	fprintf(logFile,"\n");\
	

#ifndef ANGLE_NUM
#define ANGLE_NUM 18
#endif // !ANGLE_NUM


class Kinct
{
public:
	volatile bool  bOnePicture;
	std::mutex* onePictureFlag;
	Kinct();
	~Kinct();
	int init();
	int del();
	int capThread();
	//int getAngle(float (&fAngle)[ANGLE_NUM]);
	//int getJoint(float (&fJoint)[BODU_POINT_NUM*3+1]);
	int getAngle(float* const fAngle);
	int getJoint(float* const fJoint);
	int getCmat(cv::Mat&);

private:
	k4a_device_t* dev;
	k4a_calibration_t* sensorCalibration;
	cv::Mat* colorFrame;
	std::thread* tids;
	uint32_t uintNum;
	//ʱ���
	uint64_t* timeStamp;
	//����ؽڵ�����
	k4abt_skeleton_t skeleton;
	std::mutex Flagangle1to1;
	int  iFlagAngle1to1, iFlagMaster, iMasterNum;
	bool bInitFlag, bDel;
	float fAngelUsing[ANGLE_NUM], fAmgleALL[ANGLE_NUM];
	//bool esc = false;
	void cap(k4a_device_t& dev, cv::Mat& colorFrame, const int i, k4a_calibration_t& sensorCalibration, \
		float(&fAngelUsing)[ANGLE_NUM], uint64_t& timeStamp);  //��ͨ�ĺ���������ִ���߳�
	int onePicture(const int i, k4a_capture_t sensor_capture, float(&fAngelUsing)[ANGLE_NUM], k4abt_tracker_t& tracker, uint64_t &timeStamp);
	void reKinct();
protected:
};


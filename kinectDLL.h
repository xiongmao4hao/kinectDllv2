//�ؽڽǸ���
#ifndef ANGLE_NUM
#define ANGLE_NUM 18
#endif // !ANGLE_NUM

//�ؽڵ����
#ifndef JOINT_NUM
#define JOINT_NUM 32
#endif // !JOINT_NUM

//Ϊ�˿����Ե���
#ifdef SIMPLE_CLASS_EXPORT



#define SIMPLE_CLASS_EXPORT extern "C" __declspec(dllexport)

#else

#define SIMPLE_CLASS_EXPORT extern "C" __declspec(dllimport)

#endif

#pragma once
// pch.cpp: ��Ԥ�����ͷ��Ӧ��Դ�ļ�

#include "kinect_record.h"

SIMPLE_CLASS_EXPORT kinectSubject* getKinectSubject();
SIMPLE_CLASS_EXPORT int start(kinectSubject* kinectTarget);
SIMPLE_CLASS_EXPORT int cap(kinectSubject* kinectTarget);
SIMPLE_CLASS_EXPORT int stop(kinectSubject* kinectTarget);
SIMPLE_CLASS_EXPORT Observer* getObserver(kinectSubject* kinectTarget);
SIMPLE_CLASS_EXPORT int removeObserver(Observer* observeTarget);
SIMPLE_CLASS_EXPORT float* getJoint(Observer* observeTarget);

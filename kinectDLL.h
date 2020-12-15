#pragma once
// pch.cpp: 与预编译标头对应的源文件


#ifdef SIMPLE_CLASS_EXPORT

#define SIMPLE_CLASS_EXPORT extern "C" __declspec(dllexport)

#else

#define SIMPLE_CLASS_EXPORT extern "C" __declspec(dllimport)

#endif

#ifndef ANGLE_NUM
#define ANGLE_NUM 18
#endif // !ANGLE_NUM

#ifndef BODU_POINT_NUM
#define BODU_POINT_NUM 32
#endif


typedef struct _skeletonFrame
{
	float skeletonAngle[ANGLE_NUM];
	float skeletonPoint[BODU_POINT_NUM*3+1];
}SkeletonFrame;

SIMPLE_CLASS_EXPORT int recordStart();
SIMPLE_CLASS_EXPORT int recordStop();
SIMPLE_CLASS_EXPORT uchar* recordY(int& rows, int& cols, int& channels, SkeletonFrame&);
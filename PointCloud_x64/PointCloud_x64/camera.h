#pragma once
#ifndef __MAIN_H__
#define __MAIN_H__
#include <iostream>
#include <iomanip>
#include <fstream>
#include <conio.h>
#include <opencv2/opencv.hpp>
#include "Vzense_api2.h"
#include <thread>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
using namespace std;

typedef struct {
	uint32_t slope1;
	uint32_t slope2;
	uint32_t slope3;
	int range_num;
}SlopeT;
//int dealStatus(PsReturnStatus status, string errorInfo);

/**
 * @brief		���������ȡ���������ת��Ϊ���ͼ��
 * @param[in]	slope	����ת��Ϊ����ֵ�ı���
 * @param[in]	height	ͼ���
 * @param[in]	width	ͼ���
 * @param[in]	pData	�������
 * @param[out]	dispImg ���ͼ��
*/
void 
Opencv_Depth(
	uint32_t slope, 
	int height, 
	int width, 
	uint8_t* pData,
	cv::Mat& dispImg);

/**
* @breif		��ȡslope
* @param[in]	depthRange��
* @param[in]	measuringRange��
* @return		slope
*/
uint32_t 
getSlope(
	PsDepthRange depthRange, 
	PsMeasuringRange measuringRange);

/**
* @breif			�򿪲�����Vzense���
* @param[out]		deviceHandle���豸���
* @param[in]		sessionIndex���Ự��ʶ
* @param[in][out]	wdrMode��
* @param[in][out]	setDataMode����������ģʽ���������ú����������ģʽ(����ʧ��)
* @param[out]		slope��
* @return
*/
int 
openAndConfigCamera(
	PsDeviceHandle& deviceHandle, 
	uint32_t sessionIndex, 
	PsWDROutputMode* pWdrMode,
	PsDataMode& setDataMode, 
	SlopeT& slope);

/**
 * @brief		�ر����
 * @param[in]	deviceHandle
 * @param[out]	sessionIndex
 * @return		status��״̬��
*/
int 
closeCamera(
	PsDeviceHandle deviceHandle, 
	uint32_t sessionIndex);

/**
 * @brief 
 * @param[in]	deviceHandle	�豸���
 * @param[in]	sessionIndex	�Ự��ʶ
 * @param[in]	slope			����
 * @param[out]	pointCloudVec	��������
 * @param[in]	interval		������fps��auto_updateΪtrueʱ��Ч
 * @param[in]	maxNum			��ౣ��֡��
 * @param[in]	showDepthImage	��ʾ���ͼ��
 * @param[in]	showRGBImage	��ʾRGBͼ��
 * @param[in]	auto_update		true���Զ����棻false���ֶ�ѡ�񱣴���һ֡
 * @return 
*/
int getRealTimePointClouds(
	PsDeviceHandle deviceHandle,
	uint32_t sessionIndex,
	SlopeT slope,
	vector<pcl::PointCloud<pcl::PointXYZ>::Ptr, Eigen::aligned_allocator<pcl::PointXYZ>>& pointCloudVec,
	uint32_t interval = 15,
	uint32_t maxNum = 1000,
	bool showDepthImage = true,
	bool showRGBImage = true,
	bool auto_update = true);

/**
 * @brief 
 * @param[in]	deviceHandle		�豸���
 * @param[in]	sessionIndex		�Ự��ʶ
 * @param[in]	slope				����
 * @param[in]	fp					����·��
 * @param[in]	interval			������fps��auto_saveΪtrueʱ��Ч
 * @param[in]	maxNum				��ౣ��֡��
 * @param[in]	showDepthImage		��ʾ���ͼ��
 * @param[in]	showRGBImage		��ʾRGBͼ��
 * @param[in]	auto_save			true���Զ����棻false���ֶ�ѡ�񱣴���һ֡
 * @return 
*/
int saveRealTimePointClouds(
	PsDeviceHandle deviceHandle,
	uint32_t sessionIndex,
	SlopeT slope,
	string fp = "",
	uint32_t interval = 15,
	uint32_t maxNum = 1000,
	bool showDepthImage = true,
	bool showRGBImage = true,
	bool auto_save = true);
#endif // __MAIN_H__
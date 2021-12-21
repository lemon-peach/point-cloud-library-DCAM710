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
 * @brief		将从相机获取的深度数据转换为深度图像
 * @param[in]	slope	距离转换为像素值的比例
 * @param[in]	height	图像高
 * @param[in]	width	图像宽
 * @param[in]	pData	深度数据
 * @param[out]	dispImg 深度图像
*/
void 
Opencv_Depth(
	uint32_t slope, 
	int height, 
	int width, 
	uint8_t* pData,
	cv::Mat& dispImg);

/**
* @breif		获取slope
* @param[in]	depthRange：
* @param[in]	measuringRange：
* @return		slope
*/
uint32_t 
getSlope(
	PsDepthRange depthRange, 
	PsMeasuringRange measuringRange);

/**
* @breif			打开并设置Vzense相机
* @param[out]		deviceHandle：设备句柄
* @param[in]		sessionIndex：会话标识
* @param[in][out]	wdrMode：
* @param[in][out]	setDataMode：设置数据模式，返回设置后的最终数据模式(可能失败)
* @param[out]		slope：
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
 * @brief		关闭相机
 * @param[in]	deviceHandle
 * @param[out]	sessionIndex
 * @return		status：状态码
*/
int 
closeCamera(
	PsDeviceHandle deviceHandle, 
	uint32_t sessionIndex);

/**
 * @brief 
 * @param[in]	deviceHandle	设备句柄
 * @param[in]	sessionIndex	会话标识
 * @param[in]	slope			比例
 * @param[out]	pointCloudVec	点云向量
 * @param[in]	interval		保存间隔fps，auto_update为true时有效
 * @param[in]	maxNum			最多保存帧数
 * @param[in]	showDepthImage	显示深度图像
 * @param[in]	showRGBImage	显示RGB图像
 * @param[in]	auto_update		true：自动保存；false：手动选择保存哪一帧
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
 * @param[in]	deviceHandle		设备句柄
 * @param[in]	sessionIndex		会话标识
 * @param[in]	slope				比例
 * @param[in]	fp					保存路径
 * @param[in]	interval			保存间隔fps，auto_save为true时有效
 * @param[in]	maxNum				最多保存帧数
 * @param[in]	showDepthImage		显示深度图像
 * @param[in]	showRGBImage		显示RGB图像
 * @param[in]	auto_save			true：自动保存；false：手动选择保存哪一帧
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
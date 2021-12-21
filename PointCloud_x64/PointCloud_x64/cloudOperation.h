#pragma once
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <iomanip>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/types.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/visualization/common/common.h>
#include <pcl/registration/icp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/surface/gp3.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/fast_bilateral.h>
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/surface/poisson.h>
#include <pcl/features/pfh.h>
#include <pcl/visualization/pcl_plotter.h>
#include "function.h"

using namespace std;
namespace cloud{
	using PointT = pcl::PointXYZ;
	using PointCloud = pcl::PointCloud<PointT>;
	using PointCloudPtr = PointCloud::Ptr;

	using NormalT = pcl::Normal;
	using Normal = pcl::PointCloud<NormalT>;
	using NormalPtr = Normal::Ptr;

	using PointNormalT = pcl::PointNormal;
	using PointCloudNormal = pcl::PointCloud<PointNormalT>;
	using PointCloudNormalPtr = PointCloudNormal::Ptr;

	using PointCloudVec = vector<PointCloud, Eigen::aligned_allocator<PointT>>;
	using PointCloudPtrVec = vector<PointCloudPtr, Eigen::aligned_allocator<PointT>>;

	typedef struct {
		double r, g, b;
	}Color;
}

/**
 * @brief		pcl可视化测试
 * @param[in]	filePath pcd	文件路径
 * @return		0：成功；1：失败
*/
int 
pclViewerTest(string filePath);

/**
 * @brief		可视化点云，并可以显示选择点的坐标
 * @param[in]	pointCloudPtr		要可视化的点云
 * @param[in]	pointColor			点云颜色
 * @param[in]	backgroundColor		背景颜色
 * @param[in]	is_auto				显示阻塞
 * @return		PCLVisualizer实例
*/
pcl::visualization::PCLVisualizer
visualizePointCloud(
	const cloud::PointCloudPtr& pointCloud,
	const cloud::Color pointColor = { 255.0, 255.0, 255.0 },
	const int pointSize = 1,
	const cloud::Color backgroundColor = { 0.0, 0.0, 0.0 },
	bool is_auto = true);

/**
 * @brief			可视化多个点云
 * @param[in]		pointCloudPtrVec	点云向量
 * @param[in][out]	pointColorVec		点云颜色，少于点云数量时自动补足为白色
 * @param[in]		backgroundColor		背景颜色
 * @param[in]		is_auto				显示阻塞
 * @return			PCLVisualizer实例
*/
pcl::visualization::PCLVisualizer
visualizePointCloud(
	const cloud::PointCloudPtrVec& pointCloudPtrVec,
	vector<cloud::Color>& pointColorVec,
	vector<int> pointSizeVec = {1},
	const cloud::Color& backgroundColor = { 0.0, 0.0, 0.0 },
	bool is_auto = true);

/**
 * @brief		可视化点云和网格
 * @param[in]	pointCloudPtr		点云
 * @param[in]	pointColor			点云颜色
 * @param[in]	mesh				网格
 * @param[in]	backgroundColor		背景色
 * @param[in]	is_auto				显示阻塞
 * @return
*/
pcl::visualization::PCLVisualizer
visualizePointCloud(
	const cloud::PointCloudPtr& pointCloudPtr,
	const pcl::PolygonMesh& mesh,
	const cloud::Color& pointColor = { 255.0, 255.0, 255.0 },
	int pointSize = 1,
	const cloud::Color& backgroundColor = { 0.0, 0.0, 0.0 },
	bool is_auto = true);

/**
 * @brief			可视化多个点云和网格
 * @param[in]		pointCloudPtrVec	点云向量
 * @param[in][out]	pointColorVec		点云颜色，少于点云数量时自动补足为白色
 * @param[in]		meshVec				网格向量
 * @param[in]		backgroundColor		背景颜色
 * @param[in]		is_auto				显示阻塞
 * @return			PCLVisualizer实例
*/
pcl::visualization::PCLVisualizer
visualizePointCloud(
	const cloud::PointCloudPtrVec& pointCloudPtrVec,
	const vector<pcl::PolygonMesh>& meshVec,
	vector<cloud::Color>& pointColorVec,
	vector<int> pointSizeVec = {1},
	const cloud::Color& backgroundColor = { 0.0, 0.0, 0.0 },
	bool is_auto = true);

/**
 * @brief		生成随机点云
 * @param[out]	cloudPtr	点云
 * @param[in]	width		点云的宽
 * @param[in]	height		点云的高
 * @return	
*/
void randomCloud(
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr, 
	int width, 
	int height);

/**
 * @brief		双边滤波
 * @param[in]	inPointCloudPtr		源点云
 * @param[out]	outPointCloudPtr	滤波后点云
 * @param[in]	sigmaS				sigmaS
 * @param[in]	sigmaR				sigmaR
 * @return		0：成功；-1：失败
*/
int 
fastBilateralFilter(
	const cloud::PointCloudPtr& inPointCloudPtr,
	cloud::PointCloudPtr& outPointCloudPtr,
	float sigmaS = 5,
	float sigmaR = 0.015);

/**
 * @brief		使用 RANSAC 方法进行平面分割
 * @param[in]	threshold		阈值
 * @param[in]	srcCloudPtr		原始点云
 * @param[out]	dstCloudPtr		分割后的点云
 * @param[in]	negative		提取相反的点云
 * @return		0：成功	-1：失败
*/
int planeSegmentation(
	float threshold,
	cloud::PointCloudPtr& srcCloudPtr,
	cloud::PointCloudPtr& dstCloudPtr,
	bool negative = false);

/**
 * @brief	使用RANSAC方法进行平面分割
 * @param	threshold	阈值
 * @param	srcCloudPtr 原始点云
 * @param	inliers		分割后的点云索引
 * @return	0：成功	-1：失败
*/
int planeSegmentation(
	float threshold,
	cloud::PointCloudPtr& srcCloudPtr,
	pcl::PointIndices& inliers);
/**
 * @brief		icp粗配准
 * @param[in]	srcCloudPtr		原始点云
 * @param[in]	tgtCloudPtr		目标点云
 * @param[out]	transformation	坐标变换矩阵
 * @param[out]	transCloudPtr	原始点云变换后的匹配点云
 * @return 
*/
inline int
icpRegistration(
	const cloud::PointCloudPtr& srcCloudPtr,
	const cloud::PointCloudPtr& tgtCloudPtr,
	Eigen::Matrix4f& transformation,
	pcl::PointCloud<cloud::PointT>::Ptr& transformedCloudPtr);

/**
 * @brief		icp精配准
 * @param[in]	srcCloudPtr		原始点云
 * @param[in]	tgtCloudPtr		目标点云
 * @param[out]	transformation	坐标变换矩阵
 * @param[out]	transCloudPtr	原始点云变换后的匹配点云
 * @param[in]	guess			粗配准的变换矩阵
 * @return
*/
inline int
icpRegistration(
	const cloud::PointCloudPtr& srcCloudPtr,
	const cloud::PointCloudPtr& tgtCloudPtr,
	Eigen::Matrix4f& transformation,
	const cloud::PointCloudPtr& transformedCloudPtr,
	const Eigen::Matrix4f& guess);

/**
 * @brief		计算法线
 * @param[in]	inCloudPtr			输入点云
 * @param[out]	outCloudNormalPtr	包含法线和曲率的点云（不含点信息） 
 * @param[in]	k					K邻近搜索
 * @param[in]	has_point			返回点云是否包含点数据
 * @return 
*/
inline int
normalEstimation(
	const cloud::PointCloudPtr& inCloudPtr,
	pcl::PointCloud<pcl::Normal>::Ptr& outCloudNormalPtr,
	int k = 20,
	float r = 0.05);

/**
 * @brief		计算法线
 * @param[in]	inCloudPtr			输入点云 
 * @param[out]	outCloudNormalPtr	包含点、法线、曲率的点云
 * @param[in]	k					k临近搜索
 * @return 
*/
inline int
normalEstimation(
	const cloud::PointCloudPtr& inCloudPtr,
	cloud::PointCloudNormalPtr& outCloudNormalPtr,
	int k = 20,
	float r = 0.05);

/**
 * @brief		配准：计算源点云到目标点云的变换矩阵，根据点和曲率进行配准
 * @param[in]	srcCloudPtr				源点云
 * @param[in]	tgtCloudPtr				目标点云
 * @param[out]	resCloudPtr				源点云经变换后的配准点云
 * @param[out]	final_transformation	最终的变换矩阵
 * @param[in]	downSample				是否对输入点云进行下采样
 * @return 
*/
int
pairAlignWithNormal(
	const cloud::PointCloudPtr& srcCloudPtr, 
	const cloud::PointCloudPtr& tgtCloudPtr, 
	cloud::PointCloudPtr& resCloudPtr, 
	Eigen::Matrix4f& final_transformation,
	bool downSample = false);

/**
 * @brief		配准：计算源点云到目标点云的变换矩阵，根据点进行配准
 * @param[in]	srcCloudPtr				源点云
 * @param[in]	tgtCloudPtr				目标点云
 * @param[out]	resCloudPtr				源点云经变换后的配准点云
 * @param[out]	final_transformation	最终的变换矩阵
 * @param[in]	downSample				是否对输入点云进行下采样
 * @return 
*/
int
pairAlign(
	const cloud::PointCloudPtr& srcCloudPtr,
	const cloud::PointCloudPtr& tgtCloudPtr,
	cloud::PointCloudPtr& resCloudPtr,
	Eigen::Matrix4f& final_transformation,
	bool downSample = false);

/**
 * @brief		多个点云配准，将多个点云转换到第一个点云的坐标系
 * @param[in]	pointCloudVec				待配准的点云向量
 * @param[out]	registeredPointCloudPtr		所有点云转换到第一个点云坐标系后的混合点云
 * @param[in]	saveResultToPcd				是否单独存储每个点云转换的结果到PCD
 * @return 
*/
int
registerPairsCloud(
	const cloud::PointCloudPtrVec& pointCloudVec,
	cloud::PointCloudPtr& registeredPointCloudPtr,
	bool withNormal = false,
	bool saveResultToPcd = false,
	bool downSample = false);

/**
 * @brief		多个点云配准，将多个点云转换到第一个点云的坐标系
 * @param[in]	pointCloudVec		待配准的点云向量
 * @param[put]	registeredVec		所有点云转换到第一个点云坐标系后的点云向量
 * @param[put]	transformationVec
 * @param[in]	saveResultToPcd		是否单独存储每个点云转换的结果到PCD
 * @return 
*/
int
registerPairsCloud(
	const cloud::PointCloudPtrVec& pointCloudVec,
	cloud::PointCloudPtrVec& registeredVec,
	vector<Eigen::Matrix4f>& transformationVec,
	bool withNormal = false,
	bool saveResultToPcd = false,
	bool downSample = false);

/**
 * @brief		网格重建
 * @param[in]	srcPointCloud	输入点云
 * @param[out]	trianglesMesh	网格重建结果
 * @return 
*/
int
creatMesh(
	const cloud::PointCloudPtr& srcPointCloud,
	pcl::PolygonMesh& trianglesMesh,
	int maximumNearestNeighbors = 100,
	double searchRadius = 0.5,
	double mu = 2.5,
	double minimumAngle = M_PI / 18,
	double maximumAngle = 2 * M_PI / 3,
	double maximumSurfaceAngle = M_PI_4,
	bool normalConsistency = false);

/**
 * @brief		利用泊松法表面重建
 * @param[in]	PointCloudPtr		输入点云
 * @param[out]	mesh				输出网格
 * @return 
*/
int creatMeshPassion(
	const cloud::PointCloudPtr& PointCloudPtr,
	pcl::PolygonMesh& mesh);

/**
 * @brief		利用贪婪投影算法表面重建
 * @param[in]	PointCloudNormalPtr		输入点云
 * @param[out]	mesh					输出网格
 * @return 
*/
int creatMeshPassion(
	const cloud::PointCloudNormalPtr& PointCloudNormalPtr,
	pcl::PolygonMesh& mesh);

/**
 * @brief		将点数据和三角面数据保存到obj文件中
 * @param[in]	fp				文件保存路径
 * @param[in]	pointCloud		点云数据
 * @param[in]	mesh			三角面网格数据
 * @return 
*/
int
saveToOBJ(
	string fp,
	const cloud::PointCloud& pointCloud,
	const pcl::PolygonMesh& mesh);

/**
 * @brief		计算深度图像
 * @param[in]	pointCloudPtr		输入点云
 * @param[out]	rangeImagePtr		输出深度图像
 * @param[in]	angularResolution	角度分辨率
 * @param[in]	maxAngleWidth		水平视角
 * @param[in]	maxAngleHeight		垂直视角
 * @return 
*/
int 
getRangeImage(
	const cloud::PointCloudPtr& pointCloudPtr,
	pcl::RangeImage::Ptr& rangeImagePtr,
	float angularResolution = 0.3f,
	float maxAngleWidth = 75.0f,
	float maxAngleHeight = 55.0f);

/**
 * @brief		获取NARF关键点，返回关键点的索引pcl::PointCloud<int>
 * @param[in]	pointCloudPtr		输入点云
 * @param[in]	rangeImage			深度图像
 * @param[out]	keypoint_indices	关键点索引
 * @param[in]	support_size		球半径
 * @return 
*/
int
getNARFKeypoints(
	const cloud::PointCloudPtr& pointCloudPtr,
	const pcl::RangeImage& rangeImage,
	pcl::PointCloud<int>& keypoint_indices,
	float support_size = 0.2f);

/**
 * @brief		获取NARF关键点
 * @param[in]	pointCloudPtr		输入点云
 * @param[in]	rangeImage			深度图像
 * @param[out]	keyPointCloudPtr	输出关键点
 * @param[in]	support_size		球半径
 * @return 
*/
int 
getNARFKeypoints(
	const cloud::PointCloudPtr& pointCloudPtr,
	const pcl::RangeImage& rangeImage,
	cloud::PointCloudPtr& keyPointCloudPtr,
	float support_size = 0.2f);

/**
 * @brief		获取NARF关键点，返回关键点的索引pcl::Indices
 * @param[in]	pointCloudPtr		输入点云
 * @param[in]	rangeImage			深度图像
 * @param[out]	indices				关键点索引
 * @param[in]	support_size		球半径
 * @return 
*/
int
getNARFKeypoints(
	const cloud::PointCloudPtr& pointCloudPtr,
	const pcl::RangeImage& rangeImage,
	pcl::Indices& indices,
	float support_size = 0.2f);

/**
 * @brief		计算指定多个点的PFH特征
 * @param[in]	pointCloudPtr		输入点云
 * @param[in]	normalPtr			输入法向
 * @param[in]	indices				指定多个点的索引
 * @param[out]	pfh_histogram		PFH特征
 * @param[in]	nr_split			分割数
 * @return
*/
int
computePFH(
	const cloud::PointCloudPtr& pointCloudPtr,
	const pcl::PointCloud<pcl::Normal>::Ptr& normalPtr,
	const pcl::Indices& indices,
	Eigen::VectorXf& pfh_histogram,
	int nr_split);

/**
 * @brief		查询多个指定点的临近点索引
 * @param[in]	pointCloudPtr		点云
 * @param[in]	indices				指定点的索引列表
 * @param[out]	neighborsIndices	临近点的索引列表
 * @param[in]	k					查询K个临近点的索引，k==flase 时采用半径搜索
 * @param[in]	r					搜索半径，仅当 k==false 时有效
 * @return
*/
int
getNeighbors(
	const cloud::PointCloudPtr& pointCloudPtr,
	const vector<int>& indices,
	vector<vector<int>>& neighborsIndices,
	int k = 10,
	float r = 0.01f);

/**
 * @brief 
 * @param x_data 
 * @param y_data 
 * @return 
*/
int
plotHistogram(
	vector<double> x_data,
	vector<double> y_data);


/**********************************实现*************************************/

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
 * @brief		pcl���ӻ�����
 * @param[in]	filePath pcd	�ļ�·��
 * @return		0���ɹ���1��ʧ��
*/
int 
pclViewerTest(string filePath);

/**
 * @brief		���ӻ����ƣ���������ʾѡ��������
 * @param[in]	pointCloudPtr		Ҫ���ӻ��ĵ���
 * @param[in]	pointColor			������ɫ
 * @param[in]	backgroundColor		������ɫ
 * @param[in]	is_auto				��ʾ����
 * @return		PCLVisualizerʵ��
*/
pcl::visualization::PCLVisualizer
visualizePointCloud(
	const cloud::PointCloudPtr& pointCloud,
	const cloud::Color pointColor = { 255.0, 255.0, 255.0 },
	const int pointSize = 1,
	const cloud::Color backgroundColor = { 0.0, 0.0, 0.0 },
	bool is_auto = true);

/**
 * @brief			���ӻ��������
 * @param[in]		pointCloudPtrVec	��������
 * @param[in][out]	pointColorVec		������ɫ�����ڵ�������ʱ�Զ�����Ϊ��ɫ
 * @param[in]		backgroundColor		������ɫ
 * @param[in]		is_auto				��ʾ����
 * @return			PCLVisualizerʵ��
*/
pcl::visualization::PCLVisualizer
visualizePointCloud(
	const cloud::PointCloudPtrVec& pointCloudPtrVec,
	vector<cloud::Color>& pointColorVec,
	vector<int> pointSizeVec = {1},
	const cloud::Color& backgroundColor = { 0.0, 0.0, 0.0 },
	bool is_auto = true);

/**
 * @brief		���ӻ����ƺ�����
 * @param[in]	pointCloudPtr		����
 * @param[in]	pointColor			������ɫ
 * @param[in]	mesh				����
 * @param[in]	backgroundColor		����ɫ
 * @param[in]	is_auto				��ʾ����
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
 * @brief			���ӻ�������ƺ�����
 * @param[in]		pointCloudPtrVec	��������
 * @param[in][out]	pointColorVec		������ɫ�����ڵ�������ʱ�Զ�����Ϊ��ɫ
 * @param[in]		meshVec				��������
 * @param[in]		backgroundColor		������ɫ
 * @param[in]		is_auto				��ʾ����
 * @return			PCLVisualizerʵ��
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
 * @brief		�����������
 * @param[out]	cloudPtr	����
 * @param[in]	width		���ƵĿ�
 * @param[in]	height		���Ƶĸ�
 * @return	
*/
void randomCloud(
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr, 
	int width, 
	int height);

/**
 * @brief		˫���˲�
 * @param[in]	inPointCloudPtr		Դ����
 * @param[out]	outPointCloudPtr	�˲������
 * @param[in]	sigmaS				sigmaS
 * @param[in]	sigmaR				sigmaR
 * @return		0���ɹ���-1��ʧ��
*/
int 
fastBilateralFilter(
	const cloud::PointCloudPtr& inPointCloudPtr,
	cloud::PointCloudPtr& outPointCloudPtr,
	float sigmaS = 5,
	float sigmaR = 0.015);

/**
 * @brief		ʹ�� RANSAC ��������ƽ��ָ�
 * @param[in]	threshold		��ֵ
 * @param[in]	srcCloudPtr		ԭʼ����
 * @param[out]	dstCloudPtr		�ָ��ĵ���
 * @param[in]	negative		��ȡ�෴�ĵ���
 * @return		0���ɹ�	-1��ʧ��
*/
int planeSegmentation(
	float threshold,
	cloud::PointCloudPtr& srcCloudPtr,
	cloud::PointCloudPtr& dstCloudPtr,
	bool negative = false);

/**
 * @brief	ʹ��RANSAC��������ƽ��ָ�
 * @param	threshold	��ֵ
 * @param	srcCloudPtr ԭʼ����
 * @param	inliers		�ָ��ĵ�������
 * @return	0���ɹ�	-1��ʧ��
*/
int planeSegmentation(
	float threshold,
	cloud::PointCloudPtr& srcCloudPtr,
	pcl::PointIndices& inliers);
/**
 * @brief		icp����׼
 * @param[in]	srcCloudPtr		ԭʼ����
 * @param[in]	tgtCloudPtr		Ŀ�����
 * @param[out]	transformation	����任����
 * @param[out]	transCloudPtr	ԭʼ���Ʊ任���ƥ�����
 * @return 
*/
inline int
icpRegistration(
	const cloud::PointCloudPtr& srcCloudPtr,
	const cloud::PointCloudPtr& tgtCloudPtr,
	Eigen::Matrix4f& transformation,
	pcl::PointCloud<cloud::PointT>::Ptr& transformedCloudPtr);

/**
 * @brief		icp����׼
 * @param[in]	srcCloudPtr		ԭʼ����
 * @param[in]	tgtCloudPtr		Ŀ�����
 * @param[out]	transformation	����任����
 * @param[out]	transCloudPtr	ԭʼ���Ʊ任���ƥ�����
 * @param[in]	guess			����׼�ı任����
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
 * @brief		���㷨��
 * @param[in]	inCloudPtr			�������
 * @param[out]	outCloudNormalPtr	�������ߺ����ʵĵ��ƣ���������Ϣ�� 
 * @param[in]	k					K�ڽ�����
 * @param[in]	has_point			���ص����Ƿ����������
 * @return 
*/
inline int
normalEstimation(
	const cloud::PointCloudPtr& inCloudPtr,
	pcl::PointCloud<pcl::Normal>::Ptr& outCloudNormalPtr,
	int k = 20,
	float r = 0.05);

/**
 * @brief		���㷨��
 * @param[in]	inCloudPtr			������� 
 * @param[out]	outCloudNormalPtr	�����㡢���ߡ����ʵĵ���
 * @param[in]	k					k�ٽ�����
 * @return 
*/
inline int
normalEstimation(
	const cloud::PointCloudPtr& inCloudPtr,
	cloud::PointCloudNormalPtr& outCloudNormalPtr,
	int k = 20,
	float r = 0.05);

/**
 * @brief		��׼������Դ���Ƶ�Ŀ����Ƶı任���󣬸��ݵ�����ʽ�����׼
 * @param[in]	srcCloudPtr				Դ����
 * @param[in]	tgtCloudPtr				Ŀ�����
 * @param[out]	resCloudPtr				Դ���ƾ��任�����׼����
 * @param[out]	final_transformation	���յı任����
 * @param[in]	downSample				�Ƿ��������ƽ����²���
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
 * @brief		��׼������Դ���Ƶ�Ŀ����Ƶı任���󣬸��ݵ������׼
 * @param[in]	srcCloudPtr				Դ����
 * @param[in]	tgtCloudPtr				Ŀ�����
 * @param[out]	resCloudPtr				Դ���ƾ��任�����׼����
 * @param[out]	final_transformation	���յı任����
 * @param[in]	downSample				�Ƿ��������ƽ����²���
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
 * @brief		���������׼�����������ת������һ�����Ƶ�����ϵ
 * @param[in]	pointCloudVec				����׼�ĵ�������
 * @param[out]	registeredPointCloudPtr		���е���ת������һ����������ϵ��Ļ�ϵ���
 * @param[in]	saveResultToPcd				�Ƿ񵥶��洢ÿ������ת���Ľ����PCD
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
 * @brief		���������׼�����������ת������һ�����Ƶ�����ϵ
 * @param[in]	pointCloudVec		����׼�ĵ�������
 * @param[put]	registeredVec		���е���ת������һ����������ϵ��ĵ�������
 * @param[put]	transformationVec
 * @param[in]	saveResultToPcd		�Ƿ񵥶��洢ÿ������ת���Ľ����PCD
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
 * @brief		�����ؽ�
 * @param[in]	srcPointCloud	�������
 * @param[out]	trianglesMesh	�����ؽ����
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
 * @brief		���ò��ɷ������ؽ�
 * @param[in]	PointCloudPtr		�������
 * @param[out]	mesh				�������
 * @return 
*/
int creatMeshPassion(
	const cloud::PointCloudPtr& PointCloudPtr,
	pcl::PolygonMesh& mesh);

/**
 * @brief		����̰��ͶӰ�㷨�����ؽ�
 * @param[in]	PointCloudNormalPtr		�������
 * @param[out]	mesh					�������
 * @return 
*/
int creatMeshPassion(
	const cloud::PointCloudNormalPtr& PointCloudNormalPtr,
	pcl::PolygonMesh& mesh);

/**
 * @brief		�������ݺ����������ݱ��浽obj�ļ���
 * @param[in]	fp				�ļ�����·��
 * @param[in]	pointCloud		��������
 * @param[in]	mesh			��������������
 * @return 
*/
int
saveToOBJ(
	string fp,
	const cloud::PointCloud& pointCloud,
	const pcl::PolygonMesh& mesh);

/**
 * @brief		�������ͼ��
 * @param[in]	pointCloudPtr		�������
 * @param[out]	rangeImagePtr		������ͼ��
 * @param[in]	angularResolution	�Ƕȷֱ���
 * @param[in]	maxAngleWidth		ˮƽ�ӽ�
 * @param[in]	maxAngleHeight		��ֱ�ӽ�
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
 * @brief		��ȡNARF�ؼ��㣬���عؼ��������pcl::PointCloud<int>
 * @param[in]	pointCloudPtr		�������
 * @param[in]	rangeImage			���ͼ��
 * @param[out]	keypoint_indices	�ؼ�������
 * @param[in]	support_size		��뾶
 * @return 
*/
int
getNARFKeypoints(
	const cloud::PointCloudPtr& pointCloudPtr,
	const pcl::RangeImage& rangeImage,
	pcl::PointCloud<int>& keypoint_indices,
	float support_size = 0.2f);

/**
 * @brief		��ȡNARF�ؼ���
 * @param[in]	pointCloudPtr		�������
 * @param[in]	rangeImage			���ͼ��
 * @param[out]	keyPointCloudPtr	����ؼ���
 * @param[in]	support_size		��뾶
 * @return 
*/
int 
getNARFKeypoints(
	const cloud::PointCloudPtr& pointCloudPtr,
	const pcl::RangeImage& rangeImage,
	cloud::PointCloudPtr& keyPointCloudPtr,
	float support_size = 0.2f);

/**
 * @brief		��ȡNARF�ؼ��㣬���عؼ��������pcl::Indices
 * @param[in]	pointCloudPtr		�������
 * @param[in]	rangeImage			���ͼ��
 * @param[out]	indices				�ؼ�������
 * @param[in]	support_size		��뾶
 * @return 
*/
int
getNARFKeypoints(
	const cloud::PointCloudPtr& pointCloudPtr,
	const pcl::RangeImage& rangeImage,
	pcl::Indices& indices,
	float support_size = 0.2f);

/**
 * @brief		����ָ��������PFH����
 * @param[in]	pointCloudPtr		�������
 * @param[in]	normalPtr			���뷨��
 * @param[in]	indices				ָ������������
 * @param[out]	pfh_histogram		PFH����
 * @param[in]	nr_split			�ָ���
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
 * @brief		��ѯ���ָ������ٽ�������
 * @param[in]	pointCloudPtr		����
 * @param[in]	indices				ָ����������б�
 * @param[out]	neighborsIndices	�ٽ���������б�
 * @param[in]	k					��ѯK���ٽ����������k==flase ʱ���ð뾶����
 * @param[in]	r					�����뾶������ k==false ʱ��Ч
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


/**********************************ʵ��*************************************/

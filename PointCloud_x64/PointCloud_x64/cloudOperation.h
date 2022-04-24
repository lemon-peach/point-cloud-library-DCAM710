#pragma once
#include <pcl/point_types.h>
struct EIGEN_ALIGN16 CustomPointT
{
	PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
	float data_additional[10] = {0.f};
	inline CustomPointT(float _x, float _y, float _z) {
		x = _x;
		y = _y;
		z = _z;
		data[3] = 0.f;
	}
	inline CustomPointT() :CustomPointT(0.0f, 0.0f, 0.0f) {};
	inline CustomPointT(const CustomPointT& _p) :CustomPointT(_p.x, _p.y, _p.z) {};
	friend std::ostream& operator <<(std::ostream os, CustomPointT& p) {
		return os;
	}
	PCL_MAKE_ALIGNED_OPERATOR_NEW     // make sure our new allocators are aligned
};                   // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT(CustomPointT,           // here we assume a XYZ + "test" (as fields)
	(float, x, x)
	(float, y, y)
	(float, z, z)
	(float[10], data_additional, data_additional)
)

#include <ctime>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <iomanip>

#include <pcl/memory.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/types.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/common/common.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/impl/icp.hpp>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/impl/icp_nl.hpp>
#include <pcl/surface/gp3.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/fast_bilateral.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/surface/poisson.h>
#include <pcl/features/pfh.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/correspondence.h>
#include <pcl/visualization/pcl_plotter.h>
#include <pcl/io/vtk_lib_io.h >
#include <Eigen/SVD>
#include <Eigen/Core>
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

	//using PointCloudVec = vector<PointCloud, Eigen::aligned_allocator<PointT>>;
	//using PointCloudPtrVec = vector<PointCloudPtr, Eigen::aligned_allocator<PointT>>;
	using PointCloudVec = vector<PointCloud>;
	using PointCloudPtrVec = vector<PointCloudPtr>;

	typedef struct {
		double r, g, b;
	}Color;

	using PointNormalPfhT = CustomPointT;
	//using PointNormalPfhT = PointNormalPfhTp;
	using PointNormalPfh = pcl::PointCloud<PointNormalPfhT>;
	using PointNormalPfhPtr = PointNormalPfh::Ptr;
	using PointNormalPfhPtrVec = vector<PointNormalPfhPtr, Eigen::aligned_allocator<PointNormalPfhT>>;

	using PointWithEdgeSignT = pcl::PointXYZI;
	using PointWithEdgeSign = pcl::PointCloud<PointWithEdgeSignT>;
	using PointWithEdgeSignPtr = pcl::PointCloud<PointWithEdgeSignT>::Ptr;

	using PFH27 = pcl::Histogram<27>;
	typedef struct {
		pcl::PointXYZ min;
		pcl::PointXYZ max;
		pcl::PointXYZ poistion;
		Eigen::Matrix3f rotation;
	}OBBT;
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
pcl::visualization::PCLVisualizer::Ptr
visualizePointCloud(
	const cloud::PointCloudPtr& pointCloud,
	const cloud::Color pointColor = { 255.0, 255.0, 255.0 },
	const int pointSize = 1,
	const cloud::Color backgroundColor = { 0.0, 0.0, 0.0 },
	bool showCoordinateSystem = true,
	bool is_auto = true);

/**
 * @brief			���ӻ��������
 * @param[in]		pointCloudPtrVec	��������
 * @param[in][out]	pointColorVec		������ɫ�����ڵ�������ʱ�Զ�����Ϊ��ɫ
 * @param[in]		backgroundColor		������ɫ
 * @param[in]		is_auto				��ʾ����
 * @return			PCLVisualizerʵ��
*/
pcl::visualization::PCLVisualizer::Ptr
visualizePointCloud(
	const cloud::PointCloudPtrVec& pointCloudPtrVec,
	vector<cloud::Color>& pointColorVec,
	vector<int> pointSizeVec = {1},
	const cloud::Color& backgroundColor = { 0.0, 0.0, 0.0 },
	bool showCoordinateSystem = true,
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
pcl::visualization::PCLVisualizer::Ptr
visualizePointCloud(
	const cloud::PointCloudPtr& pointCloudPtr,
	const pcl::PolygonMesh& mesh,
	const cloud::Color& pointColor = { 255.0, 255.0, 255.0 },
	int pointSize = 1,
	const cloud::Color& backgroundColor = { 0.0, 0.0, 0.0 },
	bool showCoordinateSystem = true,
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
pcl::visualization::PCLVisualizer::Ptr
visualizePointCloud(
	const cloud::PointCloudPtrVec& pointCloudPtrVec,
	const vector<pcl::PolygonMesh>& meshVec,
	vector<cloud::Color>& pointColorVec,
	vector<int> pointSizeVec = {1},
	const cloud::Color& backgroundColor = { 0.0, 0.0, 0.0 },
	bool showCoordinateSystem = true,
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
	bool downSample = false,
	float downSampleSize = 0.005f,
	float maxDis = 0.02,
	bool useMyEm=false,
	bool edgeFilter = false);

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
	bool downSample = false,
	float downSampleSize = 0.005f,
	float maxDis = 0.05);

int
pairAlignWithCustom(
	const cloud::PointNormalPfhPtr& srcCloudPtr,
	const cloud::PointNormalPfhPtr& tgtCloudPtr,
	cloud::PointCloudPtr& resCloudPtr,
	Eigen::Matrix4f& final_transformation,
	bool downSample = false,
	float downSampleSize = 0.005f,
	float maxDis = 0.02,
	bool useMyEm = false);

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
creatMeshWithTri(
	const cloud::PointCloudPtr& srcPointCloud,
	pcl::PolygonMesh& trianglesMesh,
	int maximumNearestNeighbors = 100,
	double searchRadius = 0.5,
	double mu = 2.5,
	double minimumAngle = M_PI / 18,
	double maximumAngle = 2 * M_PI / 3,
	double maximumSurfaceAngle = M_PI_4,
	bool normalConsistency = false,
	bool usePointRepresentation = true);

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

int
getSIFTKeypoint(
	const cloud::PointCloudNormalPtr& pointCloudNormalPtr,
	cloud::PointCloudPtr& result
);

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

int
computePFH(
	const cloud::PointCloudPtr& pointCloudPtr,
	const pcl::PointCloud<pcl::Normal>::Ptr& normalPtr,
	vector<Eigen::VectorXf>& pfh_histogramVec,
	const pcl::Indices& indices,
	int k = 10,
	float r = 0.01f,
	int nr_split = 5);

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

int
getNeighbors(
	const cloud::PointCloudPtr& pointCloudPtr,
	const cloud::PointCloudPtr& searchPointPtr,
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

Eigen::MatrixXf
PCA(
	Eigen::MatrixXf origin,
	int kStart,
	float error = 0.1f);


/**********************************ʵ��*************************************/
/**
 * @brief
 * @tparam PointT
 * @param pointCloudPtrVec
 * @param basePath
 * @param nameVec
 * @param prefix
 * @param suffix
 * @return
*/
template<typename PointT>
inline int savePointPCDs(
	vector<typename pcl::PointCloud<PointT>::Ptr>& pointCloudPtrVec,
	string basePath,
	vector<string> nameVec,
	string prefix,
	string suffix,
	bool saveASCII)
{
	for (int _i = 0; _i < pointCloudPtrVec.size(); ++_i) {
		string path(basePath);
		path.append("\\");
		path.append(prefix);
		path.append(nameVec[_i]);
		path.append(suffix);
		path.append(".pcd");
		if(saveASCII)pcl::io::savePCDFileASCII<PointT>(path, *(pointCloudPtrVec[_i]));
		else pcl::io::savePCDFileBinary<PointT>(path, *(pointCloudPtrVec[_i]));
	}
	return 0;
}

/**
 * @brief ���������Ƶ�PCD�ļ�
 * @tparam		PointT				��������
 * @param[in]	pointCloudPtrVec	Ҫ����ĵ�������
 * @param[in]	basePath			�����·��
 * @param[in]	prefix				����ǰ׺
 * @param[in]	suffix				���ֺ�׺������.�������չ����
 * @return 
*/
template<typename PointT>
inline int savePointPCDs(
	vector<typename pcl::PointCloud<PointT>::Ptr>& pointCloudPtrVec,
	string basePath,
	string prefix,
	string suffix,
	bool saveASCII)
{
	int indexStrSzie = 4;
	if (pointCloudPtrVec.size() > 10000) {
		int _temp = pointCloudPtrVec.size()/10000;
		while (_temp > 0) {
			++indexStrSzie;
			_temp /= 10;
		}
	}
	vector<string> nameVec(pointCloudPtrVec.size(), string(indexStrSzie, '0'));
	string indexStr;
	for (int _i = 0; _i < pointCloudPtrVec.size(); ++_i) {
		indexStr = to_string(_i);
		nameVec[_i].replace(indexStrSzie - indexStr.size(), indexStr.size(), indexStr);
	}
	for (auto& _ : nameVec) {
		cout << _ << endl;
	}
	savePointPCDs<PointT>(pointCloudPtrVec, basePath, nameVec, prefix, suffix, saveASCII);
	return 0;
}

/**
 * @brief ���ص�ǰ·���µ�����pcd�ļ�
 * @tparam		PointT				�������
 * @param[out]	pointCloudPtrVec	�洢���صĵ�������
 * @param[in]	basePath			·��
 * @return 
*/
template<typename PointT>
inline int loadPointPcds(
	vector<typename pcl::PointCloud<PointT>::Ptr>& pointCloudPtrVec,
	string basePath)
{
	vector<string> files;
	getFiles(basePath, files, "pcd", true);
	for (auto& _file : files) {
		typename pcl::PointCloud<PointT>::Ptr _temp;
		pcl::io::loadPCDFile(_file, *_temp);
		pointCloudPtrVec.push_back(_temp);
	}
	return 0;
}

/**
 * @brief ���뱳��
 * @param inPointCloudPtr 
 * @param out 
 * @return 
*/
int SeparateBG(
	const cloud::PointCloudPtr inPointCloudPtr,
	cloud::PointCloudPtr& out);

/**
 * @brief ȥ�������ӽǷ�Χ�ĵ�
 * @param[in]	inPointCloudPtr		�������
 * @param[out]	out					�������
 * @param[in]	angle				�ӽǴ�С
 * @param[in]	negative			ȥ���෴�ĵ�
 * @return 
*/
int SeparateView(
	const cloud::PointCloudPtr inPointCloudPtr,
	cloud::PointCloudPtr& out,
	float angle = 45,
	bool negative = false);

/**
 * @brief	ƴ���������ƣ��ں��ص�����
 *			cloud1�ڲ��ص�����ֱ�ӱ�����cloud1��Ե�ص���������Ȩֵ
 * @param[in]	cloud1		ƴ�ӵ���1
 * @param[in]	cloud2		ƴ�ӵ���2
 * @param[out]	resCloud	��������ƴ�ӽ��
 * @param[in]	distance	�ص������������ֵ
 * @return 
*/
int fuseTwoPointClouds(
	cloud::PointCloudPtr& cloud1,
	cloud::PointCloudPtr& cloud2,
	cloud::PointCloudPtr& resCloud,
	float distance = 0.002f,
	float edgeDistance = 0.01f,
	float normaldistance = 0.005f);

/**
 * @brief	ƴ���������ƣ��ں��ص�����
 *			cloud1�ڲ��ص�����ֱ�ӱ�����cloud1��Ե�ص���������Ȩֵ
 * @param[in]	inCloud1	ƴ�ӵ���1
 * @param[in]	inCloud2	ƴ�ӵ���2
 * @param[out]	outCloud1	����1ƴ�ӽ��
 * @param[out]	outCloud2	����2ƴ�ӽ��
 * @param[in]	distance	�ص������������ֵ
 * @return 
*/
int fuseTwoPointClouds(
	cloud::PointCloudPtr& inCloud1,
	cloud::PointCloudPtr& inCloud2,
	cloud::PointCloudPtr& outCloud1,
	cloud::PointCloudPtr& outCloud2,
	float distance = 0.002f,
	float edgeDistance = 0.01f,
	float normaldistance = 0.005f);

/**
 * @brief	ƴ���������ƣ��ں��ص�����
 *			cloud1�ڲ��ص�����ֱ�ӱ�����cloud1��Ե�ص���������Ȩֵ
 * @param[in][out]	cloud1		in ƴ�ӵ���1��out ����1ƴ�ӽ��
 * @param[in][out]	cloud2		in ƴ�ӵ���2��out ����2ƴ�ӽ��
 * @param[in]		distance	�ص������������ֵ
 * @return 
*/
int fuseTwoPointClouds(
	cloud::PointCloudPtr& cloud1,
	cloud::PointCloudPtr& cloud2,
	float distance= 0.002f,
	float edgeDistance = 0.01f,
	float normaldistance = 0.005f);

/**
 * @brief 
 * @param inPointCloudPtrVec 
 * @param outPointCloudPtr 
 * @param distance 
 * @return 
*/
int fusePointClouds(
	cloud::PointCloudPtrVec& inPointCloudPtrVec,
	cloud::PointCloudPtr& outPointCloudPtr,
	float distance = 0.002f,
	float edgeDistance = 0.01f,
	float normaldistance = 0.005f);

int edgeDetection(
	cloud::PointCloudNormalPtr& cloudPtr,
	float distance = 0.005f);

/**
 * @brief ����Ե�㲢���ñ�Եϵ��
 * @param[in][out]	cloudPtr			������ƣ���Ҫ����������Ϣ������Եϵ���洢��data_n[3]��
 * @param[out]		edgeIndicesPtr		λ�ڱ�Ե�ĵ������
 * @param[in]		distance			��Ե������
 * @return 
*/
int edgeDetection(
	cloud::PointCloudNormalPtr& cloudPtr,
	pcl::PointIndices::Ptr& edgeIndicesPtr,
	float distance = 0.005f);


int edgeViewer(
	cloud::PointCloudNormalPtr pointCloudNormalPtr);

float gaussian(
	float x,
	float u,
	float sigma);

/**
 * @brief ������Ʒֱ���
 * @param pointCloudPtr		�������
 * @return					�ֱ���
*/
float getResolution(
	cloud::PointCloudPtr pointCloudPtr);

void
NDTpair(
	cloud::PointCloudPtr srcPointCloudPtr,
	cloud::PointCloudPtr tgtPointCloudPtr,
	cloud::PointCloudPtr resPointCloudPtr,
	Eigen::Matrix4f& final_transformation,
	float transformationEpsilon = 0.01f,
	float stepSize = 0.1f,
	float resolution = 1.0f);

cloud::OBBT getOBB(
	cloud::PointCloudPtr cloudPtr);

void boxToPointCloud(
	const cloud::OBBT& box,
	cloud::PointCloudPtr& cloudPtr);

Eigen::Matrix4f getTransFromOBBT(
	cloud::OBBT box);

void segmenteByDis(
	cloud::PointCloudPtr& inCloudPtr,
	cloud::PointCloudPtrVec& outCloudPtrVec,
	float dis = 0.0,
	int min_num = 100,
	int max_num = 1000000);
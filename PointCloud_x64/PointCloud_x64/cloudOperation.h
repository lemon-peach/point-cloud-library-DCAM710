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

//struct EIGEN_ALIGN16 CustomPointT2
//{
//	float histogram[4] = { 0.f };
//	inline CustomPointT2(float _x, float _y, float _z, float _w) {
//		histogram[0] = _x;
//		histogram[1] = _y;
//		histogram[2] = _z;
//		histogram[3] = _w;
//	}
//	inline CustomPointT2() :CustomPointT2(0.0f, 0.0f, 0.0f, 0.0f) {};
//	inline CustomPointT2(const CustomPointT2& _p) :CustomPointT2(_p.histogram[0], _p.histogram[1], _p.histogram[2], _p.histogram[3]) {};
//	friend std::ostream& operator <<(std::ostream os, CustomPointT2& p) {
//		return os;
//	}
//	PCL_MAKE_ALIGNED_OPERATOR_NEW     // make sure our new allocators are aligned
//};                   // enforce SSE padding for correct memory alignment
//
//POINT_CLOUD_REGISTER_POINT_STRUCT(CustomPointT2,           // here we assume a XYZ + "test" (as fields)
//	(float[4], histogram, histogram)
//)

#include <ctime>
#include <cmath>
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
#include <pcl/features/fpfh_omp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/impl/icp_nl.hpp>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/surface/gp3.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/fast_bilateral.h>
#include <pcl/filters/bilateral.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/iss_3d.h>
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
using CustomPointT2 = pcl::FPFHSignature33;
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
	using PointCloudNormalVec = vector<PointCloudNormal>;
	using PointCloudNormalPtrVec = vector<PointCloudNormalPtr>;

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

	using PCLViewer = pcl::visualization::PCLVisualizer;
	using PCLViewerPtr = PCLViewer::Ptr;

	typedef struct {
		pcl::PointXYZ min;
		pcl::PointXYZ max;
		pcl::PointXYZ poistion;
		Eigen::Matrix3f rotation;
	}OBBT;
}
#define RED_COLOR cloud::Color({255.0,0.0,0.0})
#define GREEN_COLOR cloud::Color({ 0.0, 255.0, 0.0 })
#define BLUE_COLOR cloud::Color( { 0.0, 0.0, 255.0 })
#define YELLOW_COLOR cloud::Color({ 255.0, 255.0, 0.0 })
#define PURPLE_COLOR cloud::Color({ 255.0, 0.0, 255.0 })
#define CYAN_COLOR cloud::Color({ 0.0, 255.0, 255.0 })
#define WHITE_COLOR cloud::Color({ 255.0, 255.0, 255.0 })
#define BLACK_COLOR cloud::Color({ 0.0, 0.0, 0.0 })
#define COLOR1 cloud::Color({ 200,50,0 })
#define COLOR2 cloud::Color({ 150,100,0 })
#define COLOR3 cloud::Color({ 100,150,0 })
#define COLOR4 cloud::Color({ 50,200,0 })
#define COLOR5 cloud::Color({ 0,200,50 })
#define COLOR6 cloud::Color({ 0,150,100 })
#define COLOR7 cloud::Color({ 0,100,150 })
#define COLOR8 cloud::Color({ 0,50,200 })
#define COLOR_VEC vector<cloud::Color>({ RED_COLOR, GREEN_COLOR, BLUE_COLOR, YELLOW_COLOR, PURPLE_COLOR, CYAN_COLOR,\
COLOR1, COLOR2, COLOR3, COLOR4, COLOR5, COLOR6, COLOR7, COLOR8 })


template<typename PointT>
class MyPointRepresentationXYZ :public pcl::PointRepresentation<PointT> {
	using pcl::PointRepresentation<PointT>::nr_dimensions_;
public:
	using Ptr = shared_ptr<MyPointRepresentationXYZ>;
	MyPointRepresentationXYZ() {
		nr_dimensions_ = 3;
	}
	virtual void copyToFloatArray(const PointT& p, float* out) const
	{
		out[0] = p.x;
		out[1] = p.y;
		out[2] = p.z;
	}
};

template<typename PointT>
class MyPointRepresentationXYZCur :public pcl::PointRepresentation<PointT> {
	using pcl::PointRepresentation<PointT>::nr_dimensions_;
public:
	using Ptr = shared_ptr<MyPointRepresentationXYZCur>;
	MyPointRepresentationXYZCur() {
		nr_dimensions_ = 4;
	}
	virtual void copyToFloatArray(const PointT& p, float* out) const
	{
		out[0] = p.x;
		out[1] = p.y;
		out[2] = p.z;
		out[3] = p.curvature;
	}
};

template<typename PointT>
class MyPointRepresentationXYZNormalCur :public pcl::PointRepresentation<PointT> {
	using pcl::PointRepresentation<PointT>::nr_dimensions_;
public:
	using Ptr = shared_ptr<MyPointRepresentationXYZNormalCur>;
	MyPointRepresentationXYZNormalCur() {
		nr_dimensions_ = 7;
	}
	virtual void copyToFloatArray(const PointT& p, float* out) const
	{
		out[0] = p.x;
		out[1] = p.y;
		out[2] = p.z;
		out[3] = p.normal_x;
		out[4] = p.normal_x;
		out[5] = p.normal_x;
		out[6] = p.curvature;
	}
};
template <typename PointSource, typename PointTarget, typename FeatureT>
class SampleConsensusPrerejective2 : public pcl::SampleConsensusPrerejective<PointSource, PointTarget, FeatureT> {
public:
	using Matrix4 = typename pcl::Registration<PointSource, PointTarget>::Matrix4;

	using pcl::SampleConsensusPrerejective<PointSource, PointTarget, FeatureT>::reg_name_;
	using pcl::SampleConsensusPrerejective<PointSource, PointTarget, FeatureT>::getClassName;
	using pcl::SampleConsensusPrerejective<PointSource, PointTarget, FeatureT>::input_;
	using pcl::SampleConsensusPrerejective<PointSource, PointTarget, FeatureT>::target_;
	using pcl::SampleConsensusPrerejective<PointSource, PointTarget, FeatureT>::tree_;
	using pcl::SampleConsensusPrerejective<PointSource, PointTarget, FeatureT>::max_iterations_;
	using pcl::SampleConsensusPrerejective<PointSource, PointTarget, FeatureT>::corr_dist_threshold_;
	using pcl::SampleConsensusPrerejective<PointSource, PointTarget, FeatureT>::transformation_;
	using pcl::SampleConsensusPrerejective<PointSource, PointTarget, FeatureT>::final_transformation_;
	using pcl::SampleConsensusPrerejective<PointSource, PointTarget, FeatureT>::transformation_estimation_;
	using pcl::SampleConsensusPrerejective<PointSource, PointTarget, FeatureT>::getFitnessScore;
	using pcl::SampleConsensusPrerejective<PointSource, PointTarget, FeatureT>::converged_;
	using pcl::SampleConsensusPrerejective<PointSource, PointTarget, FeatureT>::input_features_;
	using pcl::SampleConsensusPrerejective<PointSource, PointTarget, FeatureT>::target_features_;
	using pcl::SampleConsensusPrerejective<PointSource, PointTarget, FeatureT>::inlier_fraction_;
	using pcl::SampleConsensusPrerejective<PointSource, PointTarget, FeatureT>::correspondence_rejector_poly_;
	using pcl::SampleConsensusPrerejective<PointSource, PointTarget, FeatureT>::k_correspondences_;
	using pcl::SampleConsensusPrerejective<PointSource, PointTarget, FeatureT>::nr_samples_;
	using pcl::SampleConsensusPrerejective<PointSource, PointTarget, FeatureT>::inliers_;
	using pcl::SampleConsensusPrerejective<PointSource, PointTarget, FeatureT>::getFitness;
	using pcl::SampleConsensusPrerejective<PointSource, PointTarget, FeatureT>::selectSamples;
	using pcl::SampleConsensusPrerejective<PointSource, PointTarget, FeatureT>::findSimilarFeatures;

	using PointCloudSource = typename pcl::Registration<PointSource, PointTarget>::PointCloudSource;
	using PointCloudSourcePtr = typename PointCloudSource::Ptr;
	using PointCloudSourceConstPtr = typename PointCloudSource::ConstPtr;

	using PointCloudTarget = typename pcl::Registration<PointSource, PointTarget>::PointCloudTarget;

	using PointIndicesPtr = pcl::PointIndices::Ptr;
	using PointIndicesConstPtr = pcl::PointIndices::ConstPtr;

	using FeatureCloud = pcl::PointCloud<FeatureT>;
	using FeatureCloudPtr = typename FeatureCloud::Ptr;
	using FeatureCloudConstPtr = typename FeatureCloud::ConstPtr;

	using Ptr = shared_ptr<SampleConsensusPrerejective2<PointSource, PointTarget, FeatureT> >;
	using ConstPtr = shared_ptr<const SampleConsensusPrerejective2<PointSource, PointTarget, FeatureT> >;

	using FeatureKdTreePtr = typename pcl::KdTreeFLANN<FeatureT>::Ptr;

	using CorrespondenceRejectorPoly = pcl::registration::CorrespondenceRejectorPoly<PointSource, PointTarget>;
	using CorrespondenceRejectorPolyPtr = typename CorrespondenceRejectorPoly::Ptr;
	using CorrespondenceRejectorPolyConstPtr = typename CorrespondenceRejectorPoly::ConstPtr;
	vector<int> sourceCoorIndices;
	vector<int> targetCoorIndices;
	void computeTransformation(PointCloudSource& output, const Eigen::Matrix4f& guess)override;
};
template <typename PointSource, typename PointTarget, typename FeatureT> void
SampleConsensusPrerejective2<PointSource, PointTarget, FeatureT>::computeTransformation(PointCloudSource& output, const Eigen::Matrix4f& guess)
{
	// Some sanity checks first
	if (!input_features_)
	{
		PCL_ERROR("[pcl::%s::computeTransformation] ", getClassName().c_str());
		PCL_ERROR("No source features were given! Call setSourceFeatures before aligning.\n");
		return;
	}
	if (!target_features_)
	{
		PCL_ERROR("[pcl::%s::computeTransformation] ", getClassName().c_str());
		PCL_ERROR("No target features were given! Call setTargetFeatures before aligning.\n");
		return;
	}

	if (input_->size() != input_features_->size())
	{
		PCL_ERROR("[pcl::%s::computeTransformation] ", getClassName().c_str());
		PCL_ERROR("The source points and source feature points need to be in a one-to-one relationship! Current input cloud sizes: %ld vs %ld.\n",
			input_->size(), input_features_->size());
		return;
	}

	if (target_->size() != target_features_->size())
	{
		PCL_ERROR("[pcl::%s::computeTransformation] ", getClassName().c_str());
		PCL_ERROR("The target points and target feature points need to be in a one-to-one relationship! Current input cloud sizes: %ld vs %ld.\n",
			target_->size(), target_features_->size());
		return;
	}

	if (inlier_fraction_ < 0.0f || inlier_fraction_ > 1.0f)
	{
		PCL_ERROR("[pcl::%s::computeTransformation] ", getClassName().c_str());
		PCL_ERROR("Illegal inlier fraction %f, must be in [0,1]!\n",
			inlier_fraction_);
		return;
	}

	const float similarity_threshold = correspondence_rejector_poly_->getSimilarityThreshold();
	if (similarity_threshold < 0.0f || similarity_threshold >= 1.0f)
	{
		PCL_ERROR("[pcl::%s::computeTransformation] ", getClassName().c_str());
		PCL_ERROR("Illegal prerejection similarity threshold %f, must be in [0,1[!\n",
			similarity_threshold);
		return;
	}

	if (k_correspondences_ <= 0)
	{
		PCL_ERROR("[pcl::%s::computeTransformation] ", getClassName().c_str());
		PCL_ERROR("Illegal correspondence randomness %d, must be > 0!\n",
			k_correspondences_);
		return;
	}

	// Initialize prerejector (similarity threshold already set to default value in constructor)
	correspondence_rejector_poly_->setInputSource(input_);
	correspondence_rejector_poly_->setInputTarget(target_);
	correspondence_rejector_poly_->setCardinality(nr_samples_);
	int num_rejections = 0; // For debugging

	// Initialize results
	final_transformation_ = guess;
	inliers_.clear();
	float lowest_error = std::numeric_limits<float>::max();
	converged_ = false;

	// Temporaries
	std::vector<int> inliers;
	float inlier_fraction;
	float error;

	// If guess is not the Identity matrix we check it
	if (!guess.isApprox(Eigen::Matrix4f::Identity(), 0.01f))
	{
		getFitness(inliers, error);
		inlier_fraction = static_cast<float> (inliers.size()) / static_cast<float> (input_->size());

		if (inlier_fraction >= inlier_fraction_ && error < lowest_error)
		{
			inliers_ = inliers;
			lowest_error = error;
			converged_ = true;
		}
	}

	// Feature correspondence cache
	std::vector<std::vector<int> > similar_features(input_->size());

	// Start
	for (int i = 0; i < max_iterations_; ++i)
	{
		// Temporary containers
		std::vector<int> sample_indices;
		std::vector<int> corresponding_indices;

		// Draw nr_samples_ random samples
		selectSamples(*input_, nr_samples_, sample_indices);

		// Find corresponding features in the target cloud
		findSimilarFeatures(sample_indices, similar_features, corresponding_indices);

		// Apply prerejection
		if (!correspondence_rejector_poly_->thresholdPolygon(sample_indices, corresponding_indices))
		{
			++num_rejections;
			continue;
		}

		// Estimate the transform from the correspondences, write to transformation_
		transformation_estimation_->estimateRigidTransformation(*input_, sample_indices, *target_, corresponding_indices, transformation_);

		// Take a backup of previous result
		const Matrix4 final_transformation_prev = final_transformation_;

		// Set final result to current transformation
		final_transformation_ = transformation_;

		// Transform the input and compute the error (uses input_ and final_transformation_)
		getFitness(inliers, error);

		// Restore previous result
		final_transformation_ = final_transformation_prev;

		// If the new fit is better, update results
		inlier_fraction = static_cast<float> (inliers.size()) / static_cast<float> (input_->size());

		// Update result if pose hypothesis is better
		if (inlier_fraction >= inlier_fraction_ && error < lowest_error)
		{
			inliers_ = inliers;
			lowest_error = error;
			converged_ = true;
			final_transformation_ = transformation_;
			sourceCoorIndices.resize(sample_indices.size());
			targetCoorIndices.resize(sample_indices.size());
			for (int _index = 0; _index < sample_indices.size(); ++_index) {
				sourceCoorIndices[_index] = sample_indices[_index];
				targetCoorIndices[_index] = corresponding_indices[_index];
			}
		}
	}


	// Apply the final transformation
	if (converged_)
		transformPointCloud(*input_, output, final_transformation_);

	// Debug output
	PCL_DEBUG("[pcl::%s::computeTransformation] Rejected %i out of %i generated pose hypotheses.\n",
		getClassName().c_str(), num_rejections, max_iterations_);
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
pcl::visualization::PCLVisualizer::Ptr
visualizePointCloud(
	const cloud::PointCloudPtr& pointCloud,
	const cloud::Color pointColor = { 255.0, 255.0, 255.0 },
	const int pointSize = 1,
	const cloud::Color backgroundColor = { 0.0, 0.0, 0.0 },
	bool showCoordinateSystem = true,
	bool is_auto = true);

/**
 * @brief			可视化多个点云
 * @param[in]		pointCloudPtrVec	点云向量
 * @param[in][out]	pointColorVec		点云颜色，少于点云数量时自动补足为白色
 * @param[in]		backgroundColor		背景颜色
 * @param[in]		is_auto				显示阻塞
 * @return			PCLVisualizer实例
*/
pcl::visualization::PCLVisualizer::Ptr
visualizePointCloud(
	const cloud::PointCloudPtrVec& pointCloudPtrVec,
	vector<cloud::Color> pointColorVec,
	vector<int> pointSizeVec = {1},
	const cloud::Color backgroundColor = { 0.0, 0.0, 0.0 },
	bool showCoordinateSystem = true,
	bool is_auto = true);

pcl::visualization::PCLVisualizer::Ptr
visualizePointCloud(
	const vector<cloud::PointCloudNormalPtr>& pointCloudPtrVec,
	vector<cloud::Color> pointColorVec,
	vector<int> pointSizeVec = { 1 },
	const cloud::Color backgroundColor = { 0.0, 0.0, 0.0 },
	bool showCoordinateSystem = true,
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
pcl::visualization::PCLVisualizer::Ptr
visualizePointCloud(
	const cloud::PointCloudPtr& pointCloudPtr,
	const pcl::PolygonMesh& mesh,
	const cloud::Color pointColor = { 255.0, 255.0, 255.0 },
	int pointSize = 1,
	const cloud::Color backgroundColor = { 0.0, 0.0, 0.0 },
	bool showCoordinateSystem = true,
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
pcl::visualization::PCLVisualizer::Ptr
visualizePointCloud(
	const cloud::PointCloudPtrVec& pointCloudPtrVec,
	const vector<pcl::PolygonMesh>& meshVec,
	vector<cloud::Color> pointColorVec,
	vector<int> pointSizeVec = {1},
	const cloud::Color backgroundColor = { 0.0, 0.0, 0.0 },
	bool showCoordinateSystem = true,
	bool is_auto = true);

void visualizeCorrespondences(
	cloud::PointCloudNormalPtr& cloud1,
	cloud::PointCloudNormalPtr& cloud2,
	pcl::CorrespondencesPtr& coor,
	pcl::visualization::PCLVisualizer::Ptr& viewer,
	int pointSize = 3,
	bool showNormal = false);

void visualizeCorrespondences(
	cloud::PointCloudPtr& cloud1,
	cloud::PointCloudPtr& cloud2,
	pcl::CorrespondencesPtr& coor,
	pcl::visualization::PCLVisualizer::Ptr& viewer,
	int pointSize=2);

void visualizeNormal(
	cloud::PointCloudNormalPtr& cloudNoramlPtr,
	pcl::visualization::PCLVisualizer::Ptr& viewer,
	int pointSize = 3);

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

int
fastBilateralFilter(
	const cloud::PointCloudNormalPtr& inPointCloudPtr,
	cloud::PointCloudNormalPtr& outPointCloudPtr,
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
	const cloud::PointCloudNormalPtr& srcCloudPtr, 
	const cloud::PointCloudNormalPtr& tgtCloudPtr,
	Eigen::Matrix4f& final_transformation,
	bool downSample = false,
	float downSampleSize = 0.005f,
	float maxDis = 0.02,
	bool useMyEm=false,
	bool edgeFilter = false,
	int iterations1 = 8,
	int iterations2 = 256,
	float EuclideanFitnessEpsilon = 5e-7);

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
	bool downSample = false,
	float downSampleSize = 0.005f,
	float maxDis = 0.05);

int
pairAlignWithCustom(
	const cloud::PointCloudNormalPtr& srcCloudPtr,
	const cloud::PointCloudNormalPtr& tgtCloudPtr,
	Eigen::Matrix4f& final_transformation,
	bool downSample = false,
	float downSampleSize = 0.005f,
	float maxDis = 0.02,
	bool useMyEm = false,
	bool edgeFilter = false,
	int iterations1 = 8,
	int iterations2 = 256,
	float EuclideanFitnessEpsilon = 5e-7);

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

int
getSIFTKeypoint(
	const cloud::PointCloudNormalPtr& pointCloudNormalPtr,
	cloud::PointCloudPtr& result
);

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


/**********************************实现*************************************/
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
 * @brief 保存多个点云到PCD文件
 * @tparam		PointT				点云类型
 * @param[in]	pointCloudPtrVec	要保存的点云向量
 * @param[in]	basePath			保存的路径
 * @param[in]	prefix				名字前缀
 * @param[in]	suffix				名字后缀，不含.后面的扩展名字
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
 * @brief 加载当前路径下的所有pcd文件
 * @tparam		PointT				点的类型
 * @param[out]	pointCloudPtrVec	存储加载的点云向量
 * @param[in]	basePath			路径
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
 * @brief 分离背景
 * @param inPointCloudPtr 
 * @param out 
 * @return 
*/
int SeparateBG(
	const cloud::PointCloudPtr inPointCloudPtr,
	cloud::PointCloudPtr& out);

/**
 * @brief 去除超过视角范围的点
 * @param[in]	inPointCloudPtr		输入点云
 * @param[out]	out					输出点云
 * @param[in]	angle				视角大小
 * @param[in]	negative			去除相反的点
 * @return 
*/
int SeparateView(
	const cloud::PointCloudPtr inPointCloudPtr,
	cloud::PointCloudPtr& out,
	float angle = 45,
	bool negative = false);

/**
 * @brief	拼接两个点云，融合重叠区域
 *			cloud1内部重叠区域直接保留，cloud1边缘重叠区域计算加权值
 * @param[in]	cloud1		拼接点云1
 * @param[in]	cloud2		拼接点云2
 * @param[out]	resCloud	两幅点云拼接结果
 * @param[in]	distance	重叠区域检测距离阈值
 * @return 
*/
int fuseTwoPointClouds(
	cloud::PointCloudNormalPtr& cloud1,
	cloud::PointCloudNormalPtr& cloud2,
	cloud::PointCloudNormalPtr& resCloud,
	float distance = 0.002f,
	float edgeDistance = 0.01f);

/**
 * @brief	拼接两个点云，融合重叠区域
 *			cloud1内部重叠区域直接保留，cloud1边缘重叠区域计算加权值
 * @param[in]	inCloud1	拼接点云1
 * @param[in]	inCloud2	拼接点云2
 * @param[out]	outCloud1	点云1拼接结果
 * @param[out]	outCloud2	点云2拼接结果
 * @param[in]	distance	重叠区域检测距离阈值
 * @return 
*/
int fuseTwoPointClouds(
	cloud::PointCloudNormalPtr& inCloud1,
	cloud::PointCloudNormalPtr& inCloud2,
	cloud::PointCloudNormalPtr& outCloud1,
	cloud::PointCloudNormalPtr& outCloud2,
	float distance = 0.002f,
	float edgeDistance = 0.01f);

/**
 * @brief	拼接两个点云，融合重叠区域
 *			cloud1内部重叠区域直接保留，cloud1边缘重叠区域计算加权值
 * @param[in][out]	cloud1		in 拼接点云1，out 点云1拼接结果
 * @param[in][out]	cloud2		in 拼接点云2，out 点云2拼接结果
 * @param[in]		distance	重叠区域检测距离阈值
 * @return 
*/
int fuseTwoPointClouds(
	cloud::PointCloudNormalPtr& cloud1,
	cloud::PointCloudNormalPtr& cloud2,
	float distance= 0.002f,
	float edgeDistance = 0.01f);

/**
 * @brief 
 * @param inPointCloudPtrVec 
 * @param outPointCloudPtr 
 * @param distance 
 * @return 
*/
int fusePointClouds(
	cloud::PointCloudNormalPtrVec& inPointCloudPtrVec,
	cloud::PointCloudNormalPtr& outPointCloudPtr,
	float distance = 0.002f,
	float edgeDistance = 0.01f);

int edgeDetection(
	cloud::PointCloudNormalPtr& cloudPtr,
	float distance = 0.005f);

/**
 * @brief 检测边缘点并设置边缘系数
 * @param[in][out]	cloudPtr			输入点云（需要包含法线信息），边缘系数存储于data_n[3]中
 * @param[out]		edgeIndicesPtr		位于边缘的点的索引
 * @param[in]		distance			边缘检测距离
 * @return 
*/
int edgeDetection(
	cloud::PointCloudNormalPtr& cloudPtr,
	pcl::PointIndices::Ptr& edgeIndicesPtr,
	float distance = 0.005f,
	float min = 0.2f,
	float max = 0.5f);


int edgeViewer(
	cloud::PointCloudNormalPtr pointCloudNormalPtr);

float gaussian(
	float x,
	float u,
	float sigma);

/**
 * @brief 计算点云分辨率
 * @param pointCloudPtr		输入点云
 * @return					分辨率
*/
template<typename PointT>
float getResolution(
	typename pcl::PointCloud<PointT>::Ptr pointCloudPtr)
{
	pcl::KdTreeFLANN<PointT> kdtree;
	vector<int> pointIdx(2);
	vector<float> pointSquaredDistance(2);
	float sumDis = 0.0f;
	typename MyPointRepresentationXYZ<PointT>::Ptr PointRepresentation(new MyPointRepresentationXYZ<PointT>);

	kdtree.setInputCloud(pointCloudPtr);
	kdtree.setSortedResults(true);
	kdtree.setPointRepresentation(PointRepresentation);
	for (size_t _index = 0; _index < pointCloudPtr->size(); ++_index) {
		kdtree.nearestKSearch((*pointCloudPtr).points[_index], 2, pointIdx, pointSquaredDistance);
		sumDis += pow(pointSquaredDistance[1], 0.5f);
	}
	return sumDis / (float)pointCloudPtr->size();
}

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

void segmenteByDis(
	cloud::PointCloudNormalPtr& inCloudPtr,
	cloud::PointCloudNormalPtrVec& outCloudPtrVec,
	float dis = 0.0,
	int min_num = 100,
	int max_num = 1000000);


inline int removeInvalid(
	cloud::PointCloudNormalPtr& cloudPtr)
{
	MyPointRepresentationXYZNormalCur<pcl::PointNormal>::Ptr representationPtr(new MyPointRepresentationXYZNormalCur<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr temp(new cloud::PointCloudNormal);
	size_t index = 0;
	temp->resize(cloudPtr->size());
	bool showInfo = true;
	for (auto& _point : *cloudPtr) {
		if (representationPtr->isValid(_point)) {
			(*(temp))[index] = _point;
			++index;
		}
		else if(showInfo){
			cout << "has invalid point" << endl;
		}
	}
	temp->resize(index);
	temp->width = index;
	temp->height = 1;
	temp->is_dense = true;
	pcl::copyPointCloud(*temp, *cloudPtr);
	return 0;
}

inline int removeInvalid(
	cloud::PointCloudPtr& cloudPtr)
{
	cloud::PointCloudPtr temp(new cloud::PointCloud);
	MyPointRepresentationXYZ<cloud::PointT> pointRepresentationXYZ;
	size_t index = 0;
	temp->resize(cloudPtr->size());
	for (auto& _point : *cloudPtr) {
		if (pointRepresentationXYZ.isValid(_point)) {
			(*(temp))[index] = _point;
			++index;
		}
	}
	temp->resize(index);
	temp->width = index;
	temp->height = 1;
	temp->is_dense = true;
	pcl::copyPointCloud(*temp, *cloudPtr);
	return 0;
}

void removeDiagonally(
	cloud::PointCloudNormalPtr& inCloudPtr,
	cloud::PointCloudNormalPtr& outCloudPtr,
	float th);

template<typename PointT>
float calcMSE(
	typename pcl::PointCloud<PointT>::Ptr& _srcCloudPtr,
	typename pcl::PointCloud<PointT>::Ptr& _tgtCloudPtr,
	pcl::CorrespondencesPtr& coors)
{
	float MSE = 0;
	for (auto _coor : *coors) {
		MSE += pow(_srcCloudPtr->at(_coor.index_query).x - _tgtCloudPtr->at(_coor.index_match).x, 2.0) +
			pow(_srcCloudPtr->at(_coor.index_query).y - _tgtCloudPtr->at(_coor.index_match).y, 2.0) +
			pow(_srcCloudPtr->at(_coor.index_query).z - _tgtCloudPtr->at(_coor.index_match).z, 2.0);
	}
	MSE /= coors->size();
	return MSE;
}

void ISSKeypoints(
	cloud::PointCloudNormalPtr& inCloud,
	cloud::PointCloudNormalPtr& outCloud);

void curKeypoints(
	cloud::PointCloudNormalPtr inCloud,
	cloud::PointCloudNormalPtr outCloud,
	float dis,
	int divNum = 10,
	float numTh = 4);

void Keypoints2(
	cloud::PointCloudNormalPtr inCloud,
	cloud::PointCloudNormalPtr outCloud,
	float dis,
	float minDis,
	float Th =0.45,
	float maxPersent = 0.7,
	float minPersent = 0.4);

void Keypoints3(
	cloud::PointCloudNormalPtr& inCloud,
	pcl::Indices& outIndices,
	pcl::PointCloud<CustomPointT2>& cloudFeature,
	float dis,
	float dis2,
	float alphaTh,
	float alphaTh2,
	float betaTh,
	float segDis,
	int minSize,
	int edgeN,
	bool conver=true);

int keyViewer(
	cloud::PointCloudNormalPtr pointCloudNormalPtr);

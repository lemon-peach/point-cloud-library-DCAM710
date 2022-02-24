#include "cloudOperation.h"
//#include "function.h"
#include <vtkRenderWindow.h>

namespace cloud {
	void viewerOneOff(pcl::visualization::PCLVisualizer& viewer) {
		viewer.setBackgroundColor(1.0, 0.5, 1.0);
	}
}

/**
 * @brief		pcl可视化测试
 * @param[in]	filePath pcd	文件路径
 * @return		0：成功；1：失败
*/
int 
pclViewerTest(
	string filePath) 
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_test(new pcl::PointCloud<pcl::PointXYZ>);

	if (-1 == pcl::io::loadPCDFile(filePath, *cloud_test)) {
		cout << "error input!" << endl;
		return -1;
	}

	cout << cloud_test->points.size() << endl;
	cout << "width: " << cloud_test->width << "heigth: " << cloud_test->height << endl;
	pcl::visualization::CloudViewer viewer("Cloud Viewer");
	for (int i = 0; i < 100; i++) {
		cout << cloud_test->at(i).x << "\t" << cloud_test->at(i).y << "\t" << cloud_test->at(i).z << endl;
	}

	viewer.showCloud(cloud_test);
	viewer.runOnVisualizationThreadOnce(cloud::viewerOneOff);
	cout << "show" << endl;
	system("pause");
	return 0;
}

/**
 * @brief		生成随机点云
 * @param[out]	cloudPtr	点云
 * @param[in]	width		点云的宽
 * @param[in]	height		点云的高
 * @return
*/
void 
randomCloud(
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr, 
	int width, 
	int height) 
{
	cloudPtr->width = width;
	cloudPtr->height = height;
	cloudPtr->resize(cloudPtr->width * cloudPtr->height);
	for (auto& point : *cloudPtr) {
		point.x = 1024 * rand() / (RAND_MAX + 1.0f);
		point.y = 1024 * rand() / (RAND_MAX + 1.0f);
		point.z = 1024 * rand() / (RAND_MAX + 1.0f);
	}
}

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
	pcl::PointCloud<pcl::Normal>::Ptr& outNormalPtr,
	int k,
	float r)
{
	pcl::NormalEstimation<cloud::PointT, pcl::Normal> normal_est;
	pcl::search::KdTree<cloud::PointT>::Ptr kdtree(new pcl::search::KdTree<cloud::PointT>);
	normal_est.setSearchMethod(kdtree);
	if (k) normal_est.setKSearch(k);
	else normal_est.setRadiusSearch(r);
	normal_est.setInputCloud(inCloudPtr);
	normal_est.compute(*outNormalPtr);
	return 0;
}

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
	int k,
	float r)
{
	pcl::PointCloud<pcl::Normal>::Ptr normalPtr(new pcl::PointCloud<pcl::Normal>);
	normalEstimation(inCloudPtr, normalPtr, k, r);
	pcl::concatenateFields(*inCloudPtr, *normalPtr, *outCloudNormalPtr);
	return 0;
}

/**
 * @brief		使用 RANSAC 方法进行平面分割
 * @param[in]	threshold		阈值
 * @param[in]	srcCloudPtr		原始点云
 * @param[out]	dstCloudPtr		分割后的点云
 * @param[in]	negative		提取相反的点云
 * @return		0：成功	-1：失败
*/
int
planeSegmentation(
	float threshold,
	cloud::PointCloudPtr& srcCloudPtr,
	cloud::PointCloudPtr& dstCloudPtr,
	bool negative)
{
	pcl::ModelCoefficients modelCoefficients;
	pcl::PointIndices::Ptr inliersPtr(new pcl::PointIndices);
	pcl::SACSegmentation<cloud::PointT> seg;

	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(threshold);
	seg.setInputCloud(srcCloudPtr);

	seg.segment(*inliersPtr, modelCoefficients);

	if (inliersPtr->indices.size() == 0) {
		cout << "Could not segment!" << endl;
		return -1;
	}

	pcl::ExtractIndices<cloud::PointT> extract;
	extract.setInputCloud(srcCloudPtr);
	extract.setNegative(negative);
	extract.setIndices(inliersPtr);
	extract.filter(*dstCloudPtr);
	return 0;
}

/**
 * @brief	使用 RANSAC 方法进行平面分割
 * @param	threshold	阈值
 * @param	srcCloudPtr 原始点云
 * @param	inliers		分割后的点云索引
 * @return	0：成功	-1：失败
*/
int
planeSegmentation(
	float threshold,
	cloud::PointCloudPtr& srcCloudPtr,
	pcl::PointIndices& inliers)
{
	pcl::ModelCoefficients modelCoefficients;
	pcl::SACSegmentation<cloud::PointT> seg;

	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(threshold);
	seg.setInputCloud(srcCloudPtr);

	seg.segment(inliers, modelCoefficients);

	if (inliers.indices.size() == 0) {
		cout << "Could not segment!" << endl;
		return -1;
	}
	return 0;
}

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
	pcl::PointCloud<cloud::PointT>::Ptr& transformedCloudPtr)
{
	return icpRegistration(
		srcCloudPtr,
		tgtCloudPtr,
		transformation,
		transformedCloudPtr,
		Eigen::Matrix4f::Identity());
}

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
	const Eigen::Matrix4f& guess)
{
	pcl::IterativeClosestPoint<cloud::PointT, cloud::PointT> icp;
	icp.setInputSource(srcCloudPtr);
	icp.setInputTarget(tgtCloudPtr);
	icp.align(*transformedCloudPtr, guess);
	transformation = icp.getFinalTransformation();
	return 0;
}

/**
 * @brief		配准：计算源点云到目标点云的变换矩阵，根据点进行配准
 * @param[in]	srcCloudPtr				源点云
 * @param[in]	tgtCloudPtr				目标点云
 * @param[out]	resCloudPtr				源点云经变换后的配准点云
 * @param[out]	final_transformation	最终的变换矩阵
 * @param[in]	downSample				是否对输入点云进行下采样
 * @return
*/
//int
//pairAlign(
//	const cloud::PointCloudPtr& srcCloudPtr,
//	const cloud::PointCloudPtr& tgtCloudPtr,
//	cloud::PointCloudPtr& resCloudPtr,
//	Eigen::Matrix4f& final_transformation,
//	bool downSample)
//{
//	//自定义 Point Representation
//	class MyPointRepresentation :public pcl::PointRepresentation<cloud::PointNormalT> {
//		using pcl::PointRepresentation<cloud::PointNormalT>::nr_dimensions_;
//	public:
//		MyPointRepresentation() {
//			nr_dimensions_ = 4;
//		}
//		virtual void copyToFloatArray(const cloud::PointNormalT& p, float* out) const
//		{
//			out[0] = p.x;
//			out[1] = p.y;
//			out[2] = p.z;
//			out[3] = p.curvature;
//			//bool er = false;
//			//for (int i = 0; i < nr_dimensions_; i++) {
//			//	if (!std::isfinite(out[i])) {
//			//		er = true;
//			//	}
//			//}
//			//if (er) {
//			//	cout << "copyToFloatArray er" << endl;
//			//	for (int i = 0; i < nr_dimensions_; i++) {
//			//		cout << out[i] << "\t";
//			//	}
//			//	cout << endl;
//			//}
//		}
//	};
//
//	cloud::PointCloudPtr _srcCloudPtr(new cloud::PointCloud);  //源点云下采样
//	cloud::PointCloudPtr _tgtCloudPtr(new cloud::PointCloud);  //目标点云下采样
//	//下采样
//	if (downSample) {
//		pcl::VoxelGrid<cloud::PointT> grid;
//		grid.setLeafSize(0.01, 0.01, 0.01);
//		grid.setInputCloud(srcCloudPtr);
//		grid.filter(*_srcCloudPtr);
//		grid.setInputCloud(tgtCloudPtr);
//		grid.filter(*_tgtCloudPtr);
//	}
//	else {
//		_srcCloudPtr = srcCloudPtr;
//		_tgtCloudPtr = tgtCloudPtr;
//	}
//
//	//计算法线和曲率
//	pcl::NormalEstimation<cloud::PointT, cloud::PointNormalT> normal_est;
//	pcl::search::KdTree<cloud::PointT>::Ptr kdtree(new pcl::search::KdTree<cloud::PointT>());
//	normal_est.setSearchMethod(kdtree);  //设置搜索方法
//	normal_est.setKSearch(30);  //设置 K 个临近点用于计算法线和曲率
//
//	normal_est.setInputCloud(_srcCloudPtr);
//	cloud::PointCloudNormalPtr _srcCloudNormalPtr(new cloud::PointCloudNormal);
//	normal_est.compute(*_srcCloudNormalPtr);
//	pcl::copyPointCloud(*_srcCloudPtr, *_srcCloudNormalPtr);
//
//	normal_est.setInputCloud(_tgtCloudPtr);
//	cloud::PointCloudNormalPtr _tgtCloudNormalPtr(new cloud::PointCloudNormal);
//	normal_est.compute(*_tgtCloudNormalPtr);
//	pcl::copyPointCloud(*_tgtCloudPtr, *_tgtCloudNormalPtr);
//
//	MyPointRepresentation point_representation;
//	float alpha[4] = { 1.0, 1.0, 1.0, 1.0 };
//	point_representation.setRescaleValues(alpha);
//
//	pcl::IterativeClosestPointNonLinear<cloud::PointNormalT, cloud::PointNormalT> icp_nl;  //实例 ICP 对象
//	//icp_nl.setEuclideanFitnessEpsilon(1e-6);
//	icp_nl.setTransformationEpsilon(1e-6);  //设置两个临近变换的最大平方差
//	icp_nl.setMaxCorrespondenceDistance(0.1);  //设置源点与目标点的最大匹配距离(米)
//	icp_nl.setPointRepresentation(pcl::make_shared<const MyPointRepresentation>(point_representation));
//	icp_nl.setInputSource(_srcCloudNormalPtr);
//	icp_nl.setInputTarget(_tgtCloudNormalPtr);
//
//	//多次配准
//	Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity(), pre_transformation;
//	cloud::PointCloudNormalPtr transCloudNormalPtr = _srcCloudNormalPtr;
//	icp_nl.setMaximumIterations(2);  //设置迭代次数
//	for (int i = 0; i < 64; ++i) {
//		icp_nl.setInputSource(_srcCloudNormalPtr);
//		icp_nl.align(*transCloudNormalPtr);
//		transformation = icp_nl.getFinalTransformation() * transformation;
//
//		//if (abs((icp_nl.getLastIncrementalTransformation() - pre_transformation).sum()) < icp_nl.getTransformationEpsilon()) {
//		//	icp_nl.setMaxCorrespondenceDistance(icp_nl.getMaxCorrespondenceDistance() - 0.005);
//		//}
//		pre_transformation = transformation;
//		_srcCloudNormalPtr = transCloudNormalPtr;
//	}
//	final_transformation = transformation;
//	cout << "==============" << endl;
//	cout << "in" << endl;
//	cout << transformation << endl;
//	pcl::transformPointCloud(*srcCloudPtr, *resCloudPtr, final_transformation);
//	return 0;
//}

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
	bool downSample)
{
	//自定义 Point Representation
	class MyPointRepresentationNormal :public pcl::PointRepresentation<cloud::PointNormalT> {
		using pcl::PointRepresentation<cloud::PointNormalT>::nr_dimensions_;
	public:
		MyPointRepresentationNormal() {
			nr_dimensions_ = 4;
		}
		virtual void copyToFloatArray(const cloud::PointNormalT& p, float* out) const
		{
			out[0] = p.x;
			out[1] = p.y;
			out[2] = p.z;
			out[3] = p.curvature;
		}
	};

	cloud::PointCloudPtr _srcCloudPtr(new cloud::PointCloud);  //源点云下采样
	cloud::PointCloudPtr _tgtCloudPtr(new cloud::PointCloud);  //目标点云下采样
	//下采样
	if (downSample) {
		pcl::VoxelGrid<cloud::PointT> grid;
		grid.setLeafSize(0.01, 0.01, 0.01);
		grid.setInputCloud(srcCloudPtr);
		grid.filter(*_srcCloudPtr);
		grid.setInputCloud(tgtCloudPtr);
		grid.filter(*_tgtCloudPtr);
	}
	else {
		_srcCloudPtr = srcCloudPtr;
		_tgtCloudPtr = tgtCloudPtr;
	}

	//计算法线和曲率
	pcl::NormalEstimation<cloud::PointT, cloud::PointNormalT> normal_est;
	pcl::search::KdTree<cloud::PointT>::Ptr kdtree(new pcl::search::KdTree<cloud::PointT>());
	normal_est.setSearchMethod(kdtree);  //设置搜索方法
	normal_est.setKSearch(30);  //设置 K 个临近点用于计算法线和曲率

	normal_est.setInputCloud(_srcCloudPtr);
	cloud::PointCloudNormalPtr _srcCloudNormalPtr(new cloud::PointCloudNormal);
	normal_est.compute(*_srcCloudNormalPtr);
	pcl::copyPointCloud(*_srcCloudPtr, *_srcCloudNormalPtr);

	normal_est.setInputCloud(_tgtCloudPtr);
	cloud::PointCloudNormalPtr _tgtCloudNormalPtr(new cloud::PointCloudNormal);
	normal_est.compute(*_tgtCloudNormalPtr);
	pcl::copyPointCloud(*_tgtCloudPtr, *_tgtCloudNormalPtr);

	MyPointRepresentationNormal point_representation;
	float alpha[4] = { 1.0, 1.0, 1.0, 1.0 };
	point_representation.setRescaleValues(alpha);

	pcl::IterativeClosestPointNonLinear<cloud::PointNormalT, cloud::PointNormalT> icp_nl;  //实例 ICP 对象
	//icp_nl.setEuclideanFitnessEpsilon(1e-6);
	icp_nl.setTransformationEpsilon(1e-6);  //设置两个临近变换的最大平方差
	icp_nl.setMaxCorrespondenceDistance(0.1);  //设置源点与目标点的最大匹配距离(米)
	icp_nl.setPointRepresentation(pcl::make_shared<const MyPointRepresentationNormal>(point_representation));
	icp_nl.setInputSource(_srcCloudNormalPtr);
	icp_nl.setInputTarget(_tgtCloudNormalPtr);

	//多次配准
	Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity(), pre_transformation;
	cloud::PointCloudNormalPtr transCloudNormalPtr = _srcCloudNormalPtr;
	icp_nl.setMaximumIterations(2);  //设置迭代次数
	for (int i = 0; i < 64; ++i) {
		icp_nl.setInputSource(_srcCloudNormalPtr);
		icp_nl.align(*transCloudNormalPtr);
		transformation = icp_nl.getFinalTransformation() * transformation;

		if (abs((icp_nl.getLastIncrementalTransformation() - pre_transformation).sum()) < icp_nl.getTransformationEpsilon()) {
			icp_nl.setMaxCorrespondenceDistance(icp_nl.getMaxCorrespondenceDistance() - 0.005);
		}
		pre_transformation = transformation;
		_srcCloudNormalPtr = transCloudNormalPtr;
	}
	final_transformation = transformation;
	pcl::transformPointCloud(*srcCloudPtr, *resCloudPtr, final_transformation);
	return 0;
}

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
	bool downSample)
{
	cloud::PointCloudPtr _srcCloudPtr(new cloud::PointCloud);  //源点云下采样
	cloud::PointCloudPtr _tgtCloudPtr(new cloud::PointCloud);  //目标点云下采样
	//下采样
	if (downSample) {
		pcl::VoxelGrid<cloud::PointT> grid;
		grid.setLeafSize(0.01, 0.01, 0.01);
		grid.setInputCloud(srcCloudPtr);
		grid.filter(*_srcCloudPtr);
		grid.setInputCloud(tgtCloudPtr);
		grid.filter(*_tgtCloudPtr);
	}
	else {
		_srcCloudPtr = srcCloudPtr;
		_tgtCloudPtr = tgtCloudPtr;
	}

	pcl::IterativeClosestPoint<cloud::PointT, cloud::PointT> icp_nl;  //实例 ICP 对象
	//icp_nl.setEuclideanFitnessEpsilon(1e-6);
	icp_nl.setTransformationEpsilon(1e-6);  //设置两个临近变换的最大平方差
	icp_nl.setMaxCorrespondenceDistance(0.15);  //设置源点与目标点的最大匹配距离(米)
	//icp_nl.setPointRepresentation(pcl::make_shared<const MyPointRepresentation2>(point_representation));
	icp_nl.setInputSource(_srcCloudPtr);
	icp_nl.setInputTarget(_tgtCloudPtr);

	//多次配准
	Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity(), pre_transformation;
	cloud::PointCloudPtr transCloudPtr = _srcCloudPtr;
	icp_nl.setMaximumIterations(2);  //设置迭代次数
	for (int i = 0; i < 5; ++i) {
		icp_nl.setInputSource(_srcCloudPtr);
		icp_nl.align(*transCloudPtr);
		transformation = icp_nl.getFinalTransformation() * transformation;

		if (abs((icp_nl.getLastIncrementalTransformation() - pre_transformation).sum()) < icp_nl.getTransformationEpsilon()) {
			icp_nl.setMaxCorrespondenceDistance(icp_nl.getMaxCorrespondenceDistance() - 0.005);
		}
		pre_transformation = transformation;
		_srcCloudPtr = transCloudPtr;
	}
	final_transformation = transformation;
	pcl::transformPointCloud(*srcCloudPtr, *resCloudPtr, final_transformation);
	return 0;
}

int
pairAlignWithCustom(
	const cloud::PointNormalPfhPtr& srcCloudPtr,
	const cloud::PointNormalPfhPtr& tgtCloudPtr,
	cloud::PointCloudPtr& resCloudPtr,
	Eigen::Matrix4f& final_transformation)
{
	//自定义 Point Representation
	class MyPointRepresentationNormal :public pcl::PointRepresentation<cloud::PointNormalPfhT> {
		using pcl::PointRepresentation<cloud::PointNormalPfhT>::nr_dimensions_;
	public:
		MyPointRepresentationNormal() {
			nr_dimensions_ = 7;
		}
		virtual void copyToFloatArray(const cloud::PointNormalPfhT& p, float* out) const
		{
			out[0] = p.x;
			out[1] = p.y;
			out[2] = p.z;
			//out[3] = p.pfh[0];
			//out[4] = p.pfh[1];
			//out[5] = p.pfh[2];
			//out[6] = p.pfh[3];
			//out[3] = p.curvature;
		}
	};

	cloud::PointNormalPfhPtr _srcCloudPtr(new cloud::PointNormalPfh);  //源点云下采样
	cloud::PointNormalPfhPtr _tgtCloudPtr(new cloud::PointNormalPfh);  //目标点云下采样
	//下采样
	_srcCloudPtr = srcCloudPtr;
	_tgtCloudPtr = tgtCloudPtr;

	MyPointRepresentationNormal point_representation;
	float alpha[4] = { 1.0, 1.0, 1.0, 1.0 };
	point_representation.setRescaleValues(alpha);

	//pcl::IterativeClosestPointNonLinear<cloud::PointNormalPfhT, cloud::PointNormalPfhT> icp_nl;  //实例 ICP 对象
	//icp_nl.setEuclideanFitnessEpsilon(1e-6);
	//icp_nl.setTransformationEpsilon(1e-6);  //设置两个临近变换的最大平方差
	//icp_nl.setMaxCorrespondenceDistance(0.1);  //设置源点与目标点的最大匹配距离(米)
	//icp_nl.setPointRepresentation(pcl::make_shared<const MyPointRepresentationNormal>(point_representation));
	//icp_nl.setInputSource(_srcCloudPtr);
	//icp_nl.setInputTarget(_tgtCloudPtr);

	////多次配准
	//Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity(), pre_transformation;
	//cloud::PointNormalPfhPtr transCloudNormalPtr = _srcCloudPtr;
	//icp_nl.setMaximumIterations(2);  //设置迭代次数
	//for (int i = 0; i < 64; ++i) {
	//	icp_nl.setInputSource(_srcCloudPtr);
	//	icp_nl.align(*transCloudNormalPtr);
	//	transformation = icp_nl.getFinalTransformation() * transformation;

	//	if (abs((icp_nl.getLastIncrementalTransformation() - pre_transformation).sum()) < icp_nl.getTransformationEpsilon()) {
	//		icp_nl.setMaxCorrespondenceDistance(icp_nl.getMaxCorrespondenceDistance() - 0.005);
	//	}
	//	pre_transformation = transformation;
	//	_srcCloudPtr = transCloudNormalPtr;
	//}
	//final_transformation = transformation;
	//cloud::PointCloudPtr _srcPointCloudPtr(new cloud::PointCloud);
	//pcl::copyPointCloud<cloud::PointNormalPfhT, cloud::PointT>(*_srcCloudPtr, *_srcPointCloudPtr);
	//pcl::transformPointCloud(*_srcPointCloudPtr, *resCloudPtr, final_transformation);
	return 0;
}

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
	cloud::PointCloudPtr& mixedPointCloudPtr,
	bool withNormal,
	bool saveResultToPcd,
	bool downSample)
{
	pcl::copyPointCloud(*pointCloudVec[0], *mixedPointCloudPtr);
	cloud::PointCloudPtr tgtPointCloudPtr, srcPointCloudPtr, resPointCloudPtr(new cloud::PointCloud), tmpPointCloudPtr(new cloud::PointCloud);
	Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity(), preTransformation = Eigen::Matrix4f::Identity();
	for (size_t i = 0; i < pointCloudVec.size() - 1; ++i) {
		tgtPointCloudPtr = pointCloudVec[i];
		srcPointCloudPtr = pointCloudVec[i + 1];
		if (withNormal)pairAlignWithNormal(srcPointCloudPtr, tgtPointCloudPtr, tmpPointCloudPtr, transformation, downSample);
		else pairAlign(srcPointCloudPtr, tgtPointCloudPtr, tmpPointCloudPtr, transformation, downSample);
		pcl::transformPointCloud(*tmpPointCloudPtr, *resPointCloudPtr, preTransformation);
		preTransformation *= transformation;

		if (saveResultToPcd) {
			std::stringstream filename;
			filename << i << ".pcd";
			pcl::io::savePCDFile(filename.str(), *resPointCloudPtr);
		}
		if (mixedPointCloudPtr) {
			*mixedPointCloudPtr += *resPointCloudPtr;
		}
		cout << "\rregister " << fixed << setprecision(0) << (float)(i + 1) / (float)(pointCloudVec.size() - 1) * 100 << "%";
		cout << endl;
		cout << transformation << endl;
	}
	cout << endl;
	return 0;
}

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
	bool withNormal,
	bool saveResultToPcd,
	bool downSample)
{
	registeredVec.push_back(pointCloudVec[0]);
	cloud::PointCloudPtr tgtPointCloudPtr, srcPointCloudPtr, tmpPointCloudPtr(new cloud::PointCloud);
	Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity(), preTransformation = Eigen::Matrix4f::Identity();
	transformationVec.push_back(preTransformation);
	for (size_t i = 0; i < pointCloudVec.size() - 1; ++i) {
		cloud::PointCloudPtr resPointCloudPtr(new cloud::PointCloud);
		tgtPointCloudPtr = pointCloudVec[i];
		srcPointCloudPtr = pointCloudVec[i + 1];
		if(withNormal)pairAlignWithNormal(srcPointCloudPtr, tgtPointCloudPtr, tmpPointCloudPtr, transformation, downSample);
		else pairAlign(srcPointCloudPtr, tgtPointCloudPtr, tmpPointCloudPtr, transformation, downSample);
		pcl::transformPointCloud(*tmpPointCloudPtr, *resPointCloudPtr, preTransformation);
		preTransformation *= transformation;
		transformationVec.push_back(preTransformation);
		registeredVec.push_back(resPointCloudPtr);

		if (saveResultToPcd) {
			std::stringstream filename;
			filename << i << ".pcd";
			pcl::io::savePCDFile(filename.str(), *resPointCloudPtr);
		}
		cout << "out" << endl;
		cout << transformation << endl;
	}
	return 0;
}

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
	const cloud::PointCloudPtr& pointCloudPtr,
	const cloud::Color pointColor,
	const int pointSize,
	const cloud::Color backgroundColor,
	bool is_auto)
{
	cloud::PointCloudPtrVec pointCloudPtrVec(1, pointCloudPtr);
	vector<cloud::Color> pointColorVec(1, pointColor);
	vector<int> pointSizeVec(1, pointSize);
	return visualizePointCloud(pointCloudPtrVec, pointColorVec, pointSizeVec, backgroundColor, is_auto);
}

/**
 * @brief		可视化多个点云
 * @param[in]	pointCloudPtrVec	点云向量
 * @param[in]	pointColorVec		点云颜色
 * @param[in]	backgroundColor		背景颜色
 * @param[in]	is_auto				显示阻塞
 * @return 
*/
pcl::visualization::PCLVisualizer
visualizePointCloud(
	const cloud::PointCloudPtrVec& pointCloudPtrVec,
	vector<cloud::Color>& pointColorVec,
	vector<int> pointSizeVec,
	const cloud::Color& backgroundColor,
	bool is_auto)
{
	vector<pcl::PolygonMesh> meshVec;
	return visualizePointCloud(pointCloudPtrVec, meshVec, pointColorVec, pointSizeVec, backgroundColor, is_auto);
}

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
	const cloud::Color& pointColor,
	int pointSize,
	const cloud::Color& backgroundColor,
	bool is_auto)
{
	vector<cloud::PointCloudPtr, Eigen::aligned_allocator<cloud::PointT>> pointCloudPtrVec(1, pointCloudPtr);
	vector<cloud::Color> pointColorVec(1, pointColor);
	vector<int> pointSizeVec(1, pointSize);
	vector<pcl::PolygonMesh> meshVec(1, mesh);
	return visualizePointCloud(pointCloudPtrVec, meshVec, pointColorVec, pointSizeVec, backgroundColor, is_auto);
}

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
	vector<int> pointSizeVec,
	const cloud::Color& backgroundColor,
	bool is_auto)
{
	using colorHandlerCustom = pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>;
	int pointRenderSize = 2;

	//三维点选取的回调函数
	class PointPickingCallback {
	private:
		int x_pos = 0;
		int y_pos = 0;
		string textID = "text";
		float point_x = 0, point_y = 0, point_z = 0;
		pcl::visualization::PCLVisualizer* pViewer;
	public:
		PointPickingCallback(int x, int y, string id, pcl::visualization::PCLVisualizer* _pViewer) :x_pos(x), y_pos(y), textID(id), pViewer(_pViewer) {};
		void callback(const pcl::visualization::PointPickingEvent& pointPickingEvent, void* args) {
			//pcl::visualization::PCLVisualizer* pViewer = (pcl::visualization::PCLVisualizer*)args;

			pointPickingEvent.getPoint(point_x, point_y, point_z);

			stringstream ss;
			ss << "point position\n" << "x: " << point_x << "\ny: " << point_y << "\nz: " << point_z;
			pViewer->updateText(ss.str(), this->x_pos, this->y_pos, textID);
		}
		void setPos(int x, int y) {
			this->x_pos = x;
			this->y_pos = y;
		}
		void printPos() { cout << this->x_pos << "\t" << this->y_pos << endl; }
		void getPointPos(float& x, float& y, float& z) {
			x = this->point_x;
			y = this->point_y;
			z = this->point_z;
		}
	};

	//键盘事件回调函数
	class KeyboardCallback {
	public:
		pcl::visualization::PCLVisualizer* pViewer;
		const cloud::PointCloudPtrVec* pPointCloudPtrVec;
		const vector<pcl::PolygonMesh>* pMeshVec;
		vector<string>* pCloudIDVec;
		vector<string>* pMeshIDVec;
		vector<int> cloudRec;
		vector<int> meshRec;
		vector<cloud::Color>* pColorVec;
		int showIndex = 0;
		int size = 0;
		bool eachMode = false;
		int pointRenderSize = 2;
	public:
		KeyboardCallback(pcl::visualization::PCLVisualizer* _pViewer, 
			const cloud::PointCloudPtrVec* _pPointCloudPtrVec,
			const vector<pcl::PolygonMesh>* _pMeshVec,
			vector<string>* _pCloudIDVec,
			vector<string>* _pMeshIDVec,
			vector<cloud::Color>* _pColorVec) {
			pViewer = _pViewer;
			pPointCloudPtrVec = _pPointCloudPtrVec;
			pMeshVec = _pMeshVec;
			pCloudIDVec = _pCloudIDVec;
			pMeshIDVec = _pMeshIDVec;
			pColorVec = _pColorVec;
			cloudRec.resize(_pCloudIDVec->size());
			cloudRec.assign(_pCloudIDVec->size(), 1);
			meshRec.resize(pMeshIDVec->size());
			meshRec.assign(pMeshIDVec->size(), 1);
			size = max(_pCloudIDVec->size(), pMeshIDVec->size());
		}
		void callback(const pcl::visualization::KeyboardEvent& keyboardEvent, void* args) {
			if (keyboardEvent.getKeyCode() == 'c' && keyboardEvent.keyUp()) {
				cout << "press c" << endl;
				pViewer->resetCameraViewpoint();
			}
			else if (keyboardEvent.getKeyCode() == 'p' && keyboardEvent.keyUp()) {
				eachMode = !eachMode;
				if (eachMode) {
					for (int i = 1; i < pCloudIDVec->size(); ++i) {
						pViewer->removePointCloud(pCloudIDVec->at(i));
					}
					for (int j = 1; j < pMeshIDVec->size(); ++j) {
						pViewer->removePolygonMesh(pMeshIDVec->at(j));
					}
				}
				else {
					for (int i = 0; i < pCloudIDVec->size(); ++i) {
						if (i != showIndex) {
							colorHandlerCustom color(pPointCloudPtrVec->at(i), pColorVec->at(i).r, pColorVec->at(i).g, pColorVec->at(i).b);
							pViewer->addPointCloud(pPointCloudPtrVec->at(i), color, pCloudIDVec->at(i));
							pViewer->setPointCloudRenderingProperties(
								pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 
								pointRenderSize, 
								pCloudIDVec->at(i));
						}
					}
					for (int j = 0; j < pMeshIDVec->size(); ++j) {
						if (j != showIndex) {
							pViewer->addPolygonMesh(pMeshVec->at(j), pMeshIDVec->at(j));
						}
					}
				}
			}
			else if (keyboardEvent.getKeyCode() == 'a' && eachMode && keyboardEvent.keyUp()) {
				if (showIndex < pCloudIDVec->size()) 
					pViewer->removePointCloud(pCloudIDVec->at(showIndex));
				if (showIndex < pMeshIDVec->size()) 
					pViewer->removePolygonMesh(pMeshIDVec->at(showIndex));
				showIndex = ++showIndex % size;
				if (showIndex < pCloudIDVec->size()) {
					colorHandlerCustom color(pPointCloudPtrVec->at(showIndex), pColorVec->at(showIndex).r, pColorVec->at(showIndex).g, pColorVec->at(showIndex).b);
					pViewer->addPointCloud<cloud::PointT>(pPointCloudPtrVec->at(showIndex), color, pCloudIDVec->at(showIndex));
					//pViewer->addPointCloud<cloud::PointT>(pPointCloudPtrVec->at(showIndex), pCloudIDVec->at(showIndex));
					pViewer->setPointCloudRenderingProperties(
						pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 
						pointRenderSize, 
						pCloudIDVec->at(showIndex));
				}
				if (showIndex < pMeshIDVec->size())
					pViewer->addPolygonMesh(pMeshVec->at(showIndex), pMeshIDVec->at(showIndex));
			}
		}
	};

	//框选事件回调函数
	class AreaPickingCallback {
	public:
		pcl::Indices indices;
	public:
		void callback(const pcl::visualization::AreaPickingEvent& areaPickingEvent, void* args) {
			areaPickingEvent.getPointsIndices(this->indices);
		}
		void getPointsIndices(pcl::Indices& _indices) { _indices = this->indices; }
	};

	//颜色少于点云数量时使用默认颜色
	cloud::Color defaultColor = { 255.0, 255.0, 255.0 };
	for (int i = pointCloudPtrVec.size() - pointColorVec.size(); i > 0; --i) {
		pointColorVec.push_back(defaultColor);
	}
	for (int i = pointCloudPtrVec.size() - pointSizeVec.size(); i > 0; --i) {
		pointSizeVec.push_back(2);
	}

	cout << "Shift + 鼠标左键选取点" << endl;
	cout << "单独显示点云：p" << endl;
	cout << "切换下一个点云：a" << endl;

	int text_xpos = 0;	//文本x位置
	int text_ypos = 0;	//文本y位置
	int fontsize = 20;	//字体大小
	string textID = "pointPosition";	//文本ID

	//int* winSize = new int[2];
	//int* winPos = new int[2];

	pcl::visualization::PCLVisualizer viewer("viewer");

	viewer.setBackgroundColor(backgroundColor.r, backgroundColor.g, backgroundColor.b);	//设置背景
	viewer.setShowFPS(false);
	viewer.addCoordinateSystem();

	vector<string> cloudIDVec;
	vector<string> meshIDVec;
	for (int i = 0; i < pointCloudPtrVec.size(); ++i) {
		cloudIDVec.push_back(string("cloud").append(to_string(i)));
	}
	for (int i = 0; i < min(pointCloudPtrVec.size(), meshVec.size()); ++i) {
		meshIDVec.push_back(string("mesh").append(to_string(i)));
	}

	PointPickingCallback pointPickingCallback(text_xpos, text_ypos, textID, &viewer);
	AreaPickingCallback areaPickingCallback;
	KeyboardCallback keyboardCallback(&viewer, &pointCloudPtrVec, &meshVec, &cloudIDVec, &meshIDVec, &pointColorVec); //待修改

	string pointPositionStr("point position\nx:\ny:\nz:");
	viewer.addText(pointPositionStr, text_xpos, text_ypos, fontsize, 0.0, 0.0, 1.0, textID); //添加文本
	viewer.registerPointPickingCallback<PointPickingCallback>(&PointPickingCallback::callback, pointPickingCallback);	//注册三维点选取的回调函数
	//viewer.registerAreaPickingCallback<AreaPickingCallback>(&AreaPickingCallback::callback, areaPickingCallback);
	viewer.registerKeyboardCallback<KeyboardCallback>(&KeyboardCallback::callback, keyboardCallback);

	for (int i = 0; i < pointCloudPtrVec.size(); ++i) {
		colorHandlerCustom color(pointCloudPtrVec[i], pointColorVec[i].r, pointColorVec[i].g, pointColorVec[i].b);
		viewer.addPointCloud<cloud::PointT>(pointCloudPtrVec[i], color, cloudIDVec[i]);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSizeVec[i], cloudIDVec[i]);
	}

	for (int i = 0; i < min(pointCloudPtrVec.size(), meshVec.size()); ++i) {
		viewer.addPolygonMesh<cloud::PointT>(pointCloudPtrVec[i], meshVec[i].polygons, meshIDVec[i]);
	}

	if (is_auto) {
		while (!viewer.wasStopped()) {
			viewer.spinOnce(20);
		}
	}
	return viewer;
}

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
	int maximumNearestNeighbors,
	double searchRadius,
	double mu,
	double minimumAngle,
	double maximumAngle,
	double maximumSurfaceAngle,
	bool normalConsistency)
{
	pcl::PointCloud<pcl::Normal>::Ptr normalWithCurv(new pcl::PointCloud<pcl::Normal>);
	normalEstimation(srcPointCloud, normalWithCurv, 20);
	cloud::PointCloudNormalPtr pointWithNormal(new cloud::PointCloudNormal);
	pcl::concatenateFields(*srcPointCloud, *normalWithCurv, *pointWithNormal);

	pcl::GreedyProjectionTriangulation<cloud::PointNormalT> gp3;
	pcl::search::KdTree<cloud::PointNormalT>::Ptr searchTree(new pcl::search::KdTree<cloud::PointNormalT>);

	gp3.setSearchRadius(searchRadius); //设置三角形的最大边长
	gp3.setMu(mu); //设置最近点的最大距离
	gp3.setMaximumNearestNeighbors(maximumNearestNeighbors); //设置查询的临近点的最多个数
	gp3.setMinimumAngle(minimumAngle); //设置三角面的最小角的角度
	gp3.setMaximumAngle(maximumAngle); //设置三角面的最大角的角度
	gp3.setMaximumSurfaceAngle(maximumSurfaceAngle);
	gp3.setNormalConsistency(normalConsistency);

	searchTree->setInputCloud(pointWithNormal);
	gp3.setInputCloud(pointWithNormal);
	gp3.setSearchMethod(searchTree);
	gp3.reconstruct(trianglesMesh);


	return 0;
}

/**
 * @brief		利用泊松法表面重建
 * @param[in]	PointCloudPtr		输入点云
 * @param[out]	mesh				输出网格
 * @return
*/
int 
creatMeshPassion(
	const cloud::PointCloudPtr& PointCloudPtr,
	pcl::PolygonMesh& mesh)
{
	//cloud::PointCloudNormalPtr pointCloudNormalPtr(new cloud::PointCloudNormal);
	cloud::PointCloudNormalPtr pointCloudNormalPtr(new cloud::PointCloudNormal);
	normalEstimation(PointCloudPtr, pointCloudNormalPtr, 20);
	return creatMeshPassion(pointCloudNormalPtr, mesh);
}

/**
 * @brief		利用贪婪投影算法表面重建
 * @param[in]	PointCloudNormalPtr		输入点云
 * @param[out]	mesh					输出网格
 * @return
*/
int 
creatMeshPassion(
	const cloud::PointCloudNormalPtr& PointCloudNormalPtr,
	pcl::PolygonMesh& mesh)
{
	pcl::search::KdTree<cloud::PointNormalT>::Ptr tree2(new pcl::search::KdTree<cloud::PointNormalT>());
	tree2->setInputCloud(PointCloudNormalPtr);

	pcl::Poisson<cloud::PointNormalT> pn;
	pn.setConfidence(false); //是否使用法向量的大小作为置信信息。如果false，所有法向量均归一化。
	pn.setDegree(2); //设置参数degree[1,5],值越大越精细，耗时越久。
	pn.setDepth(8);
	//树的最大深度，求解2^d x 2^d x 2^d立方体元。
	// 由于八叉树自适应采样密度，指定值仅为最大深度。
	pn.setIsoDivide(8); //用于提取ISO等值面的算法的深度
	pn.setManifold(false); //是否添加多边形的重心，当多边形三角化时。 
	// 设置流行标志，如果设置为true，则对多边形进行细分三角话时添加重心，设置false则不添加
	pn.setOutputPolygons(false); //是否输出多边形网格（而不是三角化移动立方体的结果）
	pn.setSamplesPerNode(15); //设置落入一个八叉树结点中的样本点的最小数量。无噪声，[1.0-5.0],有噪声[15.-20.]平滑
	//pn.setScale(1.25); //设置用于重构的立方体直径和样本边界立方体直径的比率。
	pn.setSolverDivide(8); //设置求解线性方程组的Gauss-Seidel迭代方法的深度
	//pn.setIndices();
	//设置搜索方法和输入点云
	pn.setSearchMethod(tree2);
	pn.setInputCloud(PointCloudNormalPtr);
	//创建多变形网格，用于存储结果
	//执行重构
	pn.reconstruct(mesh);
	return 0;
}

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
	const pcl::PolygonMesh& mesh) 
{
	fstream file(fp.c_str(), ios::out);
	if (!file.is_open()) return -1;
	file << fixed << setprecision(6);
	for (auto& point : pointCloud) {
		file << "v " << point.x << " " << point.y << " " << point.z << endl;
	}
	file << "g object" << endl;
	for (auto& face : mesh.polygons) {
		file << "f " << face.vertices[0]+1 << " " << face.vertices[1]+1 << " " << face.vertices[2]+1 << endl;
	}
	file.close();
	return 0;
}

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
	float sigmaS,
	float sigmaR)
{
	//if (!inPointCloudPtr->isOrganized()) {
	//	return -1;
	//}
	pcl::FastBilateralFilter<cloud::PointT> filter;
	filter.setInputCloud(inPointCloudPtr);
	filter.setSigmaS(sigmaS);
	filter.setSigmaR(sigmaR);
	filter.applyFilter(*outPointCloudPtr);
	return 0;
}

/**
 * @brief		计算深度图像
 * @param[in]	pointCloudPtr		输入点云
 * @param[out]	rangeImagePtr		输出深度图像
 * @param[in]	angularResolution	角度分辨率
 * @param[in]	maxAngleWidth		水平视角
 * @param[in]	maxAngleHeight		垂直视角
 * @return
*/
int getRangeImage(
	const cloud::PointCloudPtr& pointCloudPtr,
	pcl::RangeImage::Ptr& rangeImagePtr,
	float angularResolution,
	float maxAngleWidth,
	float maxAngleHeight)
{
	if (!pointCloudPtr->is_dense) {
		cout << "point cloud is not dense" << endl;
		return -1;
	}
	float _angularResolution = (float)(angularResolution * (M_PI / 180.0f));
	float _maxAngleWidth = (float)(maxAngleWidth * (M_PI / 180.0f));
	float _maxAngleHeight = (float)(maxAngleHeight * (M_PI / 180.0f));
	Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
	pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
	float noiseLevel = 0.00;
	float minRange = 0.2f; 
	int borderSize = 1;
	rangeImagePtr->createFromPointCloud(*pointCloudPtr, _angularResolution, _maxAngleWidth, _maxAngleHeight,
										sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
	return 0;
}

/**
 * @brief
 * @param pointCloudPtr
 * @param indices
 * @param neighborsIndices
 * @param k
 * @param r
 * @return
*/
int
getNARFKeypoints(
	const cloud::PointCloudPtr& pointCloudPtr,
	const pcl::RangeImage& rangeImage,
	pcl::PointCloud<int>& keypoint_indices,
	float support_size)
{
	pcl::RangeImageBorderExtractor range_image_border_extractor;
	pcl::NarfKeypoint narf_keypoint_detector(&range_image_border_extractor);
	narf_keypoint_detector.setRangeImage(&rangeImage);
	narf_keypoint_detector.getParameters().support_size = support_size;
	narf_keypoint_detector.getParameters ().add_points_on_straight_edges = true;
	narf_keypoint_detector.getParameters ().distance_for_additional_points = 0.1;

	narf_keypoint_detector.compute(keypoint_indices);

	return 0;
}

/**
 * @brief		获取NARF关键点
 * @param[in]	pointCloudPtr		输入点云
 * @param[in]	rangeImage			深度图像
 * @param[out]	keyPointCloudPtr	输出关键点
 * @return
*/
int 
getNARFKeypoints(
	const cloud::PointCloudPtr& pointCloudPtr,
	const pcl::RangeImage& rangeImage,
	cloud::PointCloudPtr& keyPointCloudPtr,
	float support_size)
{
	pcl::PointCloud<int> keypoint_indices;

	getNARFKeypoints(
		pointCloudPtr,
		rangeImage,
		keypoint_indices,
		support_size);

	keyPointCloudPtr->resize(keypoint_indices.size());
	for (int i = 0; i < keypoint_indices.size(); ++i) {
		keyPointCloudPtr->points[i].getVector3fMap() = rangeImage[keypoint_indices[i]].getVector3fMap();
	}

	return 0;
}

int
getNARFKeypoints(
	const cloud::PointCloudPtr& pointCloudPtr,
	const pcl::RangeImage& rangeImage,
	pcl::Indices& indices,
	float support_size)
{
	pcl::PointCloud<int> keypoint_indices;
	cloud::PointCloudPtr keyPointPtr(new cloud::PointCloud);

	getNARFKeypoints(
		pointCloudPtr,
		rangeImage,
		keyPointPtr,
		support_size);
	//pcl::PointCloud<int>::iterator begin = keypoint_indices.begin();
	//pcl::PointCloud<int>::iterator end = keypoint_indices.end();
	//indices.assign(begin, end);
	vector<vector<int>> _indices;
	for (auto& _point : *keyPointPtr) {
		getNeighbors(pointCloudPtr, keyPointPtr, _indices, 1);
	}

	for (auto& _index : _indices) {
		indices.push_back(_index[0]);
	}

	return 0;
}

int
getSIFTKeypoint(
	const cloud::PointCloudNormalPtr& pointCloudNormalPtr,
	cloud::PointCloudPtr& result
	)
{
	const float min_scale = 0.005f;
	const int n_octaves = 5; //3
	const int n_scales_per_octave = 6; //4
	const float min_contrast = 0.005f;

	// 使用法向量作为强度计算关键点，还可以是rgb、z值或者自定义，具体参看API
	pcl::SIFTKeypoint<cloud::PointNormalT, cloud::PointT> sift; //PointT 可以是 pcl::PointWithScale包含尺度信息
	pcl::search::KdTree<cloud::PointNormalT>::Ptr tree(new pcl::search::KdTree<cloud::PointNormalT>());
	sift.setSearchMethod(tree);
	sift.setScales(min_scale, n_octaves, n_scales_per_octave);
	sift.setMinimumContrast(min_contrast);
	sift.setInputCloud(pointCloudNormalPtr);
	sift.compute(*result);
	return 0;
}

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
	int nr_split)
{
	pcl::PFHEstimation<cloud::PointT, pcl::Normal, pcl::PFHSignature125> pfh;

	pcl::search::KdTree<cloud::PointT>::Ptr kdTreePtr(new pcl::search::KdTree<cloud::PointT>());

	pfh.setSearchMethod(kdTreePtr);
	pfh.setInputCloud(pointCloudPtr);
	pfh.setInputNormals(normalPtr);
	pfh.setRadiusSearch(0.04);
	pfh.computePointPFHSignature(*pointCloudPtr, *normalPtr, indices, nr_split, pfh_histogram);
	return 0;
}

int
computePFH(
	const cloud::PointCloudPtr& pointCloudPtr,
	const pcl::PointCloud<pcl::Normal>::Ptr& normalPtr,
	vector<Eigen::VectorXf>& pfh_histogramVec,
	const pcl::Indices& indices,
	int k,
	float r,
	int nr_split)
{
	pcl::PFHEstimation<cloud::PointT, pcl::Normal, pcl::PFHSignature125> pfh;

	pcl::search::KdTree<cloud::PointT>::Ptr kdTreePtr(new pcl::search::KdTree<cloud::PointT>());

	pfh.setSearchMethod(kdTreePtr);
	pfh.setInputCloud(pointCloudPtr);
	pfh.setInputNormals(normalPtr);
	pfh.setRadiusSearch(0.04);
	//pfh.computePointPFHSignature(*pointCloudPtr, *normalPtr, indices, nr_split, pfh_histogram);

	vector<vector<int>> neighborsIndices;
	getNeighbors(pointCloudPtr, indices, neighborsIndices, k, r);
	for (auto& _indices : neighborsIndices) {
		Eigen::VectorXf pfh_histogram;
		pfh.computePointPFHSignature(*pointCloudPtr, *normalPtr, _indices, nr_split, pfh_histogram);
		pfh_histogramVec.push_back(pfh_histogram);
	}


	return 0;
}

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
	int k,
	float r)
{
	neighborsIndices.clear();

	pcl::search::KdTree<cloud::PointT> kdTree;
	kdTree.setInputCloud(pointCloudPtr);
	for (auto index : indices) {
		vector<int> indecesTemp;
		vector<float> distanceTemp;
		if (k) {
			if (kdTree.nearestKSearch(pointCloudPtr->at(index), k, indecesTemp, distanceTemp) <= 0) {
				setTextYellow();
				cout << "Find none neighbor for " << index << "th" << endl;
				setTextWhite();
			}
		}
		else {
			if (kdTree.radiusSearch(pointCloudPtr->at(index), r, indecesTemp, distanceTemp) <= 0) {
				setTextYellow();
				cout << "Find none neighbor for " << index << "th" << endl;
				setTextWhite();
			}
		}
		neighborsIndices.push_back(indecesTemp);
	}
	return 0;
}

int
getNeighbors(
	const cloud::PointCloudPtr& pointCloudPtr,
	const cloud::PointCloudPtr& searchPointPtr,
	vector<vector<int>>& neighborsIndices,
	int k,
	float r)
{
	neighborsIndices.clear();

	pcl::search::KdTree<cloud::PointT> kdTree;
	kdTree.setInputCloud(pointCloudPtr);
	for (int i = 0; i < searchPointPtr->size(); ++i) {
		vector<int> indecesTemp;
		vector<float> distanceTemp;
		if (k) {
			if (kdTree.nearestKSearch(searchPointPtr->at(i), k, indecesTemp, distanceTemp) <= 0) {
				setTextYellow();
				cout << "Find none neighbor for " << i << "th" << endl;
				setTextWhite();
			}
		}
		else {
			if (kdTree.radiusSearch(searchPointPtr->at(i), r, indecesTemp, distanceTemp) <= 0) {
				setTextYellow();
				cout << "Find none neighbor for " << i << "th" << endl;
				setTextWhite();
			}
		}
		neighborsIndices.push_back(indecesTemp);
	}
	return 0;
}

int
plotHistogram(
	vector<double> x_data,
	vector<double> y_data)
{
	pcl::visualization::PCLPlotter plotter("histogram");
	plotter.addPlotData(x_data, y_data, "Y Axis", vtkChart::BAR);
	plotter.plot();
	return 0;
}

Eigen::MatrixXf
PCA(
	Eigen::MatrixXf origin,
	int kStart,
	float error)
{
	int cols = origin.cols();
	int rows = origin.rows();
	int i = 0;
	float mean = 0.0f;
	for (i = 0; i < cols; ++i) {
		mean = origin.col(i).mean();
		origin.col(i) = origin.col(i) - mean*Eigen::VectorXf::Ones(rows);
	}
	Eigen::MatrixXf SVDMatrix = origin.transpose() / sqrt(rows);
	Eigen::JacobiSVD<Eigen::MatrixXf> svd(SVDMatrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
	Eigen::MatrixXf U = svd.matrixU();
	Eigen::MatrixXf V = svd.matrixV();
	Eigen::VectorXf D = svd.singularValues();
	cout << "U:\n" << U << endl;
	cout << "V:\n" << V << endl;
	cout << "D:\n" << D << endl;
	int k = kStart - 1;
	float PCAerror = 1.0f;
	while (PCAerror > error && k <= cols) {
		++k;
		PCAerror = 1 - (D.head(k).sum() / D.sum());
	}
	Eigen::MatrixXf transMatrix = U.block(0, 0, U.rows(), k);
	Eigen::MatrixXf result = origin * transMatrix;
	cout << "error: " << PCAerror << endl;
	cout << "result:\n" << result;
	return result;
}

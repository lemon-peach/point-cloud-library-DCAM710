#define PCL_NO_PRECOMPILE
#include "cloudOperation.h"
#include <vtkRenderWindow.h>
//extern cloud::Color cloud::redPoint;
//extern vector<cloud::Color> cloud::colorVec;
namespace cloud {
	void viewerOneOff(pcl::visualization::PCLVisualizer& viewer) {
		viewer.setBackgroundColor(1.0, 0.5, 1.0);
	}
}

template <typename PointSource, typename PointTarget, typename Scalar = float>
class MyCorrespondenceEstimation : public pcl::registration::CorrespondenceEstimation<PointSource, PointTarget, Scalar> {
	//using pcl::registration::CorrespondenceEstimation<PointSource, PointTarget, Scalar>::
	using pcl::registration::CorrespondenceEstimation<PointSource, PointTarget, Scalar>::initCompute;
	using pcl::registration::CorrespondenceEstimation<PointSource, PointTarget, Scalar>::indices_;
	using pcl::registration::CorrespondenceEstimation<PointSource, PointTarget, Scalar>::input_;
	using pcl::registration::CorrespondenceEstimation<PointSource, PointTarget, Scalar>::target_;
	using pcl::registration::CorrespondenceEstimation<PointSource, PointTarget, Scalar>::point_representation_;
	using pcl::registration::CorrespondenceEstimation<PointSource, PointTarget, Scalar>::tree_;
	using pcl::registration::CorrespondenceEstimation<PointSource, PointTarget, Scalar>::corr_name_;
	using pcl::PCLBase<PointSource>::deinitCompute;
public:
	using Ptr = shared_ptr<MyCorrespondenceEstimation<PointSource, PointTarget, Scalar> >;
	using ConstPtr = shared_ptr<const MyCorrespondenceEstimation<PointSource, PointTarget, Scalar> >;
	//using pcl::registration::CorrespondenceEstimation<PointSource, PointTarget, Scalar>::input_;
	//using pcl::registration::CorrespondenceEstimation<PointSource, PointTarget, Scalar>::target_;
protected:
	vector<float> weight;
public:
	MyCorrespondenceEstimation()
	{
		corr_name_ = "MyCorrespondenceEstimation";
	}

	void
		determineCorrespondences(pcl::Correspondences& correspondences,
			double max_distance = std::numeric_limits<double>::max()) override;
	void setWeight(vector<float> _weight);
};
template <typename PointSource, typename PointTarget, typename Scalar> void
MyCorrespondenceEstimation<PointSource, PointTarget, Scalar>::determineCorrespondences(
	pcl::Correspondences& correspondences, 
	double max_distance) 
{
	if (!initCompute())
		return;
	double max_dist_sqr = max_distance * max_distance;
	correspondences.resize(indices_->size());

	std::vector<int> index;
	std::vector<float> distance;
	pcl::Correspondence corr;
	unsigned int nr_valid_correspondences = 0;

	cloud::PointCloudPtr inputXYZ(new cloud::PointCloud);
	pcl::copyPointCloud<PointSource, pcl::PointXYZ>(*input_, *inputXYZ);
	cloud::PointCloudPtr targetXYZ(new cloud::PointCloud);
	pcl::copyPointCloud<PointSource, pcl::PointXYZ>(*target_, *targetXYZ);
	pcl::KdTreeFLANN<pcl::PointXYZ> treeXYZ;
	treeXYZ.setInputCloud(targetXYZ);

	int __index = 0;
	float __val = -1.0;
	int numDimensions = point_representation_->getNumberOfDimensions();
	vector<float> _inputPointData;
	_inputPointData.resize(numDimensions);
	vector<float> _targetPointData;
	_targetPointData.resize(numDimensions);
	if (weight.empty()) {
		weight.resize(numDimensions);
		for (auto& _w : weight) {
			_w = 1.0;
		}
	}
	//cout << "weight: ";
	//for (auto& _w : weight) {
	//	cout << " " << _w;
	//}
	//cout << endl;
	//if (weight.size() != numDimensions) {
	//	
	//}

	if (pcl::isSamePointType<PointSource, PointTarget>())
	{
		// Iterate over the input set of source indices
		for (std::vector<int>::const_iterator idx = indices_->begin(); idx != indices_->end(); ++idx)
		{
			//r半径搜索
			//treeXYZ.radiusSearch((*inputXYZ)[*idx], max_distance, index, distance);
			treeXYZ.radiusSearchT((*inputXYZ)[*idx], max_distance, index, distance, 20);
			//cout << "1" << endl;
			point_representation_->vectorize<vector<float>>((*input_)[*idx], _inputPointData);
			//cout << "2" <<endl;
			//计算r半径内最佳匹配点
			__val = -1.0;
			for (int _i = 0; _i < index.size(); ++_i) {

				point_representation_->vectorize<vector<float>>((*target_)[index[_i]], _targetPointData);
				float __temp = 0;
				for (int _j = 0; _j < numDimensions; ++_j) {
					__temp += pow(_inputPointData[_j] - _targetPointData[_j], 2.0) * weight[_j];
				}
				if (__temp < __val || __val == -1) {
					__index = _i;
					__val = __temp;
				}
			}
			//cout << "3" << endl;
			if (index.empty() || distance[__index] > max_dist_sqr)
				continue;

			corr.index_query = *idx;
			corr.index_match = index[__index];
			corr.distance = distance[__index];
			correspondences[nr_valid_correspondences++] = corr;
		}
	}
	else
	{
		PointTarget pt;
		//无用，源代码
		// Iterate over the input set of source indices
		for (std::vector<int>::const_iterator idx = indices_->begin(); idx != indices_->end(); ++idx)
		{
			// Copy the source data to a target PointTarget format so we can search in the tree
			pcl::copyPoint<PointSource, PointTarget>((*input_)[*idx], pt);

			tree_->nearestKSearch(pt, 1, index, distance);
			if (distance[0] > max_dist_sqr)
				continue;

			corr.index_query = *idx;
			corr.index_match = index[0];
			corr.distance = distance[0];
			correspondences[nr_valid_correspondences++] = corr;
		}
	}
	correspondences.resize(nr_valid_correspondences);
	deinitCompute();
}
template <typename PointSource, typename PointTarget, typename Scalar> void
MyCorrespondenceEstimation<PointSource, PointTarget, Scalar>::setWeight(vector<float> _weight) {
	if (weight.size() != _weight.size()) weight.resize(_weight.size());
	for (int _i = 0; _i < weight.size(); ++_i) {
		weight[_i] = _weight[_i];
	}
}

class MyCorrespondenceEstimationByNormal : public pcl::registration::CorrespondenceEstimation<pcl::PointNormal, pcl::PointNormal> {
	//using pcl::registration::CorrespondenceEstimation<PointSource, PointTarget, Scalar>::
	using PointSource = pcl::PointNormal;
	using PointTarget = pcl::PointNormal;
	using Scalar = float;
	using pcl::registration::CorrespondenceEstimation<PointSource, PointTarget, Scalar>::initCompute;
	using pcl::registration::CorrespondenceEstimation<PointSource, PointTarget, Scalar>::indices_;
	using pcl::registration::CorrespondenceEstimation<PointSource, PointTarget, Scalar>::input_;
	using pcl::registration::CorrespondenceEstimation<PointSource, PointTarget, Scalar>::target_;
	using pcl::registration::CorrespondenceEstimation<PointSource, PointTarget, Scalar>::point_representation_;
	using pcl::registration::CorrespondenceEstimation<PointSource, PointTarget, Scalar>::tree_;
	using pcl::registration::CorrespondenceEstimation<PointSource, PointTarget, Scalar>::corr_name_;
	using pcl::PCLBase<PointSource>::deinitCompute;
public:
	using Ptr = shared_ptr<MyCorrespondenceEstimationByNormal>;
	using ConstPtr = shared_ptr<const MyCorrespondenceEstimationByNormal>;
protected:
	vector<float> weight;
	float cosThresold = 0.0f;
	float normalScale = 1.0f;
public:
	MyCorrespondenceEstimationByNormal()
	{
		corr_name_ = "MyCorrespondenceEstimationByNormal";
	}

	void
		determineCorrespondences(pcl::Correspondences& correspondences,
			double max_distance = std::numeric_limits<double>::max()) override;
	void setWeight(vector<float> _weight);
	void setCosThresold(float th){ cosThresold = th; }
	void setNormalSacle(float scale) { normalScale = scale; }
};
void
MyCorrespondenceEstimationByNormal::determineCorrespondences(
	pcl::Correspondences& correspondences,
	double max_distance)
{
	if (!initCompute())
		return;
	double max_dist_sqr = max_distance * max_distance;
	correspondences.resize(indices_->size());

	std::vector<int> indexVec;
	std::vector<float> distanceVec;
	pcl::Correspondence corr;
	unsigned int nr_valid_correspondences = 0;

	//cloud::PointCloudPtr inputXYZ(new cloud::PointCloud);
	//pcl::copyPointCloud<PointSource, pcl::PointXYZ>(*input_, *inputXYZ);
	//cloud::PointCloudPtr targetXYZ(new cloud::PointCloud);
	//pcl::copyPointCloud<PointSource, pcl::PointXYZ>(*target_, *targetXYZ);
	//pcl::KdTreeFLANN<pcl::PointXYZ> treeXYZ;
	//treeXYZ.setInputCloud(targetXYZ);

	int index = 0;
	float cos = 0.0f, _cos;
	int numDimensions = point_representation_->getNumberOfDimensions();
	if (weight.empty()) {
		weight.resize(numDimensions);
		for (auto& _w : weight) {
			_w = 1.0;
		}
	}
	tree_->setSortedResults(true);
	Eigen::Vector3f vector3fTemp, normalTemp, testVec;
	bool tttt = true;
	if (pcl::isSamePointType<PointSource, PointTarget>())
	{
		// Iterate over the input set of source indices
		for (std::vector<int>::const_iterator idx = indices_->begin(); idx != indices_->end(); ++idx)
		{
			//r半径搜索
			cos = 0;
			index = 0;
			const PointSource& _inputC = (*input_)[*idx];
			normalTemp = _inputC.getNormalVector3fMap();
			tree_->nearestKSearch(_inputC, 10, indexVec, distanceVec);
			for (size_t _index = 0; _index < indexVec.size(); ++_index) {
				const PointSource& _targetC = (*target_)[indexVec[_index]];
				vector3fTemp = _targetC.getVector3fMap() - _inputC.getVector3fMap();
				if (vector3fTemp.normalized().dot(normalTemp) >= 0) {
					vector3fTemp = vector3fTemp + (normalTemp * normalScale);
				}
				else {
					vector3fTemp = -1 * vector3fTemp + (normalTemp * normalScale);
				}
				_cos = abs(vector3fTemp.normalized().dot(normalTemp));
				if (_cos > cos) {
					cos = _cos;
					index = _index;
					testVec = vector3fTemp;
				}
				if (*idx == 0) {

				}
			}

			//cout << "3" << endl;
			//if (indexVec.empty() || distanceVec[index] > max_dist_sqr||cos< cosThresold)
			if (indexVec.empty() || cos < cosThresold || distanceVec[index] < 0.005)
				continue;

			corr.index_query = *idx;
			corr.index_match = indexVec[index];
			corr.distance = distanceVec[index];
			//cout << corr.index_query << "-" << corr.index_match << endl;
			correspondences[nr_valid_correspondences++] = corr;
		}
	}
	else
	{
		assert("点云类型应为pcl::PointNormal\n");
	}
	correspondences.resize(nr_valid_correspondences);
	//cout << "corr: " << nr_valid_correspondences << endl;
	deinitCompute();
}
void
MyCorrespondenceEstimationByNormal::setWeight(vector<float> _weight) {
	if (weight.size() != _weight.size()) weight.resize(_weight.size());
	for (int _i = 0; _i < weight.size(); ++_i) {
		weight[_i] = _weight[_i];
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
	bool downSample,
	float downSampleSize,
	float maxDis,
	bool useMyEm,
	bool edgeFilter,
	int iterations1,
	int iterations2,
	float EuclideanFitnessEpsilon)
{
	//自定义 Point Representation
	class MyPointRepresentationNormal :public pcl::PointRepresentation<cloud::PointNormalT> {
		using pcl::PointRepresentation<cloud::PointNormalT>::nr_dimensions_;
	public:
		using Ptr = shared_ptr<MyPointRepresentationNormal>;
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
	bool showInfo = true;
	//bool useMyEm = true;
	clock_t startTime, endTime;
	double useTime;
	cloud::PointCloudNormalPtr _srcCloudPtr(new cloud::PointCloudNormal);  //源点云下采样
	cloud::PointCloudNormalPtr _tgtCloudPtr(new cloud::PointCloudNormal);  //目标点云下采样
	//下采样
	startTime = clock();
	if (downSample) {
		pcl::VoxelGrid<pcl::PointNormal> grid;
		grid.setDownsampleAllData(true);
		grid.setLeafSize(downSampleSize, downSampleSize, downSampleSize);
		grid.setInputCloud(srcCloudPtr);
		grid.filter(*_srcCloudPtr);
		grid.setInputCloud(tgtCloudPtr);
		grid.filter(*_tgtCloudPtr);
	}
	else {
		pcl::copyPointCloud(*srcCloudPtr, *_srcCloudPtr);
		pcl::copyPointCloud(*tgtCloudPtr, *_tgtCloudPtr);
	}
	//visualizePointCloud({ _srcCloudPtr ,_tgtCloudPtr }, COLOR_VEC, { 3,3 });

	endTime = clock();
	useTime = (double)(endTime - startTime) / CLOCKS_PER_SEC;
	if(showInfo && downSample)cout << "降采样用时：" << useTime << endl;

	//法线估计
	if (downSample) {
		MyPointRepresentationXYZ<cloud::PointNormalT>::Ptr rep(new MyPointRepresentationXYZ<cloud::PointNormalT>);
		pcl::NormalEstimation<cloud::PointNormalT, cloud::PointNormalT> normal_est;
		pcl::search::KdTree<cloud::PointNormalT>::Ptr kdtree(new pcl::search::KdTree<cloud::PointNormalT>());
		kdtree->setPointRepresentation(rep);
		normal_est.setSearchMethod(kdtree);  //设置搜索方法
		//normal_est.setKSearch(30);  //设置 K 个临近点用于计算法线和曲率
		//normal_est.setRadiusSearch(resolution);
		float resolution = getResolution<cloud::PointNormalT>(_srcCloudPtr);
		normal_est.setRadiusSearch(resolution * 4.0f);
		normal_est.setInputCloud(_srcCloudPtr);
		normal_est.compute(*_srcCloudPtr);
		normal_est.setInputCloud(_tgtCloudPtr);
		normal_est.compute(*_tgtCloudPtr);
	}

	MyPointRepresentationNormal::Ptr point_representation(new MyPointRepresentationNormal);
	float alpha[4] = { 1.0, 1.0, 1.0, 1.0 };
	point_representation->setRescaleValues(alpha);

	removeInvalid(_srcCloudPtr);
	removeInvalid(_tgtCloudPtr);

	MyCorrespondenceEstimation<cloud::PointNormalT, cloud::PointNormalT>::Ptr 
		corrEstimation(new MyCorrespondenceEstimation<cloud::PointNormalT, cloud::PointNormalT>);
	corrEstimation->setPointRepresentation(point_representation);
	vector<float> weight = { 1.0, 1.0, 1.0, 1.0 };
	corrEstimation->setWeight(weight);

	pcl::IterativeClosestPointNonLinear<cloud::PointNormalT, cloud::PointNormalT> icp_nl;  //实例 ICP 对象
	icp_nl.setEuclideanFitnessEpsilon(EuclideanFitnessEpsilon);
	icp_nl.setTransformationEpsilon(1e-6);  //设置两个临近变换的最大平方差
	//icp_nl.setUseReciprocalCorrespondences(true);
	icp_nl.setMaxCorrespondenceDistance(maxDis);  //设置源点与目标点的最大匹配距离(米)
	icp_nl.setPointRepresentation(point_representation);
	if(useMyEm)icp_nl.setCorrespondenceEstimation(corrEstimation);
	icp_nl.setInputSource(_srcCloudPtr);
	icp_nl.setInputTarget(_tgtCloudPtr);

	//多次配准
	Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity(), pre_transformation;
	cloud::PointCloudNormalPtr transCloudNormalPtr = _srcCloudPtr;
	icp_nl.setMaximumIterations(iterations1);  //设置迭代次数
	icp_nl.setRANSACIterations(0);
	pcl::CorrespondencesPtr coors;
	startTime = clock();
	float MSE = maxDis * maxDis;
#define SHOW_CORR
#ifdef SHOW_CORR
	int showIndex = -1;
	cout << "showIndex= ";
	cin >> showIndex;
	cout << endl;
#endif
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer);
	visualizeNormal(_srcCloudPtr, viewer);
	for (int i = 0; i < iterations2; ++i) {
		icp_nl.setInputSource(_srcCloudPtr);
		icp_nl.align(*transCloudNormalPtr);
#ifdef SHOW_CORR
		if (showInfo&&i==showIndex) {
			cout << "showIndex= ";
			cin >> showIndex;
			cout << endl;
			cloud::PCLViewerPtr viewerPtr(new cloud::PCLViewer);
			visualizeCorrespondences(_srcCloudPtr, _tgtCloudPtr, icp_nl.correspondences_, viewerPtr);
			while (!viewerPtr->wasStopped()) {
				viewerPtr->spinOnce(20);
			}
		}
#endif
		MSE = icp_nl.getFitnessScore();
		transformation = icp_nl.getFinalTransformation() * transformation;

		if (abs((icp_nl.getLastIncrementalTransformation() - pre_transformation).sum()) < icp_nl.getTransformationEpsilon()) {
			//icp_nl.setMaxCorrespondenceDistance(icp_nl.getMaxCorrespondenceDistance() - pow(sum, 0.5f)*0.05);
			//icp_nl.setMaxCorrespondenceDistance(icp_nl.getMaxCorrespondenceDistance() - pow(sum, 0.5f)*0.3);
			icp_nl.setMaxCorrespondenceDistance(pow(MSE, 0.5f)*1.5);
			if (icp_nl.getMaxCorrespondenceDistance() < 1e-4)break;
		}
		pre_transformation = icp_nl.getLastIncrementalTransformation();
		//pre_transformation = transformation;
		_srcCloudPtr = transCloudNormalPtr;

		if (useMyEm) {
			//float _weight2 = pow(2.718, -100000.0 * sum * 0.05);
			////cout << _weight2 << endl;
			//if (_weight2 > 0.8)_weight2 = 0.8;
			//else if (_weight2 < 0.2)_weight2 = 0;
			//_weight2 = 0.25 - 0.25 * _weight2;
			float _weight2 = -0.15 * log10(MSE) - 0.35;
			if (_weight2 > 0.5)_weight2 = 0.5;
			else if (_weight2 < 0.2)_weight2 = 0.2;
			float _weight1 = (1 - _weight2) / 3;
			for (int _j = 0; _j < weight.size(); ++_j) {
				if (_j < 3) {
					weight[_j] = _weight1;
				}
				else {
					weight[_j] = _weight2;
				}
			}
			corrEstimation->setWeight(weight);
			cout << "set weight to";
			for (auto& _ : weight) {
				cout << " " << _;
			}
			cout << "\t";
		}
		if (showInfo) {
			cout << "\r";
			cout << i + 1 << "th align----";
			cout << "distance:" << icp_nl.getMaxCorrespondenceDistance() << "----";
			cout << "MSE:" << MSE << "----";
		}
	}
	cout << endl;
	endTime = clock();
	useTime = (double)(endTime - startTime) / CLOCKS_PER_SEC;
	if (showInfo)cout << "配准用时：" << useTime << endl;
	final_transformation = transformation;
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
	bool downSample,
	float downSampleSize,
	float maxDis)
{
	bool showInfo = true;
	cloud::PointCloudPtr _srcCloudPtr(new cloud::PointCloud);  //源点云下采样
	cloud::PointCloudPtr _tgtCloudPtr(new cloud::PointCloud);  //目标点云下采样
	//下采样
	if (downSample) {
		pcl::VoxelGrid<cloud::PointT> grid;
		grid.setLeafSize(downSampleSize, downSampleSize, downSampleSize);
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
	icp_nl.setTransformationEpsilon(1e-7);  //设置两个临近变换的最大平方差
	icp_nl.setMaxCorrespondenceDistance(maxDis);  //设置源点与目标点的最大匹配距离(米)
	//icp_nl.setPointRepresentation(pcl::make_shared<const MyPointRepresentation2>(point_representation));
	icp_nl.setInputSource(_srcCloudPtr);
	icp_nl.setInputTarget(_tgtCloudPtr);

	//多次配准
	Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity(), pre_transformation;
	cloud::PointCloudPtr transCloudPtr = _srcCloudPtr;
	icp_nl.setMaximumIterations(2);  //设置迭代次数
	pcl::CorrespondencesPtr coors;
	float sum = 0.0f;
	for (int i = 0; i < 256; ++i) {
		cout << i << "th align" << endl;
		icp_nl.setInputSource(_srcCloudPtr);
		icp_nl.align(*transCloudPtr);
		coors = icp_nl.correspondences_;
		if (showInfo) {
			coors = icp_nl.correspondences_;
			sum = 0;
			for (auto _coor : *coors) {
				sum += pow(_srcCloudPtr->at(_coor.index_query).x - _tgtCloudPtr->at(_coor.index_match).x, 2.0) +
					pow(_srcCloudPtr->at(_coor.index_query).y - _tgtCloudPtr->at(_coor.index_match).y, 2.0) +
					pow(_srcCloudPtr->at(_coor.index_query).z - _tgtCloudPtr->at(_coor.index_match).z, 2.0);
			}
			sum /= coors->size();
			if (showInfo)cout << "MSE: " << sum << endl;
		}
		transformation = icp_nl.getFinalTransformation() * transformation;

		if (abs((icp_nl.getLastIncrementalTransformation() - pre_transformation).sum()) < icp_nl.getTransformationEpsilon()) {
			icp_nl.setMaxCorrespondenceDistance(icp_nl.getMaxCorrespondenceDistance() - pow(sum, 0.5f) * 0.05);
			//if (showInfo)cout << "set distance to" << icp_nl.getMaxCorrespondenceDistance() << endl;
			if (icp_nl.getMaxCorrespondenceDistance() < 0)break;
		}
		if (showInfo) cout << "distance:" << icp_nl.getMaxCorrespondenceDistance() << endl;
		pre_transformation = icp_nl.getLastIncrementalTransformation();
		_srcCloudPtr = transCloudPtr;
	}
	final_transformation = transformation;
	pcl::transformPointCloud(*srcCloudPtr, *resCloudPtr, final_transformation);
	return 0;
}

int
pairAlignWithCustom(
	const cloud::PointCloudNormalPtr& srcCloudPtr,
	const cloud::PointCloudNormalPtr& tgtCloudPtr,
	Eigen::Matrix4f& final_transformation,
	bool downSample,
	float downSampleSize,
	float maxDis,
	bool useMyEm,
	bool edgeFilter,
	int iterations1,
	int iterations2,
	float EuclideanFitnessEpsilon)
{
	//自定义 Point Representation
	bool showInfo = true;
	//bool useMyEm = true;
	clock_t startTime, endTime;
	double useTime;
	cloud::PointCloudNormalPtr _srcCloudPtr(new cloud::PointCloudNormal);  //源点云下采样
	cloud::PointCloudNormalPtr _tgtCloudPtr(new cloud::PointCloudNormal);  //目标点云下采样
	//下采样
	startTime = clock();
	if (downSample) {
		pcl::VoxelGrid<cloud::PointNormalT> grid;
		//grid.setDownsampleAllData(true);
		grid.setLeafSize(downSampleSize, downSampleSize, downSampleSize);
		grid.setInputCloud(srcCloudPtr);
		grid.filter(*_srcCloudPtr);
		grid.setInputCloud(tgtCloudPtr);
		grid.filter(*_tgtCloudPtr);
	}
	else {
		pcl::copyPointCloud(*srcCloudPtr, *_srcCloudPtr);
		pcl::copyPointCloud(*tgtCloudPtr, *_tgtCloudPtr);
	}
	endTime = clock();
	useTime = (double)(endTime - startTime) / CLOCKS_PER_SEC;
	if (showInfo && downSample)cout << "降采样用时：" << useTime << endl;

	//法线估计
	MyPointRepresentationXYZ<cloud::PointNormalT>::Ptr rep(new MyPointRepresentationXYZ<cloud::PointNormalT>);
	pcl::NormalEstimation<cloud::PointNormalT, cloud::PointNormalT> normal_est;
	pcl::search::KdTree<cloud::PointNormalT>::Ptr kdtree(new pcl::search::KdTree<cloud::PointNormalT>());
	kdtree->setPointRepresentation(rep);
	normal_est.setSearchMethod(kdtree);  //设置搜索方法
	//normal_est.setKSearch(30);  //设置 K 个临近点用于计算法线和曲率
	//normal_est.setRadiusSearch(resolution);
	float resolution = getResolution<cloud::PointNormalT>(_srcCloudPtr);
	normal_est.setRadiusSearch(resolution * 4.0f);
	normal_est.setInputCloud(_srcCloudPtr);
	normal_est.compute(*_srcCloudPtr);
	normal_est.setInputCloud(_tgtCloudPtr);
	normal_est.compute(*_tgtCloudPtr);

	removeInvalid(_srcCloudPtr);
	removeInvalid(_tgtCloudPtr);

	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer);
	visualizeNormal(_srcCloudPtr, viewer);
	visualizePointCloud({ _srcCloudPtr ,_tgtCloudPtr }, COLOR_VEC);

	//计算法线和曲率
	/*
	float normalRadius = 4.0f * resolution;
	//cout << "normalRadius:" << normalRadius << endl;
	pcl::NormalEstimation<cloud::PointNormalT, cloud::PointNormalT> normal_est;
	pcl::search::KdTree<cloud::PointNormalT>::Ptr kdtree(new pcl::search::KdTree<cloud::PointNormalT>());
	normal_est.setSearchMethod(kdtree);  //设置搜索方法
	//normal_est.setKSearch(30);  //设置 K 个临近点用于计算法线和曲率
	normal_est.setRadiusSearch(normalRadius);

	bool is_dense;
	normal_est.setInputCloud(_srcCloudPtr);
	cloud::PointCloudNormalPtr _srcCloudNormalPtr(new cloud::PointCloudNormal);
	normal_est.compute(*_srcCloudNormalPtr);
	is_dense = _srcCloudNormalPtr->is_dense;
	pcl::copyPointCloud(*_srcCloudPtr, *_srcCloudNormalPtr);
	_srcCloudNormalPtr->is_dense = is_dense;

	normal_est.setInputCloud(_tgtCloudPtr);
	cloud::PointCloudNormalPtr _tgtCloudNormalPtr(new cloud::PointCloudNormal);
	normal_est.compute(*_tgtCloudNormalPtr);
	is_dense = _tgtCloudNormalPtr->is_dense;
	pcl::copyPointCloud(*_tgtCloudPtr, *_tgtCloudNormalPtr);
	_tgtCloudNormalPtr->is_dense = is_dense;*/


	//滤除边界点
	//if (edgeFilter) {
	//	//cout << "edge filter" << endl;
	//	pcl::PointIndices::Ptr indicesPtr(new pcl::PointIndices);
	//	pcl::ExtractIndices<cloud::PointNormalT> extract;
	//	edgeDetection(_srcCloudNormalPtr, indicesPtr, normalRadius);
	//	//edgeViewer(_srcCloudNormalPtr);
	//	extract.setInputCloud(_srcCloudNormalPtr);
	//	extract.setIndices(indicesPtr);
	//	extract.setNegative(true);
	//	extract.filter(*_srcCloudNormalPtr);
	//	edgeDetection(_tgtCloudNormalPtr, indicesPtr, normalRadius);
	//	//edgeViewer(_tgtCloudNormalPtr);
	//	extract.setInputCloud(_tgtCloudNormalPtr);
	//	extract.setIndices(indicesPtr);
	//	extract.setNegative(true);
	//	extract.filter(*_tgtCloudNormalPtr);
	//}
	//removeDiagonally(_srcCloudNormalPtr, _srcCloudNormalPtr, -0.34);
	//removeDiagonally(_tgtCloudNormalPtr, _tgtCloudNormalPtr, -0.34);

	//滤除离群点
	//pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
	//outrem.setMinNeighborsInRadius(12);
	//outrem.setKeepOrganized(false);
	//outrem.setRadiusSearch(resolution * 4.0f);
	//outrem.setInputCloud(_srcCloudPtr);
	//outrem.filter(*_srcCloudPtr);
	//outrem.setInputCloud(_tgtCloudPtr);
	//outrem.filter(*_tgtCloudPtr);


	MyPointRepresentationXYZ<cloud::PointNormalT>::Ptr point_representation(new MyPointRepresentationXYZ<cloud::PointNormalT>);
	float alpha[4] = { 1.0, 1.0, 1.0, 1.0};
	point_representation->setRescaleValues(alpha);

	//if (!_srcCloudNormalPtr->is_dense)removeInvalid(_srcCloudNormalPtr, point_representation);
	//if (!_tgtCloudNormalPtr->is_dense)removeInvalid(_tgtCloudNormalPtr, point_representation);

	MyCorrespondenceEstimationByNormal::Ptr
		corrEstimation(new MyCorrespondenceEstimationByNormal);
	corrEstimation->setPointRepresentation(point_representation);
	vector<float> weight = { 1.0, 1.0, 1.0, 1.0};
	corrEstimation->setWeight(weight);
	corrEstimation->setNormalSacle(0.003);
	corrEstimation->setCosThresold(0.8);

	pcl::IterativeClosestPointNonLinear<cloud::PointNormalT, cloud::PointNormalT> icp_nl;  //实例 ICP 对象
	icp_nl.setEuclideanFitnessEpsilon(EuclideanFitnessEpsilon);
	icp_nl.setTransformationEpsilon(1e-7);  //设置两个临近变换的最大平方差
	//icp_nl.setUseReciprocalCorrespondences(true);
	icp_nl.setMaxCorrespondenceDistance(maxDis);  //设置源点与目标点的最大匹配距离(米)
	icp_nl.setPointRepresentation(point_representation);
	if (useMyEm)icp_nl.setCorrespondenceEstimation(corrEstimation);
	//icp_nl.setInputSource(_srcCloudPtr);
	icp_nl.setInputTarget(_tgtCloudPtr);
	icp_nl.setRANSACIterations(0);

	//多次配准
	Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity(), pre_transformation;
	cloud::PointCloudNormalPtr transCloudNormalPtr = _srcCloudPtr;
	icp_nl.setMaximumIterations(iterations1);  //设置迭代次数
	//cloud::Color redPoint = { 255.0, 0.0, 0.0 };
	//cloud::Color greenPoint = { 0.0, 255.0, 0.0 };
	//cloud::Color bluePoint = { 0.0, 0.0, 255.0 };
	//cloud::Color yellowPoint = { 255.0, 255.0, 0.0 };
	//cloud::Color purplePoint = { 255.0, 0.0, 255.0 };
	//cloud::Color cyanPoint = { 255.0, 0.0, 255.0 };
	//cloud::Color whitePoint = { 255.0, 255.0, 255.0 };
	//vector<cloud::Color> colorVec = { redPoint, greenPoint, bluePoint, yellowPoint, purplePoint, cyanPoint };
	//vector<cloud::Color> colorWhiteVec = { whitePoint };
	//vector<cloud::PointCloudNormalPtr> vec;
	//vec.push_back(_srcCloudNormalPtr);
	//vec.push_back(_tgtCloudNormalPtr);
	//visualizePointCloud(vec, colorVec);
	pcl::CorrespondencesPtr coors;
	startTime = clock();
	float MSE = maxDis * maxDis;
#define SHOW_CORR
#ifdef SHOW_CORR
	int showIndex = 0;
	cout << "showIndex= ";
	cin >> showIndex;
	cout << endl;
#endif
	cloud::PointCloudNormalPtr temp_(new cloud::PointCloudNormal);
	for (int i = 0; i < iterations2; ++i) {
		pcl::copyPointCloud(*_srcCloudPtr, *temp_);
		icp_nl.setInputSource(temp_);
		icp_nl.align(*transCloudNormalPtr);
#ifdef SHOW_CORR
		if (showInfo && i == showIndex) {
			cout << "showIndex= ";
			cin >> showIndex;
			cloud::PCLViewerPtr viewerPtr(new cloud::PCLViewer);
			visualizeCorrespondences(_srcCloudPtr, _tgtCloudPtr, icp_nl.correspondences_, viewerPtr,3,false);
			while (!viewerPtr->wasStopped()) {
				viewerPtr->spinOnce(20);
			}
		}
#endif

		transformation = icp_nl.getFinalTransformation() * transformation;

		//if (abs((icp_nl.getLastIncrementalTransformation() - pre_transformation).sum()) < icp_nl.getTransformationEpsilon()) {
		//	//icp_nl.setMaxCorrespondenceDistance(icp_nl.getMaxCorrespondenceDistance() - pow(sum, 0.5f)*0.05);
		//	//icp_nl.setMaxCorrespondenceDistance(icp_nl.getMaxCorrespondenceDistance() - pow(sum, 0.5f)*0.3);
		//	icp_nl.setMaxCorrespondenceDistance(pow(sum, 0.5f) * 5.0f);
		//	if (icp_nl.getMaxCorrespondenceDistance() < 1e-4)break;
		//}
		pre_transformation = icp_nl.getLastIncrementalTransformation();
		//pre_transformation = transformation;
		_srcCloudPtr = transCloudNormalPtr;

		//MSE
		if (showInfo) {
			coors = icp_nl.correspondences_;
			MSE = 0;
			for (auto _coor : *coors) {
				MSE += pow(_srcCloudPtr->at(_coor.index_query).x - _tgtCloudPtr->at(_coor.index_match).x, 2.0) +
					pow(_srcCloudPtr->at(_coor.index_query).y - _tgtCloudPtr->at(_coor.index_match).y, 2.0) +
					pow(_srcCloudPtr->at(_coor.index_query).z - _tgtCloudPtr->at(_coor.index_match).z, 2.0);
			}
			MSE /= coors->size();
		}
		
		if (showInfo) {
			cout << "\r";
			cout << i + 1 << "th align    ";
			cout << "distance:" << icp_nl.getMaxCorrespondenceDistance() << "    ";
			cout << "MSE:" << MSE << "    ";
		}
	}
	cout << endl;
	endTime = clock();
	useTime = (double)(endTime - startTime) / CLOCKS_PER_SEC;
	if (showInfo)cout << "配准用时：" << useTime << endl;
	final_transformation = transformation;
	return 0;
}

/**
 * @brief		多个点云配准，将多个点云转换到第一个点云的坐标系
 * @param[in]	pointCloudVec				待配准的点云向量
 * @param[out]	registeredPointCloudPtr		所有点云转换到第一个点云坐标系后的混合点云
 * @param[in]	saveResultToPcd				是否单独存储每个点云转换的结果到PCD
 * @return
*/
//int
//registerPairsCloud(
//	const cloud::PointCloudPtrVec& pointCloudVec,
//	cloud::PointCloudPtr& mixedPointCloudPtr,
//	bool withNormal,
//	bool saveResultToPcd,
//	bool downSample)
//{
//	pcl::copyPointCloud(*pointCloudVec[0], *mixedPointCloudPtr);
//	cloud::PointCloudPtr tgtPointCloudPtr, srcPointCloudPtr, resPointCloudPtr(new cloud::PointCloud), tmpPointCloudPtr(new cloud::PointCloud);
//	Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity(), preTransformation = Eigen::Matrix4f::Identity();
//	for (size_t i = 0; i < pointCloudVec.size() - 1; ++i) {
//		tgtPointCloudPtr = pointCloudVec[i];
//		srcPointCloudPtr = pointCloudVec[i + 1];
//		if (withNormal)pairAlignWithNormal(srcPointCloudPtr, tgtPointCloudPtr, tmpPointCloudPtr, transformation, downSample);
//		else pairAlign(srcPointCloudPtr, tgtPointCloudPtr, tmpPointCloudPtr, transformation, downSample);
//		pcl::transformPointCloud(*tmpPointCloudPtr, *resPointCloudPtr, preTransformation);
//		preTransformation *= transformation;
//
//		if (saveResultToPcd) {
//			std::stringstream filename;
//			filename << i << ".pcd";
//			pcl::io::savePCDFile(filename.str(), *resPointCloudPtr);
//		}
//		if (mixedPointCloudPtr) {
//			*mixedPointCloudPtr += *resPointCloudPtr;
//		}
//		cout << "\rregister " << fixed << setprecision(0) << (float)(i + 1) / (float)(pointCloudVec.size() - 1) * 100 << "%";
//		cout << endl;
//		cout << transformation << endl;
//	}
//	cout << endl;
//	return 0;
//}

/**
 * @brief		多个点云配准，将多个点云转换到第一个点云的坐标系
 * @param[in]	pointCloudVec		待配准的点云向量
 * @param[put]	registeredVec		所有点云转换到第一个点云坐标系后的点云向量
 * @param[put]	transformationVec
 * @param[in]	saveResultToPcd		是否单独存储每个点云转换的结果到PCD
 * @return
*/
//int
//registerPairsCloud(
//	const cloud::PointCloudPtrVec& pointCloudVec,
//	cloud::PointCloudPtrVec& registeredVec,
//	vector<Eigen::Matrix4f>& transformationVec,
//	bool withNormal,
//	bool saveResultToPcd,
//	bool downSample)
//{
//	cloud::Color redPoint = { 255.0, 0.0, 0.0 };
//	cloud::Color greenPoint = { 0.0, 255.0, 0.0 };
//	vector<cloud::Color>colors = { redPoint, greenPoint };
//	bool showInfo = false;
//	cloud::PointCloudPtr tgtPointCloudPtr, srcPointCloudPtr, tmpPointCloudPtr(new cloud::PointCloud);
//	pcl::copyPointCloud(*pointCloudVec[0], *tmpPointCloudPtr);
//	registeredVec.push_back(tmpPointCloudPtr);
//	Eigen::Matrix4f preTransformation = Eigen::Matrix4f::Identity();
//	transformationVec.push_back(preTransformation);
//	for (size_t i = 0; i < pointCloudVec.size() - 1; ++i) {
//		Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
//		cloud::PointCloudPtr resPointCloudPtr(new cloud::PointCloud);
//		tgtPointCloudPtr = pointCloudVec[i];
//		srcPointCloudPtr = pointCloudVec[i + 1];
//		if(withNormal)pairAlignWithNormal(srcPointCloudPtr, tgtPointCloudPtr, tmpPointCloudPtr, transformation, downSample);
//		else pairAlign(srcPointCloudPtr, tgtPointCloudPtr, tmpPointCloudPtr, transformation, downSample);
//		if (showInfo) {
//			pcl::transformPointCloud(*srcPointCloudPtr, *tmpPointCloudPtr, transformation);
//			cout << "pair with " << i << "th and " << i + 1 << "th" << endl;
//			visualizePointCloud({ tgtPointCloudPtr, tmpPointCloudPtr }, colors);
//		}
//		pcl::transformPointCloud(*tmpPointCloudPtr, *resPointCloudPtr, preTransformation);
//		preTransformation *= transformation;
//		transformationVec.push_back(transformation);
//		registeredVec.push_back(resPointCloudPtr);
//
//		if (saveResultToPcd) {
//			std::stringstream filename;
//			filename << i << ".pcd";
//			pcl::io::savePCDFile(filename.str(), *resPointCloudPtr);
//		}
//		if (showInfo) {
//			cout << "out" << endl;
//			cout << transformation << endl;
//		}
//	}
//	return 0;
//}

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
	const cloud::PointCloudPtr& pointCloudPtr,
	const cloud::Color pointColor,
	const int pointSize,
	const cloud::Color backgroundColor,
	bool showCoordinateSystem,
	bool is_auto)
{
	cloud::PointCloudPtrVec pointCloudPtrVec(1, pointCloudPtr);
	vector<cloud::Color> pointColorVec(1, pointColor);
	vector<int> pointSizeVec(1, pointSize);
	return visualizePointCloud(pointCloudPtrVec, pointColorVec, pointSizeVec, backgroundColor, showCoordinateSystem, is_auto);
}

/**
 * @brief		可视化多个点云
 * @param[in]	pointCloudPtrVec	点云向量
 * @param[in]	pointColorVec		点云颜色
 * @param[in]	backgroundColor		背景颜色
 * @param[in]	is_auto				显示阻塞
 * @return 
*/
pcl::visualization::PCLVisualizer::Ptr
visualizePointCloud(
	const cloud::PointCloudPtrVec& pointCloudPtrVec,
	vector<cloud::Color> pointColorVec,
	vector<int> pointSizeVec,
	const cloud::Color backgroundColor,
	bool showCoordinateSystem,
	bool is_auto)
{
	vector<pcl::PolygonMesh> meshVec;
	return visualizePointCloud(pointCloudPtrVec, meshVec, pointColorVec, pointSizeVec, backgroundColor, showCoordinateSystem, is_auto);
}

pcl::visualization::PCLVisualizer::Ptr
visualizePointCloud(
	const vector<cloud::PointCloudNormalPtr>& pointCloudPtrVec,
	vector<cloud::Color> pointColorVec,
	vector<int> pointSizeVec,
	const cloud::Color backgroundColor,
	bool showCoordinateSystem,
	bool is_auto)
{
	vector<pcl::PolygonMesh> meshVec;
	cloud::PointCloudPtrVec tempVec;
	for (auto& _p : pointCloudPtrVec) {
		cloud::PointCloudPtr _temp(new cloud::PointCloud);
		pcl::copyPointCloud(*_p, *_temp);
		tempVec.push_back(_temp);
	}
	return visualizePointCloud(tempVec, meshVec, pointColorVec, pointSizeVec, backgroundColor, showCoordinateSystem, is_auto);
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
pcl::visualization::PCLVisualizer::Ptr
visualizePointCloud(
	const cloud::PointCloudPtr& pointCloudPtr,
	const pcl::PolygonMesh& mesh,
	const cloud::Color pointColor,
	int pointSize,
	const cloud::Color backgroundColor,
	bool showCoordinateSystem,
	bool is_auto)
{
	vector<cloud::PointCloudPtr> pointCloudPtrVec(1, pointCloudPtr);
	vector<cloud::Color> pointColorVec(1, pointColor);
	vector<int> pointSizeVec(1, pointSize);
	vector<pcl::PolygonMesh> meshVec(1, mesh);
	return visualizePointCloud(pointCloudPtrVec, meshVec, pointColorVec, pointSizeVec, backgroundColor, showCoordinateSystem, is_auto);
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
pcl::visualization::PCLVisualizer::Ptr
visualizePointCloud(
	const cloud::PointCloudPtrVec& pointCloudPtrVec,
	const vector<pcl::PolygonMesh>& meshVec,
	vector<cloud::Color> pointColorVec,
	vector<int> pointSizeVec,
	const cloud::Color backgroundColor,
	bool showCoordinateSystem,
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
			else if (keyboardEvent.getKeyCode() == 27) {
				*((int*)args)=0;
				args = nullptr;
				cout << "close viewer" << endl;
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
	pcl::visualization::PCLVisualizer::Ptr pViewer(new pcl::visualization::PCLVisualizer("viewer"));
	pcl::visualization::PCLVisualizer viewer = *pViewer;

	viewer.setBackgroundColor(backgroundColor.r, backgroundColor.g, backgroundColor.b);	//设置背景
	viewer.setShowFPS(false);
	if (showCoordinateSystem) {
		viewer.addCoordinateSystem();
	}
	else {
		viewer.removeCoordinateSystem();
	}

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
	int stopFlag = 1;
	viewer.registerKeyboardCallback<KeyboardCallback>(&KeyboardCallback::callback, keyboardCallback, (void*)(&stopFlag));

	for (int i = 0; i < pointCloudPtrVec.size(); ++i) {
		colorHandlerCustom color(pointCloudPtrVec[i], pointColorVec[i].r, pointColorVec[i].g, pointColorVec[i].b);
		viewer.addPointCloud<cloud::PointT>(pointCloudPtrVec[i], color, cloudIDVec[i]);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSizeVec[i], cloudIDVec[i]);
	}

	for (int i = 0; i < min(pointCloudPtrVec.size(), meshVec.size()); ++i) {
		viewer.addPolygonMesh<cloud::PointT>(pointCloudPtrVec[i], meshVec[i].polygons, meshIDVec[i]);
	}

	if (is_auto) {
		while ((!pViewer->wasStopped())&& stopFlag) {
			viewer.spinOnce(20);
		}
	}
	return pViewer;
}

void visualizeCorrespondences(
	cloud::PointCloudNormalPtr& cloud1,
	cloud::PointCloudNormalPtr& cloud2,
	pcl::CorrespondencesPtr& coor,
	pcl::visualization::PCLVisualizer::Ptr& viewer,
	int pointSize,
	bool showNormal)
{
	using colorHandlerCustom = pcl::visualization::PointCloudColorHandlerCustom<cloud::PointNormalT>;
	colorHandlerCustom color1(255.0, 0.0, 0.0);
	colorHandlerCustom color2(0.0, 255.0, 0.0);
	//viewer->addCoordinateSystem();

	viewer->addPointCloud<cloud::PointNormalT>(cloud1, "cloud1");
	viewer->addPointCloud<cloud::PointNormalT>(cloud2, "cloud2");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "cloud1");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "cloud2");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, "cloud1");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, "cloud2");
	if (showNormal) {
		viewer->addPointCloudNormals<cloud::PointNormalT>(cloud1, 10, 0.01f, "normal1");
		viewer->addPointCloudNormals<cloud::PointNormalT>(cloud2, 10, 0.01f, "normal2");
	}
	viewer->addCorrespondences<cloud::PointNormalT>(cloud1, cloud2, *coor);

}

void visualizeCorrespondences(
	cloud::PointCloudPtr& cloud1,
	cloud::PointCloudPtr& cloud2,
	pcl::CorrespondencesPtr& coor,
	pcl::visualization::PCLVisualizer::Ptr& viewer,
	int pointSize)
{
	using colorHandlerCustom = pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>;
	colorHandlerCustom color1(255.0, 0.0, 0.0);
	colorHandlerCustom color2(0.0, 255.0, 0.0);
	viewer->addPointCloud(cloud1, color1, "cloud1");
	viewer->addPointCloud(cloud2, color2, "cloud2");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, "cloud1");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, "cloud2");
	viewer->addCorrespondences<cloud::PointT>(cloud1, cloud2, *coor);

}

void visualizeNormal(
	cloud::PointCloudNormalPtr& cloudNoramlPtr,
	pcl::visualization::PCLVisualizer::Ptr& viewer,
	int pointSize)
{
	viewer->addPointCloud<cloud::PointNormalT>(cloudNoramlPtr);
	viewer->addPointCloudNormals<cloud::PointNormalT>(cloudNoramlPtr, 3,0.02f,"normal");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize);
	while (!viewer->wasStopped()) {
		viewer->spinOnce(20);
	}
}

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
	int maximumNearestNeighbors,
	double searchRadius,
	double mu,
	double minimumAngle,
	double maximumAngle,
	double maximumSurfaceAngle,
	bool normalConsistency,
	bool usePointRepresentation)
{
	class MyPointRepresentation :public pcl::PointRepresentation<cloud::PointNormalT> {
		using pcl::PointRepresentation<cloud::PointNormalT>::nr_dimensions_;
	public:
		using Ptr = shared_ptr<MyPointRepresentation>;
		MyPointRepresentation() {
			nr_dimensions_ = 3;
		}
		virtual void copyToFloatArray(const cloud::PointNormalT& p, float* out) const
		{
			out[0] = p.x;
			out[1] = p.y;
			out[2] = p.z;
		}
	};

	pcl::PointCloud<pcl::Normal>::Ptr normalWithCurv(new pcl::PointCloud<pcl::Normal>);
	normalEstimation(srcPointCloud, normalWithCurv, 0, getResolution<cloud::PointT>(srcPointCloud)*5.0f);
	cloud::PointCloudNormalPtr pointWithNormal(new cloud::PointCloudNormal);
	pcl::concatenateFields(*srcPointCloud, *normalWithCurv, *pointWithNormal);

	pcl::GreedyProjectionTriangulation<cloud::PointNormalT> gp3;
	pcl::search::KdTree<cloud::PointNormalT>::Ptr searchTree(new pcl::search::KdTree<cloud::PointNormalT>);
	if (usePointRepresentation) {
		MyPointRepresentation::Ptr myPointRepresentationPtr(new MyPointRepresentation());
		searchTree->setPointRepresentation(myPointRepresentationPtr);
	}
	searchTree->setInputCloud(pointWithNormal);

	gp3.setSearchRadius(searchRadius); //设置三角形的最大边长
	gp3.setMu(mu); //设置最近点的最大距离
	gp3.setMaximumNearestNeighbors(maximumNearestNeighbors); //设置查询的临近点的最多个数
	gp3.setMinimumAngle(minimumAngle); //设置三角面的最小角的角度
	gp3.setMaximumAngle(maximumAngle); //设置三角面的最大角的角度
	gp3.setMaximumSurfaceAngle(maximumSurfaceAngle);
	gp3.setNormalConsistency(normalConsistency);

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
	pn.setIsoDivide(16); //用于提取ISO等值面的算法的深度
	pn.setManifold(false); //是否添加多边形的重心，当多边形三角化时。 
	// 设置流行标志，如果设置为true，则对多边形进行细分三角话时添加重心，设置false则不添加
	pn.setOutputPolygons(false); //是否输出多边形网格（而不是三角化移动立方体的结果）
	pn.setSamplesPerNode(5); //设置落入一个八叉树结点中的样本点的最小数量。无噪声，[1.0-5.0],有噪声[15.-20.]平滑
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

int
fastBilateralFilter(
	const cloud::PointCloudNormalPtr& inPointCloudPtr,
	cloud::PointCloudNormalPtr& outPointCloudPtr,
	float sigmaS,
	float sigmaR)
{
	//if (!inPointCloudPtr->isOrganized()) {
	//	return -1;
	//}
	pcl::FastBilateralFilter<cloud::PointNormalT> filter;
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
	//pcl::PFHEstimation<cloud::PointT, pcl::Normal, cloud::PFH27> pfh;

	pcl::search::KdTree<cloud::PointT>::Ptr kdTreePtr(new pcl::search::KdTree<cloud::PointT>());

	pfh.setSearchMethod(kdTreePtr);
	pfh.setInputCloud(pointCloudPtr);
	pfh.setInputNormals(normalPtr);
	pfh.setRadiusSearch(0.04);
	//pfh.computePointPFHSignature(*pointCloudPtr, *normalPtr, indices, nr_split, pfh_histogram);

	vector<vector<int>> neighborsIndices;
	getNeighbors(pointCloudPtr, indices, neighborsIndices, k, r);
	cout << "neighborsIndices size: " << neighborsIndices.size() << endl;
	int cont = 0;
	for (auto& _indices : neighborsIndices) {
		cout << "cont: " << ++cont << endl;
		cout << "indices size: " << _indices.size() << endl;
		Eigen::VectorXf pfh_histogram(125);
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
			//if (maxNum<float>(distanceTemp) > r) {
			//	setTextYellow();
			//	cout << "距离过远，采用半径搜索" << endl;
			//	setTextWhite();
			//	if (kdTree.radiusSearch(pointCloudPtr->at(index), r, indecesTemp, distanceTemp) <= 0) {
			//		setTextYellow();
			//		cout << "Find none neighbor for " << index << "th" << endl;
			//		setTextWhite();
			//	}
			//}
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


/**
 * @brief PCA算法降维，对矩阵的列维度进行降维
 * @param origin		需要降维的矩阵
 * @param kStart		降维到k维
 * @param error			丢失率阈值，当降到k维不满足丢失率要求时提升维度
 * @return 
*/
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
	//cout << "U:\n" << U << endl;
	//cout << "V:\n" << V << endl;
	//cout << "D:\n" << D << endl;
	int k = kStart;
	float PCAerror = 1.0f;
	if (PCAerror > error) --k;
	while (PCAerror > error && k <= cols) {
		++k;
		PCAerror = 1 - (D.head(k).sum() / D.sum());
	}
	PCAerror = 1 - (D.head(k).sum() / D.sum());
	Eigen::MatrixXf transMatrix = U.block(0, 0, U.rows(), k);
	Eigen::MatrixXf result = origin * transMatrix;
	cout << "error: " << PCAerror << endl;
	//cout << "result:\n" << result;
	return result;
}

int SeparateView(
	const cloud::PointCloudPtr inPointCloudPtr,
	cloud::PointCloudPtr& out,
	float angle,
	bool negative)
{
	float cosineLimit = cos(angle/180.0f*3.1416);
	Eigen::Vector3f z(0, 0, 1);
	Eigen::Vector3f pVec;
	float cosine;
	out->clear();
	for (auto& _point : *inPointCloudPtr) {
		pVec(0) = _point.x;
		pVec(1) = _point.y;
		pVec(2) = _point.z;
		pVec.normalize();
		cosine = pVec.dot(z);

		//修改
		if ((cosine > cosineLimit && !negative) || (cosine < cosineLimit && negative)) {
			cloud::PointT newP;
			newP.x = _point.x;
			newP.y = _point.y;
			newP.z = _point.z;
			out->push_back(newP);
		}
	}
	return 0;
}


int SeparateBG(
	const cloud::PointCloudPtr inPointCloudPtr,
	cloud::PointCloudPtr& out)
{
	bool show = false;
	cloud::PointCloudPtr _temp(new cloud::PointCloud);
	if (show) cout << "距离滤波" << endl;
	pcl::copyPointCloud<cloud::PointT, cloud::PointT>(*inPointCloudPtr, *out);
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0.25, 0.6);
	pass.setInputCloud(out);
	pass.filter(*out);
	pass.setFilterFieldName("x");
	pass.setFilterLimits(-0.25, 0.25);
	pass.setInputCloud(out);
	pass.filter(*out);
	if (show) visualizePointCloud(out);

	if (show) cout << "平面分割" << endl;
	planeSegmentation(0.005, out, _temp, true);
	pcl::copyPointCloud(*_temp, *out);
	if (show) visualizePointCloud(out);

	if (show) cout << "离群点消除" << endl;
	SeparateView(out, _temp, 20);
	pcl::copyPointCloud(*_temp, *out);
	pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
	outrem.setRadiusSearch(0.03);
	outrem.setMinNeighborsInRadius(400);
	outrem.setKeepOrganized(false);
	outrem.setInputCloud(out);
	outrem.filter(*out);
	outrem.setRadiusSearch(0.005);
	outrem.setMinNeighborsInRadius(5);
	outrem.setKeepOrganized(false);
	outrem.setInputCloud(out);
	outrem.filter(*out);
	if (show) visualizePointCloud(out);

	return 0;
}

int fuseTwoPointClouds(
	cloud::PointCloudNormalPtr& cloud1,
	cloud::PointCloudNormalPtr& cloud2,
	cloud::PointCloudNormalPtr& resCloud,
	float distance,
	float edgeDistance)
{
	cloud::PointCloudNormalPtr _cloud1(new cloud::PointCloudNormal), _cloud2(new cloud::PointCloudNormal);
	pcl::copyPointCloud(*cloud1, *_cloud1);
	pcl::copyPointCloud(*cloud2, *_cloud2);
	visualizePointCloud({ _cloud1 , _cloud2 }, COLOR_VEC, {2,2});
	fuseTwoPointClouds(_cloud1, _cloud2, distance, edgeDistance);
	visualizePointCloud({ _cloud1 , _cloud2 }, COLOR_VEC, { 2,2 });
	*resCloud = *_cloud1 + *_cloud2;
	return 0;
}

int fuseTwoPointClouds(
	cloud::PointCloudNormalPtr& inCloud1,
	cloud::PointCloudNormalPtr& inCloud2,
	cloud::PointCloudNormalPtr& outCloud1,
	cloud::PointCloudNormalPtr& outCloud2,
	float distance,
	float edgeDistance)
{
	pcl::copyPointCloud(*inCloud1, *outCloud1);
	pcl::copyPointCloud(*inCloud2, *outCloud2);
	fuseTwoPointClouds(outCloud1, outCloud2, distance, edgeDistance);
	return 0;
}

int fuseTwoPointClouds(
	cloud::PointCloudNormalPtr& pointCloudNormalPtr1,
	cloud::PointCloudNormalPtr& pointCloudNormalPtr2,
	float distance,
	float edgeDistance)
{
	MyPointRepresentationXYZNormalCur<cloud::PointNormalT> PointRepresentation;
	int nr = PointRepresentation.getNumberOfDimensions();
	pcl::PointIndices::Ptr edgeIndicesPtr1(new pcl::PointIndices);
	pcl::PointIndices::Ptr edgeIndicesPtr2(new pcl::PointIndices);
	cout << "边缘检测" << endl;
	edgeDetection(pointCloudNormalPtr1, edgeIndicesPtr1, edgeDistance);
	edgeDetection(pointCloudNormalPtr2, edgeIndicesPtr2, edgeDistance);
	//edgeViewer(pointCloudNormalPtr1);
	//edgeViewer(pointCloudNormalPtr2);

	MyPointRepresentationXYZ<cloud::PointNormalT>::Ptr pointRepresentation(new MyPointRepresentationXYZ<cloud::PointNormalT>());
	pcl::KdTreeFLANN<pcl::PointNormal> kdtree;
	kdtree.setPointRepresentation(pointRepresentation);
	kdtree.setInputCloud(pointCloudNormalPtr1);
	kdtree.setSortedResults(true);
	vector<int> indexSearch, _indices(pointCloudNormalPtr1->size(), -1);
	vector<float> squareDisSearch;
	pcl::PointIndices::Ptr cloud1_indices(new pcl::PointIndices()), cloud2_indices(new pcl::PointIndices());
	cloud1_indices->indices.resize(pointCloudNormalPtr1->size());
	cloud2_indices->indices.resize(pointCloudNormalPtr2->size());
	size_t indices1Size = 0, indices2Size = 0;
	float edgeCoefficient1, edgeCoefficient2, coe, disScale = 1.5;
	float sqDistance = distance * distance;
	float realDistance = 0.0f;
	float temp = 0.0f;
	Eigen::Vector3f edgeVec1, edgeVec2,tempVec3f;
	//cout << "拼接" << endl;
	for (size_t _index = 0; _index < pointCloudNormalPtr2->size(); ++_index) { //遍历cloud2
		kdtree.nearestKSearch((*pointCloudNormalPtr2)[_index], 1, indexSearch, squareDisSearch);
		if (indexSearch.size() == 0 || squareDisSearch[0] > sqDistance) {
			//距离阈值内无临近点
			continue;
		}
		else if ((*pointCloudNormalPtr1)[indexSearch[0]].data_n[3] == 0)
		{
			//临近点不是边缘点
			cloud2_indices->indices[indices2Size++] = _index;
			continue;
		}
		kdtree.radiusSearch((*pointCloudNormalPtr2)[_index], distance, indexSearch, squareDisSearch);
		if (indexSearch.size()) { //cloud1与cloud2有重叠
			edgeCoefficient1 = (*pointCloudNormalPtr1)[indexSearch[0]].data_n[3];
			if (edgeCoefficient1){	//cloud2与cloud1边缘有重叠
				realDistance = pow(squareDisSearch[0], 0.5f);
				edgeCoefficient2 = (*pointCloudNormalPtr2)[_index].data_n[3];
				//以cloud1的边缘系数为权重，计算两点加权值，设置cloud2点的位置
				coe = edgeCoefficient1 * (edgeCoefficient1 / (edgeCoefficient2 + edgeCoefficient1));
				coe = 1.0f - (1.0f - coe) * (1- (realDistance / distance));
				for (size_t _xyzIndex = 0; _xyzIndex < 3; ++_xyzIndex) {
					(*pointCloudNormalPtr2)[_index].data[_xyzIndex] = ((*pointCloudNormalPtr2)[_index].data[_xyzIndex]) * coe
						+ (*pointCloudNormalPtr1)[indexSearch[0]].data[_xyzIndex] * (1-coe);
				}
				for (size_t _normalIndex = 0; _normalIndex < 3; ++_normalIndex) {
					(*pointCloudNormalPtr2)[_index].data_n[_normalIndex] = ((*pointCloudNormalPtr2)[_index].data_n[_normalIndex]) * coe
						+ (*pointCloudNormalPtr1)[indexSearch[0]].data_n[_normalIndex] * (1-coe);
				}
				(*pointCloudNormalPtr2)[_index].curvature = ((*pointCloudNormalPtr2)[_index].curvature) * coe
					+ (*pointCloudNormalPtr1)[indexSearch[0]].curvature * (1 - coe);
				for (auto __index : indexSearch) {
					tempVec3f = (*pointCloudNormalPtr1)[__index].getVector3fMap() 
						- (*pointCloudNormalPtr2)[_index].getVector3fMap();
					tempVec3f += (*pointCloudNormalPtr1)[__index].getNormalVector3fMap();
					tempVec3f.normalize();
					temp = (*pointCloudNormalPtr2)[_index].getNormalVector3fMap().dot(tempVec3f);
					if(temp >0.9)_indices[__index] = __index; //cloud1边缘重叠的索引号放入滤除的indices中
				}
			}
			else {//cloud2与cloud1内部有重叠
				//cloud2索引号放入滤除的indices中
				cloud2_indices->indices[indices2Size++] = _index;
			}
		}
	}
	for (auto _index : _indices) {
		if (_index != -1 && (*pointCloudNormalPtr1)[_index].data_n[3]>0.0f) {
			cloud1_indices->indices[indices1Size++] = _index;
		}
	}
	cloud1_indices->indices.resize(indices1Size);
	cloud2_indices->indices.resize(indices2Size);

	pcl::ExtractIndices<pcl::PointNormal> extract;
	//去除cloud1边缘点
	extract.setInputCloud(pointCloudNormalPtr1);
	extract.setIndices(cloud1_indices);
	extract.setNegative(true);
	extract.filter(*pointCloudNormalPtr1);
	//去除cloud2中与cloud1内部重叠的点
	extract.setInputCloud(pointCloudNormalPtr2);
	extract.setIndices(cloud2_indices);
	extract.setNegative(true);
	extract.filter(*pointCloudNormalPtr2);
	//visualizePointCloud({ cloud1, cloud2 }, COLOR_VEC, { 2,2 });
	//*resCloud += *cloud1;
	return 0;
}

int fusePointClouds(
	cloud::PointCloudNormalPtrVec& inPointCloudPtrVec,
	cloud::PointCloudNormalPtr& outPointCloudPtr,
	float distance,
	float edgeDistance)
{
	//if (outPointCloudPtrVec.size() != inPointCloudPtrVec.size()) {
	//	outPointCloudPtrVec.resize(inPointCloudPtrVec.size());
	//}
	//for (size_t _index = 0; _index < inPointCloudPtrVec.size(); ++_index) {
	//	if (!outPointCloudPtrVec[_index]) outPointCloudPtrVec[_index] = std::make_shared<cloud::PointCloud>();
	//	pcl::copyPointCloud(*(inPointCloudPtrVec[_index]), *(outPointCloudPtrVec[_index]));
	//}
	//cloud::PointCloudPtr outPointCloudPtr;
	bool showInfo = true;
	//visualizePointCloud(inPointCloudPtrVec, COLOR_VEC);
	pcl::copyPointCloud(*(inPointCloudPtrVec[0]), *outPointCloudPtr);
	for (size_t _index = 1; _index < inPointCloudPtrVec.size(); ++_index) {
		if (showInfo) {
			cout << endl;
			cout << "\rfusing " << _index - 1 << "th and " << _index << "th " 
				<< _index << "/" << inPointCloudPtrVec.size()-1 << endl;
		}
		//visualizePointCloud({ outPointCloudPtr }, COLOR_VEC);
		removeInvalid(outPointCloudPtr);
		//visualizePointCloud({ outPointCloudPtr, inPointCloudPtrVec[_index] }, COLOR_VEC);
		if(edgeDistance <=0)fuseTwoPointClouds(outPointCloudPtr, inPointCloudPtrVec[_index], outPointCloudPtr, distance, -1.0f * edgeDistance * getResolution<cloud::PointNormalT>(inPointCloudPtrVec[_index]));
		else fuseTwoPointClouds(outPointCloudPtr, inPointCloudPtrVec[_index], outPointCloudPtr, distance, edgeDistance);
		//visualizePointCloud({ outPointCloudPtr }, COLOR_VEC);
	}
	return 0;
}

int edgeDetection(
	cloud::PointCloudNormalPtr& cloudPtr,
	float distance) 
{
	pcl::PointIndices::Ptr edgeIndicesPtr(new pcl::PointIndices);
	edgeDetection(cloudPtr, edgeIndicesPtr, distance);
	return 0;
}

int edgeDetection(
	cloud::PointCloudNormalPtr& cloudPtr,
	pcl::PointIndices::Ptr& edgeIndicesPtr,
	float distance,
	float min,
	float max) 
{
	
	//PointRepresentation，只取x，y，z数据
	int ii = 200;
	pcl::KdTreeFLANN<cloud::PointNormalT> kdtree;
	MyPointRepresentationXYZ<cloud::PointNormalT>::Ptr pointRepresentation(new MyPointRepresentationXYZ<cloud::PointNormalT>());
	kdtree.setPointRepresentation(pointRepresentation);
	kdtree.setInputCloud(cloudPtr);

	vector<int> k_indices;
	vector<float> k_sqr_distances;
	Eigen::Vector3f normal_n, pointVec_n, edgeSign=Eigen::Vector3f::Zero(), crossTemp, _edgeSign;
	pcl::PointNormal test;
	size_t edgeIndicesIndex = 0;
	edgeIndicesPtr->indices.resize(cloudPtr->size());
	for (int _index = 0; _index < cloudPtr->size(); ++_index) {
		kdtree.radiusSearch((*cloudPtr)[_index], distance, k_indices, k_sqr_distances); //查找指定点P0的半径distance内的点
		normal_n = (*cloudPtr)[_index].getNormalVector3fMap(); //点的法线
		edgeSign = Eigen::Vector3f::Zero();
		for (size_t __index = 0; __index < k_indices.size(); ++__index) {
			if (k_sqr_distances[__index]) { //不是该点本身的情况
				pointVec_n = (*cloudPtr)[k_indices[__index]].getVector3fMap() - (*cloudPtr)[_index].getVector3fMap(); //求向量Pi - P0
				//pointVec_n.normalize();
				edgeSign += normal_n.cross(pointVec_n).cross(normal_n); //edgeSign += (pointVec_n向量) 在P0切平面上的投影
				//_edgeSign = edgeSign.normalized();
				//for (size_t _edgeSignIndex = 0; _edgeSignIndex < 3; ++_edgeSignIndex) {
				//	(*cloudPtr)[_index].data_c[_edgeSignIndex] = _edgeSign[_edgeSignIndex];
				//}
				//edgeSign += crossTemp.cross(normal_n);
			}
		}
		if (k_indices.size()) { //P0附近有其他点
			(*cloudPtr)[_index].data_n[3] = edgeSign.norm() / (float)k_indices.size() / distance; //计算边缘系数并归一化
			// 边缘系数映射 [0,1] 映射到 [min,1]
			if ((*cloudPtr)[_index].data_n[3] < min) {
				(*cloudPtr)[_index].data_n[3] = 0.0f;
			}
			else if ((*cloudPtr)[_index].data_n[3] >= max) {
				(*cloudPtr)[_index].data_n[3] = 1.0f;
				edgeIndicesPtr->indices[edgeIndicesIndex++] = _index;
			}
			else {
				(*cloudPtr)[_index].data_n[3] = ((*cloudPtr)[_index].data_n[3] - min) / (max - min);
				edgeIndicesPtr->indices[edgeIndicesIndex++] = _index;
			}
		}
		else { //P0附近无其他点
			(*cloudPtr)[_index].data_n[3] = 1.0f;
			edgeIndicesPtr->indices[edgeIndicesIndex++] = _index;
		}
	}
	//edgeViewer(cloudPtr);
	cloud::PointCloudNormal _cloudTemp;
	pcl::copyPointCloud(*cloudPtr, _cloudTemp);
	vector<float> weight;
	float weightSum = 0, edgeSignTemp=0;
	for (int _index = 0; _index < cloudPtr->size(); ++_index) {
		kdtree.radiusSearch((*cloudPtr)[_index], distance, k_indices, k_sqr_distances, 4);
		weight.resize(k_indices.size());
		weightSum = 0;
		for (size_t __index = 0; __index < k_indices.size(); ++__index) {
			//weight[__index] = pow(2.71828, k_sqr_distances[__index] * -0.5f);
			weight[__index] = gaussian(pow(k_sqr_distances[__index], 0.5f) * 1000.0f, 0.0f, 2.0f);
			//cout << "weight[__index]:" << weight[__index] << endl;
			weightSum += weight[__index];
		}
		if(weightSum)edgeSignTemp = 0.0f;
		for (size_t __index = 0; __index < k_indices.size(); ++__index) {
			edgeSignTemp += _cloudTemp[k_indices[__index]].data_n[3] * weight[__index] / weightSum;
			//cout << (*cloudPtr)[_index].data_n[3] << endl;
		}
		(*cloudPtr)[_index].data_n[3] = edgeSignTemp;
	}
	return 0;
}

int edgeViewer(
	cloud::PointCloudNormalPtr pointCloudNormalPtr)
{
	pcl::PointCloud<pcl::PointXYZRGB> pointRGB;
	pointRGB.resize(pointCloudNormalPtr->size());
	for (int _index = 0; _index < pointCloudNormalPtr->size(); ++_index) {
		pcl::copyPoint(pointCloudNormalPtr->at(_index), pointRGB[_index]);
		pointRGB[_index].r = 255 - (*pointCloudNormalPtr)[_index].data_n[3] * 255;
		pointRGB[_index].b = 255 - (*pointCloudNormalPtr)[_index].data_n[3] * 255;
		pointRGB[_index].g = 255;
	}
	pcl::visualization::PCLVisualizer viewer("3D Viewer");
	viewer.addPointCloud(pointRGB.makeShared());
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3);
	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
	}

	return 0;
}

float gaussian(
	float x,
	float u,
	float sigma)
{
	float result = (x - u) / sigma;
	result = result * result * -0.5f;
	result = pow(2.71828, result) / sigma / pow(2 * 3.14159, 0.5);
	return result;
}


void 
NDTpair(
	cloud::PointCloudPtr srcPointCloudPtr,
	cloud::PointCloudPtr tgtPointCloudPtr,
	cloud::PointCloudPtr resPointCloudPtr,
	Eigen::Matrix4f& final_transformation,
	float transformationEpsilon,
	float stepSize,
	float resolution)
{
	cloud::PointCloudPtr filtered_cloud(new cloud::PointCloud);
	pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
	approximate_voxel_filter.setLeafSize(0.005, 0.005, 0.005);
	approximate_voxel_filter.setInputCloud(srcPointCloudPtr);
	approximate_voxel_filter.filter(*filtered_cloud);

	pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;

	ndt.setTransformationEpsilon(transformationEpsilon);
	ndt.setStepSize(stepSize);
	ndt.setResolution(getResolution<cloud::PointT>(filtered_cloud));
	ndt.setMaximumIterations(256);
	ndt.setInputSource(filtered_cloud);
	ndt.setInputTarget(tgtPointCloudPtr);
	ndt.align(*resPointCloudPtr);
	final_transformation = ndt.getFinalTransformation();
}

cloud::OBBT getOBB(
	cloud::PointCloudPtr cloudPtr) 
{
	cloud::OBBT obb;
	pcl::MomentOfInertiaEstimation<cloud::PointT> feature_extractor;
	feature_extractor.setInputCloud(cloudPtr);
	feature_extractor.compute();
	feature_extractor.getOBB(obb.min, obb.max, obb.poistion, obb.rotation);
	return obb;
}

void boxToPointCloud(
	const cloud::OBBT& box,
	cloud::PointCloudPtr& cloudPtr)
{
	cloudPtr->resize(8);
	float widthOBB = box.max.x - box.min.x;
	float heightOBB = box.max.y - box.min.y;
	float depthOBB = box.max.z - box.min.z;
	int xx = 1, yy = 1, zz = 1;
	for (int _index = 0; _index < 8; ++_index) {
		cloudPtr->at(_index).x = xx * widthOBB / 2.0;
		cloudPtr->at(_index).y = yy * heightOBB / 2.0;
		cloudPtr->at(_index).z = zz * depthOBB / 2.0;
		xx *= -1;
		if ((_index + 1) % 2 == 0)yy *= -1;
		if ((_index + 1) % 4 == 0)zz *= -1;
	}
	Eigen::Matrix4f trans;
	Eigen::Translation3f translation(box.poistion.x, box.poistion.y, box.poistion.z);
	Eigen::Quaternionf quat(box.rotation);
	trans = (translation * quat).matrix();
	pcl::transformPointCloud(*cloudPtr, *cloudPtr, trans);
}

Eigen::Matrix4f getTransFromOBBT(
	cloud::OBBT box)
{
	Eigen::Translation3f translation(box.poistion.x, box.poistion.y, box.poistion.z);
	Eigen::Quaternionf quat(box.rotation);
	return (translation * quat).matrix();
}

void segmenteByDis(
	cloud::PointCloudPtr& inCloudPtr,
	cloud::PointCloudPtrVec& outCloudPtrVec,
	float dis,
	int min_num,
	int max_num)
{
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(inCloudPtr);
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	if (dis <= 0)ec.setClusterTolerance(getResolution<cloud::PointT>(inCloudPtr) * 2);
	else ec.setClusterTolerance(dis);
	ec.setMinClusterSize(min_num);
	ec.setMaxClusterSize(max_num);
	ec.setSearchMethod(tree);
	ec.setInputCloud(inCloudPtr);
	ec.extract(cluster_indices);

	outCloudPtrVec.clear();
	for (vector<pcl::PointIndices>::const_iterator indicesIt = cluster_indices.begin(); indicesIt != cluster_indices.end(); ++indicesIt) {
		cloud::PointCloudPtr tempPtr(new cloud::PointCloud);
		for (vector<int>::const_iterator indexIt = indicesIt->indices.begin(); indexIt != indicesIt->indices.end(); ++indexIt) {
			tempPtr->push_back(inCloudPtr->points[*indexIt]);
		}
		tempPtr->width = tempPtr->size();
		tempPtr->height = 1;
		//tempPtr->is_dense = true;
		outCloudPtrVec.push_back(tempPtr);
	}
}

void segmenteByDis(
	cloud::PointCloudNormalPtr& inCloudPtr,
	cloud::PointCloudNormalPtrVec& outCloudPtrVec,
	float dis,
	int min_num,
	int max_num)
{
	MyPointRepresentationXYZ<cloud::PointNormalT>::Ptr rep(new MyPointRepresentationXYZ<cloud::PointNormalT>);
	pcl::search::KdTree<cloud::PointNormalT>::Ptr tree(new pcl::search::KdTree<cloud::PointNormalT>);
	tree->setPointRepresentation(rep);
	tree->setInputCloud(inCloudPtr);
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<cloud::PointNormalT> ec;
	if (dis <= 0)ec.setClusterTolerance(getResolution<cloud::PointNormalT>(inCloudPtr) * 2);
	else ec.setClusterTolerance(dis);
	ec.setMinClusterSize(min_num);
	ec.setMaxClusterSize(max_num);
	ec.setSearchMethod(tree);
	ec.setInputCloud(inCloudPtr);
	ec.extract(cluster_indices);

	outCloudPtrVec.clear();
	for (vector<pcl::PointIndices>::const_iterator indicesIt = cluster_indices.begin(); indicesIt != cluster_indices.end(); ++indicesIt) {
		cloud::PointCloudNormalPtr tempPtr(new cloud::PointCloudNormal);
		for (vector<int>::const_iterator indexIt = indicesIt->indices.begin(); indexIt != indicesIt->indices.end(); ++indexIt) {
			tempPtr->push_back(inCloudPtr->points[*indexIt]);
		}
		tempPtr->width = tempPtr->size();
		tempPtr->height = 1;
		//tempPtr->is_dense = true;
		outCloudPtrVec.push_back(tempPtr);
	}
}

void removeDiagonally(
	cloud::PointCloudNormalPtr& inCloudPtr,
	cloud::PointCloudNormalPtr& outCloudPtr,
	float th)
{
	outCloudPtr->resize(inCloudPtr->size());
	size_t size = 0;
	for (size_t _index = 0; _index < inCloudPtr->size(); ++_index) {
		pcl::PointNormal& p= (*inCloudPtr)[_index];
		if (p.getVector3fMap().normalized().dot(p.getNormalVector3fMap()) <= th) {
			(*outCloudPtr)[size++] = p;
		}
	}
	outCloudPtr->resize(size);
}

void ISSKeypoints(
	cloud::PointCloudPtr& inCloud, 
	cloud::PointCloudPtr& outCloud) 
{
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

	// Fill in the model cloud

	double model_resolution = getResolution<pcl::PointXYZ>(inCloud);

	// Compute model_resolution

	pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> iss_detector;

	iss_detector.setSearchMethod(tree);
	iss_detector.setSalientRadius(6 * model_resolution);
	iss_detector.setNonMaxRadius(4 * model_resolution);
	iss_detector.setThreshold21(0.975);
	iss_detector.setThreshold32(0.975);
	iss_detector.setMinNeighbors(5);
	iss_detector.setNumberOfThreads(4);
	iss_detector.setInputCloud(inCloud);
	iss_detector.compute(*outCloud);
}

void ISSKeypoints(
	cloud::PointCloudNormalPtr& inCloud,
	cloud::PointCloudNormalPtr& outCloud)
{
	MyPointRepresentationXYZ<cloud::PointNormalT>::Ptr rep(new MyPointRepresentationXYZ<cloud::PointNormalT>);
	pcl::search::KdTree<cloud::PointNormalT>::Ptr tree(new pcl::search::KdTree<cloud::PointNormalT>());
	tree->setPointRepresentation(rep);

	// Fill in the model cloud

	double model_resolution = getResolution<cloud::PointNormalT>(inCloud);

	// Compute model_resolution

	pcl::ISSKeypoint3D<cloud::PointNormalT, cloud::PointNormalT> iss_detector;

	iss_detector.setSearchMethod(tree);
	iss_detector.setSalientRadius(6 * model_resolution);
	iss_detector.setNonMaxRadius(4 * model_resolution);
	iss_detector.setThreshold21(0.975);
	iss_detector.setThreshold32(0.975);
	iss_detector.setMinNeighbors(5);
	iss_detector.setNumberOfThreads(4);
	iss_detector.setInputCloud(inCloud);
	iss_detector.compute(*outCloud);
}


void curKeypoints(
	cloud::PointCloudNormalPtr inCloud,
	cloud::PointCloudNormalPtr outCloud,
	float dis,
	int divNum,
	float numTh)
{
	MyPointRepresentationXYZ<cloud::PointNormalT>::Ptr rep(new MyPointRepresentationXYZ<cloud::PointNormalT>);
	pcl::search::KdTree<cloud::PointNormalT>::Ptr kdtreePtr(new pcl::search::KdTree<cloud::PointNormalT>);
	kdtreePtr->setPointRepresentation(rep);
	kdtreePtr->setInputCloud(inCloud);
	vector<int> indices;
	vector<float> distances;
	float maxC, minC, cur_, curTh;
	int count = 0;
	vector<int> divVec(divNum, 0);
	for (size_t _index = 0; _index < inCloud->size(); ++_index) {
		kdtreePtr->radiusSearch((*inCloud).points[_index], dis, indices, distances);
		maxC = (*inCloud)[_index].curvature;
		minC = maxC;
		cur_ = (*inCloud)[_index].curvature;
		for (size_t __index = 0; __index < indices.size(); ++__index) {
			if (distances[__index] == 0)continue;
			if ((*inCloud)[indices[__index]].curvature < minC) minC = (*inCloud)[indices[__index]].curvature;
			if ((*inCloud)[indices[__index]].curvature > maxC) maxC = (*inCloud)[indices[__index]].curvature;
		}
		curTh = (maxC - minC) / (float)divNum;
		if (curTh < 1e-3)continue;
		//cout << curTh << endl;
		count = 0;
		for (size_t __index = 0; __index < indices.size(); ++__index) {
			if (abs((*inCloud)[__index].curvature - cur_) <= curTh)++count;
		}
		if ((float)count * numTh < indices.size()) {
			outCloud->push_back((*inCloud)[_index]);
		}
	}
}

void Keypoints2(
	cloud::PointCloudNormalPtr inCloud,
	cloud::PointCloudNormalPtr outCloud,
	float dis,
	float minDis,
	float Th,
	float maxPersent,
	float minPersent)
{
	outCloud->clear();
	MyPointRepresentationXYZ<cloud::PointNormalT>::Ptr rep(new MyPointRepresentationXYZ<cloud::PointNormalT>);
	pcl::search::KdTree<cloud::PointNormalT>::Ptr kdtreePtr(new pcl::search::KdTree<cloud::PointNormalT>);
	kdtreePtr->setPointRepresentation(rep);
	kdtreePtr->setInputCloud(inCloud);
	vector<int> indices;
	vector<float> distances;
	Eigen::Vector3f pVec;
	float dot1, dot2;
	int count0, count1, count_1;
	for (size_t _index = 0; _index < inCloud->size(); ++_index) {
		kdtreePtr->radiusSearch((*inCloud).points[_index], dis, indices, distances);
		cloud::PointNormalT& _p = (*inCloud)[_index];
		count0 = count1 = count_1 = 0;
		for (size_t __index = 0; __index < indices.size(); ++__index) {
			if (distances[__index] < minDis)continue;
			pVec = ((*inCloud)[indices[__index]].getVector3fMap() - _p.getVector3fMap()).normalized();
			dot1 = pVec.dot(_p.getNormalVector3fMap());
			dot2 = (-1 * pVec).dot((*inCloud)[indices[__index]].getNormalVector3fMap());
			if (dot1 > Th && dot2 > Th)++count1;
			else if (dot1 < Th*-1 && dot2 < Th*-1)++count_1;
			else ++count0;
		}
		if(count0  * 1.2 > indices.size())(*inCloud)[_index].data_n[3] = 0;
		else if(count1 > count_1) (*inCloud)[_index].data_n[3] = 1;
		else (*inCloud)[_index].data_n[3] = -1;
	}
	keyViewer(inCloud);
	float label, tempF;
	int sameNum;
	for (size_t _index = 0; _index < inCloud->size(); ++_index) {
		label = (*inCloud)[_index].data_n[3];
		kdtreePtr->nearestKSearch((*inCloud).points[_index], 8, indices, distances);
		//kdtreePtr->radiusSearch((*inCloud).points[_index], dis, indices, distances);
		sameNum = 0;
		for (size_t __index = 0; __index < indices.size(); ++__index) {
			if (label == (*inCloud)[indices[__index]].data_n[3] && distances[__index] < dis) ++sameNum;
		}
		tempF = (float)sameNum / (float)indices.size();
		if (tempF < maxPersent && tempF > minPersent) {
			cloud::PointNormalT _temp;
			pcl::copyPoint((*inCloud)[_index], _temp);
			outCloud->push_back((*inCloud)[_index]);
		}
	}
}

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
	bool conver)
{
	cloud::PointCloudNormalPtr outCloud(new cloud::PointCloudNormal);
	pcl::copyPointCloud(*inCloud, *outCloud);
	outIndices.clear();
	outIndices.resize(outCloud->size());
	//pcl::PointCloud<pcl::Histogram<4>> cloudFeature;
	cloudFeature.resize(outCloud->size());
	int outIndicesIndex = 0;
	//构建kd-tree
	MyPointRepresentationXYZ<cloud::PointNormalT>::Ptr rep(new MyPointRepresentationXYZ<cloud::PointNormalT>);
	pcl::search::KdTree<cloud::PointNormalT>::Ptr kdtreePtr(new pcl::search::KdTree<cloud::PointNormalT>);
	kdtreePtr->setPointRepresentation(rep);
	kdtreePtr->setInputCloud(outCloud);
	vector<int> indices;
	vector<float> distances;

	Eigen::Vector3f pVec;
	float dot1, dot2;
	float theta, beta;
	float alpha1, alpha2, alpha3;
	int count0, count1, count_1, count;
	for (size_t _index = 0; _index < outCloud->size(); ++_index) {
		kdtreePtr->radiusSearch((*outCloud).points[_index], dis, indices, distances);
		cloud::PointNormalT& _p = (*outCloud)[_index];
		count0 = count1 = count_1 = 0;
		for (size_t __index = 0; __index < indices.size(); ++__index) {
			if (distances[__index] <= dis2)continue; //是该点本身
			pVec = ((*outCloud)[indices[__index]].getVector3fMap() - _p.getVector3fMap()).normalized();
			dot1 = pVec.dot(_p.getNormalVector3fMap());
			dot2 = (-1 * pVec).dot((*outCloud)[indices[__index]].getNormalVector3fMap());
			theta = acosf(dot1);
			//theta = (acosf(dot1) + acosf(dot2)) * 0.5;
			beta = (dot1 + dot2)/2.0;
			//beta = (theta / PI - 0.5) * 2.0;
			if (beta > betaTh)++count_1; //凹点对
			else if (beta < -1*betaTh)++count1; //凸点对
			else ++count0; //平面点对
		}
		count = count1 + count0 + count_1;
		alpha1 = (float)count1 / (float)count; //凸点对百分比
		alpha2 = (float)count0 / (float)count; //平面点对百分比
		alpha3 = (float)count_1 / (float)count; //凹点对百分比
		if (alpha1 > alphaTh)(*outCloud)[_index].data_n[3] = 1;	//凸峰面，红色
		else if(alpha2 > alphaTh)(*outCloud)[_index].data_n[3] = 2;	//平面，绿色
		else if(alpha3 > alphaTh)(*outCloud)[_index].data_n[3] = 3;	//凹谷面，蓝色
		else if (alpha1 > alphaTh2 && alpha2 > alphaTh2 && alpha3 > alphaTh2) {
			(*outCloud)[_index].data_n[3] = 0;
		}
		else if (alpha1 > alpha2) {
			if (alpha3 > alpha2) {
				(*outCloud)[_index].data_n[3] = 4;	//鞍面，黄色
			}
			else {
				(*outCloud)[_index].data_n[3] = 5;	//凸脊面，紫色
			}
		}
		else {
			if (alpha1 > alpha3) {
				(*outCloud)[_index].data_n[3] = 6;	//凹脊面，青色
			}
			else {
				(*outCloud)[_index].data_n[3] = 5;	//凸脊面，紫色
			}
		}
	}
	class MyPointRepresentation :public pcl::PointRepresentation<cloud::PointNormalT> {
		using pcl::PointRepresentation<cloud::PointNormalT>::nr_dimensions_;
	public:
		using Ptr = shared_ptr<MyPointRepresentation>;
		MyPointRepresentation() {
			nr_dimensions_ = 4;
		}
		virtual void copyToFloatArray(const cloud::PointNormalT& p, float* out) const
		{
			out[0] = p.x;
			out[1] = p.y;
			out[2] = p.z;
			out[3] = p.data_n[3];
		}
	};
	MyPointRepresentation::Ptr pointRepresentationPtr(new MyPointRepresentation);
	pcl::search::KdTree<cloud::PointNormalT>::Ptr tree(new pcl::search::KdTree<cloud::PointNormalT>);
	tree->setPointRepresentation(pointRepresentationPtr);
	tree->setInputCloud(outCloud);
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<cloud::PointNormalT> ec;
	ec.setClusterTolerance(segDis); // 2cm
	ec.setMinClusterSize(0);
	ec.setMaxClusterSize(25000);
	ec.setSearchMethod(tree);
	ec.setInputCloud(outCloud);
	ec.extract(cluster_indices);
	for (const auto& _indices : cluster_indices) {
		if (_indices.indices.size() > minSize) continue;
		for (const auto& _indice : _indices.indices) {
			(*outCloud)[_indice].data_n[3] = 0;
		}
	}
	map<int, int, less<int>> numberCountMap;
	map<int, int, less<int>>::iterator mapIter;
	int minNum, maxNumID;
	if (conver) {
		for (int _index = 0; _index < outCloud->size(); ++_index) {
			if ((*outCloud)[_index].data_n[3] != 0)continue;
			kdtreePtr->nearestKSearch((*outCloud).points[_index], 8, indices, distances);
			numberCountMap.clear();
			for (const auto& _indice : indices) {
				mapIter = numberCountMap.find(_indice);
				if ((*outCloud)[_indice].data_n[3] == 0)continue;
				if (mapIter == numberCountMap.end()) {
					numberCountMap.insert(pair<int, int>((*outCloud)[_indice].data_n[3], 1));
				}
				else {
					++(*mapIter).second;
				}
			}
			minNum = 0;
			maxNumID = 0;
			for (const auto& _e : numberCountMap) {
				if (_e.second > minNum) {
					minNum = _e.second;
					maxNumID = _e.first;
				}
			}
			(*outCloud)[_index].data_n[3] = maxNumID;
		}
	}
	tree->setInputCloud(outCloud);
	ec.setSearchMethod(tree);
	ec.setInputCloud(outCloud);
	ec.extract(cluster_indices);
	float percent;
	for (const auto& _indices : cluster_indices) {
		percent = _indices.indices.size() / (float)outCloud->size();
		for (const auto& _indice : _indices.indices) {
			(*outCloud)[_indice].data_c[1] = percent;
		}
	}
	keyViewer(outCloud);
	Eigen::Vector3f x_normal, y_normal, z_normal;
	float label, tempF;
	float xTemp, yTemp;
	//int count;
	int nearestIndex;
	float minDis = dis;
	map<float, int, less<float>> dict;
	map<int, float, less<float>> dict2;
	for (size_t _index = 0; _index < outCloud->size(); ++_index) {
		label = (*outCloud)[_index].data_n[3];
		if (label == 0)continue;
		kdtreePtr->nearestKSearch((*outCloud).points[_index], 8, indices, distances);
		minDis = dis;
		nearestIndex = 1;
		for (int __index = 0; __index < distances.size(); ++__index) {
			if (distances[__index] == 0)continue; //是该点本身
			if (distances[__index] < minDis) {
				nearestIndex = __index;
				minDis = distances[__index];
			}
		}

		z_normal = (*outCloud)[_index].getNormalVector3fMap();
		pVec = ((*outCloud).points[nearestIndex].getVector3fMap() - (*outCloud)[_index].getVector3fMap()).normalized();
		x_normal = z_normal.cross(pVec);
		y_normal = z_normal.cross(x_normal);

		dict.clear();
		for (size_t __index = 0; __index < indices.size(); ++__index) {
			if (distances[__index] == 0)continue; //是该点本身
			pVec = (*outCloud)[indices[__index]].getVector3fMap() - (*outCloud)[_index].getVector3fMap();
			xTemp = pVec.dot(x_normal);
			yTemp = pVec.dot(y_normal);
			theta = atan2f(yTemp, xTemp);
			dict.insert(pair<float, int>(theta, (*outCloud)[indices[__index]].data_n[3]));
			dict2.insert(pair<int, float>((*outCloud)[indices[__index]].data_n[3], (*outCloud)[indices[__index]].data_c[1]));
		}
		count = 0;
		for (const auto& e : dict) {
			if (e.second == label || e.second == 0)count=0;
			else ++count;
			if (count > edgeN && e.second < label) {
				outIndices[outIndicesIndex] = _index;
				cloudFeature[outIndicesIndex].histogram[0] = e.second;
				cloudFeature[outIndicesIndex].histogram[1] = label;
				cloudFeature[outIndicesIndex].histogram[2] = dict2[e.second];
				cloudFeature[outIndicesIndex].histogram[3] = (*outCloud)[_index].data_c[1];
				++outIndicesIndex;
				break;
			}
		}
	}
	cloudFeature.resize(outIndicesIndex);
	outIndices.resize(outIndicesIndex);
}

int keyViewer(
	cloud::PointCloudNormalPtr pointCloudNormalPtr)
{
	pcl::PointCloud<pcl::PointXYZRGB> pointRGB;
	pointRGB.resize(pointCloudNormalPtr->size());
	vector<cloud::Color> colorVec;
	colorVec = COLOR_VEC;
	colorVec.insert(colorVec.begin(),BLACK_COLOR);
	int colorIndex;
	for (int _index = 0; _index < pointCloudNormalPtr->size(); ++_index) {
		pcl::copyPoint(pointCloudNormalPtr->at(_index), pointRGB[_index]);
		//colorIndex = (int)(*pointCloudNormalPtr)[_index].data_n[3] - 1;
		colorIndex = (int)(*pointCloudNormalPtr)[_index].data_n[3];
		pointRGB[_index].r = colorVec[colorIndex % colorVec.size()].r;
		pointRGB[_index].g = colorVec[colorIndex % colorVec.size()].g;
		pointRGB[_index].b = colorVec[colorIndex % colorVec.size()].b;
		//if ((*pointCloudNormalPtr)[_index].data_n[3] == 1) {
		//	pointRGB[_index].r = 255;
		//	pointRGB[_index].g = 255;
		//	pointRGB[_index].b = 255;
		//}
		//else if ((*pointCloudNormalPtr)[_index].data_n[3] == 2) {
		//	pointRGB[_index].r = 255;
		//	pointRGB[_index].g = 0;
		//	pointRGB[_index].b = 0;
		//}
		//else {
		//	pointRGB[_index].r = 0;
		//	pointRGB[_index].g = 255;
		//	pointRGB[_index].b = 0;
		//}
	}
	pcl::visualization::PCLVisualizer viewer("3D Viewer");
	viewer.addPointCloud(pointRGB.makeShared());
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1);
	viewer.setBackgroundColor(255, 255, 255);
	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
	}

	return 0;
}

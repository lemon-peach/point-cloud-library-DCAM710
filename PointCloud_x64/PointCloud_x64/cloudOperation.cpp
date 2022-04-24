#define PCL_NO_PRECOMPILE
#include "cloudOperation.h"
#include <vtkRenderWindow.h>

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
			//r�뾶����
			//treeXYZ.radiusSearch((*inputXYZ)[*idx], max_distance, index, distance);
			treeXYZ.radiusSearchT((*inputXYZ)[*idx], max_distance, index, distance, 20);
			//cout << "1" << endl;
			point_representation_->vectorize<vector<float>>((*input_)[*idx], _inputPointData);
			//cout << "2" <<endl;
			//����r�뾶�����ƥ���
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
			if (index.empty() || distance[0] > max_dist_sqr)
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
		//���ã�Դ����
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

/**
 * @brief		pcl���ӻ�����
 * @param[in]	filePath pcd	�ļ�·��
 * @return		0���ɹ���1��ʧ��
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
 * @brief		�����������
 * @param[out]	cloudPtr	����
 * @param[in]	width		���ƵĿ�
 * @param[in]	height		���Ƶĸ�
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
	int k,
	float r)
{
	pcl::PointCloud<pcl::Normal>::Ptr normalPtr(new pcl::PointCloud<pcl::Normal>);
	normalEstimation(inCloudPtr, normalPtr, k, r);
	pcl::concatenateFields(*inCloudPtr, *normalPtr, *outCloudNormalPtr);
	return 0;
}

/**
 * @brief		ʹ�� RANSAC ��������ƽ��ָ�
 * @param[in]	threshold		��ֵ
 * @param[in]	srcCloudPtr		ԭʼ����
 * @param[out]	dstCloudPtr		�ָ��ĵ���
 * @param[in]	negative		��ȡ�෴�ĵ���
 * @return		0���ɹ�	-1��ʧ��
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
 * @brief	ʹ�� RANSAC ��������ƽ��ָ�
 * @param	threshold	��ֵ
 * @param	srcCloudPtr ԭʼ����
 * @param	inliers		�ָ��ĵ�������
 * @return	0���ɹ�	-1��ʧ��
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
	bool downSample,
	float downSampleSize,
	float maxDis,
	bool useMyEm,
	bool edgeFilter)
{
	//�Զ��� Point Representation
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
	cloud::PointCloudPtr _srcCloudPtr(new cloud::PointCloud);  //Դ�����²���
	cloud::PointCloudPtr _tgtCloudPtr(new cloud::PointCloud);  //Ŀ������²���
	//�²���
	startTime = clock();
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
	endTime = clock();
	useTime = (double)(endTime - startTime) / CLOCKS_PER_SEC;
	if(showInfo && downSample)cout << "��������ʱ��" << useTime << endl;

	//�˳���Ⱥ��
	float resolution = getResolution(_srcCloudPtr);
	cout << "resolution" << resolution <<endl;
	pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
	outrem.setMinNeighborsInRadius(12);
	outrem.setKeepOrganized(false);
	outrem.setRadiusSearch(resolution * 4.0f);
	outrem.setInputCloud(_srcCloudPtr);
	outrem.filter(*_srcCloudPtr);
	outrem.setInputCloud(_tgtCloudPtr);
	outrem.filter(*_tgtCloudPtr);

	//���㷨�ߺ�����
	float normalRadius = 4.0f * resolution;
	cout << "normalRadius:" << normalRadius << endl;
	pcl::NormalEstimation<cloud::PointT, cloud::PointNormalT> normal_est;
	pcl::search::KdTree<cloud::PointT>::Ptr kdtree(new pcl::search::KdTree<cloud::PointT>());
	normal_est.setSearchMethod(kdtree);  //������������
	//normal_est.setKSearch(30);  //���� K ���ٽ������ڼ��㷨�ߺ�����
	normal_est.setRadiusSearch(normalRadius);

	normal_est.setInputCloud(_srcCloudPtr);
	cloud::PointCloudNormalPtr _srcCloudNormalPtr(new cloud::PointCloudNormal);
	normal_est.compute(*_srcCloudNormalPtr);
	pcl::copyPointCloud(*_srcCloudPtr, *_srcCloudNormalPtr);

	normal_est.setInputCloud(_tgtCloudPtr);
	cloud::PointCloudNormalPtr _tgtCloudNormalPtr(new cloud::PointCloudNormal);
	normal_est.compute(*_tgtCloudNormalPtr);
	pcl::copyPointCloud(*_tgtCloudPtr, *_tgtCloudNormalPtr);

	//�˳��߽��
	if (edgeFilter) {
		cout << "edge" << endl;
		pcl::PointIndices::Ptr indicesPtr(new pcl::PointIndices);
		pcl::ExtractIndices<cloud::PointNormalT> extract;
		edgeDetection(_srcCloudNormalPtr, indicesPtr, normalRadius * 2.0f);
		edgeViewer(_srcCloudNormalPtr);
		extract.setInputCloud(_srcCloudNormalPtr);
		extract.setIndices(indicesPtr);
		extract.setNegative(true);
		extract.filter(*_srcCloudNormalPtr);
		edgeDetection(_tgtCloudNormalPtr, indicesPtr, normalRadius * 2.0f);
		edgeViewer(_tgtCloudNormalPtr);
		extract.setInputCloud(_tgtCloudNormalPtr);
		extract.setIndices(indicesPtr);
		extract.setNegative(true);
		extract.filter(*_tgtCloudNormalPtr);
	}


	MyPointRepresentationNormal::Ptr point_representation(new MyPointRepresentationNormal);
	float alpha[4] = { 1.0, 1.0, 1.0, 1.0 };
	point_representation->setRescaleValues(alpha);

	MyCorrespondenceEstimation<cloud::PointNormalT, cloud::PointNormalT>::Ptr 
		corrEstimation(new MyCorrespondenceEstimation<cloud::PointNormalT, cloud::PointNormalT>);
	corrEstimation->setPointRepresentation(point_representation);
	vector<float> weight = { 1.0, 1.0, 1.0, 1.0 };
	corrEstimation->setWeight(weight);

	pcl::IterativeClosestPointNonLinear<cloud::PointNormalT, cloud::PointNormalT> icp_nl;  //ʵ�� ICP ����
	icp_nl.setEuclideanFitnessEpsilon(1e-6);
	icp_nl.setTransformationEpsilon(1e-7);  //���������ٽ��任�����ƽ����
	//icp_nl.setUseReciprocalCorrespondences(true);
	icp_nl.setMaxCorrespondenceDistance(maxDis);  //����Դ����Ŀ�������ƥ�����(��)
	icp_nl.setPointRepresentation(point_representation);
	if(useMyEm)icp_nl.setCorrespondenceEstimation(corrEstimation);
	icp_nl.setInputSource(_srcCloudNormalPtr);
	icp_nl.setInputTarget(_tgtCloudNormalPtr);
	//icp_nl.

	//�����׼
	Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity(), pre_transformation;
	cloud::PointCloudNormalPtr transCloudNormalPtr = _srcCloudNormalPtr;
	icp_nl.setMaximumIterations(64);  //���õ�������
	cloud::Color redPoint = { 255.0, 0.0, 0.0 };
	cloud::Color greenPoint = { 0.0, 255.0, 0.0 };
	cloud::Color bluePoint = { 0.0, 0.0, 255.0 };
	cloud::Color yellowPoint = { 255.0, 255.0, 0.0 };
	cloud::Color purplePoint = { 255.0, 0.0, 255.0 };
	cloud::Color cyanPoint = { 255.0, 0.0, 255.0 };
	cloud::Color whitePoint = { 255.0, 255.0, 255.0 };
	vector<cloud::Color> colorVec = { redPoint, greenPoint, bluePoint, yellowPoint, purplePoint, cyanPoint };
	vector<cloud::Color> colorWhiteVec = { whitePoint };
	pcl::CorrespondencesPtr coors;
	startTime = clock();
	float sum = 0;
	for (int i = 0; i < 256; ++i) {
		if (showInfo)cout << i << "th align" << endl;
		icp_nl.setInputSource(_srcCloudNormalPtr);
		icp_nl.align(*transCloudNormalPtr);
		if (i == 0 && showInfo) {
			coors = icp_nl.correspondences_;
			for (auto _coor : *coors) {
				sum += pow(_srcCloudNormalPtr->at(_coor.index_query).x - _tgtCloudNormalPtr->at(_coor.index_match).x, 2.0) +
					pow(_srcCloudNormalPtr->at(_coor.index_query).y - _tgtCloudNormalPtr->at(_coor.index_match).y, 2.0) +
					pow(_srcCloudNormalPtr->at(_coor.index_query).z - _tgtCloudNormalPtr->at(_coor.index_match).z, 2.0);
			}
			sum /= coors->size();
			if (showInfo)cout << "ԴMSE: " << sum << endl;
		}
		transformation = icp_nl.getFinalTransformation() * transformation;
		//if (pre_transformation == icp_nl.getLastIncrementalTransformation())break;
		if (abs((icp_nl.getLastIncrementalTransformation() - pre_transformation).sum()) < icp_nl.getTransformationEpsilon()) {
			icp_nl.setMaxCorrespondenceDistance(icp_nl.getMaxCorrespondenceDistance() - pow(sum, 0.5f)*0.05);
			//if (showInfo)cout << "set distance to" << icp_nl.getMaxCorrespondenceDistance() << endl;
			if (icp_nl.getMaxCorrespondenceDistance() < 0)break;
		}
		if (showInfo) cout << "distance:" << icp_nl.getMaxCorrespondenceDistance() << endl;
		pre_transformation = icp_nl.getLastIncrementalTransformation();
		//pre_transformation = transformation;
		_srcCloudNormalPtr = transCloudNormalPtr;

		//MSE
		if (showInfo|| useMyEm) {
			coors = icp_nl.correspondences_;
			sum = 0;
			for (auto _coor : *coors) {
				sum += pow(_srcCloudNormalPtr->at(_coor.index_query).x - _tgtCloudNormalPtr->at(_coor.index_match).x, 2.0) +
					pow(_srcCloudNormalPtr->at(_coor.index_query).y - _tgtCloudNormalPtr->at(_coor.index_match).y, 2.0) +
					pow(_srcCloudNormalPtr->at(_coor.index_query).z - _tgtCloudNormalPtr->at(_coor.index_match).z, 2.0);
			}
			sum /= coors->size();
			if (showInfo)cout << "MSE: " << sum << endl;
		}
		if (useMyEm) {
			float _weight2 = pow(2.718, -100000.0 * sum * 0.05);
			//cout << _weight2 << endl;
			if (_weight2 > 0.8)_weight2 = 0.8;
			else if (_weight2 < 0.2)_weight2 = 0;
			_weight2 = 0.25 - 0.25 * _weight2;
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
			cout << endl;
		}
	}
	endTime = clock();
	useTime = (double)(endTime - startTime) / CLOCKS_PER_SEC;
	if (showInfo)cout << "��׼��ʱ��" << useTime << endl;
	final_transformation = transformation;
	pcl::transformPointCloud(*srcCloudPtr, *resCloudPtr, final_transformation);
	return 0;
}

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
	bool downSample,
	float downSampleSize,
	float maxDis)
{
	bool showInfo = true;
	cloud::PointCloudPtr _srcCloudPtr(new cloud::PointCloud);  //Դ�����²���
	cloud::PointCloudPtr _tgtCloudPtr(new cloud::PointCloud);  //Ŀ������²���
	//�²���
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

	pcl::IterativeClosestPoint<cloud::PointT, cloud::PointT> icp_nl;  //ʵ�� ICP ����
	//icp_nl.setEuclideanFitnessEpsilon(1e-6);
	icp_nl.setTransformationEpsilon(1e-7);  //���������ٽ��任�����ƽ����
	icp_nl.setMaxCorrespondenceDistance(maxDis);  //����Դ����Ŀ�������ƥ�����(��)
	//icp_nl.setPointRepresentation(pcl::make_shared<const MyPointRepresentation2>(point_representation));
	icp_nl.setInputSource(_srcCloudPtr);
	icp_nl.setInputTarget(_tgtCloudPtr);

	//�����׼
	Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity(), pre_transformation;
	cloud::PointCloudPtr transCloudPtr = _srcCloudPtr;
	icp_nl.setMaximumIterations(2);  //���õ�������
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
	const cloud::PointNormalPfhPtr& srcCloudPtr,
	const cloud::PointNormalPfhPtr& tgtCloudPtr,
	cloud::PointCloudPtr& resCloudPtr,
	Eigen::Matrix4f& final_transformation,
	bool downSample,
	float downSampleSize,
	float maxDis,
	bool useMyEm)
{
	//�Զ��� Point Representation
	class MyPointRepresentationNormal :public pcl::PointRepresentation<cloud::PointNormalPfhT> {
		using pcl::PointRepresentation<cloud::PointNormalPfhT>::nr_dimensions_;
	public:
		using Ptr = shared_ptr<MyPointRepresentationNormal>;
		MyPointRepresentationNormal() {
			nr_dimensions_ = 3;
		}
		MyPointRepresentationNormal(int n) {
			nr_dimensions_ = n;
		}
		virtual void copyToFloatArray(const cloud::PointNormalPfhT& p, float* out) const
		{
			out[0] = p.x;
			out[1] = p.y;
			out[2] = p.z;
			//out[3] = p.data_additional[0];
			for (int _i = 0; _i < nr_dimensions_-3; ++_i) {
				out[_i+3] = p.data_additional[_i];
			}
		}
	};

	bool showInfo = false;
	cloud::PointNormalPfhPtr _srcCloudPtr(new cloud::PointNormalPfh);  //Դ�����²���
	cloud::PointNormalPfhPtr _tgtCloudPtr(new cloud::PointNormalPfh);  //Ŀ������²���
	//�²���
	_srcCloudPtr = srcCloudPtr;
	_tgtCloudPtr = tgtCloudPtr;

	#define nr_dimensions 4
	MyPointRepresentationNormal::Ptr point_representation(new MyPointRepresentationNormal(nr_dimensions));
	float alpha[nr_dimensions] = { 1.0, 1.0, 1.0, 1.0};
	//float alpha[3] = { 1.0, 1.0, 1.0 };
	point_representation->setRescaleValues(alpha);

	MyCorrespondenceEstimation<cloud::PointNormalPfhT, cloud::PointNormalPfhT>::Ptr
		corrEstimation(new MyCorrespondenceEstimation<cloud::PointNormalPfhT, cloud::PointNormalPfhT>);
	corrEstimation->setPointRepresentation(point_representation);
	vector<float> weight = { 1.0, 1.0, 1.0, 1.0};
	corrEstimation->setWeight(weight);

	pcl::IterativeClosestPointNonLinear<cloud::PointNormalPfhT, cloud::PointNormalPfhT> icp_nl;  //ʵ�� ICP ����
	//icp_nl.setEuclideanFitnessEpsilon(1e-6);
	icp_nl.setTransformationEpsilon(1e-5);  //���������ٽ��任�����ƽ����
	icp_nl.setMaxCorrespondenceDistance(1);  //����Դ����Ŀ�������ƥ�����(��)
	icp_nl.setPointRepresentation(point_representation);
	icp_nl.setMaximumIterations(2);  //���õ�������
	//icp_nl.setCorrespondenceEstimation(corrEstimation);
	icp_nl.setInputSource(_srcCloudPtr);
	icp_nl.setInputTarget(_tgtCloudPtr);

	////�����׼
	Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity(), pre_transformation;
	cloud::PointNormalPfhPtr transCloudNormalPtr = _srcCloudPtr;
	float sum = 0.0f;
	pcl::CorrespondencesPtr coors;
	clock_t startTime = clock();
	for (int i = 0; i < 64; ++i) {
		if (showInfo)cout << i << "th align" << endl;
		icp_nl.setInputSource(_srcCloudPtr);
		icp_nl.align(*transCloudNormalPtr);
		if (i == 0 && showInfo) {
			coors = icp_nl.correspondences_;
			for (auto _coor : *coors) {
				sum += pow(_srcCloudPtr->at(_coor.index_query).x - _tgtCloudPtr->at(_coor.index_match).x, 2.0) +
					pow(_srcCloudPtr->at(_coor.index_query).y - _tgtCloudPtr->at(_coor.index_match).y, 2.0) +
					pow(_srcCloudPtr->at(_coor.index_query).z - _tgtCloudPtr->at(_coor.index_match).z, 2.0);
			}
			sum /= coors->size();
			if (showInfo)cout << "ԴMSE: " << sum << endl;
		}
		transformation = icp_nl.getFinalTransformation() * transformation;
		//if (pre_transformation == icp_nl.getLastIncrementalTransformation())break;
		if (abs((icp_nl.getLastIncrementalTransformation() - pre_transformation).sum()) < icp_nl.getTransformationEpsilon() &&
			icp_nl.getMaxCorrespondenceDistance() > 0.005) {
			icp_nl.setMaxCorrespondenceDistance(icp_nl.getMaxCorrespondenceDistance() - 0.001);
			if (showInfo)cout << "set distance to" << icp_nl.getMaxCorrespondenceDistance() << endl;
			if (icp_nl.getMaxCorrespondenceDistance() < 0)break;
		}
		if (icp_nl.getMaxCorrespondenceDistance() <= 0.005)break;
		pre_transformation = icp_nl.getLastIncrementalTransformation();
		//pre_transformation = transformation;
		_srcCloudPtr = transCloudNormalPtr;

		//MSE
		if (showInfo || useMyEm) {
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
		if (useMyEm) {
			float _weight2 = pow(2.718, -100000.0 * sum * 0.223);
			cout << _weight2 << endl;
			if (_weight2 > 0.8)_weight2 = 0.8;
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
			cout << endl;
		}
	}
	final_transformation = transformation;
	cloud::PointCloudPtr _srcPointCloudPtr(new cloud::PointCloud);
	pcl::copyPointCloud<cloud::PointNormalPfhT, cloud::PointT>(*_srcCloudPtr, *_srcPointCloudPtr);
	pcl::transformPointCloud(*_srcPointCloudPtr, *resCloudPtr, final_transformation);
	return 0;
}

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
	bool withNormal,
	bool saveResultToPcd,
	bool downSample)
{
	cloud::Color redPoint = { 255.0, 0.0, 0.0 };
	cloud::Color greenPoint = { 0.0, 255.0, 0.0 };
	vector<cloud::Color>colors = { redPoint, greenPoint };
	bool showInfo = false;
	cloud::PointCloudPtr tgtPointCloudPtr, srcPointCloudPtr, tmpPointCloudPtr(new cloud::PointCloud);
	pcl::copyPointCloud(*pointCloudVec[0], *tmpPointCloudPtr);
	registeredVec.push_back(tmpPointCloudPtr);
	Eigen::Matrix4f preTransformation = Eigen::Matrix4f::Identity();
	transformationVec.push_back(preTransformation);
	for (size_t i = 0; i < pointCloudVec.size() - 1; ++i) {
		Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
		cloud::PointCloudPtr resPointCloudPtr(new cloud::PointCloud);
		tgtPointCloudPtr = pointCloudVec[i];
		srcPointCloudPtr = pointCloudVec[i + 1];
		if(withNormal)pairAlignWithNormal(srcPointCloudPtr, tgtPointCloudPtr, tmpPointCloudPtr, transformation, downSample);
		else pairAlign(srcPointCloudPtr, tgtPointCloudPtr, tmpPointCloudPtr, transformation, downSample);
		if (showInfo) {
			pcl::transformPointCloud(*srcPointCloudPtr, *tmpPointCloudPtr, transformation);
			cout << "pair with " << i << "th and " << i + 1 << "th" << endl;
			visualizePointCloud({ tgtPointCloudPtr, tmpPointCloudPtr }, colors);
		}
		pcl::transformPointCloud(*tmpPointCloudPtr, *resPointCloudPtr, preTransformation);
		preTransformation *= transformation;
		transformationVec.push_back(transformation);
		registeredVec.push_back(resPointCloudPtr);

		if (saveResultToPcd) {
			std::stringstream filename;
			filename << i << ".pcd";
			pcl::io::savePCDFile(filename.str(), *resPointCloudPtr);
		}
		if (showInfo) {
			cout << "out" << endl;
			cout << transformation << endl;
		}
	}
	return 0;
}

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
 * @brief		���ӻ��������
 * @param[in]	pointCloudPtrVec	��������
 * @param[in]	pointColorVec		������ɫ
 * @param[in]	backgroundColor		������ɫ
 * @param[in]	is_auto				��ʾ����
 * @return 
*/
pcl::visualization::PCLVisualizer::Ptr
visualizePointCloud(
	const cloud::PointCloudPtrVec& pointCloudPtrVec,
	vector<cloud::Color>& pointColorVec,
	vector<int> pointSizeVec,
	const cloud::Color& backgroundColor,
	bool showCoordinateSystem,
	bool is_auto)
{
	vector<pcl::PolygonMesh> meshVec;
	return visualizePointCloud(pointCloudPtrVec, meshVec, pointColorVec, pointSizeVec, backgroundColor, showCoordinateSystem, is_auto);
}

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
	const cloud::Color& pointColor,
	int pointSize,
	const cloud::Color& backgroundColor,
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
	vector<int> pointSizeVec,
	const cloud::Color& backgroundColor,
	bool showCoordinateSystem,
	bool is_auto)
{
	using colorHandlerCustom = pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>;
	int pointRenderSize = 2;

	//��ά��ѡȡ�Ļص�����
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

	//�����¼��ص�����
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

	//��ѡ�¼��ص�����
	class AreaPickingCallback {
	public:
		pcl::Indices indices;
	public:
		void callback(const pcl::visualization::AreaPickingEvent& areaPickingEvent, void* args) {
			areaPickingEvent.getPointsIndices(this->indices);
		}
		void getPointsIndices(pcl::Indices& _indices) { _indices = this->indices; }
	};

	//��ɫ���ڵ�������ʱʹ��Ĭ����ɫ
	cloud::Color defaultColor = { 255.0, 255.0, 255.0 };
	for (int i = pointCloudPtrVec.size() - pointColorVec.size(); i > 0; --i) {
		pointColorVec.push_back(defaultColor);
	}
	for (int i = pointCloudPtrVec.size() - pointSizeVec.size(); i > 0; --i) {
		pointSizeVec.push_back(2);
	}

	cout << "Shift + ������ѡȡ��" << endl;
	cout << "������ʾ���ƣ�p" << endl;
	cout << "�л���һ�����ƣ�a" << endl;

	int text_xpos = 0;	//�ı�xλ��
	int text_ypos = 0;	//�ı�yλ��
	int fontsize = 20;	//�����С
	string textID = "pointPosition";	//�ı�ID

	//int* winSize = new int[2];
	//int* winPos = new int[2];
	pcl::visualization::PCLVisualizer::Ptr pViewer(new pcl::visualization::PCLVisualizer("viewer"));
	pcl::visualization::PCLVisualizer viewer = *pViewer;

	viewer.setBackgroundColor(backgroundColor.r, backgroundColor.g, backgroundColor.b);	//���ñ���
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
	KeyboardCallback keyboardCallback(&viewer, &pointCloudPtrVec, &meshVec, &cloudIDVec, &meshIDVec, &pointColorVec); //���޸�

	string pointPositionStr("point position\nx:\ny:\nz:");
	viewer.addText(pointPositionStr, text_xpos, text_ypos, fontsize, 0.0, 0.0, 1.0, textID); //����ı�
	viewer.registerPointPickingCallback<PointPickingCallback>(&PointPickingCallback::callback, pointPickingCallback);	//ע����ά��ѡȡ�Ļص�����
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
	normalEstimation(srcPointCloud, normalWithCurv, 0,0.003);
	cloud::PointCloudNormalPtr pointWithNormal(new cloud::PointCloudNormal);
	pcl::concatenateFields(*srcPointCloud, *normalWithCurv, *pointWithNormal);

	pcl::GreedyProjectionTriangulation<cloud::PointNormalT> gp3;
	pcl::search::KdTree<cloud::PointNormalT>::Ptr searchTree(new pcl::search::KdTree<cloud::PointNormalT>);
	if (usePointRepresentation) {
		MyPointRepresentation::Ptr myPointRepresentationPtr(new MyPointRepresentation());
		searchTree->setPointRepresentation(myPointRepresentationPtr);
	}
	searchTree->setInputCloud(pointWithNormal);

	gp3.setSearchRadius(searchRadius); //���������ε����߳�
	gp3.setMu(mu); //����������������
	gp3.setMaximumNearestNeighbors(maximumNearestNeighbors); //���ò�ѯ���ٽ����������
	gp3.setMinimumAngle(minimumAngle); //�������������С�ǵĽǶ�
	gp3.setMaximumAngle(maximumAngle); //��������������ǵĽǶ�
	gp3.setMaximumSurfaceAngle(maximumSurfaceAngle);
	gp3.setNormalConsistency(normalConsistency);

	gp3.setInputCloud(pointWithNormal);
	gp3.setSearchMethod(searchTree);
	gp3.reconstruct(trianglesMesh);


	return 0;
}
/**
 * @brief		���ò��ɷ������ؽ�
 * @param[in]	PointCloudPtr		�������
 * @param[out]	mesh				�������
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
 * @brief		����̰��ͶӰ�㷨�����ؽ�
 * @param[in]	PointCloudNormalPtr		�������
 * @param[out]	mesh					�������
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
	pn.setConfidence(false); //�Ƿ�ʹ�÷������Ĵ�С��Ϊ������Ϣ�����false�����з���������һ����
	pn.setDegree(2); //���ò���degree[1,5],ֵԽ��Խ��ϸ����ʱԽ�á�
	pn.setDepth(8);
	//���������ȣ����2^d x 2^d x 2^d������Ԫ��
	// ���ڰ˲�������Ӧ�����ܶȣ�ָ��ֵ��Ϊ�����ȡ�
	pn.setIsoDivide(16); //������ȡISO��ֵ����㷨�����
	pn.setManifold(false); //�Ƿ���Ӷ���ε����ģ�����������ǻ�ʱ�� 
	// �������б�־���������Ϊtrue����Զ���ν���ϸ�����ǻ�ʱ������ģ�����false�����
	pn.setOutputPolygons(false); //�Ƿ������������񣨶��������ǻ��ƶ�������Ľ����
	pn.setSamplesPerNode(5); //��������һ���˲�������е����������С��������������[1.0-5.0],������[15.-20.]ƽ��
	//pn.setScale(1.25); //���������ع���������ֱ���������߽�������ֱ���ı��ʡ�
	pn.setSolverDivide(8); //����������Է������Gauss-Seidel�������������
	//pn.setIndices();
	//���������������������
	pn.setSearchMethod(tree2);
	pn.setInputCloud(PointCloudNormalPtr);
	//����������������ڴ洢���
	//ִ���ع�
	pn.reconstruct(mesh);
	return 0;
}

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
 * @brief		�������ͼ��
 * @param[in]	pointCloudPtr		�������
 * @param[out]	rangeImagePtr		������ͼ��
 * @param[in]	angularResolution	�Ƕȷֱ���
 * @param[in]	maxAngleWidth		ˮƽ�ӽ�
 * @param[in]	maxAngleHeight		��ֱ�ӽ�
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
 * @brief		��ȡNARF�ؼ���
 * @param[in]	pointCloudPtr		�������
 * @param[in]	rangeImage			���ͼ��
 * @param[out]	keyPointCloudPtr	����ؼ���
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

	// ʹ�÷�������Ϊǿ�ȼ���ؼ��㣬��������rgb��zֵ�����Զ��壬����ο�API
	pcl::SIFTKeypoint<cloud::PointNormalT, cloud::PointT> sift; //PointT ������ pcl::PointWithScale�����߶���Ϣ
	pcl::search::KdTree<cloud::PointNormalT>::Ptr tree(new pcl::search::KdTree<cloud::PointNormalT>());
	sift.setSearchMethod(tree);
	sift.setScales(min_scale, n_octaves, n_scales_per_octave);
	sift.setMinimumContrast(min_contrast);
	sift.setInputCloud(pointCloudNormalPtr);
	sift.compute(*result);
	return 0;
}

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
			//	cout << "�����Զ�����ð뾶����" << endl;
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
 * @brief PCA�㷨��ά���Ծ������ά�Ƚ��н�ά
 * @param origin		��Ҫ��ά�ľ���
 * @param kStart		��ά��kά
 * @param error			��ʧ����ֵ��������kά�����㶪ʧ��Ҫ��ʱ����ά��
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

		//�޸�
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
	if (show) cout << "�����˲�" << endl;
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

	if (show) cout << "ƽ��ָ�" << endl;
	planeSegmentation(0.005, out, _temp, true);
	pcl::copyPointCloud(*_temp, *out);
	if (show) visualizePointCloud(out);

	if (show) cout << "��Ⱥ������" << endl;
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
	cloud::PointCloudPtr& cloud1,
	cloud::PointCloudPtr& cloud2,
	cloud::PointCloudPtr& resCloud,
	float distance,
	float edgeDistance,
	float normaldistance)
{
	cloud::PointCloudPtr _cloud1(new cloud::PointCloud), _cloud2(new cloud::PointCloud);
	pcl::copyPointCloud(*cloud1, *_cloud1);
	pcl::copyPointCloud(*cloud2, *_cloud2);
	fuseTwoPointClouds(_cloud1, _cloud2, distance, edgeDistance, normaldistance);
	*resCloud = *_cloud1 + *_cloud2;
	return 0;
}

int fuseTwoPointClouds(
	cloud::PointCloudPtr& inCloud1,
	cloud::PointCloudPtr& inCloud2,
	cloud::PointCloudPtr& outCloud1,
	cloud::PointCloudPtr& outCloud2,
	float distance,
	float edgeDistance,
	float normaldistance)
{
	pcl::copyPointCloud(*inCloud1, *outCloud1);
	pcl::copyPointCloud(*inCloud2, *outCloud2);
	fuseTwoPointClouds(outCloud1, outCloud2, distance, edgeDistance, normaldistance);
	return 0;
}

int fuseTwoPointClouds(
	cloud::PointCloudPtr& cloud1,
	cloud::PointCloudPtr& cloud2,
	float distance,
	float edgeDistance,
	float normaldistance)
{
	cloud::PointCloudNormalPtr pointCloudNormalPtr1(new cloud::PointCloudNormal), pointCloudNormalPtr2(new cloud::PointCloudNormal);
	pcl::PointIndices::Ptr edgeIndicesPtr(new pcl::PointIndices);
	//cloud::NormalPtr normalPtr1(new cloud::Normal);
	//cout << "���߹���" << endl;
	normalEstimation(cloud1, pointCloudNormalPtr1, 0, normaldistance);
	normalEstimation(cloud2, pointCloudNormalPtr2, 0, normaldistance);
	//cout << "��Ե���" << endl;
	edgeDetection(pointCloudNormalPtr1, edgeIndicesPtr, edgeDistance);
	edgeDetection(pointCloudNormalPtr2, edgeIndicesPtr, edgeDistance);
	//edgeViewer(pointCloudNormalPtr1);
	//edgeViewer(pointCloudNormalPtr2);

	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud1);
	kdtree.setSortedResults(true);
	vector<int> indexSearch, _indices(cloud1->size(), -1);
	vector<float> squareDisSearch;
	pcl::PointIndices::Ptr cloud1_indices(new pcl::PointIndices()), cloud2_indices(new pcl::PointIndices());
	cloud1_indices->indices.resize(cloud1->size());
	cloud2_indices->indices.resize(cloud2->size());
	size_t indices1Size = 0, indices2Size = 0;
	float edgeCoefficient1, edgeCoefficient2, coe, disScale = 1.5;
	//float maxDistance = disScale * distance;
	float realDistance = 0.0f;
	float temp = 0.0f;
	Eigen::Vector3f edgeVec1, edgeVec2,tempVec3f;
	//cout << "ƴ��" << endl;
	for (size_t _index = 0; _index < cloud2->size(); ++_index) { //����cloud2
		kdtree.radiusSearch((*cloud2)[_index], distance, indexSearch, squareDisSearch);
		if (indexSearch.size()) { //cloud1��cloud2���ص�
			realDistance = pow(squareDisSearch[0], 0.5f);
			edgeCoefficient1 = (*pointCloudNormalPtr1)[indexSearch[0]].data_n[3];
			edgeCoefficient2 = (*pointCloudNormalPtr2)[_index].data_n[3];
			if (edgeCoefficient1){	//cloud2��cloud1��Ե���ص�
				//��cloud1�ı�Եϵ��ΪȨ�أ����������Ȩֵ������cloud2���λ��
				coe = edgeCoefficient1 * (edgeCoefficient1 / (edgeCoefficient2 + edgeCoefficient1));
				coe = 1.0f - (1.0f - coe) * (1- (realDistance / distance));
				for (size_t _xyzIndex = 0; _xyzIndex < 3; ++_xyzIndex) {
					(*cloud2)[_index].data[_xyzIndex] = ((*cloud2)[_index].data[_xyzIndex]) * coe
						+ (*pointCloudNormalPtr1)[indexSearch[0]].data[_xyzIndex] * (1-coe);
				}
				for (auto __index : indexSearch) {
					tempVec3f = (*pointCloudNormalPtr1)[__index].getVector3fMap() 
						- (*pointCloudNormalPtr2)[_index].getVector3fMap();
					tempVec3f += (*pointCloudNormalPtr1)[__index].getNormalVector3fMap();
					tempVec3f.normalize();
					temp = (*pointCloudNormalPtr2)[_index].getNormalVector3fMap().dot(tempVec3f);
					if(temp >0.9)_indices[__index] = __index; //cloud1��Ե�ص��������ŷ����˳���indices��
				}
			}
			else {//cloud2��cloud1�ڲ����ص�
				//cloud2�����ŷ����˳���indices��
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

	pcl::ExtractIndices<pcl::PointXYZ> extract;
	//ȥ��cloud1��Ե��
	extract.setInputCloud(cloud1);
	extract.setIndices(cloud1_indices);
	extract.setNegative(true);
	extract.filter(*cloud1);
	//ȥ��cloud2����cloud1�ڲ��ص��ĵ�
	extract.setInputCloud(cloud2);
	extract.setIndices(cloud2_indices);
	extract.setNegative(true);
	extract.filter(*cloud2);
	//*resCloud += *cloud1;
	return 0;
}

int fusePointClouds(
	cloud::PointCloudPtrVec& inPointCloudPtrVec,
	cloud::PointCloudPtr& outPointCloudPtr,
	float distance,
	float edgeDistance,
	float normaldistance)
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
	pcl::copyPointCloud(*(inPointCloudPtrVec[0]), *outPointCloudPtr);
	for (size_t _index = 1; _index < inPointCloudPtrVec.size(); ++_index) {
		if (showInfo) {
			cout << endl;
			cout << "\rfusing " << _index - 1 << "th and " << _index << "th " 
				<< _index << "/" << inPointCloudPtrVec.size()-1 << endl;
		}
		fuseTwoPointClouds(outPointCloudPtr, inPointCloudPtrVec[_index], outPointCloudPtr, distance, edgeDistance, normaldistance);
		visualizePointCloud(outPointCloudPtr, {255.0,255.0,255.0}, 3);
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
	float distance) 
{
	//PointRepresentation��ֻȡx��y��z����
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
	int ii = 200;
	pcl::KdTreeFLANN<cloud::PointNormalT> kdtree;
	MyPointRepresentation::Ptr pointRepresentation(new MyPointRepresentation());
	kdtree.setPointRepresentation(pointRepresentation);
	kdtree.setInputCloud(cloudPtr);

	vector<int> k_indices;
	vector<float> k_sqr_distances;
	Eigen::Vector3f normal_n, pointVec_n, edgeSign=Eigen::Vector3f::Zero(), crossTemp, _edgeSign;
	pcl::PointNormal test;
	float min = 0.15;
	float max = 0.5;
	size_t edgeIndicesIndex = 0;
	edgeIndicesPtr->indices.resize(cloudPtr->size());
	for (int _index = 0; _index < cloudPtr->size(); ++_index) {
		kdtree.radiusSearch((*cloudPtr)[_index], distance, k_indices, k_sqr_distances); //����ָ����P0�İ뾶distance�ڵĵ�
		normal_n = (*cloudPtr)[_index].getNormalVector3fMap(); //��ķ���
		edgeSign = Eigen::Vector3f::Zero();
		for (size_t __index = 0; __index < k_indices.size(); ++__index) {
			if (k_sqr_distances[__index]) { //���Ǹõ㱾������
				pointVec_n = (*cloudPtr)[k_indices[__index]].getVector3fMap() - (*cloudPtr)[_index].getVector3fMap(); //������Pi - P0
				//pointVec_n.normalize();
				edgeSign += normal_n.cross(pointVec_n).cross(normal_n); //edgeSign += (pointVec_n����) ��P0��ƽ���ϵ�ͶӰ
				_edgeSign = edgeSign.normalized();
				for (size_t _edgeSignIndex = 0; _edgeSignIndex < 3; ++_edgeSignIndex) {
					(*cloudPtr)[_index].data_c[_edgeSignIndex] = _edgeSign[_edgeSignIndex];
				}
				//edgeSign += crossTemp.cross(normal_n);
			}
		}
		if (k_indices.size()) { //P0������������
			(*cloudPtr)[_index].data_n[3] = edgeSign.norm() / (float)k_indices.size() / distance; //�����Եϵ������һ��
			// ��Եϵ��ӳ�� [0,1] ӳ�䵽 [min,1]
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
		else { //P0������������
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

float getResolution(
	cloud::PointCloudPtr pointCloudPtr)
{
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	vector<int> pointIdx(2);
	vector<float> pointSquaredDistance(2);
	float sumDis = 0.0f;

	kdtree.setInputCloud(pointCloudPtr);
	kdtree.setSortedResults(true);
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
	ndt.setResolution(getResolution(filtered_cloud));
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
	if (dis <= 0)ec.setClusterTolerance(getResolution(inCloudPtr) * 2);
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
		tempPtr->is_dense = true;
		outCloudPtrVec.push_back(tempPtr);
	}
}

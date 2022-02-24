#pragma warning
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <opencv2/imgproc/imgproc_c.h>
#include "camera.h"
#include "cloudOperation.h"
#include "function.h"
#include <string>

using namespace std;
bool showRGBImage = true;
bool showDepthImage = true;
ofstream PointCloudWriter;
string rabbitPcdFile = "D:\\剑走偏锋\\毕设\\硕士\\TOF\\数据\\rabbit.pcd";

cloud::Color redPoint = { 255.0, 0.0, 0.0 };
cloud::Color greenPoint = { 0.0, 255.0, 0.0 };
cloud::Color bluePoint = { 0.0, 0.0, 255.0 };
cloud::Color yellowPoint = { 255.0, 255.0, 0.0 };
cloud::Color purplePoint = { 255.0, 0.0, 255.0 };
cloud::Color cyanPoint = { 255.0, 0.0, 255.0 };
cloud::Color whitePoint = { 255.0, 255.0, 255.0 };
vector<cloud::Color> colorVec = {redPoint, greenPoint, bluePoint, yellowPoint, purplePoint, cyanPoint};
vector<cloud::Color> colorWhiteVec = { whitePoint };

int vzense_test();
int pcl_test();
void cloudViewerKeyboardCallback(const pcl::visualization::KeyboardEvent& keyboardEvent, void* args);
void viewerOneOff(pcl::visualization::PCLVisualizer& viewer) {
	viewer.setBackgroundColor(1.0, 0.5, 1.0);
}
int process();
int test();
int registerWithKeypoint();

int main() {
	//int key;
	//cin >> key;
	//switch (key)
	//{
	//case 0:
	//	test();
	//	break;
	//case 1:
	//	process();
	//	break;
	//case 2:
	//	registerWithKeypoint();
	//	break;
	//default:
	//	break;
	//}
	//string filePath("D:\\剑走偏锋\\毕设\\硕士\\工程\\GetData\\GetData\\PCD\\001.pcd");
	//cloud::PointCloudPtr pointCloudPtr(new cloud::PointCloud);
	//if (pcl::io::loadPCDFile(filePath, *pointCloudPtr) == -1) {
	//	cout << "Can not load" << filePath << endl;
	//}

	//pcl::PassThrough<cloud::PointT> pass;
	//pass.setFilterFieldName("z");
	//pass.setFilterLimits(0.1, 0.6);
	//pass.setInputCloud(pointCloudPtr);
	//pass.filter(*pointCloudPtr);

	////pcl::RangeImage::Ptr rangeImagePtr(new pcl::RangeImage());
	////getRangeImage(pointCloudPtr, rangeImagePtr, 0.3);

	////pcl::visualization::RangeImageVisualizer rangeImageViewer("range image");
	////rangeImageViewer.showRangeImage(*rangeImagePtr);
	////while (!rangeImageViewer.wasStopped())
	////{
	////	rangeImageViewer.spinOnce(20);
	////}

	//vector<int> indices({ 15789,16144,18129,27721,29808 });
	//vector<vector<int>> searchRs;
	//float radius = 0.02f;
	//getNeighbors(pointCloudPtr, indices, searchRs, 0, radius);
	////pcl::Indices indices({ 15789, 15790, 15791 });
	////getNARFKeypoints(pointCloudPtr, *rangeImagePtr, indices, 0.2);

	//pcl::ExtractIndices<pcl::PointXYZ> extract;
	//cloud::PointCloudPtr keyPointCloudPtr(new cloud::PointCloud);
	//pcl::PointIndicesPtr pointIndicesPtr(new pcl::PointIndices);
	//pointIndicesPtr->indices = indices;
	//extract.setInputCloud(pointCloudPtr);
	//extract.setIndices(pointIndicesPtr);
	//extract.setNegative(false);
	//extract.filter(*keyPointCloudPtr);

	//cloud::PointCloudPtrVec pointCloudPtrVec = { pointCloudPtr, keyPointCloudPtr };

	//visualizePointCloud(pointCloudPtrVec, colorVec, { 1, 4 });

	//cloud::NormalPtr normalPtr(new cloud::Normal);
	//normalEstimation(pointCloudPtr, normalPtr, 0, 0.01f);

	//cout << "normal finish" << endl;

	//Eigen::VectorXf pfh_histogram(125);
	//for (auto& ner : searchRs) {
	//	computePFH(pointCloudPtr, normalPtr, ner, pfh_histogram, 5);

	//	cout << pfh_histogram.size() << "=" << pfh_histogram.rows() << "*" << pfh_histogram.cols() << endl;
	//	//cout << pfh_histogram << endl;
	//	for (int i = 0; i < pfh_histogram.size(); ++i) {
	//		cout << pfh_histogram(i) << "\t";
	//	}
	//	cout << endl;
	//	cout << "===================" << endl;
	//}
	//vector<double> x_data, y_data;
	//for (int i = 0; i < pfh_histogram.size(); ++i) {
	//	x_data.push_back(i + 1);
	//	y_data.push_back(pfh_histogram(i));
	//}
	//pcl::visualization::PCLPlotter plotter("histogram");
	//plotter.addPlotData(x_data, y_data, "test", vtkChart::BAR);
	//for (auto& i : y_data) {
	//	i = i + 1;
	//}
	//plotter.addPlotData(x_data, y_data, "test2", vtkChart::BAR);
	//plotter.plot();
	// 
	//Eigen::MatrixXd X(6, 2);
	//X << -1, 1, -2, -1, -3, -2, 1, 1, 2, 1, 3, 2;
	//cout << "X:\n" << X << endl;
	//PCA(X,1,0.1);

	//string filePath = "D:\\剑走偏锋\\毕设\\硕士\\TOF\\数据\\PCL官方数据\\data\\tutorials\\pairwise";
	//string filePath = "D:\\剑走偏锋\\毕设\\硕士\\TOF\\数据\\PCL官方数据\\data\\tutorials\\template_alignment";
	vector<string> files;
	//string path = "D:\\剑走偏锋\\毕设\\硕士\\TOF\\数据\\bunny\\data";
	//string path = "D:\\剑走偏锋\\毕设\\硕士\\TOF\\数据\\PCL官方数据\\data\\tutorials\\pairwise";
	string path = "D:\\剑走偏锋\\毕设\\硕士\\工程\\GetData\\GetData\\PCD";
	cloud::PointCloudPtrVec pointCloudPtrVec;
	getFiles(path, files, "pcd", true);
	vector<int> pointCloudIndexVec({ 0, 1 });
	
	//for (auto& _index : pointCloudIndexVec) {
	//	//if (_file.find("data\\bun") != -1) {
	//	//	cloud::PointCloudPtr _temp(new cloud::PointCloud);
	//	//	pcl::io::loadPLYFile(_file, *_temp);
	//	//	pointCloudPtrVec.push_back(_temp);
	//	//}
	//	cloud::PointCloudPtr _temp(new cloud::PointCloud);
	//	pcl::io::loadPCDFile(files[_index], *_temp);
	//	_temp->width = 640;
	//	_temp->height = 480;
	//	pointCloudPtrVec.push_back(_temp);
	//}

	//pcl::PassThrough<pcl::PointXYZ> pass;
	//for (auto& _pointCloudPtr : pointCloudPtrVec) {
	//	pass.setFilterFieldName("z");
	//	pass.setFilterLimits(0.05, 1);
	//	pass.setInputCloud(_pointCloudPtr);
	//	pass.filter(*_pointCloudPtr);
	//}

	//visualizePointCloud(pointCloudPtrVec, colorVec);

	////int index = 2;
	//vector<pcl::RangeImage::Ptr> rangeImagePtrVec;
	//for (auto _pointCloudPtr : pointCloudPtrVec) {
	//	pcl::RangeImage::Ptr rangeImagePtr(new pcl::RangeImage());
	//	getRangeImage(_pointCloudPtr, rangeImagePtr, 0.3f);
	//	rangeImagePtrVec.push_back(rangeImagePtr);
	//}

	//for (auto& _rangeImagePtr : rangeImagePtrVec) {
	//	pcl::visualization::RangeImageVisualizer rangeImageViewer("range image");
	//	rangeImageViewer.showRangeImage(*_rangeImagePtr);
	//	while (!rangeImageViewer.wasStopped())
	//	{
	//		rangeImageViewer.spinOnce(20);
	//	}
	//}

	//vector<vector<int>> keyPointsIndicesVec;
	//cloud::PointCloudPtrVec keyPointPtrVec;
	//pcl::ExtractIndices<pcl::PointXYZ> extract;
	//pcl::PointIndicesPtr pointIndicesPtr(new pcl::PointIndices);
	//for (int i = 0; i < pointCloudPtrVec.size(); ++i) {
	//	vector<int> _indices;
	//	cloud::PointCloudPtr _keyPointPtr(new cloud::PointCloud);
	//	getNARFKeypoints(pointCloudPtrVec[i], *(rangeImagePtrVec[i]), _indices, 0.1f);
	//	pointIndicesPtr->indices = _indices;
	//	extract.setInputCloud(pointCloudPtrVec[i]);
	//	extract.setIndices(pointIndicesPtr);
	//	extract.setNegative(false);
	//	extract.filter(*_keyPointPtr);
	//	keyPointsIndicesVec.push_back(_indices);
	//	keyPointPtrVec.push_back(_keyPointPtr);
	//}

	////cloud::PointCloudNormalPtr cloudNormalPtr(new cloud::PointCloudNormal);
	////cloud::PointCloudPtr keyPointCloudPtr(new cloud::PointCloud);
	////normalEstimation(pointCloudPtrVec[index], cloudNormalPtr);
	////getSIFTKeypoint(cloudNormalPtr, keyPointCloudPtr);

	//for (int i = 0; i < pointCloudPtrVec.size(); ++i) {
	//	cloud::PointCloudPtrVec _viewerPointCloudPtrVec = { pointCloudPtrVec[i], keyPointPtrVec[i] };
	//	visualizePointCloud(_viewerPointCloudPtrVec, colorVec, { 1, 4 });
	//}

	//vector<cloud::NormalPtr> NormalPtrVec;
	//for (int i = 0; i < pointCloudPtrVec.size(); ++i) {
	//	cloud::NormalPtr _NormalPtr;
	//	normalEstimation(pointCloudPtrVec[i], _NormalPtr);
	//	NormalPtrVec.push_back(_NormalPtr);
	//}

	//int n_split = 3;
	//int pfhNum = pow(n_split, 3);
	//vector<vector<Eigen::VectorXf>> pfh_histogramVec;
	//for (int i = 0; i < pointCloudPtrVec.size(); ++i) {
	//	vector<Eigen::VectorXf> _pfh_histograms;
	//	computePFH(pointCloudPtrVec[i], NormalPtrVec[i], _pfh_histograms, keyPointsIndicesVec[i], 20, 0.01, n_split);
	//	pfh_histogramVec.push_back(_pfh_histograms);
	//}

	//cloud::PointNormalPfhPtrVec pointNormalPfhPtrVec;
	//vector<cloud::PointNormalPfhPtrVec> pairPoints;
	//pairPoints.resize(pfh_histogramVec.size() - 1);
	//for (int i = 0; i < pfh_histogramVec.size() - 1; ++i) {
	//	Eigen::MatrixXf pfhPCA(pfh_histogramVec[i].size() + pfh_histogramVec[i + 1].size(), pfhNum);
	//	int j = 0;
	//	for (auto& pfh : pfh_histogramVec[i]) {
	//		pfhPCA.row(j) = pfh;
	//		++j;
	//	}
	//	for (auto& pfh : pfh_histogramVec[i + 1]) {
	//		pfhPCA.row(j) = pfh;
	//		++j;
	//	}
	//	Eigen::MatrixXf _pfhReduce = PCA(pfhPCA, 4, 1.0);
	//	cloud::PointNormalPfhPtr _temp(new cloud::PointNormalPfh);
	//	for (int _index = 0; _index < keyPointPtrVec[i]->size(); ++_index) {
	//		_temp->points[_index].x = keyPointPtrVec[i]->points[_index].x;
	//		_temp->points[_index].y = keyPointPtrVec[i]->points[_index].y;
	//		_temp->points[_index].z = keyPointPtrVec[i]->points[_index].z;
	//		_temp->points[_index].pfh = _pfhReduce.row(_index);
	//	}
	//	cloud::PointNormalPfhPtr _temp2(new cloud::PointNormalPfh);
	//	for (int _index = 0; _index < keyPointPtrVec[i+1]->size(); ++_index) {
	//		_temp2->points[_index].x = keyPointPtrVec[i]->points[_index].x;
	//		_temp2->points[_index].y = keyPointPtrVec[i]->points[_index].y;
	//		_temp2->points[_index].z = keyPointPtrVec[i]->points[_index].z;
	//		_temp2->points[_index].pfh = _pfhReduce.row(_index+ keyPointPtrVec[i]->size());
	//	}
	//	pairPoints[i] = { _temp, _temp2 };
	//}

	//cloud::PointCloudPtr resCloudPtr(new cloud::PointCloud);
	//Eigen::Matrix4f trans;
	//pairAlignWithCustom(pairPoints[0][0], pairPoints[0][1], resCloudPtr, trans);
	//cloud::PointCloudPtr _transPtr(new cloud::PointCloud);
	//pcl::transformPointCloud(*(pointCloudPtrVec[pointCloudIndexVec[0]]), *_transPtr, trans);

	//cloud::PointCloudPtrVec viewerPointPtrVec = { (_transPtr, pointCloudPtrVec[pointCloudIndexVec[1]]) };
	//visualizePointCloud(viewerPointPtrVec, colorVec);




	//registerWithKeypoint();

	return 0;
}

int process() {
	//获取目录下的文件
	string key;
	vector<string> files;
	string dataPath("D:\\剑走偏锋\\毕设\\硕士\\工程\\GetData\\GetData\\PCD");
	//string dataPath("D:\\剑走偏锋\\毕设\\硕士\\TOF\\数据\\PCL官方数据\\data\\tutorials\\pairwise");
	getFiles(dataPath, files, "pcd", true);

	cloud::PointCloud tempCloud;
	cloud::PointCloudPtr tempCloudPtr(new cloud::PointCloud);

	//读取PCD文件
	cloud::PointCloudPtrVec pointCloudPtrVec;
	for (auto& file : files) {
		cloud::PointCloudPtr pointCloudPtr(new cloud::PointCloud);
		if (pcl::io::loadPCDFile(file, *pointCloudPtr) != -1) {
			pointCloudPtr->width = 640;
			pointCloudPtr->height = 480;
			pointCloudPtrVec.push_back(pointCloudPtr);
			cout << "Load " << file << " success" << endl;
		}
		else {
			setTextRed();
			cout << "Load " << file << " failed" << endl;
			setTextWhite();
			return -1;
		}
	}
	visualizePointCloud(pointCloudPtrVec, colorVec);
	cout << "0 to stop, else to continue" << endl;
	cin >> key;
	if (key == "0") return 0;

	////双边滤波
	//cout << "BilateralFilter" << endl;
	//for (auto& _cloudPtr : pointCloudPtrVec) {
	//	if (fastBilateralFilter(_cloudPtr, tempCloudPtr) == -1)return -1;
	//	pcl::copyPointCloud(*tempCloudPtr, *_cloudPtr);
	//}
	//visualizePointCloud(pointCloudPtrVec, colorVec);
	//cout << "0 to stop, else to continue" << endl;
	//cin >> key;
	//if (key == "0") return 0;

	//下采样
	cout << "Downsample" << endl;
	pcl::VoxelGrid<cloud::PointT> grid;
	grid.setLeafSize(0.005, 0.005, 0.005);
	for (auto& cloud : pointCloudPtrVec) {
		grid.setInputCloud(cloud);
		grid.filter(*cloud);
	}
	visualizePointCloud(pointCloudPtrVec, colorVec);
	cout << "0 to stop, else to continue" << endl;
	cin >> key;
	if (key == "0") return 0;

	//距离滤波
	cout << "Start filtering" << endl;
	pcl::PassThrough<pcl::PointXYZ> pass;
	for (auto& _pointCloudPtr : pointCloudPtrVec) {
		pass.setFilterFieldName("z");
		pass.setFilterLimits(0.35, 0.6);
		pass.setInputCloud(_pointCloudPtr);
		pass.filter(*_pointCloudPtr);
		pass.setFilterFieldName("x");
		pass.setFilterLimits(-0.2, 0.2);
		pass.setInputCloud(_pointCloudPtr);
		pass.filter(*_pointCloudPtr);
		pass.setFilterFieldName("y");
		pass.setFilterLimits(-0.2, 0.2);
		pass.setInputCloud(_pointCloudPtr);
		pass.filter(*_pointCloudPtr);
	}
	cout << "Filter 0.2 - 0.55" << endl;
	visualizePointCloud(pointCloudPtrVec, colorVec);
	cout << "0 to stop, else to continue" << endl;
	cin >> key;
	if (key == "0") return 0;

	//分割
	cout << "Start Segmentation" << endl;
	cloud::PointCloudPtr segedPointCloudPtr;
	for (auto& _pointCloudPtr : pointCloudPtrVec) {
		cloud::PointCloudPtr _segedPointCloudPtr(new cloud::PointCloud);
		planeSegmentation(0.015, _pointCloudPtr, _segedPointCloudPtr, true);
		pcl::copyPointCloud(*_segedPointCloudPtr, *_pointCloudPtr);
	}
	cout << "Segmentation finished" << endl;
	visualizePointCloud(pointCloudPtrVec, colorVec);
	cout << "0 to stop, else to continue" << endl;
	cin >> key;
	if (key == "0") return 0;

	//消除离群点
	cout << "Removing outliers" << endl;
	pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
	outrem.setRadiusSearch(0.035);
	outrem.setMinNeighborsInRadius(25);
	outrem.setKeepOrganized(false);
	for (auto& cloud : pointCloudPtrVec) {
		outrem.setInputCloud(cloud);
		outrem.filter(*cloud);
	}
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setMeanK(8);
	sor.setStddevMulThresh(1.0);
	for (auto& _pointCloudPtr : pointCloudPtrVec) {
		sor.setInputCloud(_pointCloudPtr);
		sor.filter(tempCloud);
		pcl::copyPointCloud(tempCloud, *_pointCloudPtr);
	}
	visualizePointCloud(pointCloudPtrVec, colorVec);
	cout << "0 to stop, else to continue" << endl;
	cin >> key;
	if (key == "0") return 0;

	//点云配准
	cloud::PointCloudPtr registeredPointCloudPtr(new cloud::PointCloud);
	cloud::PointCloudPtrVec registeredVec;
	vector<Eigen::Matrix4f> transformationVec;
	cout << "Start registering" << endl;
	registerPairsCloud(pointCloudPtrVec, registeredVec, transformationVec, true);
	//registerPairsCloud(pointCloudPtrVec, registeredVec);
	cout << "Register finished" << endl;
	cout << registeredVec.size() << endl;
	visualizePointCloud(registeredVec, colorVec);
	cout << "0 to stop, else to continue" << endl;
	cin >> key;
	if (key == "0") return 0;

	//重建网格
	//cout << "Start meshing" << endl;
	//pcl::PolygonMesh mesh;
	//creatMesh(registeredPointCloudPtr, mesh);
	//cout << "Mesh finished" << endl;
	//visualizePointCloud(registeredPointCloudPtr, { 255.0, 255.0, 255.0 }, mesh);

	//可视化
	//pcl::visualization::PCLVisualizer viewer("viewer");
	//viewer.addPointCloud<cloud::PointT>(rabbitPointCloudPtr);
	//viewer.addPolygonMesh<cloud::PointT>(rabbitPointCloudPtr, rabbitTrianglesMesh.polygons);
	//while (!viewer.wasStopped()) {
	//	viewer.spinOnce(20);
	//}

	//保存为obj文件
	//if (saveToOBJ("rabbit.obj", *rabbitPointCloudPtr, rabbitTrianglesMesh) == -1) {
	//	cout << "save obj failed" << endl;
	//}
	//cout << "save obj success" << endl;
	
	return 0;
}

int registerWithKeypoint() {
	//获取目录下的文件
	string key;
	vector<string> files;
	string dataPath("D:\\剑走偏锋\\毕设\\硕士\\工程\\GetData\\GetData\\PCD");
	//string dataPath("D:\\剑走偏锋\\毕设\\硕士\\TOF\\数据\\PCL官方数据\\data\\tutorials\\pairwise");
	getFiles(dataPath, files, "pcd", true);

	//读取PCD文件
	cloud::PointCloudPtrVec pointCloudPtrVec;
	//for (auto& file : files) {
	//	cloud::PointCloudPtr pointCloudPtr(new cloud::PointCloud);
	//	if (pcl::io::loadPCDFile(file, *pointCloudPtr) != -1) {
	//		pointCloudPtr->width = 640;
	//		pointCloudPtr->height = 480;
	//		pointCloudPtrVec.push_back(pointCloudPtr);
	//		cout << "Load " << file << " success" << endl;
	//	}
	//	else {
	//		setTextRed();
	//		cout << "Load " << file << " failed" << endl;
	//		setTextWhite();
	//		return -1;
	//	}
	//}

	cloud::PointCloudPtr pointCloudPtr(new cloud::PointCloud);
	if (pcl::io::loadPCDFile(files[0], *pointCloudPtr) != -1) {
		pointCloudPtr->width = 640;
		pointCloudPtr->height = 480;
		pointCloudPtrVec.push_back(pointCloudPtr);
		cout << "Load " << files[0] << " success" << endl;
	}

	visualizePointCloud(pointCloudPtrVec, colorVec);
	cout << "0 to stop, else to continue" << endl;
	cin >> key;
	if (key == "0") return 0;

	////距离滤波
	//cout << "Start filtering" << endl;
	//pcl::PassThrough<pcl::PointXYZ> pass;
	//for (auto& _pointCloudPtr : pointCloudPtrVec) {
	//	pass.setFilterFieldName("z");
	//	pass.setFilterLimits(0.25, 0.6);
	//	pass.setInputCloud(_pointCloudPtr);
	//	pass.filter(*_pointCloudPtr);
	//}
	//visualizePointCloud(pointCloudPtrVec, colorVec);
	//cout << "0 to stop, else to continue" << endl;
	//cin >> key;
	//if (key == "0") return 0;

	//下采样
	//cout << "Downsample" << endl;
	//pcl::VoxelGrid<cloud::PointT> grid;
	//grid.setLeafSize(0.0025, 0.0025, 0.0025);
	//for (auto& cloud : pointCloudPtrVec) {
	//	grid.setInputCloud(cloud);
	//	grid.filter(*cloud);
	//}
	//visualizePointCloud(pointCloudPtrVec, colorVec);
	//cout << "0 to stop, else to continue" << endl;
	//cin >> key;
	//if (key == "0") return 0;

	//距离滤波
	cout << "Start filtering" << endl;
	pcl::PassThrough<pcl::PointXYZ> pass;
	for (auto& _pointCloudPtr : pointCloudPtrVec) {
		pass.setFilterFieldName("z");
		pass.setFilterLimits(0.05, 1);
		pass.setInputCloud(_pointCloudPtr);
		pass.filter(*_pointCloudPtr);
	//	pass.setFilterFieldName("x");
	//	pass.setFilterLimits(-0.2, 0.2);
	//	pass.setInputCloud(_pointCloudPtr);
	//	pass.filter(*_pointCloudPtr);
	//	pass.setFilterFieldName("y");
	//	pass.setFilterLimits(-0.2, 0.2);
	//	pass.setInputCloud(_pointCloudPtr);
	//	pass.filter(*_pointCloudPtr);
	}
	//visualizePointCloud(pointCloudPtrVec, colorVec);
	//cout << "0 to stop, else to continue" << endl;
	//cin >> key;
	//if (key == "0") return 0;

	//分割
	//cout << "Start Segmentation" << endl;
	//cloud::PointCloudPtr segedPointCloudPtr;
	//for (auto& _pointCloudPtr : pointCloudPtrVec) {
	//	cloud::PointCloudPtr _segedPointCloudPtr(new cloud::PointCloud);
	//	planeSegmentation<cloud::PointT>(0.015, _pointCloudPtr, _segedPointCloudPtr, true);
	//	pcl::copyPointCloud(*_segedPointCloudPtr, *_pointCloudPtr);
	//}
	//cout << "Segmentation finished" << endl;
	//visualizePointCloud(pointCloudPtrVec, colorVec);
	//cout << "0 to stop, else to continue" << endl;
	//cin >> key;
	//if (key == "0") return 0;

	//消除离群点
	//cout << "Removing outliers" << endl;
	//pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
	//outrem.setRadiusSearch(0.035);
	//outrem.setMinNeighborsInRadius(50);
	//outrem.setKeepOrganized(false);
	//for (auto& cloud : pointCloudPtrVec) {
	//	outrem.setInputCloud(cloud);
	//	outrem.filter(*cloud);
	//}
	////pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	////sor.setMeanK(8);
	////sor.setStddevMulThresh(1.0);
	////for (auto& _pointCloudPtr : pointCloudPtrVec) {
	////	sor.setInputCloud(_pointCloudPtr);
	////	sor.filter(*_pointCloudPtr);
	////}
	//visualizePointCloud(pointCloudPtrVec, colorVec);
	//cout << "0 to stop, else to continue" << endl;
	//cin >> key;
	//if (key == "0") return 0;

	//RangeImage
	cout << "Range Image" << endl;
	vector< pcl::RangeImage::Ptr > rangeImagePtrVec;
	int processing = 0;
	cout << "Computing range Image 0%";
	for (auto& _cloudPtr : pointCloudPtrVec) {
		pcl::RangeImage::Ptr _rangeImagePtr(new pcl::RangeImage);
		getRangeImage(_cloudPtr, _rangeImagePtr, 0.3f);
		rangeImagePtrVec.push_back(_rangeImagePtr);
		++processing;
		cout << "\rComputing range Image " << fixed << setprecision(0) << (float)processing / (float)pointCloudPtrVec.size() * 100.0f << "%";
	}
	cout << endl;
	for (auto& _rangeImagePtr : rangeImagePtrVec) {
		pcl::visualization::RangeImageVisualizer rangeImageViewer("rangeImage");
		rangeImageViewer.showRangeImage(*_rangeImagePtr);
		while (!rangeImageViewer.wasStopped()) {
			rangeImageViewer.spinOnce(20);
		}
	}

	//计算特征点
	cout << "Compute keypoints" << endl;
	cloud::PointCloudPtrVec keyPointCloudPtrVec;
	processing = 0;
	cout << "Computing keypoints 0%";
	for (int i = 0; i < pointCloudPtrVec.size(); ++i) {
		cloud::PointCloudPtr keyPointCloudPtr(new cloud::PointCloud);
		getNARFKeypoints(pointCloudPtrVec[i], *(rangeImagePtrVec[i]), keyPointCloudPtr,0.1f);
		keyPointCloudPtrVec.push_back(keyPointCloudPtr);
		++processing;
		cout << "\rComputing keypoints " << fixed << setprecision(0) << (float)processing / (float)pointCloudPtrVec.size() * 100.0f << "%";
	}
	cout << endl;
	cloud::PointCloudPtrVec viewerCloud;
	viewerCloud.resize(2);
	for (int i = 0; i < pointCloudPtrVec.size(); ++i) {
		viewerCloud[0] = pointCloudPtrVec[i];
		viewerCloud[1] = keyPointCloudPtrVec[i];
		visualizePointCloud(viewerCloud, colorVec, { 1, 4 });
	}

	//配准
	cout << "Register by keypoints" << endl;
	cloud::PointCloudPtrVec registeredKeypointVec;
	vector<Eigen::Matrix4f> transformationVec;
	registerPairsCloud(keyPointCloudPtrVec, registeredKeypointVec, transformationVec, false);

	//将特征点的配准结果应用到源点云
	cout << "Transform source point cloud" << endl;
	cloud::PointCloudPtrVec registeredPointCloudVec;
	for (int i = 0; i < pointCloudPtrVec.size(); ++i) {
		cloud::PointCloudPtr temp(new cloud::PointCloud);
		pcl::transformPointCloud(*(pointCloudPtrVec[i]), *temp, transformationVec[i]);
		registeredPointCloudVec.push_back(temp);
	}
	visualizePointCloud(registeredPointCloudVec, colorVec);

	return 0;
}

int test() {
	string file("D:\\剑走偏锋\\毕设\\硕士\\工程\\GetData\\GetData\\PCD\\009.pcd");
	cloud::PointCloudPtr pointCloudPtr(new cloud::PointCloud), tempPointCloudPtr(new cloud::PointCloud);
	cloud::PointCloudPtrVec pointCloudPtrVec;

	if (pcl::io::loadPCDFile(file, *pointCloudPtr) == -1) {
		cout << "load file " << file << " failed" << endl;
		return -1;
	}
	cout << pointCloudPtr->points.size() << endl;
	pointCloudPtr->width = 640;
	pointCloudPtr->height = 480;
	pointCloudPtrVec.push_back(pointCloudPtr);

	cout << "FastBilateralFilter" << endl;
	float sigmaS, sigmaR;
	cout << "Input sigmaS(0 for default 9)" << endl;
	cin >> sigmaS;
	if (sigmaS == 0) sigmaS = 9;
	cout << "Input sigmaR(0 for default 0.01)" << endl;
	cin >> sigmaR;
	if (sigmaR == 0) sigmaR = 0.01;
	cloud::PointCloudPtr bilateralFilter(new cloud::PointCloud);
	fastBilateralFilter(pointCloudPtr, bilateralFilter, sigmaS, sigmaR);
	pointCloudPtrVec.push_back(bilateralFilter);
	visualizePointCloud(bilateralFilter);

	cout << "pass" << endl;
	cloud::PointCloudPtr passPointCloud(new cloud::PointCloud);
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0.35, 0.6);
	pass.setInputCloud(bilateralFilter);
	pass.filter(*passPointCloud);
	pass.setFilterFieldName("x");
	pass.setFilterLimits(-0.2, 0.2);
	pass.setInputCloud(passPointCloud);
	pass.filter(*passPointCloud);
	pass.setFilterFieldName("y");
	pass.setFilterLimits(-0.2, 0.2);
	pass.setInputCloud(passPointCloud);
	pass.filter(*passPointCloud);
	pointCloudPtrVec.push_back(passPointCloud);
	visualizePointCloud(passPointCloud);

	cout << "Downsample" << endl;
	cloud::PointCloudPtr downsample(new cloud::PointCloud);
	pcl::VoxelGrid<cloud::PointT> grid;
	grid.setLeafSize(0.005, 0.005, 0.005);
	grid.setInputCloud(passPointCloud);
	grid.setMinimumPointsNumberPerVoxel(8);
	grid.filter(*downsample);
	pointCloudPtrVec.push_back(downsample);
	visualizePointCloud(downsample);

	cout << "Segmentation" << endl;
	cloud::PointCloudPtr segmentation(new cloud::PointCloud);
	planeSegmentation(0.01, downsample, segmentation, true);
	pointCloudPtrVec.push_back(segmentation);
	visualizePointCloud(segmentation);

	cout << "ourline" << endl;
	cloud::PointCloudPtr outline(new cloud::PointCloud);
	pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
	outrem.setRadiusSearch(0.035);
	outrem.setMinNeighborsInRadius(25);
	outrem.setKeepOrganized(false);
	outrem.setInputCloud(segmentation);
	outrem.filter(*outline);
	pointCloudPtrVec.push_back(outline);
	visualizePointCloud(outline);

	cout << "all" << endl;
	visualizePointCloud(pointCloudPtrVec, colorWhiteVec);

	return 0;
}

int vzense_test() {
	//PsReturnStatus status;
	PsDeviceHandle deviceHandle;
	uint32_t sessionIndex = 0;
	PsWDROutputMode wdrMode = { PsWDRTotalRange_Two, PsNearRange, 1, PsFarRange, 1, PsUnknown, 1 };
	PsDataMode setDataMode = PsDataMode::PsDepthAndRGB_30;
	PsDataMode dataMode = setDataMode;
	SlopeT slope;
	openAndConfigCamera(deviceHandle, sessionIndex, &wdrMode, dataMode, slope);

	//获取图像并显示
	PsFrameReady frameReady = { 0 };
	PsFrame depthFrame = { 0 }, RGBFrame = { 0 };
	cv::Mat imageMat;
	const string depthImageWinName = "深度图像";
	const string RGBImageWinName = "RGB图像";
	const string depthImageWDRWiNane1 = "深度图像WDR1";
	const string depthImageWDRWiNane2 = "深度图像WDR2";
	const string depthImageWDRWiNane3 = "深度图像WDR3";
	bool convertToCloud = false;
	bool saveCloudToPcd = false;
	bool continuingMode = false;
	bool showCloud = false;
	bool stop = false;
	bool b_test = false;
	bool orderedCloud = false;
	int i, key;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PassThrough<pcl::PointXYZ> passZ;
	pcl::visualization::CloudViewer viewer("Cloud Viewer");
	viewer.runOnVisualizationThreadOnce(viewerOneOff);
	viewer.registerKeyboardCallback(cloudViewerKeyboardCallback, (void*)&cloud);
	while (!stop) {
		key = cv::waitKey(10);
		switch (key)
		{
		case 27:
			cv::destroyAllWindows();
			stop = true;
			break;
		case 'm':
			continuingMode = !continuingMode;
			break;
		case 's':
			saveCloudToPcd = true;
			break;
		case 'c':
			showCloud = true;
			break;
		case 't' :
			b_test = true;
			break;
		default:
			break;
		}
		if (Ps2_ReadNextFrame(deviceHandle, sessionIndex, &frameReady) != PsReturnStatus::PsRetOK) {
			cout << "ReadNextFrame failed" << endl;
			continue;
		}

		//显示深度图像
		if (frameReady.depth == 1) {
			if (Ps2_GetFrame(deviceHandle, sessionIndex, PsFrameType::PsDepthFrame, &depthFrame) != PsReturnStatus::PsRetOK) {
				std::cout << "GetDepthFrame failed!" << endl;
			}
			else {
				//转换为点云
				convertToCloud = saveCloudToPcd || showCloud;
				if (convertToCloud) {
					const int len = depthFrame.width * depthFrame.height;
					PsVector3f* worldV = new PsVector3f[len];
					Ps2_ConvertDepthFrameToWorldVector(deviceHandle, sessionIndex, depthFrame, worldV);
					if (orderedCloud) {
						cloud->width = depthFrame.width;
						cloud->height = depthFrame.height;						
					}
					else {
						cloud->width = len;
						cloud->height = 1;
					}
					cloud->resize(len);
					for (i = 0; i < len; i++) {
						cloud->at(i).x = worldV[i].x / 100;
						cloud->at(i).y = worldV[i].y / 100;
						cloud->at(i).z = worldV[i].z / 100;
					}

					delete[] worldV;
					worldV = NULL;
					cout << "convert depth image to cloud" << endl;
				}
				//点云可视化
				if (showCloud) {					
					viewer.showCloud(cloud, "cloud");
					//showCloud = false;
					//保存点云为pcd文件
					showCloud = false;
				}
				if (b_test) {
					b_test = false;
					int count = 10;
					cv::Mat testImage(depthFrame.height, depthFrame.width, CV_16UC1, depthFrame.pFrameData);
					PsDepthVector3* depthV = new PsDepthVector3[count];
					PsVector3f* worldV0 = new PsVector3f[count];
					for (int i = 0; i < count; i++) {
						cout << "(0," << i << "):" << testImage.at<uint16_t>(0, i) << '\t';
						depthV[i].depthX = i;
						depthV[i].depthY= 0;
						depthV[i].depthZ = testImage.at<uint16_t>(0, i);
					}
					cout << endl;

					PsCameraParameters cameraParam; 
					Ps2_GetCameraParameters(deviceHandle, sessionIndex, PsSensorType::PsDepthSensor, &cameraParam);
					Ps2_ConvertDepthToWorld(deviceHandle, sessionIndex, depthV, worldV0, count, &cameraParam);
					for (int i = 0; i < count; i++) {
						cout << "(" << worldV0[i].x << "," << worldV0[i].y << "):" << worldV0[i].z << '\t';
					}
					cout << endl;

					PsVector3f* worldV1 = new PsVector3f[depthFrame.height * depthFrame.width];
					Ps2_ConvertDepthFrameToWorldVector(deviceHandle, sessionIndex, depthFrame, worldV1);
					for (int i = 0; i < count; i++) {
						cout << "(" << worldV1[i].x << "," << worldV1[i].y << "):" << worldV1[i].z << '\t';
					}
					cout << endl;
					delete[] depthV;
					delete[] worldV0;
					delete[] worldV1;
					depthV = NULL;
					worldV0 = NULL;
					worldV1 = NULL;
				}
				//WDR模式
				if (dataMode == PsDataMode::PsWDR_Depth && showDepthImage) {
					if (depthFrame.depthRange == wdrMode.range1 && wdrMode.range1Count != 0) {
						Opencv_Depth(slope.slope1, depthFrame.height, depthFrame.width, depthFrame.pFrameData, imageMat);
						cv::imshow(depthImageWDRWiNane1, imageMat);
					}
					if (depthFrame.depthRange == wdrMode.range2 && wdrMode.range2Count != 0) {
						Opencv_Depth(slope.slope2, depthFrame.height, depthFrame.width, depthFrame.pFrameData, imageMat);
						cv::imshow(depthImageWDRWiNane2, imageMat);
					}
					if (depthFrame.depthRange == wdrMode.range3 && wdrMode.range3Count != 0) {
						Opencv_Depth(slope.slope3, depthFrame.height, depthFrame.width, depthFrame.pFrameData, imageMat);
						cv::imshow(depthImageWDRWiNane3, imageMat);
					}
				}
				//非WDR模式
				else {
					Opencv_Depth(slope.slope1, depthFrame.height, depthFrame.width, depthFrame.pFrameData, imageMat);
					cv::imshow(depthImageWinName, imageMat);
				}
			}
		}
		//显示RGB图像
		if (frameReady.rgb == 1 && showRGBImage) {
			if (Ps2_GetFrame(deviceHandle, sessionIndex, PsFrameType::PsRGBFrame, &RGBFrame) != PsReturnStatus::PsRetOK) {
				std::cout << "GetRGBFrame failed!" << endl;
			}
			else {
				imageMat = cv::Mat(RGBFrame.height, RGBFrame.width, CV_8UC3, RGBFrame.pFrameData);
				cv::imshow(RGBImageWinName, imageMat);
			}
		}
	}
	//关闭处理
	closeCamera(deviceHandle, sessionIndex);
	return 0;
}

void cloudViewerKeyboardCallback(const pcl::visualization::KeyboardEvent& keyboardEvent, void* args) {
	unsigned char key = 0;
	key = keyboardEvent.getKeyCode();
	if (key == 's') {
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr = *((pcl::PointCloud<pcl::PointXYZ>::Ptr*)args);
		pcl::io::savePCDFileASCII("test.pcd", *cloudPtr);
		cout << "Saved point to test.pcd" << endl;
	}
}

int pcl_test() {
	cloud::PointCloudPtr cloud(new cloud::PointCloud);
	cloud::PointCloudPtr cloudseg(new cloud::PointCloud);
	
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(rabbitPcdFile, *cloud) == -1) {
		cout << "Load " << rabbitPcdFile << " failed!" << endl;
		return -1;
	}

	if (planeSegmentation(1.0f, cloud, cloudseg) == -1) {
		return -1;
	}
	/*if (pcl::io::loadPCDFile<pcl::PointXYZ>("D:\\浏览器下载\\rabbit.pcd", *cloudseg) == -1) {
		cout << "Load " << rabbitPcdFile << " failed!" << endl;
		return -1;
	}*/
	cout << "Load success" << endl;
	pcl::visualization::PCLVisualizer::Ptr viewerPtr(new pcl::visualization::PCLVisualizer("Before"));
	int viewerPort1 = 0, viewerPort2 = 1;
	viewerPtr->createViewPort(0.0, 0.0, 0.5, 1.0, viewerPort1);
	viewerPtr->createViewPort(0.5, 0.0, 1.0, 1.0, viewerPort2);
	cout << "createViewPort" << endl;
	viewerPtr->createViewPortCamera(viewerPort1);
	viewerPtr->createViewPortCamera(viewerPort2);
	cout << "createViewPortCamera" << endl;
	viewerPtr->setBackgroundColor(0.7, 0.7, 0.7, viewerPort1);
	viewerPtr->setBackgroundColor(0.0, 0.0, 0.0, viewerPort2);
	cout << "setBackgroundColor" << endl;
	viewerPtr->addPointCloud<pcl::PointXYZ>(cloud, "before", viewerPort1);
	viewerPtr->addPointCloud<pcl::PointXYZ>(cloudseg, "after", viewerPort2);
	cout << "addPointCloud" << endl;
	viewerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "before", viewerPort1);
	viewerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "after", viewerPort2);
	cout << "setPointCloudRenderingProperties" << endl;
	viewerPtr->addCoordinateSystem(1.0, "before", viewerPort1);
	viewerPtr->addCoordinateSystem(1.0, "after", viewerPort2);
	cout << "addCoordinateSystem" << endl;
	viewerPtr->initCameraParameters();
	cout << "initCameraParameters" << endl;
	while (!viewerPtr->wasStopped())
	{
		viewerPtr->spinOnce(100);
		std::this_thread::sleep_for(100ms);
	}

	system("pause");
	
	return 0;
}

#pragma warning
#define __COLOR__
#include <iostream>
#include <opencv2/imgproc/imgproc_c.h>
#include "camera.h"
#include "cloudOperation.h"
#include "function.h"
#include <string>
#include <ctime>

using namespace std;
bool showRGBImage = true;
bool showDepthImage = true;
ofstream PointCloudWriter;
string rabbitPcdFile = "D:\\����ƫ��\\����\\˶ʿ\\TOF\\����\\rabbit.pcd";
cloud::Color redPoint = { 255.0, 0.0, 0.0 };

int vzense_test();
int pcl_test();
void cloudViewerKeyboardCallback(const pcl::visualization::KeyboardEvent& keyboardEvent, void* args);
void viewerOneOff(pcl::visualization::PCLVisualizer& viewer) {
	viewer.setBackgroundColor(1.0, 0.5, 1.0);
}
int process();
int test();
int registerWithKeypoint();
int mode = 0;
int num = 2;
bool showCloud = true;
clock_t startTime, endTime;
double useTime;

int main() {
	cout << "input mode -1(�鿴),0(Ԥ����),1(����),2(��׼),3(ƴ��),4(�ؽ�),����(test)" << endl;
	cin >> mode;
	cout << "input num(0 for all)" << endl;
	cin >> num;
	string passPath("D:\\����ƫ��\\����\\˶ʿ\\TOF\\����\\bunny\\��������640-480\\pass");
	string keypointPath("D:\\����ƫ��\\����\\˶ʿ\\TOF\\����\\bunny\\��������640-480\\keypoint");
	string normalPath("D:\\����ƫ��\\����\\˶ʿ\\TOF\\����\\bunny\\��������640-480\\normal");
	//string separatePath("D:\\����ƫ��\\����\\˶ʿ\\TOF\\����\\������������\\separate");
	string separatePath("D:\\����ƫ��\\����\\˶ʿ\\TOF\\����\\DCAM710����\\ģ��03\\Ԥ����");
	vector<string> files;
	string path;
	//if (mode == 0) path = "D:\\����ƫ��\\����\\˶ʿ\\����\\GetData\\GetData\\PCD";
	//else path = "D:\\����ƫ��\\����\\˶ʿ\\����\\PointCloud_x64\\PointCloud_x64\\pass";
	if (mode == 0) path = "D:\\����ƫ��\\����\\˶ʿ\\TOF\\����\\bunny\\��������640-480\\ԭʼPCD";
	else path = "D:\\����ƫ��\\����\\˶ʿ\\TOF\\����\\bunny\\��������640-480\\pass";
	cloud::PointCloudPtrVec pointCloudPtrVec;
	getFiles(path, files, "pcd", true);
	if (num == 0)num = files.size();
	vector<int> pointCloudIndexVec({0,1 });
	int rows = 1080;
	int cols = 1920;

	if (mode == 0 || mode == 1) {
		cout << "��ȡPCD�ļ�" << endl;
		for (int _index = 0; _index < num; ++_index) {
			cout << "loading " << files[_index] << endl;
			cloud::PointCloudPtr _temp(new cloud::PointCloud);
			pcl::io::loadPCDFile(files[_index], *_temp);
			//_temp->width = 1920;
			//_temp->height = 1080;
			pointCloudPtrVec.push_back(_temp);
		}
		visualizePointCloud(pointCloudPtrVec, COLOR_VEC);
	}
	//cout << "��������" << endl;
	//cloud::PointCloudPtrVec objectPointCloudPtrVec;
	//objectPointCloudPtrVec.resize(pointCloudPtrVec.size());
	//for (int _index = 0; _index < pointCloudPtrVec.size(); ++_index) {
	//	cloud::PointCloudPtr _temp(new cloud::PointCloud);
	//	SeparateBG(pointCloudPtrVec[_index], _temp);
	//	objectPointCloudPtrVec[_index] = _temp;
	//}
	//visualizePointCloud(objectPointCloudPtrVec, COLOR_VEC);
	
	
	if (mode == 0) {
		cout << "�����˲�" << endl;
		pcl::PassThrough<pcl::PointXYZ> pass;
		for (auto& _pointCloudPtr : pointCloudPtrVec) {
			pass.setFilterFieldName("z");
			pass.setFilterLimits(0.05, 0.9);
			pass.setInputCloud(_pointCloudPtr);
			pass.filter(*_pointCloudPtr);
		}
		if (showCloud)visualizePointCloud(pointCloudPtrVec, COLOR_VEC);		
		////vector<pcl::PointCloud<cloud::PointT>::Ptr> a;
		//cout << "�洢�����˲����������" << endl;
		//savePointPCDs<cloud::PointT>(
		//	pointCloudPtrVec,
		//	passPath,
		//	"pass",
		//	"");

		//cout << "����range image" << endl;
		////int index = 2;
		//
		//vector<pcl::RangeImage::Ptr> rangeImagePtrVec;
		//for (auto _pointCloudPtr : pointCloudPtrVec) {
		//	pcl::RangeImage::Ptr rangeImagePtr(new pcl::RangeImage());
		//	getRangeImage(_pointCloudPtr, rangeImagePtr, 0.1f);
		//	rangeImagePtrVec.push_back(rangeImagePtr);
		//}
		//if (showCloud) {
		//	for (auto& _rangeImagePtr : rangeImagePtrVec) {
		//		pcl::visualization::RangeImageVisualizer rangeImageViewer("range image");
		//		rangeImageViewer.showRangeImage(*_rangeImagePtr);
		//		while (!rangeImageViewer.wasStopped())
		//		{
		//			rangeImageViewer.spinOnce(20);
		//		}
		//	}
		//}
		//cout << "��ȡNARF�ؼ���" << endl;
		//startTime = clock();
		//vector<vector<int>> keyPointsIndicesVec;
		//cloud::PointCloudPtrVec keyPointPtrVec;
		//pcl::ExtractIndices<pcl::PointXYZ> extract;
		//pcl::PointIndicesPtr pointIndicesPtr(new pcl::PointIndices);
		//for (int i = 0; i < pointCloudPtrVec.size(); ++i) {
		//	vector<int> _indices;
		//	cloud::PointCloudPtr _keyPointPtr(new cloud::PointCloud);
		//	getNARFKeypoints(pointCloudPtrVec[i], *(rangeImagePtrVec[i]), _indices, 0.02f);
		//	pointIndicesPtr->indices = _indices;
		//	extract.setInputCloud(pointCloudPtrVec[i]);
		//	extract.setIndices(pointIndicesPtr);
		//	extract.setNegative(false);
		//	extract.filter(*_keyPointPtr);
		//	keyPointsIndicesVec.push_back(_indices);
		//	keyPointPtrVec.push_back(_keyPointPtr);
		//	cout << "\r" << "NARF: " << (float)i / (float)pointCloudPtrVec.size() * 100 << "%";
		//}
		//endTime = clock();
		//useTime = (double)(endTime - startTime) / CLOCKS_PER_SEC;
		//cout << "�ؼ�����ȡ��ʱ: " << useTime << "s" << endl;
		//cout << endl;
		//if (showCloud) {
		//	for (int i = 0; i < pointCloudPtrVec.size(); ++i) {
		//		cloud::PointCloudPtrVec _viewerPointCloudPtrVec = { pointCloudPtrVec[i], keyPointPtrVec[i] };
		//		visualizePointCloud(_viewerPointCloudPtrVec, COLOR_VEC);
		//	}
		//}

		//cout << "�洢�ؼ�������" << endl;
		//vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> keyIndicesSavePtrVec;
		//for (int i = 0; i < keyPointsIndicesVec.size(); ++i) {
		//	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr _keyIndicesSavePtr(new pcl::PointCloud<pcl::PointXYZRGBA>);
		//	_keyIndicesSavePtr->resize(keyPointsIndicesVec[i].size());
		//	for (int _index = 0; _index < keyPointsIndicesVec[i].size(); ++_index) {
		//		_keyIndicesSavePtr->points[_index].rgba = keyPointsIndicesVec[i][_index];
		//	}
		//	keyIndicesSavePtrVec.push_back(_keyIndicesSavePtr);
		//}
		//savePointPCDs<pcl::PointXYZRGBA>(keyIndicesSavePtrVec, keypointPath, "keypoints", "", false);

		//cout << "�����˲�" << endl;
		//pcl::PassThrough<pcl::PointXYZ> pass;
		//for (auto& _pointCloudPtr : pointCloudPtrVec) {
		//	pass.setFilterFieldName("z");
		//	pass.setFilterLimits(0.05, 0.9);
		//	pass.setInputCloud(_pointCloudPtr);
		//	pass.filter(*_pointCloudPtr);
		//}
		//if (showCloud)visualizePointCloud(pointCloudPtrVec, COLOR_VEC);
		////vector<pcl::PointCloud<cloud::PointT>::Ptr> a;
		//cout << "�洢�����˲����������" << endl;
		//savePointPCDs<cloud::PointT>(
		//	pointCloudPtrVec,
		//	separatePath,
		//	"separate",
		//	"",
		//	false);

		//float resolution = 0.0f;
		//pcl::NormalEstimation<cloud::PointT, cloud::PointNormalT> normal_est;
		//pcl::search::KdTree<cloud::PointT>::Ptr kdtree(new pcl::search::KdTree<cloud::PointT>());
		//normal_est.setSearchMethod(kdtree);  //������������
		////normal_est.setKSearch(30);  //���� K ���ٽ������ڼ��㷨�ߺ�����
		//normal_est.setRadiusSearch(resolution);

		cout << "����" << endl;
		cloud::PointCloudPtrVec segmenteByDisVec;
		cloud::PointCloudPtrVec largestCloudPtrVec(pointCloudPtrVec);
		size_t maxNum = 0, maxIndex = 0;
		//for (size_t _index = 0; _index < pointCloudPtrVec.size(); ++_index) {
		//	segmenteByDis(pointCloudPtrVec[_index], segmenteByDisVec, getResolution<cloud::PointT>(pointCloudPtrVec[_index])*4.0);
		//	maxNum =  0;
		//	maxIndex = 0;
		//	for (size_t __index = 0; __index < segmenteByDisVec.size(); ++__index) {
		//		if (segmenteByDisVec[__index]->size() > maxNum) {
		//			maxNum = segmenteByDisVec[__index]->size();
		//			maxIndex = __index;
		//		}
		//	}
		//	largestCloudPtrVec.push_back(segmenteByDisVec[maxIndex]);
		//}
		visualizePointCloud(largestCloudPtrVec, COLOR_VEC);

		cout << "���߹���" << endl;
		vector<cloud::PointCloudNormalPtr> NormalPtrVec;
		for (int i = 0; i < largestCloudPtrVec.size(); ++i) {
			cloud::PointCloudNormalPtr _NormalPtr(new cloud::PointCloudNormal);
			normalEstimation(largestCloudPtrVec[i], _NormalPtr, 0, getResolution<cloud::PointT>(largestCloudPtrVec[i]) * 4.0);
			NormalPtrVec.push_back(_NormalPtr);
		}

		savePointPCDs<cloud::PointNormalT>(
			NormalPtrVec,
			"D:\\����ƫ��\\����\\˶ʿ\\TOF\\����\\bunny\\��������640-480\\����",
			"separate",
			"",
			false);

		//cout << "��������" << endl;
		//cloud::PointCloudPtrVec objectPointCloudPtrVec;
		//objectPointCloudPtrVec.resize(pointCloudPtrVec.size());
		//for (int _index = 0; _index < pointCloudPtrVec.size(); ++_index) {
		//	cloud::PointCloudPtr _temp(new cloud::PointCloud);
		//	SeparateBG(pointCloudPtrVec[_index], _temp);
		//	objectPointCloudPtrVec[_index] = _temp;
		//}
		//if (showCloud)visualizePointCloud(objectPointCloudPtrVec, COLOR_VEC);		
		//cout << "�洢���������������" << endl;
		//savePointPCDs<cloud::PointT>(
		//	objectPointCloudPtrVec,
		//	separatePath,
		//	"separate",
		//	"");

		//cout << "���߹���" << endl;
		//vector<cloud::NormalPtr> NormalPtrVec;
		//for (int i = 0; i < pointCloudPtrVec.size(); ++i) {
		//	cloud::NormalPtr _NormalPtr(new cloud::Normal);
		//	normalEstimation(pointCloudPtrVec[i], _NormalPtr);
		//	NormalPtrVec.push_back(_NormalPtr);
		//}
		//cout << "���ߴ洢" << endl;
		////for (int _i = 0; _i < NormalPtrVec.size(); ++_i) {
		////	string _path("D:\\����ƫ��\\����\\˶ʿ\\����\\PointCloud_x64\\PointCloud_x64\\normal\\");
		////	_path.append(to_string(_i).append("normal.pcd"));
		////	pcl::io::savePCDFileASCII(_path, *(NormalPtrVec[_i]));
		////}
		//savePointPCDs<cloud::NormalT>(
		//	NormalPtrVec,
		//	normalPath,
		//	"normal",
		//	"",
		//	false);
		//for (auto& _temp : NormalPtrVec) {
		//	_temp->clear();
		//}
	}
	else if (mode == 1){/*
		cout << "���ر��������������" << endl;
		vector<string> separateFiles;
		getFiles(separatePath, separateFiles, "pcd", true);
		cout << "1" << endl;
		cloud::PointCloudPtrVec objectPointCloudPtrVec;
		for (int _i = 0; _i < separateFiles.size(); ++_i) {
			cout << "i" << endl;
			cloud::PointCloudPtr _temp(new cloud::PointCloud);
			pcl::io::loadPCDFile(separateFiles[_i], *_temp);
			objectPointCloudPtrVec.push_back(_temp);
		}

		cout << "���عؼ�������" << endl;
		pcl::search::KdTree<cloud::PointT> kdTree;
		vector<vector<int>> keyPointsIndicesVec;
		vector<int> kdTreeIndicesTemp;
		vector<float> kdTreeDisTemp;
		vector<string> keyPointFiles;
		cloud::PointCloudPtrVec keyPointPtrVec;
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		pcl::PointIndicesPtr pointIndicesPtr(new pcl::PointIndices);
		getFiles(keypointPath, keyPointFiles, "pcd", true);
		for (int _i = 0; _i < num; ++_i) {
			kdTree.setInputCloud(objectPointCloudPtrVec[_i]);
			pcl::PointCloud<pcl::PointXYZRGBA> _point;
			cloud::PointCloudPtr _keyPointPtr(new cloud::PointCloud);
			vector<int> _indices;
			pcl::io::loadPCDFile(keyPointFiles[_i], _point);
			//_indices.resize(_point.points.size());
			for (int _index = 0; _index < _point.points.size(); ++_index) {
				kdTree.nearestKSearch(
					pointCloudPtrVec[_i]->at(_point.points[_index].rgba),
					1,
					kdTreeIndicesTemp,
					kdTreeDisTemp);
				if ( kdTreeDisTemp[0] == 0 ) {
					cout << "distance: " << kdTreeDisTemp[0] << endl;
					_indices.push_back(_point.points[_index].rgba);
				}
				kdTreeIndicesTemp.clear();
				kdTreeDisTemp.clear();

				_indices.push_back(_point.points[_index].rgba);
			}
			//�ֶ�ȥ���ؼ���
			//if (_i == 0) {
			//	vector<int>::iterator _iterL = _indices.begin() + 10;
			//	vector<int>::iterator _iterR = _indices.end();
			//	_indices.erase(_iterL, _iterR);
			//	_iterL = _indices.begin() + 6;
			//	_iterR = _indices.end() - 1;
			//	_indices.erase(_iterL, _iterR);
			//}
			pointIndicesPtr->indices = _indices;
			extract.setInputCloud(pointCloudPtrVec[_i]);
			extract.setIndices(pointIndicesPtr);
			extract.setNegative(false);
			extract.filter(*_keyPointPtr);
			cloud::PointCloudPtr _temp(new cloud::PointCloud);
			//if (_i == 0) {
			//	SeparateView(_keyPointPtr, _temp, 5, true);
			//	keyPointPtrVec.push_back(_temp);
			//}
			//else {
			//	keyPointPtrVec.push_back(_keyPointPtr);
			//}		
			//SeparateView(_keyPointPtr, _temp, 5, true);
			keyPointPtrVec.push_back(_keyPointPtr);
			keyPointsIndicesVec.push_back(_indices);
		}
		//for (int _j = 0; _j < keyPointPtrVec.size(); ++_j) {
		//	for (int _i = 0; _i < keyPointPtrVec[_j]->size(); ++_i) {
		//		cout << _i << "th key point in the " << _j << "th cloud" << endl;
		//		cloud::PointCloudPtr _temp(new cloud::PointCloud);
		//		_temp->resize(1);
		//		_temp->at(0).x = keyPointPtrVec[_j]->at(_i).x;
		//		_temp->at(0).y = keyPointPtrVec[_j]->at(_i).y;
		//		_temp->at(0).z = keyPointPtrVec[_j]->at(_i).z;
		//		cloud::PointCloudPtrVec _viewerPointCloudPtrVec = { objectPointCloudPtrVec[_j], _temp };
		//		visualizePointCloud(_viewerPointCloudPtrVec, COLOR_VEC, { 1, 4 });
		//	}
		//}
		if (showCloud) {
			for (int i = 0; i < pointCloudPtrVec.size(); ++i) {
				cloud::PointCloudPtrVec _viewerPointCloudPtrVec = { objectPointCloudPtrVec[i], keyPointPtrVec[i] };
				visualizePointCloud(_viewerPointCloudPtrVec, COLOR_VEC, { 1, 4 });
			}
		}

		////cloud::PointCloudNormalPtr cloudNormalPtr(new cloud::PointCloudNormal);
		////cloud::PointCloudPtr keyPointCloudPtr(new cloud::PointCloud);
		////normalEstimation(pointCloudPtrVec[index], cloudNormalPtr);
		////getSIFTKeypoint(cloudNormalPtr, keyPointCloudPtr);

		cout << "�������ݶ�ȡ" << endl;
		vector<cloud::NormalPtr> NormalPtrVec;
		vector<string> normalFiles;
		getFiles(normalPath, normalFiles, "pcd", true);
		for (int _index = 0; _index < num; ++_index) {
			cout << normalFiles[_index] << endl;
			cloud::NormalPtr _temp(new cloud::Normal);
			pcl::io::loadPCDFile(normalFiles[_index], *_temp);
			NormalPtrVec.push_back(_temp);
		}

		cout << "����PFH" << endl;
		int n_split = 5;
		int pfhNum = pow(n_split, 3);
		vector<vector<Eigen::VectorXf>> pfh_histogramVec;
		for (int i = 0; i < pointCloudPtrVec.size(); ++i) {
			vector<Eigen::VectorXf> _pfh_histograms;
			computePFH(pointCloudPtrVec[i], NormalPtrVec[i], _pfh_histograms, keyPointsIndicesVec[i], 40, 0.01, n_split);
			pfh_histogramVec.push_back(_pfh_histograms);
		}
		cout << "pfh1: " << pfh_histogramVec[0].size() << endl;
		cout << "pfh2: " << pfh_histogramVec[1].size() << endl;

		cout << "��������ת��" << endl;
		cloud::PointNormalPfhPtrVec pointNormalPfhPtrVec;
		vector<cloud::PointNormalPfhPtrVec> pairPoints;
		pairPoints.resize(pfh_histogramVec.size() - 1);
		for (int i = 0; i < pfh_histogramVec.size() - 1; ++i) {
			Eigen::MatrixXf pfhPCA(pfh_histogramVec[i].size() + pfh_histogramVec[i + 1].size(), pfhNum);
			int j = 0;
			for (auto& pfh : pfh_histogramVec[i]) {
				pfhPCA.row(j) = pfh;
				++j;
			}
			for (auto& pfh : pfh_histogramVec[i + 1]) {
				pfhPCA.row(j) = pfh;
				++j;
			}
			cout << "all rows: " << keyPointPtrVec[i]->size() + keyPointPtrVec[i + 1]->size() << endl;
			cout << "pfhPCA rows: " << pfhPCA.rows() << endl;
			Eigen::MatrixXf _pfhReduce = PCA(pfhPCA, 4, 1.0);
			cout << "_pfhReduce rows: " << _pfhReduce.rows() << " cols: " << _pfhReduce.cols() << endl;
			//cout << "_pfhReduce(256, 0): " << _pfhReduce(256, 0)<<endl;
			cloud::PointNormalPfhPtr _temp(new cloud::PointNormalPfh);
			_temp->resize(keyPointPtrVec[i]->size());
			for (int _index = 0; _index < keyPointPtrVec[i]->size(); ++_index) {
				_temp->points[_index].x = keyPointPtrVec[i]->points[_index].x;
				_temp->points[_index].y = keyPointPtrVec[i]->points[_index].y;
				_temp->points[_index].z = keyPointPtrVec[i]->points[_index].z;
				for (int __index = 0; __index < 4; ++__index) {
					_temp->points[_index].data_n[__index] = _pfhReduce(_index, __index) / 100;
				}
			}
			cloud::PointNormalPfhPtr _temp2(new cloud::PointNormalPfh);
			_temp2->resize(keyPointPtrVec[i + 1]->size());
			for (int _index = 0; _index < keyPointPtrVec[i + 1]->size(); ++_index) {
				_temp2->points[_index].x = keyPointPtrVec[i + 1]->points[_index].x;
				_temp2->points[_index].y = keyPointPtrVec[i + 1]->points[_index].y;
				_temp2->points[_index].z = keyPointPtrVec[i + 1]->points[_index].z;
				for (int __index = 0; __index < 4; ++__index) {
					_temp2->points[_index].data_n[__index] = _pfhReduce(_index + keyPointPtrVec[i]->size(), __index) / 100;
				}
			}
			pairPoints[i] = { _temp, _temp2 };
		}

		if (showCloud) {
			cout << "������׼�ĵ���" << endl;
			cloud::PointCloudPtr _viewer1(new cloud::PointCloud);
			cloud::PointCloudPtr _viewer2(new cloud::PointCloud);
			pcl::copyPointCloud<cloud::PointNormalPfhT, cloud::PointT>(*pairPoints[0][0], *_viewer1);
			pcl::copyPointCloud<cloud::PointNormalPfhT, cloud::PointT>(*pairPoints[0][1], *_viewer2);
			cloud::PointCloudPtrVec _viewerVec = { _viewer1,_viewer2 };
			visualizePointCloud(_viewerVec, COLOR_VEC, { 3, 3 });
		}
		//for (int _i = 0; _i < 100; ++_i) {
		//	cout << pairPoints[0][0]->at(_i).x << "\t";
		//	cout << pairPoints[0][0]->at(_i).y << "\t";
		//	cout << pairPoints[0][0]->at(_i).z << "\n";
		//	cout << pairPoints[0][0]->at(_i).data_n[0] << "\t";
		//	cout << pairPoints[0][0]->at(_i).data_n[1] << "\t";
		//	cout << pairPoints[0][0]->at(_i).data_n[2] << "\t";
		//	cout << pairPoints[0][0]->at(_i).data_n[3] << endl;
		//	cout << "===========" << endl;
		//}
		
		cout << "��׼" << endl;
		cloud::PointCloudPtr resCloudPtr(new cloud::PointCloud);
		Eigen::Matrix4f trans;
		pairAlignWithCustom(pairPoints[0][0], pairPoints[0][1], resCloudPtr, trans);
		cloud::PointCloudPtr _transPtr(new cloud::PointCloud);
		cloud::PointCloudPtr _transKeyPtr(new cloud::PointCloud);
		pcl::transformPointCloud(*(objectPointCloudPtrVec[0]), *_transPtr, trans);
		pcl::transformPointCloud(*(keyPointPtrVec[0]), *_transKeyPtr, trans);
		
		//pairAlignWithNormal(objectPointCloudPtrVec[0], objectPointCloudPtrVec[1], resCloudPtr, trans);
		cloud::PointCloudPtrVec viewerPointPtrVec = { _transPtr, objectPointCloudPtrVec[1] };
		cloud::PointCloudPtrVec viewerPointPtrVec2 = { _transKeyPtr, keyPointPtrVec[1] };
		visualizePointCloud(viewerPointPtrVec, COLOR_VEC);
		visualizePointCloud(viewerPointPtrVec2, COLOR_VEC, {1, 4});
		*/
	}
	else if (mode == 2) {
		class MyPointRepresentationNormal :public pcl::PointRepresentation<cloud::PointNormalT> {
			using pcl::PointRepresentation<cloud::PointNormalT>::nr_dimensions_;
		public:
			using Ptr = shared_ptr<MyPointRepresentationNormal>;
			MyPointRepresentationNormal() {
				nr_dimensions_ = 7;
			}
			virtual void copyToFloatArray(const cloud::PointNormalT& p, float* out) const
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

		vector<string> separateFiles;
		//getFiles("D:\\����ƫ��\\����\\˶ʿ\\TOF\\����\\DCAM710����\\ģ��03\\Ԥ����", separateFiles, "pcd", true);
		getFiles("D:\\����ƫ��\\����\\˶ʿ\\TOF\\����\\��������men_640_480\\����", separateFiles, "pcd", true);
		//getFiles("D:\\����ƫ��\\����\\˶ʿ\\TOF\\����\\bunny\\��������640-480\\����", separateFiles, "pcd", true);
		if (num == 0 || num > separateFiles.size())num = separateFiles.size();
		cloud::PointCloudNormalPtrVec objectPointCloudPtrVec;
		vector<cloud::PointCloudNormalPtr> preTreatPointCloudPtrVec;

		pcl::PassThrough<cloud::PointNormalT> pass;
		pass.setFilterFieldName("z");
		pass.setFilterLimits(0.0001, 1.0);

		pcl::VoxelGrid<cloud::PointNormalT> grid;
		grid.setLeafSize(0.001, 0.001, 0.001);

		float resolution = 0.0f;
		MyPointRepresentationXYZ<cloud::PointNormalT>::Ptr rep(new MyPointRepresentationXYZ<cloud::PointNormalT>);
		pcl::NormalEstimation<cloud::PointNormalT, cloud::PointNormalT> normal_est;
		pcl::search::KdTree<cloud::PointNormalT>::Ptr kdtree(new pcl::search::KdTree<cloud::PointNormalT>());
		kdtree->setPointRepresentation(rep);
		normal_est.setSearchMethod(kdtree);  //������������
		//normal_est.setKSearch(30);  //���� K ���ٽ������ڼ��㷨�ߺ�����
		normal_est.setRadiusSearch(resolution);

		pcl::RadiusOutlierRemoval<cloud::PointNormalT> outrem;
		outrem.setMinNeighborsInRadius(12);
		outrem.setKeepOrganized(false);
		outrem.setRadiusSearch(resolution);

		pcl::ExtractIndices<cloud::PointNormalT> extractNomral;

		MyPointRepresentationNormal::Ptr pointRepresentationNormal(new MyPointRepresentationNormal);
		size_t start_ = 1;
		cloud::PointCloudNormalPtr viewerNormal(new cloud::PointCloudNormal);
		for (int _i = start_; _i < num; ++_i) {
			cloud::PointCloudNormalPtr _temp(new cloud::PointCloudNormal);
			cloud::PointCloudNormalPtr _treatTemp(new cloud::PointCloudNormal);
			cloud::PointCloudNormalPtr _pointNormalTemp(new cloud::PointCloudNormal);
			pcl::io::loadPCDFile(separateFiles[_i], *_temp);
			//������ֵ�˳�
			pass.setInputCloud(_temp);
			pass.filter(*_temp);
			cout << "load " << separateFiles[_i] << endl;

			////ȥ����
			//Eigen::Matrix<float, 4, 1> centroid;
			//pcl::compute3DCentroid(*_temp, centroid);
			//pcl::demeanPointCloud<cloud::PointNormalT>(*_temp, centroid, *_temp);

			//fastBilateralFilter(_temp, _temp);

			objectPointCloudPtrVec.push_back(_temp);
			//�²���
			grid.setInputCloud(_temp);
			grid.filter(*_treatTemp);
			//����ֱ���
			resolution = getResolution < cloud::PointNormalT > (_treatTemp);
			cout << "resolution " << resolution << endl;
			//�˳���Ⱥ��
			outrem.setRadiusSearch(resolution * 5.0f);
			outrem.setInputCloud(_treatTemp);
			outrem.filter(*_treatTemp);
			pcl::copyPointCloud(*_temp, *viewerNormal);

			//���߹���
			normal_est.setRadiusSearch(resolution * 2.5);
			normal_est.setInputCloud(_treatTemp);
			normal_est.compute(*_treatTemp);
			removeInvalid(_treatTemp);
			//cloud::PointCloudNormalPtr _keytemp(new cloud::PointCloudNormal);
			//curKeypoints(_treatTemp, _keytemp, getResolution<cloud::PointNormalT>(_treatTemp) * 8.0f, 5, 10);
			//pcl::copyPointCloud(*_treatTemp, *_pointNormalTemp);
			//if(!_pointNormalTemp->is_dense)removeInvalid(_pointNormalTemp);
			//�˳���б��
			//removeDiagonally(_treatTemp, _pointNormalTemp, -0.4);
			pcl::PointIndices::Ptr edgeIndicesPtr(new pcl::PointIndices);
			edgeDetection(_treatTemp, edgeIndicesPtr, resolution * 4.0f, 0.25,0.5);
			//edgeViewer(_treatTemp);
			extractNomral.setNegative(true);
			extractNomral.setInputCloud(_treatTemp);
			extractNomral.setIndices(edgeIndicesPtr);
			extractNomral.filter(*_pointNormalTemp);
			preTreatPointCloudPtrVec.push_back(_pointNormalTemp);
		}
		//for (auto _p : preTreatPointCloudPtrVec) {
		//	cloud::PointCloudNormalPtr _temp(new cloud::PointCloudNormal);
		//	ISSKeypoints(_p, _temp);
		//	visualizePointCloud({ _p, _temp }, COLOR_VEC);
		//}		
		//cout << "�ؼ�����" << endl;
		//for (auto _p : preTreatPointCloudPtrVec) {
		//	cloud::PointCloudNormalPtr _temp(new cloud::PointCloudNormal);
		//	curKeypoints(_p, _temp, getResolution<cloud::PointNormalT>(_p)*8.0f, 20,50);
		//	visualizePointCloud({ _p, _temp }, COLOR_VEC, {2,4});
		//}
		visualizePointCloud( preTreatPointCloudPtrVec ,COLOR_VEC, { 1,1 }, WHITE_COLOR, false );
		cloud::PointCloudNormalPtrVec keypointVec;
		vector<pcl::PointCloud<CustomPointT2>> cloudFeatureVec;
		cout << "�ؼ�����" << endl;
		pcl::FPFHEstimationOMP<cloud::PointNormalT, cloud::PointNormalT, pcl::FPFHSignature33> fest;
		vector<pcl::PointCloud<pcl::FPFHSignature33>::Ptr> fpfhVec;
		int ii=0;
		for (auto _p : preTreatPointCloudPtrVec) {
			cloud::PointCloudNormalPtr _temp(new cloud::PointCloudNormal);
			pcl::Indices keyIndices;
			resolution = getResolution<cloud::PointNormalT>(_p);
			pcl::PointCloud<CustomPointT2> cloudFeature;
			Keypoints3(_p, keyIndices, cloudFeature, resolution * 4.0, resolution * resolution * 0, 0.85, 0.2, 1e-3, resolution * 1.5, 75, 1, true);
			cloudFeatureVec.push_back(cloudFeature);

			//FPFH
			pcl::PointCloud<pcl::FPFHSignature33>::Ptr _fpfhPtr(new pcl::PointCloud<pcl::FPFHSignature33>);
			pcl::PointIndices::Ptr _PointIndicesPtr(new pcl::PointIndices);
			_PointIndicesPtr->indices = keyIndices;
			fest.setIndices(_PointIndicesPtr);
			fest.setRadiusSearch(resolution*4.0);
			fest.setInputCloud(_p);
			fest.setInputNormals(_p);
			fest.compute(*_fpfhPtr);
			fpfhVec.push_back(_fpfhPtr);

			for (const auto& _indice : keyIndices) {
				_temp->push_back((*_p)[_indice]);
			}
			keypointVec.push_back(_temp);

			vector<cloud::Color> _color_vec(2);
			if (ii == 0)_color_vec[0] = RED_COLOR;
			else _color_vec[0] = GREEN_COLOR;
			_color_vec[1] = BLUE_COLOR;
			++ii;
			visualizePointCloud({ _p, _temp }, _color_vec, { 2,3 }, WHITE_COLOR, false);
		}

		//for (auto _p : preTreatPointCloudPtrVec) {
		//	//FPFH
		//	pcl::PointCloud<pcl::FPFHSignature33>::Ptr _fpfhPtr(new pcl::PointCloud<pcl::FPFHSignature33>);
		//	fest.setRadiusSearch(resolution * 4.0);
		//	fest.setInputCloud(_p);
		//	fest.setInputNormals(_p);
		//	fest.compute(*_fpfhPtr);
		//	fpfhVec.push_back(_fpfhPtr);
		//	keypointVec.push_back(_p);
		//}
		SampleConsensusPrerejective2<cloud::PointNormalT, cloud::PointNormalT, pcl::FPFHSignature33> align;
		align.setInputSource(keypointVec[0]);
		align.setSourceFeatures(fpfhVec[0]);
		align.setInputTarget(keypointVec[1]);
		align.setTargetFeatures(fpfhVec[1]);
		align.setMaximumIterations(50000); // Number of RANSAC iterations
		align.setNumberOfSamples(3); // Number of points to sample for generating/prerejecting a pose
		align.setCorrespondenceRandomness(33); // Number of nearest features to use
		align.setSimilarityThreshold(0.9f); // Polygonal edge length similarity threshold
		align.setMaxCorrespondenceDistance(2.5f * getResolution < cloud::PointNormalT >(objectPointCloudPtrVec[0])); // Inlier threshold
		align.setInlierFraction(0.25f); // Required inlier fraction for accepting a pose hypothesis
		cloud::PointCloudNormalPtr cloudAligned(new cloud::PointCloudNormal);
		align.align(*cloudAligned);
		Eigen::Matrix4f chushiMatrix;
		chushiMatrix=align.getFinalTransformation();
		pcl::transformPointCloudWithNormals(*(objectPointCloudPtrVec[0]), *(objectPointCloudPtrVec[0]), chushiMatrix);
		visualizePointCloud(objectPointCloudPtrVec, COLOR_VEC, { 1,1 }, WHITE_COLOR, false);
		vector<int> sourceIndices = align.sourceCoorIndices;
		vector<int> targetIndices = align.targetCoorIndices;
		pcl::CorrespondencesPtr coor(new pcl::Correspondences);
		coor->resize(sourceIndices.size());
		for (int _index = 0; _index < coor->size(); ++_index) {
			(*coor)[_index].index_query = sourceIndices[_index];
			(*coor)[_index].index_match = targetIndices[_index];
			(*coor)[_index].distance = 0;
		}
		pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer);
		visualizeCorrespondences(keypointVec[0], keypointVec[1], coor, viewer,2);
		for (int _index = 0; _index < coor->size(); ++_index) {
			viewer->addLine(keypointVec[0]->at(sourceIndices[_index]), keypointVec[1]->at(targetIndices[_index]), to_string(_index));
			viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 4, to_string(_index));
		}
		
		while (!viewer->wasStopped()) {
			viewer->spinOnce(20);
		}
		pcl::registration::CorrespondenceEstimation<cloud::PointNormalT, cloud::PointNormalT> correspondenceEstimation;
		MyPointRepresentationXYZ<cloud::PointNormalT>::Ptr pointRepresentationXYZ(new MyPointRepresentationXYZ<cloud::PointNormalT>());
		correspondenceEstimation.setInputSource(preTreatPointCloudPtrVec[0]);
		correspondenceEstimation.setInputTarget(preTreatPointCloudPtrVec[1]);
		correspondenceEstimation.setPointRepresentation(pointRepresentationXYZ);
		coor->clear();
		correspondenceEstimation.determineCorrespondences(*coor, 0.01);
		float sumS = 0;
		for (int _index = 0; _index < coor->size(); ++_index) {
			sumS += ((*preTreatPointCloudPtrVec[0])[coor->at(_index).index_query].getVector3fMap() - (*preTreatPointCloudPtrVec[1])[coor->at(_index).index_match].getVector3fMap()).squaredNorm();
		}
		float MSE = sumS / (float)coor->size();
		cout << "MSE: " << MSE << endl;
		return 0;
		
		//preTreatPointCloudPtrVec.clear();
		//cloud::PointCloudNormalPtr p1(new cloud::PointCloudNormal);
		//cloud::PointCloudNormalPtr p2(new cloud::PointCloudNormal);
		//for (int i = 0; i < 100; ++i) {
		//	for (int j = 0; j < 10; ++j) {
		//		cloud::PointNormalT p;
		//		p.x = i * 0.001+0.2;
		//		p.y = j * 0.001+0.2;
		//		p.z = 0;
		//		p.normal_x = 0;
		//		p.normal_y = 0;
		//		p.normal_z = 1;
		//		p1->push_back(p);
		//	}
		//}
		//for (int i = 0; i < 100; ++i) {
		//	for (int j = 0; j < 10; ++j) {
		//		cloud::PointNormalT p;
		//		p.x = i * 0.001+0.2;
		//		p.y = j * 0.001+0.2;
		//		p.z = 0.005;
		//		p.normal_x = 0;
		//		p.normal_y = 0;
		//		p.normal_z = 1;
		//		p2->push_back(p);
		//	}
		//}
		//preTreatPointCloudPtrVec.push_back(p1);
		//preTreatPointCloudPtrVec.push_back(p2);
		//grid.setInputCloud(objectPointCloudPtrVec[0]);
		//grid.filter(*objectPointCloudPtrVec[0]);
		//grid.setInputCloud(objectPointCloudPtrVec[1]);
		//grid.filter(*objectPointCloudPtrVec[1]);
		//cout << "����OBB��Χ��" << endl;
		//cloud::OBBT obb = getOBB(objectPointCloudPtrVec[0]);
		//cloud::OBBT obb2 = getOBB(objectPointCloudPtrVec[1]);
		//cloud::PointCloudPtr obbCloud(new cloud::PointCloud);
		//cloud::PointCloudPtr obbCloud2(new cloud::PointCloud);
		//boxToPointCloud(obb, obbCloud);
		//boxToPointCloud(obb2, obbCloud2);
		//Eigen::Matrix4f trans1 = getTransFromOBBT(obb);
		//Eigen::Matrix4f trans2 = getTransFromOBBT(obb2);
		//pcl::transformPointCloud(*(objectPointCloudPtrVec[0]), *(objectPointCloudPtrVec[0]), trans1.inverse());
		//pcl::transformPointCloud(*(objectPointCloudPtrVec[0]), *(objectPointCloudPtrVec[0]), trans2);
		//pcl::transformPointCloud(*obbCloud, *obbCloud, trans1.inverse());
		//pcl::transformPointCloud(*obbCloud, *obbCloud, trans2); 
		cloud::PointCloudNormalPtrVec segmenteByDisVec;
		//cloud::PointCloudPtrVec largestCloudPtrVec;
		//vector<cloud::PointCloudNormalPtr> largestCloudPtrVec(preTreatPointCloudPtrVec);
		vector<cloud::PointCloudNormalPtr> largestCloudPtrVec;
		size_t maxNum = 0, maxIndex = 0;
		for (size_t _index = 0; _index < objectPointCloudPtrVec.size(); ++_index) {
			segmenteByDis(preTreatPointCloudPtrVec[_index], segmenteByDisVec, getResolution<cloud::PointNormalT>(objectPointCloudPtrVec[_index])*4.0);
			maxNum =  0;
			maxIndex = 0;
			for (size_t __index = 0; __index < segmenteByDisVec.size(); ++__index) {
				if (segmenteByDisVec[__index]->size() > maxNum) {
					maxNum = segmenteByDisVec[__index]->size();
					maxIndex = __index;
				}
			}
			largestCloudPtrVec.push_back(segmenteByDisVec[maxIndex]);
		}
		//visualizePointCloud(largestCloudPtrVec, COLOR_VEC);
		vector<Eigen::Matrix4f> transVec(largestCloudPtrVec.size(), Eigen::Matrix4f::Identity());
		//transVec[0] = Eigen::Matrix4f::Identity();
		cloud::PointCloudNormalPtr tempCloudPtr(new cloud::PointCloudNormal), keyPointTempPtr(new cloud::PointCloudNormal);
		Eigen::Matrix4f tempTrans = Eigen::Matrix4f::Identity();
		Eigen::AngleAxisf rotationVector(M_PI / -6, Eigen::Vector3f(0, 1, 0));
		//Eigen::Matrix3f R12(rotationVector);
		Eigen::Matrix4f rotationMatrix = Eigen::Matrix4f::Identity();
		//rotationMatrix.block<3, 3>(0, 0) = rotationVector.matrix();
		//rotationMatrix.block<3, 1>(0, 3) = Eigen::Vector3f(0.1,0.0,-0.1);
		for (size_t _index = 1; _index < largestCloudPtrVec.size(); ++_index) {
			cout << "Pairing " << _index - 1 << "th and" << _index << "th" << endl;
			pcl::copyPointCloud(*(largestCloudPtrVec[_index]), *tempCloudPtr);
			pcl::copyPointCloud(*(keypointVec[_index]), *keyPointTempPtr);
			//pcl::transformPointCloudWithNormals(*(largestCloudPtrVec[_index]), *tempCloudPtr, rotationMatrix);
			//transVec[_index] *= rotationMatrix;
			//visualizePointCloud({ tempCloudPtr , largestCloudPtrVec[_index - 1] }, COLOR_VEC, { 3,3 });
			//pairAlignWithCustom(
			//	keyPointTempPtr,
			//	keypointVec[_index - 1],
			//	tempTrans,
			//	false,
			//	0.002,
			//	0.05,
			//	false,
			//	false,
			//	1,
			//	16,
			//	1e-7);
			//transVec[_index] *= tempTrans;
			//pcl::transformPointCloudWithNormals(*tempCloudPtr, *tempCloudPtr, tempTrans);
			//pcl::transformPointCloudWithNormals(*keyPointTempPtr, *keyPointTempPtr, tempTrans);
			//visualizePointCloud({ tempCloudPtr , largestCloudPtrVec[_index - 1] }, COLOR_VEC, { 3,3 });
			//pairAlignWithCustom(
			//	keyPointTempPtr,
			//	keypointVec[_index - 1],
			//	tempTrans,
			//	false,
			//	0.002,
			//	0.005,
			//	false,
			//	false,
			//	1,
			//	16,
			//	1e-7);
			//transVec[_index] *= tempTrans;
			//pcl::transformPointCloudWithNormals(*tempCloudPtr, *tempCloudPtr, tempTrans);
			//visualizePointCloud({ tempCloudPtr , largestCloudPtrVec[_index - 1] }, COLOR_VEC, { 3,3 });
			pairAlignWithNormal(
				tempCloudPtr,
				largestCloudPtrVec[_index - 1],
				tempTrans,
				true,
				0.003,
				0.01,
				false,
				false,
				2,
				32,
				1e-7);
			transVec[_index] *= tempTrans;
			pcl::transformPointCloudWithNormals(*tempCloudPtr, *tempCloudPtr, tempTrans);
			visualizePointCloud({ tempCloudPtr , largestCloudPtrVec[_index - 1] }, COLOR_VEC, { 1,1 }, WHITE_COLOR, false);
			//pairAlignWithNormal(
			//	tempCloudPtr,
			//	largestCloudPtrVec[_index - 1],
			//	tempTrans,
			//	true,
			//	0.002,
			//	0.01,
			//	false,
			//	false,
			//	1,
			//	32,
			//	1e-7);
			//transVec[_index] *= tempTrans;
			//pcl::transformPointCloudWithNormals(*tempCloudPtr, *tempCloudPtr, tempTrans);
			//removeInvalid(tempCloudPtr);
			//visualizePointCloud({ tempCloudPtr , largestCloudPtrVec[_index - 1] }, COLOR_VEC, { 3,3 });
			//pairAlignWithNormal(
			//	tempCloudPtr,
			//	largestCloudPtrVec[_index - 1],
			//	tempTrans,
			//	false,
			//	0.002,
			//	0.02,
			//	false,
			//	false,
			//	1,
			//	8,
			//	1e-7);
			//transVec[_index] *= tempTrans;
			//pcl::transformPointCloudWithNormals(*tempCloudPtr, *tempCloudPtr, tempTrans);
			//removeInvalid(tempCloudPtr);
			pairAlignWithCustom(
				tempCloudPtr,
				largestCloudPtrVec[_index - 1],
				tempTrans,
				true,
				0.002,
				0.05,
				true,
				false,
				1,
				32,
				1e-7);
			transVec[_index] *= tempTrans;
			pcl::transformPointCloudWithNormals(*tempCloudPtr, *tempCloudPtr, tempTrans);
			visualizePointCloud({ tempCloudPtr , largestCloudPtrVec[_index - 1] }, COLOR_VEC, { 3,3 });
		}

		Eigen::Matrix4f wholeTrans = transVec[0];
		for (size_t _index = 1; _index < objectPointCloudPtrVec.size(); ++_index) {
			wholeTrans *= transVec[_index];
			pcl::transformPointCloud(*(objectPointCloudPtrVec[_index]), *(objectPointCloudPtrVec[_index]), wholeTrans);
		}
		visualizePointCloud(objectPointCloudPtrVec, COLOR_VEC, { 2,2 });
		cout << "���棿(0:��1:��)" << endl;
		int inputNum = 0;
		cin >> inputNum;
		if (inputNum == 1) {
			savePointPCDs<cloud::PointNormalT>(
				objectPointCloudPtrVec,
				"D:\\����ƫ��\\����\\˶ʿ\\TOF\\����\\��������men_640_480\\��׼",
				"registration",
				"",
				false);
		}
	}
	//ƴ��
	else if (mode == 3) {
		bool isAll = true;
		cout << "���ص�������" << endl;
		string savePath("D:\\����ƫ��\\����\\˶ʿ\\TOF\\����\\bunny\\��������640-480\\ƴ��");
		string saveName("fuse.pcd");
		vector<string> separateFiles;
		//getFiles("D:\\����ƫ��\\����\\˶ʿ\\TOF\\����\\bunny\\��������640-480\\��׼2", separateFiles, "pcd", true);
		getFiles("D:\\����ƫ��\\����\\˶ʿ\\TOF\\����\\bunny\\��������640-480\\��׼3", separateFiles, "pcd", true);
		cloud::PointCloudPtrVec objectPointCloudPtrVec;
		if (num == 0 || num > separateFiles.size())num = separateFiles.size();
		pcl::PassThrough<pcl::PointXYZ> pass;
		pass.setFilterFieldName("z");
		pass.setFilterLimits(0.0001, 1.0);
		pcl::NormalEstimation<cloud::PointT, cloud::PointNormalT> normal_est;
		pcl::search::KdTree<cloud::PointT>::Ptr kdtree(new pcl::search::KdTree<cloud::PointT>());
		normal_est.setSearchMethod(kdtree);  //������������
		pcl::VoxelGrid<cloud::PointT> grid;
		grid.setLeafSize(0.001, 0.001, 0.001);
		//normal_est.setRadiusSearch(resolution);
		cloud::PointCloudNormalPtrVec pointCloudNormalPtrVec;
		for (int _i = 0; _i < num; ++_i) {
			cloud::PointCloudPtr _temp(new cloud::PointCloud);
			if (isAll) {
				pcl::io::loadPCDFile(separateFiles[_i], *_temp);
				pass.setInputCloud(_temp);
				pass.filter(*_temp);
				cout << "load " << separateFiles[_i] << endl;
			}
			else {
				pcl::io::loadPCDFile(separateFiles[pointCloudIndexVec[_i]], *_temp);
				pass.setInputCloud(_temp);
				pass.filter(*_temp);
				cout << "load " << separateFiles[pointCloudIndexVec[_i]] << endl;
			}
			grid.setInputCloud(_temp);
			grid.filter(*_temp);
			cloud::PointCloudNormalPtr pointNormalPtrTemp(new cloud::PointCloudNormal);
			normal_est.setRadiusSearch(getResolution<cloud::PointT>(_temp)*2.0);
			normal_est.setInputCloud(_temp);
			normal_est.compute(*pointNormalPtrTemp);
			pcl::copyPointCloud(*_temp, *pointNormalPtrTemp);
			pointCloudNormalPtrVec.push_back(pointNormalPtrTemp);
			objectPointCloudPtrVec.push_back(_temp);
		}
		reverse(pointCloudNormalPtrVec.begin(), pointCloudNormalPtrVec.begin()+2);
		visualizePointCloud(pointCloudNormalPtrVec, COLOR_VEC);
		//visualizePointCloud(pointCloudNormalPtrVec, COLOR_VEC);
		cloud::PointCloudNormalPtr resPointCloudNormalPtr(new cloud::PointCloudNormal);

		cloud::PointCloudPtr resPointCloudPtr(new cloud::PointCloud);
		//for (auto _pointPtr : objectPointCloudPtrVec) {
		//	*resPointCloudPtr += *_pointPtr;
		//}
		//grid.setInputCloud(resPointCloudPtr);
		//grid.filter(*resPointCloudPtr);
		//visualizePointCloud({ resPointCloudPtr }, COLOR_VEC);
		//cout << "�����ļ���" << endl;
		//cin >> saveName;
		//pcl::io::savePCDFileBinary<cloud::PointT>(savePath + "\\" + saveName, *resPointCloudPtr);
		//return 0;
		if (isAll) {
			fusePointClouds(pointCloudNormalPtrVec, resPointCloudNormalPtr, 0.002f, -5.0);
			pcl::copyPointCloud(*resPointCloudNormalPtr, *resPointCloudPtr);
			visualizePointCloud({ resPointCloudPtr }, COLOR_VEC);
			int saveBool = 0;
			cout << "���棿(0����1����)" << endl;
			cin >> saveBool;
			if (saveBool == 1) {
				cout << "�����ļ���" << endl;
				cin >> saveName;
				pcl::io::savePCDFileBinary<cloud::PointT>(savePath + "\\" + saveName, *resPointCloudPtr);
			}
		}
		else {
			//fuseTwoPointClouds(objectPointCloudPtrVec[0], objectPointCloudPtrVec[1], 0.003, 0.005, 0.003);
			fusePointClouds(pointCloudNormalPtrVec, resPointCloudNormalPtr, 0.003, 0.005);
			cout << "view" << endl;
			visualizePointCloud(objectPointCloudPtrVec, COLOR_VEC, { 3,3 });
		}
	}
	//�����ؽ�
	else if (mode == 4) {
		string fp("D:\\����ƫ��\\����\\˶ʿ\\TOF\\����\\bunny\\��������640-480\\ƴ��\\fuse.pcd");
		cloud::PointCloudPtr pointCloudPtr(new cloud::PointCloud);
		pcl::io::loadPCDFile(fp, *pointCloudPtr);

		pcl::VoxelGrid<cloud::PointT> grid;
		//grid.setLeafSize(0.001, 0.001, 0.001);
		//grid.setInputCloud(pointCloudPtr);
		//grid.filter(*pointCloudPtr);
		removeInvalid(pointCloudPtr);
		visualizePointCloud(pointCloudPtr);
		pcl::PolygonMesh mesh;
		cout << "�ؽ�" << endl;
		creatMeshWithTri(
			pointCloudPtr, 
			mesh, 
			100, 
			0.5, 
			3.0, 
			M_PI / 18.0, 
			5.0f * M_PI / 6.0, 
			M_PI / 4.0);	
		//creatMeshWithTri(
		//	pointCloudPtr, 
		//	mesh, 
		//	100, 
		//	0.1, 
		//	2.5);
		//creatMeshWithTri(pointCloudPtr, mesh);
		//creatMeshPassion(pointCloudPtr, mesh);
		pcl::io::savePolygonFilePLY("D:\\����ƫ��\\����\\˶ʿ\\TOF\\����\\bunny\\��������640-480\\����\\mesh.ply", mesh, true);
		visualizePointCloud(pointCloudPtr, mesh, RED_COLOR, 1, {0.0f,0.0f,0.0f}, false);
	}
	else if (mode == -1) {
		vector<string> separateFiles;
		//getFiles("D:\\����ƫ��\\����\\˶ʿ\\TOF\\����\\bunny\\��������640-480\\��׼2", separateFiles, "pcd", true);
		getFiles(separatePath, separateFiles, "pcd", true);
		cloud::PointCloudPtrVec objectPointCloudPtrVec;
		if (num == 0 || num > separateFiles.size())num = separateFiles.size();
		//pcl::PassThrough<pcl::PointXYZ> pass;
		//pass.setFilterFieldName("z");
		//pass.setFilterLimits(0.0001, 1.0);
		for (int _i = 0; _i < num; ++_i) {
			cloud::PointCloudPtr _temp(new cloud::PointCloud);
			pcl::io::loadPCDFile(separateFiles[_i], *_temp);
			//pass.setInputCloud(_temp);
			//pass.filter(*_temp);
			cout << "load " << separateFiles[_i] << endl;
			objectPointCloudPtrVec.push_back(_temp);
		}
		visualizePointCloud(objectPointCloudPtrVec, COLOR_VEC);
	}
	else {
		//string fp("D:\\����ƫ��\\����\\˶ʿ\\TOF\\����\\��\\PLY\\dragon_vrip.ply");
		//string saveFp("D:\\����ƫ��\\����\\˶ʿ\\TOF\\����\\��\\PCD\\dragon_vrip.pcd");
		//cloud::PointCloudPtr cloudPtr(new cloud::PointCloud);
		//pcl::io::loadPLYFile(fp, *cloudPtr);
		//pcl::io::savePCDFileBinary(saveFp, *cloudPtr);
		//visualizePointCloud(cloudPtr);
		//return 0;
		//string fp("D:\\����ƫ��\\����\\˶ʿ\\TOF\\����\\��\\PCD\\dragon_vrip.pcd");
		string fp("D:\\����ƫ��\\����\\˶ʿ\\TOF\\����\\ETH\\pcd\\00.pcd");
		cloud::PointCloudPtr cloudPtr(new cloud::PointCloud);
		pcl::io::loadPCDFile(fp, *cloudPtr);
		visualizePointCloud(cloudPtr);

		float resolution = getResolution<cloud::PointT>(cloudPtr);
		pcl::search::KdTree<cloud::PointT> kdtree;
		pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree1(resolution * 0.25);

		kdtree.setInputCloud(cloudPtr);
		octree1.setInputCloud(cloudPtr);
		octree1.addPointsFromInputCloud();

		pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree2(resolution);
		//octree2.setTreeDepth(4);
		octree2.setInputCloud(cloudPtr);
		octree2.addPointsFromInputCloud();

		pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree3(resolution * 4.0);
		//octree3.setTreeDepth(octree1.getTreeDepth() + 2);
		octree3.setInputCloud(cloudPtr);
		octree3.addPointsFromInputCloud();

		pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree4(resolution * 16.0);
		//octree4.setTreeDepth(octree1.getTreeDepth() + 2);
		octree4.setInputCloud(cloudPtr);
		octree4.addPointsFromInputCloud();

		cout << "octree depth=" << octree1.getTreeDepth() << " resolution=" << octree1.getResolution() << " VoxelSquaredDiameter=" << octree1.getVoxelSquaredDiameter() << endl;
		cout << "octree depth=" << octree2.getTreeDepth() << " resolution=" << octree2.getResolution() << " VoxelSquaredDiameter=" << octree2.getVoxelSquaredDiameter() << endl;
		cout << "octree depth=" << octree3.getTreeDepth() << " resolution=" << octree3.getResolution() << " VoxelSquaredDiameter=" << octree3.getVoxelSquaredDiameter() << endl;
		cout << "octree depth=" << octree4.getTreeDepth() << " resolution=" << octree4.getResolution() << " VoxelSquaredDiameter=" << octree4.getVoxelSquaredDiameter() << endl;


		std::vector<int> pointIdxNKNSearch;
		std::vector<float> pointNKNSquaredDistance;

		cout << "��ѯ����" << endl;
		int searchNum = 100;
		vector<int> searchIndex(searchNum);
		for (int index = 0; index < searchNum; ++index) {
			searchIndex[index] = (float)cloudPtr->size() * rand() / (RAND_MAX + 1.0f);
			cout << searchIndex[index] << "\t";
		}
		cout << "===========================" << endl;
		cout << "���ҵ���: " << searchIndex.size() << endl;
		cout << "===========================" << endl;

		int startTime, endTime;
		float useTime;
		vector<double> kdtreeTime;
		vector<double> octree1Time;
		vector<double> octree2Time;
		vector<double> octree3Time;
		vector<double> octree4Time;
		vector<double> kVec;
		kVec.push_back(1);
		for (int i = 0; i < 20; ++i) {
			kVec.push_back((i + 1)*10);
		}
		for (auto _k : kVec) {
			cout << "k=" << _k << " max=" << kVec.back()<<endl;
			startTime = clock();
			for (auto index : searchIndex) {
				kdtree.nearestKSearch((*cloudPtr)[index], _k, pointIdxNKNSearch, pointNKNSquaredDistance);
			}
			endTime = clock();
			useTime = (double)(endTime - startTime) / CLOCKS_PER_SEC;
			kdtreeTime.push_back(useTime);

			startTime = clock();
			for (auto index : searchIndex) {
				octree1.nearestKSearch((*cloudPtr)[index], _k, pointIdxNKNSearch, pointNKNSquaredDistance);
			}
			endTime = clock();
			useTime = (double)(endTime - startTime) / CLOCKS_PER_SEC;
			octree1Time.push_back(useTime);

			startTime = clock();
			for (auto index : searchIndex) {
				octree2.nearestKSearch((*cloudPtr)[index], _k, pointIdxNKNSearch, pointNKNSquaredDistance);
			}
			endTime = clock();
			useTime = (double)(endTime - startTime) / CLOCKS_PER_SEC;
			octree2Time.push_back(useTime);

			startTime = clock();
			for (auto index : searchIndex) {
				octree3.nearestKSearch((*cloudPtr)[index], _k, pointIdxNKNSearch, pointNKNSquaredDistance);
			}
			endTime = clock();
			useTime = (double)(endTime - startTime) / CLOCKS_PER_SEC;
			octree3Time.push_back(useTime);

			startTime = clock();
			for (auto index : searchIndex) {
				octree4.nearestKSearch((*cloudPtr)[index], _k, pointIdxNKNSearch, pointNKNSquaredDistance);
			}
			endTime = clock();
			useTime = (double)(endTime - startTime) / CLOCKS_PER_SEC;
			octree4Time.push_back(useTime);
		}

		pcl::visualization::PCLPlotter plotter("k search");
		plotter.setShowLegend(true);
		plotter.setXTitle("k");
		plotter.setYTitle("Time/s");
		plotter.addPlotData(kVec, kdtreeTime, "kd-tree", vtkChart::LINE);
		plotter.spinOnce(2000);
		plotter.addPlotData(kVec, octree1Time, "octree1", vtkChart::LINE);
		plotter.spinOnce(2000);
		plotter.addPlotData(kVec, octree2Time, "octree2", vtkChart::LINE);
		plotter.spinOnce(2000);
		plotter.addPlotData(kVec, octree3Time, "octree3", vtkChart::LINE);
		plotter.spinOnce(2000);
		plotter.addPlotData(kVec, octree4Time, "octree4", vtkChart::LINE);
		plotter.spinOnce(2000);
		plotter.plot();
		
		kdtreeTime.clear();
		octree1Time.clear();
		octree2Time.clear();
		octree3Time.clear();
		octree4Time.clear();
		vector<double> rVec;
		for (int i = 0; i < 50; ++i) {
			rVec.push_back(resolution* i*0.5+ resolution);
		}
		for (auto _r : rVec) {
			cout << "k=" << _r << " max=" << rVec.back() << endl;
			startTime = clock();
			for (auto index : searchIndex) {
				kdtree.radiusSearch((*cloudPtr)[index], _r, pointIdxNKNSearch, pointNKNSquaredDistance);
			}
			endTime = clock();
			useTime = (double)(endTime - startTime) / CLOCKS_PER_SEC;
			kdtreeTime.push_back(useTime);

			startTime = clock();
			for (auto index : searchIndex) {
				octree1.radiusSearch((*cloudPtr)[index], _r, pointIdxNKNSearch, pointNKNSquaredDistance);
			}
			endTime = clock();
			useTime = (double)(endTime - startTime) / CLOCKS_PER_SEC;
			octree1Time.push_back(useTime);

			startTime = clock();
			for (auto index : searchIndex) {
				octree2.radiusSearch((*cloudPtr)[index], _r, pointIdxNKNSearch, pointNKNSquaredDistance);
			}
			endTime = clock();
			useTime = (double)(endTime - startTime) / CLOCKS_PER_SEC;
			octree2Time.push_back(useTime);

			startTime = clock();
			for (auto index : searchIndex) {
				octree3.radiusSearch((*cloudPtr)[index], _r, pointIdxNKNSearch, pointNKNSquaredDistance);
			}
			endTime = clock();
			useTime = (double)(endTime - startTime) / CLOCKS_PER_SEC;
			octree3Time.push_back(useTime);

			startTime = clock();
			for (auto index : searchIndex) {
				octree4.radiusSearch((*cloudPtr)[index], _r, pointIdxNKNSearch, pointNKNSquaredDistance);
			}
			endTime = clock();
			useTime = (double)(endTime - startTime) / CLOCKS_PER_SEC;
			octree4Time.push_back(useTime);
		}
		pcl::visualization::PCLPlotter plotter2("r search");
		plotter2.setShowLegend(true);
		plotter2.setXTitle("r/mm");
		plotter2.setYTitle("Time/s");
		plotter2.addPlotData(rVec, kdtreeTime, "kd-tree", vtkChart::LINE);
		plotter2.spinOnce(2000);
		plotter2.addPlotData(rVec, octree1Time, "octree1", vtkChart::LINE);
		plotter2.spinOnce(2000);
		plotter2.addPlotData(rVec, octree2Time, "octree2", vtkChart::LINE);
		plotter2.spinOnce(2000);
		plotter2.addPlotData(rVec, octree3Time, "octree3", vtkChart::LINE);
		plotter2.spinOnce(2000);
		plotter2.addPlotData(rVec, octree4Time, "octree4", vtkChart::LINE);
		plotter2.spinOnce(2000);
		plotter2.plot();
	}
	//	vector<int> kVec({1,10,100,1000});
	//	for (auto _k : kVec) {
	//		startTime = clock();
	//		for (auto index : searchIndex) {
	//			kdtree.nearestKSearch((*cloudPtr)[index], _k, pointIdxNKNSearch, pointNKNSquaredDistance);
	//		}
	//		endTime = clock();
	//		useTime = (double)(endTime - startTime) / CLOCKS_PER_SEC;
	//		cout << "kdtree k=" << _k << " ��ʱ��" << useTime << endl;

	//		startTime = clock();
	//		for (auto index : searchIndex) {
	//			octree1.nearestKSearch((*cloudPtr)[index], _k, pointIdxNKNSearch, pointNKNSquaredDistance);
	//		}
	//		endTime = clock();
	//		useTime = (double)(endTime - startTime) / CLOCKS_PER_SEC;
	//		cout << "octree1 k=" << _k << " ��ʱ��" << useTime << endl;

	//		startTime = clock();
	//		for (auto index : searchIndex) {
	//			octree2.nearestKSearch((*cloudPtr)[index], _k, pointIdxNKNSearch, pointNKNSquaredDistance);
	//		}
	//		endTime = clock();
	//		useTime = (double)(endTime - startTime) / CLOCKS_PER_SEC;
	//		cout << "octree2 k=" << _k << " ��ʱ��" << useTime << endl;

	//		startTime = clock();
	//		for (auto index : searchIndex) {
	//			octree3.nearestKSearch((*cloudPtr)[index], _k, pointIdxNKNSearch, pointNKNSquaredDistance);
	//		}
	//		endTime = clock();
	//		useTime = (double)(endTime - startTime) / CLOCKS_PER_SEC;
	//		cout << "octree3 k=" << _k << " ��ʱ��" << useTime << endl;

	//		startTime = clock();
	//		for (auto index : searchIndex) {
	//			octree4.nearestKSearch((*cloudPtr)[index], _k, pointIdxNKNSearch, pointNKNSquaredDistance);
	//		}
	//		endTime = clock();
	//		useTime = (double)(endTime - startTime) / CLOCKS_PER_SEC;
	//		cout << "octree4 k=" << _k << " ��ʱ��" << useTime << endl;

	//		cout << "===========================" << endl;
	//	}
	//	cout << "%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;

	//	vector<float> rVec;
	//	rVec.push_back(resolution);
	//	rVec.push_back(resolution * 5);
	//	rVec.push_back(resolution * 10);
	//	rVec.push_back(resolution * 20);
	//	for (auto _r:rVec) {
	//		startTime = clock();
	//		for (auto index : searchIndex) {
	//			kdtree.radiusSearch((*cloudPtr)[index], _r, pointIdxNKNSearch, pointNKNSquaredDistance);
	//		}
	//		endTime = clock();
	//		useTime = (double)(endTime - startTime) / CLOCKS_PER_SEC;
	//		cout << "kdtree r=" << _r << " ��ʱ��" << useTime << endl;

	//		startTime = clock();
	//		for (auto index : searchIndex) {
	//			octree1.radiusSearch((*cloudPtr)[index], _r, pointIdxNKNSearch, pointNKNSquaredDistance);
	//		}
	//		endTime = clock();
	//		useTime = (double)(endTime - startTime) / CLOCKS_PER_SEC;
	//		cout << "octree1 r=" << _r << " ��ʱ��" << useTime << endl;

	//		startTime = clock();
	//		for (auto index : searchIndex) {
	//			octree2.radiusSearch((*cloudPtr)[index], _r, pointIdxNKNSearch, pointNKNSquaredDistance);
	//		}
	//		endTime = clock();
	//		useTime = (double)(endTime - startTime) / CLOCKS_PER_SEC;
	//		cout << "octree2 r=" << _r << " ��ʱ��" << useTime << endl;

	//		startTime = clock();
	//		for (auto index : searchIndex) {
	//			octree3.radiusSearch((*cloudPtr)[index], _r, pointIdxNKNSearch, pointNKNSquaredDistance);
	//		}
	//		endTime = clock();
	//		useTime = (double)(endTime - startTime) / CLOCKS_PER_SEC;
	//		cout << "octree3 r=" << _r << " ��ʱ��" << useTime << endl;

	//		startTime = clock();
	//		for (auto index : searchIndex) {
	//			octree4.radiusSearch((*cloudPtr)[index], _r, pointIdxNKNSearch, pointNKNSquaredDistance);
	//		}
	//		endTime = clock();
	//		useTime = (double)(endTime - startTime) / CLOCKS_PER_SEC;
	//		cout << "octree4 r=" << _r << " ��ʱ��" << useTime << endl;
	//		
	//		cout << "===========================" << endl;
	//	}
	//}
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

	//��ȡͼ����ʾ
	PsFrameReady frameReady = { 0 };
	PsFrame depthFrame = { 0 }, RGBFrame = { 0 };
	cv::Mat imageMat;
	const string depthImageWinName = "���ͼ��";
	const string RGBImageWinName = "RGBͼ��";
	const string depthImageWDRWiNane1 = "���ͼ��WDR1";
	const string depthImageWDRWiNane2 = "���ͼ��WDR2";
	const string depthImageWDRWiNane3 = "���ͼ��WDR3";
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

		//��ʾ���ͼ��
		if (frameReady.depth == 1) {
			if (Ps2_GetFrame(deviceHandle, sessionIndex, PsFrameType::PsDepthFrame, &depthFrame) != PsReturnStatus::PsRetOK) {
				std::cout << "GetDepthFrame failed!" << endl;
			}
			else {
				//ת��Ϊ����
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
				//���ƿ��ӻ�
				if (showCloud) {					
					viewer.showCloud(cloud, "cloud");
					//showCloud = false;
					//�������Ϊpcd�ļ�
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
				//WDRģʽ
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
				//��WDRģʽ
				else {
					Opencv_Depth(slope.slope1, depthFrame.height, depthFrame.width, depthFrame.pFrameData, imageMat);
					cv::imshow(depthImageWinName, imageMat);
				}
			}
		}
		//��ʾRGBͼ��
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
	//�رմ���
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


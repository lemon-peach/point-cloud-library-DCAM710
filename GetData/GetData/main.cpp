#include "camera.h"
#include <fstream>
#include <io.h>
#include <pcl/visualization/pcl_visualizer.h>
namespace cloud {
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

	//using PointNormalPfhT = CustomPointT;
	////using PointNormalPfhT = PointNormalPfhTp;
	//using PointNormalPfh = pcl::PointCloud<PointNormalPfhT>;
	//using PointNormalPfhPtr = PointNormalPfh::Ptr;
	//using PointNormalPfhPtrVec = vector<PointNormalPfhPtr, Eigen::aligned_allocator<PointNormalPfhT>>;

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
cloud::Color redPoint = { 255.0, 0.0, 0.0 };
cloud::Color greenPoint = { 0.0, 255.0, 0.0 };
cloud::Color bluePoint = { 0.0, 0.0, 255.0 };
cloud::Color yellowPoint = { 255.0, 255.0, 0.0 };
cloud::Color purplePoint = { 255.0, 0.0, 255.0 };
cloud::Color cyanPoint = { 0.0, 255.0, 255.0 };
cloud::Color whitePoint = { 255.0, 255.0, 255.0 };
cloud::Color color1 = { 200,50,0 };
cloud::Color color2 = { 150,100,0 };
cloud::Color color3 = { 100,150,0 };
cloud::Color color4 = { 50,200,0 };
cloud::Color color5 = { 0,200,50 };
cloud::Color color6 = { 0,150,100 };
cloud::Color color7 = { 0,100,150 };
cloud::Color color8 = { 0,50,200 };
vector<cloud::Color> colorVec = { redPoint, greenPoint, bluePoint, yellowPoint, purplePoint, cyanPoint,
									color1,color2,color3,color4,color5,color6,color7,color8 };
vector<cloud::Color> colorWhiteVec = { whitePoint };

pcl::visualization::PCLVisualizer::Ptr
visualizePointCloud(
	const cloud::PointCloudPtrVec& pointCloudPtrVec,
	vector<cloud::Color>& pointColorVec = colorVec,
	vector<int> pointSizeVec = {1},
	const cloud::Color& backgroundColor = {0.0f,0.0f,0.0f},
	bool showCoordinateSystem=true,
	bool is_auto = true);

int mode = 1;


int main(int argc, char* argv[]) {
	if (mode == 0) {
		//相机打开
		string fp("D:\\剑走偏锋\\毕设\\硕士\\TOF\\数据\\自建数据\\原始PCD");
		//cout << "Please input save path(0 for default)" << endl;
		//cout << "\\ for split" << endl;
		//cin >> fp;
		//if (fp == "0") fp = ".\\PCD";

		_finddata_t fileInfo;
		intptr_t handle;
		handle = _findfirst(fp.c_str(), &fileInfo);
		if (handle == -1) {
			cout << "Dir " << fp << " does not exit" << endl;
			return 0;
		}
		_findclose(handle);

		fp.append("\\");
		PsDeviceHandle deviceHandle;
		uint32_t sessionIndex = 0;
		PsWDROutputMode wdrMode = { PsWDRTotalRange_Two, PsNearRange, 1, PsFarRange, 1, PsUnknown, 1 };
		PsDataMode setDataMode = PsDataMode::PsDepthAndRGB_30;
		PsDataMode dataMode = setDataMode;
		SlopeT slope;
		//打开相机，设置模式
		openAndConfigCamera(deviceHandle, sessionIndex, &wdrMode, dataMode, slope);
		//获取点云数据
		saveRealTimePointClouds(deviceHandle, sessionIndex, slope, fp, 15, 1000, true, true, false);
		//关闭相机
		closeCamera(deviceHandle, sessionIndex);
	}
	else {
		string pcdPath("D:\\剑走偏锋\\毕设\\硕士\\TOF\\数据\\bunny\\程序生成640-480\\原始PCD");
		string pngPath("D:\\剑走偏锋\\毕设\\硕士\\Blender\\bunny_seq_640_480");
		vector<string> files;
		getFiles(pngPath, files, "png", true);
		cv::Mat image;
		float d = 0.036f / 480.0f;
		float f = 0.05f;
		cloud::PointCloudPtrVec cloudPtrVec;
		for (int _i = 0; _i < files.size(); ++_i) {
			cloud::PointCloudPtr pointCloudPtr(new cloud::PointCloud);
			cout << "converting " << _i + 1 << "/" << files.size() << endl;
			image = cv::imread(files[_i], cv::IMREAD_UNCHANGED);
			depthToPointCloud(image, pointCloudPtr, f, d, d);
			string _path(pcdPath);
			string _fileName("0000.pcd");
			string _num = to_string(_i);
			_fileName.replace(4 - _num.size(), _num.size(), _num);
			_path.append("\\");
			_path.append(_fileName);
			//pcl::io::savePCDFileASCII(_path, *pointCloudPtr);
			pcl::io::savePCDFileBinary(_path, *pointCloudPtr);
			cloudPtrVec.push_back(pointCloudPtr);
		}
		visualizePointCloud(cloudPtrVec);
		//cv::waitKey();
		
		cout << "close";
	}

	return 0;
}

pcl::visualization::PCLVisualizer::Ptr
visualizePointCloud(
	const cloud::PointCloudPtrVec& pointCloudPtrVec,
	vector<cloud::Color>& pointColorVec,
	vector<int> pointSizeVec,
	const cloud::Color& backgroundColor,
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
				*((int*)args) = 0;
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
	vector<pcl::PolygonMesh> meshVec = {};
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
		while ((!pViewer->wasStopped()) && stopFlag) {
			viewer.spinOnce(20);
		}
	}
	return pViewer;
}


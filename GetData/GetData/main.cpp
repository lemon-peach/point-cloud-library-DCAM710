#include "camera.h"
#include <fstream>
#include <io.h>

int mode = 1;

int main(int argc, char* argv[]) {
	if (mode == 0) {
		//�����
		string fp("D:\\����ƫ��\\����\\˶ʿ\\TOF\\����\\�Խ�����\\ԭʼPCD");
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
		//�����������ģʽ
		openAndConfigCamera(deviceHandle, sessionIndex, &wdrMode, dataMode, slope);
		//��ȡ��������
		saveRealTimePointClouds(deviceHandle, sessionIndex, slope, fp, 15, 1000, true, true, false);
		//�ر����
		closeCamera(deviceHandle, sessionIndex);
	}
	else {
		string pcdPath("D:\\����ƫ��\\����\\˶ʿ\\TOF\\����\\������������");
		string pngPath("D:\\����ƫ��\\����\\˶ʿ\\Blender\\sequence");
		vector<string> files;
		getFiles(pngPath, files, "png", true);
		cv::Mat image;
		pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
		float d = 0.036f / 1080.0f;
		float f = 0.05f;
		
		pcl::visualization::CloudViewer viewer("Cloud Viewer");
		for (int _i = 0; _i < files.size(); ++_i) {
			image = cv::imread(files[_i], cv::IMREAD_UNCHANGED);
			depthToPointCloud(image, pointCloudPtr, f, d, d);
			string _path(pcdPath);
			string _fileName("0000.pcd");
			string _num = to_string(_i);
			_fileName.replace(4 - _num.size(), _num.size(), _num);
			_path.append("\\");
			_path.append(_fileName);
			pcl::io::savePCDFileASCII(_path, *pointCloudPtr);
		}
		//cv::waitKey();

		viewer.showCloud(pointCloudPtr);
		cv::waitKey();
	}

	return 0;
}
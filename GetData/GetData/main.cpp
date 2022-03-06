#include "camera.h"
#include <fstream>
#include <io.h>

int main(int argc, char* argv[]) {
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
	return 0;
}
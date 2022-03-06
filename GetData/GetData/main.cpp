#include "camera.h"
#include <fstream>
#include <io.h>

int main(int argc, char* argv[]) {
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
	return 0;
}
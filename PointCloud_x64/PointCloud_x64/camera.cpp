#include "camera.h"
using namespace std;

/**
* @breif		�����ͼ����ӳ��
* @param[in]	slope
* @param[in]	height�����ͼ�ĸ�
* @param[in]	width�����ͼ�Ŀ�
* @param[in]	pData��Vzense���ص����֡�ṹ��ָ��
* @return		dispImg��ӳ���Ľ��ͼ
*/
void Opencv_Depth(uint32_t slope, int height, int width, uint8_t* pData, cv::Mat& dispImg)
{
	dispImg = cv::Mat(height, width, CV_16UC1, pData);
	cv::Point2d pointxy(width / 2, height / 2);
	int val = dispImg.at<ushort>(pointxy);
	char text[20];
#ifdef _WIN32
	sprintf_s(text, "%d", val);
#else
	snprintf(text, sizeof(text), "%d", val);
#endif
	dispImg.convertTo(dispImg, CV_8U, 255.0 / slope);
	applyColorMap(dispImg, dispImg, cv::COLORMAP_RAINBOW);
	int color;
	if (val > 2500)
		color = 0;
	else
		color = 4096;
	circle(dispImg, pointxy, 4, cv::Scalar(color, color, color), -1, 8, 0);
	putText(dispImg, text, pointxy, cv::FONT_HERSHEY_DUPLEX, 2, cv::Scalar(color, color, color));
}

/**
* @breif		��ȡslope
* @param[in]	depthRange��
* @param[in]	measuringRange��
* @return		slope
*/
uint32_t getSlope(PsDepthRange depthRange, PsMeasuringRange measuringRange) {
	switch (depthRange)
	{
	case PsNearRange:
	case PsXNearRange:
	case PsXXNearRange:
		return measuringRange.effectDepthMaxNear;
		break;

	case PsMidRange:
	case PsXMidRange:
	case PsXXMidRange:
		return measuringRange.effectDepthMaxMid;
		break;

	case PsFarRange:
	case PsXFarRange:
	case PsXXFarRange:
		return measuringRange.effectDepthMaxFar;
		break;
	default:
		break;
	}
	return -1;
}

/**
* @breif			�򿪲�����Vzense���
* @param[out]		deviceHandle���豸���
* @param[in]		sessionIndex���Ự��ʶ
* @param[in][out]	wdrMode��
* @param[in][out]	setDataMode����������ģʽ���������ú����������ģʽ(����ʧ��)
* @param[out]		slope��
* @return			
*/
int openAndConfigCamera(PsDeviceHandle& deviceHandle, uint32_t sessionIndex, PsWDROutputMode* pWdrMode, PsDataMode& setDataMode, SlopeT& slope) {
	PsReturnStatus status = PsReturnStatus::PsRetOK;
	uint32_t deviceIndex = 0;
	uint32_t deviceCount = 0;  //���ӵ��豸����
	string setDataModeStr = "NOT WDR";
	if (setDataMode == PsDataMode::PsWDR_Depth) {
		setDataModeStr = "WDR";
	}

	//��ʼ��
	status = Ps2_Initialize();
	if (status != PsReturnStatus::PsRetOK) {
		cout << "Initialize failed!" << endl;
		return status;
	}
	cout << "Initialize success" << endl;

	//��ȡ�豸����
	status = Ps2_GetDeviceCount(&deviceCount);
	if (status != PsReturnStatus::PsRetOK) {
		cout << "GetDeviceCount failed!" << endl;
		return status;
	}
	if (deviceCount == 0) {
		//���豸����
		cout << "None camera connecting!" << endl;
		return 1;
	}
	cout << "Camera count: " << deviceCount << endl;

	//��ȡ��0���豸��Ϣ
	PsDeviceInfo DeviceListInfo = { 0 };
	status = Ps2_GetDeviceInfo(&DeviceListInfo, 0);
	if (status != PsReturnStatus::PsRetOK) {
		cout << "GetDeviceInfo failed!" << endl;
		return status;
	}
	std::cout << "uri: " << DeviceListInfo.uri << endl;

	//���豸
	status = Ps2_OpenDevice(DeviceListInfo.uri, &deviceHandle);
	if (status != PsReturnStatus::PsRetOK) {
		cout << "OpenDevice failed!" << endl;
		return status;
	}
	cout << "OpenDevice success" << endl;

	//����
	status = Ps2_StartStream(deviceHandle, sessionIndex);
	if (status != PsReturnStatus::PsRetOK) {
		cout << "StartStream failed!" << endl;
		return status;
	}
	cout << "StartStream success" << endl;

	//WDRģʽ��Ҫ����Ps2_SetWDROutputMode()
	bool b_WDRMode = false;
	if (setDataMode == PsDataMode::PsWDR_Depth) {
		status = Ps2_SetWDROutputMode(deviceHandle, sessionIndex, pWdrMode);
		if (status == PsReturnStatus::PsRetOK) {
			std::cout << "Set WDR output mode" << endl;
			b_WDRMode = true;
		}
		else {
			std::cout << "Set WDR output mode failed!!!" << endl;
			b_WDRMode = false;
		}
	}

	//��ȡ�������ģʽ
	PsDataMode dataMode = PsDataMode::PsDepthAndRGB_30;
	Ps2_GetDataMode(deviceHandle, sessionIndex, &dataMode);

	//�����������ģʽ
	if (b_WDRMode || setDataMode != PsDataMode::PsWDR_Depth) {
		status = Ps2_SetDataMode(deviceHandle, sessionIndex, setDataMode);
		if (status == PsReturnStatus::PsRetOK) {
			std::cout << "set dataMode to " << setDataModeStr << endl;
			dataMode = setDataMode;
		}
		else {
			std::cout << "set dataMode failed!!!" << endl;
		}
	}
	setDataMode = dataMode;

	//��ȡ������Χ
	PsDepthRange depth_Range;
	PsMeasuringRange measuringRange;
	//WDRģʽ
	if (dataMode == PsDataMode::PsWDR_Depth) {
		status = Ps2_GetWDROutputMode(deviceHandle, sessionIndex, pWdrMode);
		if (status != PsReturnStatus::PsRetOK) {
			cout << "GetWDROutputMode failed" << endl;
		}

		status = Ps2_GetMeasuringRange(deviceHandle, sessionIndex, pWdrMode->range1, &measuringRange);
		if (status != PsReturnStatus::PsRetOK) {
			cout << "GetMeasuringRange1 failed" << endl;
		}
		slope.slope1 = getSlope(pWdrMode->range1, measuringRange);

		status = Ps2_GetMeasuringRange(deviceHandle, sessionIndex, pWdrMode->range2, &measuringRange);
		if (status != PsReturnStatus::PsRetOK) {
			cout << "GetMeasuringRange2 failed" << endl;
		}
		slope.slope2 = getSlope(pWdrMode->range2, measuringRange);

		slope.range_num = 2;
		if (pWdrMode->totalRange == 3) {
			status = Ps2_GetMeasuringRange(deviceHandle, sessionIndex, pWdrMode->range3, &measuringRange);
			if (status != PsReturnStatus::PsRetOK) {
				cout << "GetMeasuringRange3 failed" << endl;
			}
			slope.slope3 = getSlope(pWdrMode->range3, measuringRange);
			slope.range_num = 3;
		}
	}
	//��WDRģʽ
	else {
		status = Ps2_GetDepthRange(deviceHandle, sessionIndex, &depth_Range);
		if (status != PsReturnStatus::PsRetOK) {
			cout << "GetDepthRange failed!" << endl;
		}

		status = Ps2_GetMeasuringRange(deviceHandle, sessionIndex, depth_Range, &measuringRange);
		if (status != PsReturnStatus::PsRetOK) {
			cout << "GetMeasuringRange failed!" << endl;
		}
		slope.slope1 = getSlope(depth_Range, measuringRange);
		slope.range_num = 1;
	}
	Ps2_SetDepthDistortionCorrectionEnabled(deviceHandle, sessionIndex, false);
	return PsReturnStatus::PsRetOK;
}

/**
 * @brief		�ر����
 * @param[in]	deviceHandle 
 * @param[out]	sessionIndex 
 * @return		status��״̬��
*/
int closeCamera(PsDeviceHandle deviceHandle, uint32_t sessionIndex) {
	PsReturnStatus status = PsReturnStatus::PsRetOK;

	status = Ps2_StopStream(deviceHandle, sessionIndex);
	if (status != PsReturnStatus::PsRetOK) {
		cout << "StopStream failed!" << endl;
		return status;
	}
	cout << "StopStream" << endl;

	status = Ps2_CloseDevice(&deviceHandle);
	if (status != PsReturnStatus::PsRetOK) {
		cout << "CloseDevice failed!" << endl;
		return status;
	}
	cout << "CloseDevice" << endl;

	status = Ps2_Shutdown();
	if (status != PsReturnStatus::PsRetOK) {
		cout << "Shutdown failed!" << endl;
		return status;
	}
	std::cout << "Shutdown" << endl;
	return status;
}

/**
 * @brief
 * @param[in]	deviceHandle	�豸���
 * @param[in]	sessionIndex	�Ự��ʶ
 * @param[in]	slope			����
 * @param[out]	pointCloudVec	��������
 * @param[in]	interval		������fps��auto_updateΪtrueʱ��Ч
 * @param[in]	maxNum			��ౣ��֡��
 * @param[in]	showDepthImage	��ʾ���ͼ��
 * @param[in]	showRGBImage	��ʾRGBͼ��
 * @param[in]	auto_update		true���Զ����棻false���ֶ�ѡ�񱣴���һ֡
 * @return
*/
int
getRealTimePointClouds(
	PsDeviceHandle deviceHandle,
	uint32_t sessionIndex,
	SlopeT slope,
	vector<pcl::PointCloud<pcl::PointXYZ>::Ptr, Eigen::aligned_allocator<pcl::PointXYZ>>& pointCloudVec,
	uint32_t interval,
	uint32_t maxNum,
	bool showDepthImage,
	bool showRGBImage,
	bool auto_update)
{
	bool saveStart = false; //��ʼ�洢�ı�־
	PsFrameReady frameReady = { 0 };
	PsFrame RGBFrame = { 0 };
	size_t Index = 0;
	cv::Mat imageMat;
	vector<PsVector3f*> worldVec;
	int key = -1;
	int len = 0;
	int waitKeyTime = 10;
	pcl::visualization::CloudViewer viewer("Cloud Viewer");

	showDepthImage = showDepthImage && (slope.range_num > 0);

	cout << "Press s to start" << endl;
	cout << "Press esc to stop" << endl;

	//��ȡ�������
	while (true) {
		key = cv::waitKey(waitKeyTime);
		if (key == 27)break;
		else if (key == 's') saveStart = true;
		if (saveStart)++Index;
		//��������Ƿ���Զ�ȡͼ��
		if (Ps2_ReadNextFrame(deviceHandle, sessionIndex, &frameReady) != PsReturnStatus::PsRetOK) {
			cout << "ReadNextFrame failed" << endl;
			continue;
		}
		//���ͼ���Ƿ���Զ�ȡ
		if (frameReady.depth == 1) {
			PsFrame depthFrame = { 0 };
			//��ȡ���ͼ��ʧ��
			if (Ps2_GetFrame(deviceHandle, sessionIndex, PsFrameType::PsDepthFrame, &depthFrame) != PsReturnStatus::PsRetOK) {				
				cout << "GetDepthFrame failed!" << endl;
			}
			//��ȡ���ͼ��ɹ�
			else {
				//��ʾ���ͼ��
				if (showDepthImage) {
					Opencv_Depth(slope.slope1, depthFrame.height, depthFrame.width, depthFrame.pFrameData, imageMat);
					cv::imshow("ʵʱ���ͼ��", imageMat);
				}
				//��ʼ����
				if (saveStart && (Index % interval == 1 || !auto_update)) {
					len = depthFrame.width * depthFrame.height;
					PsVector3f* pWorld = new PsVector3f[len];
					Ps2_ConvertDepthFrameToWorldVector(deviceHandle, sessionIndex, depthFrame, pWorld);
					pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZ>);
					pointCloud->width = depthFrame.width;
					pointCloud->height = depthFrame.height;
					pointCloud->resize(len);
					for (int i = 0; i < len; i++) {
						pointCloud->points[i].x = pWorld[i].x / 1000;
						pointCloud->points[i].y = pWorld[i].y / 1000;
						pointCloud->points[i].z = pWorld[i].z / 1000;
					}
					if (!auto_update) {
						viewer.showCloud(pointCloud);
						if (cv::waitKey() == 's') pointCloudVec.push_back(pointCloud);
						cout << "\rcatch " << Index << " frame";
						saveStart = !saveStart;
					}
					else {
						pointCloudVec.push_back(pointCloud);
						cout << "\rcatch " << (Index - 1) / interval + 1 << " frame";
					}
					//worldVec.push_back(pWorld);
					//if (auto_update){
					//	cout << "\rcatch " << (Index - 1) / interval + 1 << " frame";
					//}
					//else {
					//	saveStart = auto_update;
					//	cout << "\rcatch " << Index << " frame";
					//}
				}
			}
		}
		//��ʾRGBͼ��
		if (frameReady.rgb == 1 && showRGBImage) {
			if (Ps2_GetFrame(deviceHandle, sessionIndex, PsFrameType::PsRGBFrame, &RGBFrame) != PsReturnStatus::PsRetOK) {
				cout << "GetRGBFrame failed!" << endl;
			}
			else {
				imageMat = cv::Mat(RGBFrame.height, RGBFrame.width, CV_8UC3, RGBFrame.pFrameData);
				cv::imshow("RGBͼ��", imageMat);
			}
		}
		if (Index / interval + 1 > maxNum) break; //������󱣴���Ŀʱ�˳�
	}

	//�������ת��ΪPCL��������
	int index = 0;
	int num = worldVec.size();
	//for (auto& w : worldVec) {
	//	pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZ>);
	//	pointCloud->width = len;
	//	pointCloud->height = 1;
	//	pointCloud->resize(len);
	//	for (int i = 0; i < len; i++) {
	//		pointCloud->points[i].x = w[i].x / 1000;
	//		pointCloud->points[i].y = w[i].y / 1000;
	//		pointCloud->points[i].z = w[i].z / 1000;
	//	}
	//	pointCloudVec.push_back(pointCloud);
	//	cout << "\rtransform to point cloud: " << fixed << setprecision(0) << (((float)++index) / (float)num) * 100.0 << "%";
	//}

	//�ͷ�ָ��
	for (auto& w : worldVec) {
		delete[] w;
		w = NULL;
	}

	//�رմ���
	cv::destroyAllWindows();

	return 0;
}

/**
 * @brief
 * @param[in]	deviceHandle		�豸���
 * @param[in]	sessionIndex		�Ự��ʶ
 * @param[in]	slope				����
 * @param[in]	fp					����·��
 * @param[in]	interval			������fps��auto_saveΪtrueʱ��Ч
 * @param[in]	maxNum				��ౣ��֡��
 * @param[in]	showDepthImage		��ʾ���ͼ��
 * @param[in]	showRGBImage		��ʾRGBͼ��
 * @param[in]	auto_save			true���Զ����棻false���ֶ�ѡ�񱣴���һ֡
 * @return
*/
int saveRealTimePointClouds(
	PsDeviceHandle deviceHandle,
	uint32_t sessionIndex,
	SlopeT slope,
	string fp,
	uint32_t interval,
	uint32_t maxNum,
	bool showDepthImage,
	bool showRGBImage,
	bool auto_save) 
{
	//��ȡ���Ʋ�����������
	vector<pcl::PointCloud<pcl::PointXYZ>::Ptr, Eigen::aligned_allocator<pcl::PointXYZ>> pointCloudVec;
	getRealTimePointClouds(deviceHandle, sessionIndex, slope, pointCloudVec, interval, maxNum, showDepthImage, showRGBImage, auto_save);
	if (pointCloudVec.size() == 0) return -1; //Ϊ��ȡ����������
	

	//����������ݵ�pcd�ļ�
	uint32_t index = 0;
	for (auto& pointcloud : pointCloudVec) {
		string wholeFp(fp);
		wholeFp.append(to_string(index).append(".pcd"));
		pcl::io::savePCDFileBinary(wholeFp, *pointcloud);
		cout << "\rsave point cloud: " << fixed << setprecision(0) << (((float)++index) / (float)pointCloudVec.size()) * 100.0 << "%";
	}
	cout << endl;
	return 0;
}

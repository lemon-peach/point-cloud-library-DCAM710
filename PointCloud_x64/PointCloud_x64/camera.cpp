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
	//int color;
	//if (val > 2500)
	//	color = 0;
	//else
	//	color = 4096;
	//circle(dispImg, pointxy, 4, cv::Scalar(color, color, color), -1, 8, 0);
	//putText(dispImg, text, pointxy, cv::FONT_HERSHEY_DUPLEX, 2, cv::Scalar(color, color, color));
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

	bool jibian;
	Ps2_SetDepthDistortionCorrectionEnabled(deviceHandle, sessionIndex, true);
	Ps2_GetDepthDistortionCorrectionEnabled(deviceHandle, sessionIndex, &jibian);
	if (jibian)cout << "����У��" << endl;
	else cout << "�޼���У��" << endl;

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
	Ps2_SetDepthDistortionCorrectionEnabled(deviceHandle, sessionIndex, true);
	Ps2_SetTimeFilterEnabled(deviceHandle, sessionIndex, true);
	Ps2_SetSpatialFilterEnabled(deviceHandle, sessionIndex, false);
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
					/*cv::Mat _testImg = cv::Mat(depthFrame.height, depthFrame.width, CV_16UC1, depthFrame.pFrameData);
					//cv::imshow("test", _dispImg);
					vector<int> weight;
					weight.resize(4);
					vector<vector<int>> neighborPixle;
					neighborPixle.resize(4);
					cv::Point2d pointxy(0, 0);
					for (auto& _v : neighborPixle) {
						_v.resize(2);
					}
					//cout << "image rows: " << _testImg.rows << endl;
					//cout << "image cols: " << _testImg.cols << endl;
					//cout << "(0,0): " << _testImg.at<ushort>(0, 0) << endl;
					//cout << "eeeee" << endl;
					for (int _row = 0; _row < _testImg.rows; ++_row) {
						for (int _col = 0; _col < _testImg.cols; ++_col) {
							//cout << "(row, col): " << _row << ", " << _col << endl;
							if (_testImg.at<ushort>(_row, _col) == 0 || _testImg.at<ushort>(_row, _col) == 65535) {
								//cout << "pixel == 0" << endl;
								int _r = _row;
								int _c = _col - 1;
								while (_c >= 0 && _r >= 0 && (_testImg.at<ushort>(_r, _c) == 0 || _testImg.at<ushort>(_r, _c) == 65535)) {
									--_c;
								}
								neighborPixle[0][0] = _r;
								neighborPixle[0][1] = _c;
								_r = _row - 1;
								_c = _col;
								while (_c >= 0 && _r >= 0 && (_testImg.at<ushort>(_r, _c) == 0 || _testImg.at<ushort>(_r, _c) == 65535)) {
									--_r;
								}
								neighborPixle[1][0] = _r;
								neighborPixle[1][1] = _c;
								_r = _row;
								_c = _col + 1;
								while (_c < _testImg.cols && _r < _testImg.rows && (_testImg.at<ushort>(_r, _c) == 0 || _testImg.at<ushort>(_r, _c) == 65535)) {
									++_c;
								}
								if (_c < _testImg.cols && _r < _testImg.rows) {
									neighborPixle[2][0] = _r;
									neighborPixle[2][1] = _c;
								}
								else {
									neighborPixle[2][0] = -1;
									neighborPixle[2][1] = -1;
								}
								_r = _row + 1;
								_c = _col;
								while (_c < _testImg.cols && _r < _testImg.rows && (_testImg.at<ushort>(_r, _c) == 0 || _testImg.at<ushort>(_r, _c) == 65535)) {
									++_r;
								}
								if (_c < _testImg.cols && _r < _testImg.rows) {
									neighborPixle[3][0] = _r;
									neighborPixle[3][1] = _c;
								}
								else {
									neighborPixle[3][0] = -1;
									neighborPixle[3][1] = -1;
								}
								vector<float> weight;
								float __temp = 0;
								calcWeight(neighborPixle, { _row, _col }, weight);
								for (int _index = 0; _index < neighborPixle.size(); ++_index) {
									if (weight[_index] != 0) {
										//cout << neighborPixle[_index][0]<<", "<<neighborPixle[_index][1] << endl;
										__temp += weight[_index] * _testImg.at<ushort>(neighborPixle[_index][0], neighborPixle[_index][1]);
									}
								}
								_testImg.at<ushort>(_row, _col) = (uint16_t)__temp;
							}
						}
					}
					//for (int _row = 0; _row < _testImg.rows; ++_row) {
					//	for (int _col = 0; _col < _testImg.cols; ++_col) {
					//		int _pos = _row * _testImg.cols + _col;
					//		ushort _val = _testImg.at<ushort>(_row, _col);
					//		*(depthFrame.pFrameData + _pos * 2 + 1) = (uint8_t)(_val & 0x0f);
					//		*(depthFrame.pFrameData + _pos * 2) = (uint8_t)(_val >> 8 & 0x0f);
					//	}
					//}
					_testImg.convertTo(_testImg, CV_8U, 255.0 / slope.slope1);
					applyColorMap(_testImg, _testImg, cv::COLORMAP_RAINBOW);
					cv::imshow("test", _testImg);
					cv::Mat _testImg2 = cv::Mat(depthFrame.height, depthFrame.width, CV_16UC1, depthFrame.pFrameData);
					_testImg2.convertTo(_testImg2, CV_8U, 255.0 / slope.slope1);
					applyColorMap(_testImg2, _testImg2, cv::COLORMAP_RAINBOW);
					cv::imshow("test2", _testImg2);
					cout << "num: "<<_testImg.total() << endl;
					cout << "press any continue" << endl;
					cv::waitKey();*/
				}
				//��ʼ����
				if (saveStart && (Index % interval == 1 || !auto_update)) {
					cout << "save===========================" << endl;
					flyingPixel(deviceHandle, sessionIndex, depthFrame, slope.slope1);
					cv::Mat _testImg = cv::Mat(depthFrame.height, depthFrame.width, CV_16UC1, depthFrame.pFrameData);
					//vector<int> weight;
					//weight.resize(4);
					//vector<vector<int>> neighborPixle;
					//neighborPixle.resize(4);
					//cv::Point2d pointxy(0, 0);
					//for (auto& _v : neighborPixle) {
					//	_v.resize(2);
					//}
					//for (int _row = 0; _row < _testImg.rows; ++_row) {
					//	for (int _col = 0; _col < _testImg.cols; ++_col) {
					//		if (_testImg.at<ushort>(_row, _col) == 0 || _testImg.at<ushort>(_row, _col) == 65535) {
					//			int _r = _row;
					//			int _c = _col - 1;
					//			while (_c >= 0 && _r >= 0 && (_testImg.at<ushort>(_r, _c) == 0 || _testImg.at<ushort>(_r, _c) == 65535)) {
					//				--_c;
					//			}
					//			neighborPixle[0][0] = _r;
					//			neighborPixle[0][1] = _c;
					//			_r = _row - 1;
					//			_c = _col;
					//			while (_c >= 0 && _r >= 0 && (_testImg.at<ushort>(_r, _c) == 0 || _testImg.at<ushort>(_r, _c) == 65535)) {
					//				--_r;
					//			}
					//			neighborPixle[1][0] = _r;
					//			neighborPixle[1][1] = _c;
					//			_r = _row;
					//			_c = _col + 1;
					//			while (_c < _testImg.cols && _r < _testImg.rows && (_testImg.at<ushort>(_r, _c) == 0 || _testImg.at<ushort>(_r, _c) == 65535)) {
					//				++_c;
					//			}
					//			if (_c < _testImg.cols && _r < _testImg.rows) {
					//				neighborPixle[2][0] = _r;
					//				neighborPixle[2][1] = _c;
					//			}
					//			else {
					//				neighborPixle[2][0] = -1;
					//				neighborPixle[2][1] = -1;
					//			}
					//			_r = _row + 1;
					//			_c = _col;
					//			while (_c < _testImg.cols && _r < _testImg.rows && (_testImg.at<ushort>(_r, _c) == 0 || _testImg.at<ushort>(_r, _c) == 65535)) {
					//				++_r;
					//			}
					//			if (_c < _testImg.cols && _r < _testImg.rows) {
					//				neighborPixle[3][0] = _r;
					//				neighborPixle[3][1] = _c;
					//			}
					//			else {
					//				neighborPixle[3][0] = -1;
					//				neighborPixle[3][1] = -1;
					//			}
					//			vector<float> weight;
					//			float __temp = 0;
					//			calcWeight(neighborPixle, { _row, _col }, weight);
					//			for (int _index = 0; _index < neighborPixle.size(); ++_index) {
					//				if (weight[_index] != 0) {
					//					//cout << neighborPixle[_index][0]<<", "<<neighborPixle[_index][1] << endl;
					//					__temp += weight[_index] * _testImg.at<ushort>(neighborPixle[_index][0], neighborPixle[_index][1]);
					//				}
					//			}
					//			_testImg.at<ushort>(_row, _col) = (uint16_t)__temp;
					//		}
					//	}
					//}
					_testImg.convertTo(_testImg, CV_8U, 255.0 / slope.slope1);
					applyColorMap(_testImg, _testImg, cv::COLORMAP_RAINBOW);
					cv::imshow("�ն��", _testImg);

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

int calcWeight(
	vector<vector<int>> postionVec,
	vector<int> pos,
	vector<float>& weight)
{
	vector<float> _weightTemp;
	_weightTemp.resize(postionVec.size());
	weight.resize(postionVec.size());
	float dis;
	float e = 2.718281828459045;
	float sum = 0.0;
	for (int _index = 0; _index < postionVec.size(); ++_index) {
		if (postionVec[_index][0] >= 0 && postionVec[_index][1] >= 0) {
			dis = pow((float)(postionVec[_index][0] - pos[0]), 2.0) + pow((float)(postionVec[_index][1] - pos[1]), 2.0);			
			_weightTemp[_index] = pow(e, dis * -0.5);
			sum += _weightTemp[_index];
		}
		else {
			_weightTemp[_index] = 0.0;
			//cout << "weight: " << _index << ", " << _weightTemp[_index] << endl;
		}
	}
	if (sum != 0) {
		for (int _index = 0; _index < _weightTemp.size(); ++_index) {
			weight[_index] = _weightTemp[_index] / sum;
		}
	}
	return 0;
}

void depthToPointCloud(
	const cv::Mat& image,
	pcl::PointCloud<pcl::PointXYZ>::Ptr& pointCloudPtr,
	float f,
	float dx,
	float dy)
{
	pointCloudPtr->resize(image.total());
	pointCloudPtr->width = image.cols;
	pointCloudPtr->height = image.rows;
	float centerX = (float)image.cols / 2.0 - 0.5;
	float centerY = (float)image.rows / 2.0 - 0.5;
	float disX;
	float disY;
	float scale;
	for (int _row = 0; _row < image.rows; ++_row) {
		for (int _col = 0; _col < image.cols; ++_col) {
			disX = ((float)_col - centerX) * dx;
			disY = ((float)_row - centerY) * dy;
			//scale = (float)image.at<ushort>(_row, _col) / 65535.0f / pow(disX * disX + disY * disY + f * f, 0.5);
			scale = (float)image.at<ushort>(_row, _col) / 65535.0f / f;
			//scale = 0.5 / f;
			pointCloudPtr->at(_col, _row).x = disX * scale;
			pointCloudPtr->at(_col, _row).y = disY * scale;
			pointCloudPtr->at(_col, _row).z = f * scale;
		}
	}
}

void flyingPixel(
	PsDeviceHandle& deviceHandle, 
	uint32_t sessionIndex,
	PsFrame& depthFrame,
	uint32_t slpoe)
{
	cv::Mat flyingEm = cv::Mat(depthFrame.height, depthFrame.width, CV_16UC1, depthFrame.pFrameData);
	cv::Mat showImg;
	//cv::Mat showImg = cv::Mat(depthFrame.height, depthFrame.width, CV_16UC1, depthFrame.pFrameData);
	flyingEm.convertTo(showImg, CV_32F);
	//applyColorMap(showImg, showImg, cv::COLORMAP_RAINBOW);
	cv::Mat showImg2;
	bilateralFilter(showImg, showImg2, 15, 20, 50);
	cv::imshow("�˲�ǰ", showImg);
	cv::imshow("�˲���", showImg2);
	showImg2.convertTo(showImg2, CV_16UC1);
	for (size_t row = 1; row < flyingEm.rows - 1; ++row) {
		for (size_t col = 1; col < flyingEm.cols - 1; ++col) {
			flyingEm.at<ushort>(row, col) = showImg2.at<ushort>(row, col);
		}
	}
	int len, pWorldIndex, winSize=1;
	int winWidth = winSize * 2 + 1;
	int winNum = winWidth * winWidth;
	len = depthFrame.width * depthFrame.height;
	PsVector3f* pWorld = new PsVector3f[len];
	Ps2_ConvertDepthFrameToWorldVector(deviceHandle, sessionIndex, depthFrame, pWorld);
	vector<Eigen::Vector3f> win;
	Eigen::Vector3f normal = Eigen::Vector3f::Zero();
	Eigen::Vector3f pVec;
	vector<int> search({ 0,1,2,5,8,7,6,3 });
	vector<int> search2({ 0,1,2,5,8,7,6,3 });
	win.resize(9);
	Eigen::MatrixXf winMatrix(winNum, 3);
	
	float flyingI, maxZdis,minZdis;
	//int row_, col_;
	for (size_t row = winSize; row < flyingEm.rows - winSize; ++row) {
		cout <<"\rnormal row"<<row;
		for (size_t col = winSize; col < flyingEm.cols - winSize; ++col) {
			//cout << " col" << col;
			if (flyingEm.at<ushort>(row, col) == 65535 || flyingEm.at<ushort>(row, col) == 0) {
				flyingEm.at<ushort>(row, col) = 0;
				continue;
			}
			pWorldIndex = row * depthFrame.width + col;
			pVec << pWorld[pWorldIndex].x, pWorld[pWorldIndex].y, pWorld[pWorldIndex].z;
			minZdis = 65535;
			maxZdis = 0;
			for (int _index = 0; _index < winNum; ++_index) {
				pWorldIndex = (row + _index / winWidth - winSize) * depthFrame.width + col - winSize + (_index % winWidth);
				winMatrix.row(_index) << pWorld[pWorldIndex].x, pWorld[pWorldIndex].y, pWorld[pWorldIndex].z;
				minZdis = minZdis < pWorld[pWorldIndex].z ? minZdis : pWorld[pWorldIndex].z;
				maxZdis = maxZdis > pWorld[pWorldIndex].z ? maxZdis : pWorld[pWorldIndex].z;
			}
			//cout <<"\r" << maxZdis - minZdis;
			//if (maxZdis - minZdis < 10)continue;
			getNormal(winMatrix, normal);
			//for (int _index = 0; _index < 9; ++_index) {
			//	pWorldIndex = (row + _index / 3 - 1) * depthFrame.width + col - 1 + (_index % 3);
			//		win[_index] << pWorld[pWorldIndex].x, pWorld[pWorldIndex].y, pWorld[pWorldIndex].z;
			//}
			//normal.setZero();
			//for (int _index = 0; _index < 8; _index++) {
			//	normal += (win[search[_index]] - win[4]).cross(win[search[(_index + 2) % 8]] - win[4]).normalized();
			//}
			normal.normalized();
			flyingI = abs(pVec.normalized().dot(normal));
			if (flyingI >0.8)flyingEm.at<ushort>(row, col) = 65535;
			//flyingEm.at<ushort>(row, col) = 65535;
			//else flyingEm.at<ushort>(col, row) = 0;
		}
	}
	return;
	cout << endl;
	int mindis, disTemp;
	vector<vector<int>> neighborPixle;
	//vector<vector<int>> neighborPixle;
	neighborPixle.resize(4);
	for (auto& _v : neighborPixle) {
		_v.resize(2);
	}
	for (int _row = 0; _row < flyingEm.rows; ++_row) {
		cout << "\rrow " << _row;
		for (int _col = 0; _col < flyingEm.cols; ++_col) {
			if (flyingEm.at<ushort>(_row, _col) != 65535) continue;
			int _r = _row;
			int _c = _col - 1;
			while (_c >= 0 && _r >= 0 && (flyingEm.at<ushort>(_r, _c) == 65535)) {
				--_c;
			}
			neighborPixle[0][0] = _r > 0 ? _r : 0;
			neighborPixle[0][1] = _c > 0 ? _c : 0;
			_r = _row - 1;
			_c = _col;
			while (_c >= 0 && _r >= 0 && (flyingEm.at<ushort>(_r, _c) == 65535)) {
				--_r;
			}
			neighborPixle[1][0] = _r > 0 ? _r : 0;
			neighborPixle[1][1] = _c > 0 ? _c : 0;
			_r = _row;
			_c = _col + 1;
			while (_c < flyingEm.cols && _r < flyingEm.rows && (flyingEm.at<ushort>(_r, _c) == 65535)) {
				++_c;
			}
			if (_c < flyingEm.cols && _r < flyingEm.rows) {
				neighborPixle[2][0] = _r < flyingEm.rows ? _r : flyingEm.rows;
				neighborPixle[2][1] = _c < flyingEm.rows ? _c : flyingEm.rows;
			}
			_r = _row + 1;
			_c = _col;
			while (_c < flyingEm.cols && _r < flyingEm.rows && (flyingEm.at<ushort>(_r, _c) == 65535)) {
				++_r;
			}
			neighborPixle[3][0] = _r < flyingEm.rows ? _r : flyingEm.rows;
			neighborPixle[3][1] = _c < flyingEm.rows ? _c : flyingEm.rows;

			int pixel = flyingEm.at<ushort>(_row, _col);
			mindis = 65536;
			for (int __index = 0; __index < 4; ++__index) {
				disTemp = abs(pixel- flyingEm.at<ushort>(neighborPixle[__index][0], neighborPixle[__index][1]));
				if (disTemp < mindis) {
					mindis = disTemp;
					flyingEm.at<ushort>(_row, _col) = flyingEm.at<ushort>(neighborPixle[__index][0], neighborPixle[__index][1]);
				}
			}
		}
	}
}

void getNormal(
	Eigen::MatrixXf matrix,
	Eigen::Vector3f& normal)
{
	int pointNum = matrix.rows();
	float mean_x = 0, mean_y = 0, mean_z = 0;
	mean_x = matrix.col(0).mean();
	mean_y = matrix.col(1).mean();
	mean_z = matrix.col(2).mean();
	for (int _row = 0; _row < pointNum; ++_row) {
		matrix(_row, 0) -= mean_x;
		matrix(_row, 1) -= mean_y;
		matrix(_row, 2) -= mean_z;
	}
	Eigen::Matrix3f covMatrix = (matrix.transpose()* matrix) / pointNum;
	Eigen::EigenSolver<Eigen::Matrix3f> em(covMatrix);
	Eigen::Matrix3f D = em.pseudoEigenvalueMatrix();
	Eigen::Matrix3f V = em.pseudoEigenvectors();
	int row, col;
	D.maxCoeff(&row, &col);
	normal = V.col(row);
}

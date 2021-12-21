#include "pclTest.h"

int pclVisualizationTest(string cloudFilePath1, string cloudFilePath2) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudseg(new pcl::PointCloud<pcl::PointXYZ>);

	if (pcl::io::loadPCDFile<pcl::PointXYZ>(cloudFilePath1, *cloud) == -1) {
		cout << "Load " << cloudFilePath1 << " failed!" << endl;
		return -1;
	}

	/*if (planeSegmentation<pcl::PointXYZ>(1, cloud, cloudseg) == -1) {
		return -1;
	}*/
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(cloudFilePath2, *cloudseg) == -1) {
		cout << "Load " << cloudFilePath2 << " failed!" << endl;
		return -1;
	}
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
		//std::this_thread::sleep_for(100ms);
	}

	//system("pause");

	return 0;
}

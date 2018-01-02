// ======================================================================
//
//	include
//
// =======================================================================
#include "pcProcess.h"

//#include "RTC_PointCloudProcess.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/viz.hpp>

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcProcessLib.h>

// ======================================================================
//
//	prototype declaration
//
// =======================================================================

// ======================================================================
//
//	main program
//
// =======================================================================
bool pcProcess(
		RTC::PointCloud& rtc_pc,
		pcl::PointCloud<pcl::PointXYZRGB>& pcl_pc) {

	// ========== point cloud convert RTC -> PCL ==============
	cvt_RTCpc_to_PCLpc(rtc_pc, pcl_pc);

	// ========== cut near 0 (under 0.001) ====================
	pcl::PointCloud<pcl::PointXYZRGB> cloud_filtered;

	// Create the filtering object
	pcl::PassThrough<pcl::PointXYZRGB> pass;
	pass.setInputCloud(pcl_pc.makeShared());
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0.001, 1);
	//pass.setFilterLimitsNegative (true);
	pass.filter(cloud_filtered);

	return false;
}


// ======================================================================
//
//	include
//
// ======================================================================

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/viz.hpp>

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>

#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <rtm/idl/InterfaceDataTypesSkel.h>
// ======================================================================
//
//	arguments
//
// =======================================================================

// ======================================================================
//
//	prototype declaration
//
// =======================================================================
bool pcProcess(
		RTC::PointCloud& rtc_pc,
		pcl::PointCloud<pcl::PointXYZRGB>& pcl_pc);


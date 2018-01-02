// -*-C++-*-
/*!
 * @file  pcProcessPortSVC_impl.cpp
 * @brief Service implementation code of pcProcessPort.idl
 *
 */

#include "pcProcessPortSVC_impl.h"
#include "pcProcessLib.h"

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcProcessLib.h>

/*
 * Example implementational code for IDL interface ComPcProcess
 */
ComPcProcessSVC_impl::ComPcProcessSVC_impl() {
	// Please add extra constructor code here.
}
ComPcProcessSVC_impl::ComPcProcessSVC_impl(
		RTC::PointCloud *m_p,
		RTC::InPort<RTC::PointCloud> *m_p_i) {

	m_pointCloud_in = m_p;
	m_pointCloud_inIn = m_p_i;

	// Please add extra constructor code here.
}

ComPcProcessSVC_impl::~ComPcProcessSVC_impl() {
	// Please add extra destructor code here.
}

/*
 * Methods corresponding to IDL attributes and operations
 */
// ======================================================================
//
//	process point cloud
//
// =======================================================================
// ======================================================================
//
//	get point cloud
//
// =======================================================================
RTC::PointCloud* ComPcProcessSVC_impl::get_pointCloud(::CORBA::Boolean& flag) {

	RTC::PointCloud *pc = new RTC::PointCloud();

	std::cout << "exe get_pointCloud" << std::endl;

	// check new point cloud
	if (m_pointCloud_inIn->isNew()) {
		// read new point cloud
		m_pointCloud_inIn->read();

		std::cout << "discover new data" << std::endl;

		flag = true;
		*pc = *m_pointCloud_in;

		return pc;
	}

	std::cout << "not discover new data" << std::endl;

	flag = false;
	return pc;
}

// ======================================================================
//
//	save point cloud
//
// =======================================================================

::CORBA::Boolean ComPcProcessSVC_impl::save_pointCloud(const char* str) {

	std::cout << "exe save_pointCloud" << std::endl;

	if (m_pointCloud_inIn->isNew()) {
		// read new point cloud
		m_pointCloud_inIn->read();

		std::cout << "discover new data" << std::endl;

		// ========== point cloud convert RTC -> PCL ==============
		pcl::PointCloud<pcl::PointXYZRGB> cloud;
		cvt_RTCpc_to_PCLpc(*m_pointCloud_in, cloud);

		// ========== cut near 0 (under 0.001) ====================
		pcl::PointCloud<pcl::PointXYZRGB> cloud_filtered;
		cut_pointCloud_z(cloud, cloud_filtered, 0.001);

		// ========== save point cloud ==============
		pcl::io::savePCDFileASCII(str, cloud);

		return true;
	}
	std::cout << "not discover new data" << std::endl;
	return false;
}

//***********************************
//**Dong's Code
//***********************************

char* ComPcProcessSVC_impl::Capture_PointClould(const ::ComPcProcess::Matrix4_4 TransformData)
{
	std::cout << "Capture!" << std::endl;
	for(int i = 0;i < 4;i++)
		for(int j = 0;j < 4;j++)
			std::cout << TransformData[i][j] << std::endl;
	return const_cast<char*>("Success!\n");
}

char* ComPcProcessSVC_impl::SwitchSysMode(const char* ModeStr)
{
	string str = ModeStr;
	std::cout << str << std::endl;
	return const_cast<char*>(str.c_str());
}

char* ComPcProcessSVC_impl::Clear_QueueAndPoints()
{
	string str = "ModeStr";
	std::cout << str << std::endl;
	return const_cast<char*>(str.c_str());
}

ComPcProcess::CupInfo_slice* ComPcProcessSVC_impl::GetCupInfo()
{
	ComPcProcess::CupInfo Res;

	for(int i = 0;i < 7;i++)
		ComPcProcessSVC_impl::Result_CupInfo[i] = i * 3;
	for(int i = 0;i < 5;i++)
		for(int j = 0;j < 7;j++)
			Res[i][j] = (CORBA::Double)(i * 7 + j);

	return Res;
}

// -*- C++ -*-
/*!
 * @file  RTC_PointCloudProcess.cpp
 * @brief ModuleDescription
 * @date $Date$
 *
 * $Id$
 */

// ======================================================================
//
//	include
//
// =======================================================================
#include "RTC_PointCloudProcess.h"
#include "pcProcess.h"

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/viz.hpp>

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>

//*******************************************************
//**Edit by Dong.
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include "Functions.h"
#include <thread>
#include "Visualizer.h"
#include "pcProcessLib.h"
#include "pcProcessPortSVC_impl.h"

void Transform_PointCloud();
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
//**Edit by Dong. (END)
//*******************************************************

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

// Module specification
// <rtc-template block="module_spec">
static const char* rtc_pointcloudprocess_spec[] = { "implementation_id",
		"RTC_PointCloudProcess", "type_name", "RTC_PointCloudProcess",
		"description", "ModuleDescription", "version", "1.0.0", "vendor",
		"Keisuke Nagano", "category", "Category", "activity_type", "PERIODIC",
		"kind", "DataFlowComponent", "max_instance", "1", "language", "C++",
		"lang_type", "compile", "" };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
RTC_PointCloudProcess::RTC_PointCloudProcess(RTC::Manager* manager)
// <rtc-template block="initializer">
:
				RTC::DataFlowComponentBase(manager),
				m_command_inIn("command_in", m_command_in),
				m_pointCloud_inIn("pointCloud_in", m_pointCloud_in),
				m_command_outOut("command_out", m_command_out),
				m_value_outOut("value_out", m_value_out),
				m_svPcProcessPort("svPcProcess"),
				m_sv_pp(&m_pointCloud_in, &m_pointCloud_inIn)

// </rtc-template>
{
}

/*!
 * @brief destructor
 */
RTC_PointCloudProcess::~RTC_PointCloudProcess() {
}

RTC::ReturnCode_t RTC_PointCloudProcess::onInitialize() {
	// Registration: InPort/OutPort/Service
	// <rtc-template block="registration">
	// Set InPort buffers
	addInPort("command_in", m_command_inIn);
	addInPort("pointCloud_in", m_pointCloud_inIn);

	// Set OutPort buffer
	addOutPort("command_out", m_command_outOut);
	addOutPort("value_out", m_value_outOut);

	// Set service provider to Ports
	m_svPcProcessPort.registerProvider("sv_pp", "ComPcProcess", m_sv_pp);

	// Set service consumers to Ports

	// Set CORBA Service Ports
	addPort(m_svPcProcessPort);

	// </rtc-template>

	// <rtc-template block="bind_config">
	// </rtc-template>

	return RTC::RTC_OK;
}

/*
 RTC::ReturnCode_t RTC_PointCloudProcess::onFinalize()
 {
 return RTC::RTC_OK;
 }
 */

/*
 RTC::ReturnCode_t RTC_PointCloudProcess::onStartup(RTC::UniqueId ec_id)
 {
 return RTC::RTC_OK;
 }
 */

/*
 RTC::ReturnCode_t RTC_PointCloudProcess::onShutdown(RTC::UniqueId ec_id)
 {
 return RTC::RTC_OK;
 }
 */

RTC::ReturnCode_t RTC_PointCloudProcess::onActivated(RTC::UniqueId ec_id) {
	return RTC::RTC_OK;
}

/*
 RTC::ReturnCode_t RTC_PointCloudProcess::onDeactivated(RTC::UniqueId ec_id)
 {
 return RTC::RTC_OK;
 }
 */

RTC::ReturnCode_t RTC_PointCloudProcess::onExecute(RTC::UniqueId ec_id) {

	// check new command
	if (m_command_inIn.isNew()) {
		//read new command
		m_command_inIn.read();

		// check new point cloud
		if (m_pointCloud_inIn.isNew()) {
			// read new point cloud
			m_pointCloud_inIn.read();

			std::cout << "length : " << m_pointCloud_in.points.length()
					<< std::endl;

//			pcl::PointCloud<pcl::PointXYZRGB> cloud;
//			pcProcess(m_pointCloud_in, cloud);

		}
	}
	//*******************************************************
	//**Edit by Dong.
	//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
	if(ComPcProcessSVC_impl::SystemMode == ComPcProcessSVC_impl::Capture)
	{
		//std::cout << "###### onExecute. Capture Mode!" << std::endl;
		Transform_PointCloud();
	}
	else
		std::cout << "###### onExecute. Process Mode!" << std::endl;
	//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
	//**Edit by Dong. (END)
	//*******************************************************
	
	return RTC::RTC_OK;
}

//*******************************************************
//**Edit by Dong.
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void Transform_PointCloud()
{
	if (ComPcProcessSVC_impl::SystemMode != ComPcProcessSVC_impl::Capture)
		return;

	if (!ComPcProcessSVC_impl::QueueMutex.try_lock())
	{
		std::cout << "Transform_PointCloud() Queue is now using, exit!" << std::endl;
		return;
	}
	if (ComPcProcessSVC_impl::queue_PointsOfCapture.empty() || ComPcProcessSVC_impl::queue_TransformData.empty())
	{
		std::cout << "Transform_PointCloud() Queue is empty, exit!" << std::endl;
		ComPcProcessSVC_impl::QueueMutex.unlock();
		return;
	}

	std::cout << "#################################################" << std::endl;
	std::cout << "#----Entering Transform_PointCloud() process----#" << std::endl;
	std::cout << "#################################################" << std::endl;

	PCXYZ_Ptr tmp(new PCXYZ);
	PCXYZ_Ptr tmp2(new PCXYZ);
	PCXYZ_Ptr tmp3(new PCXYZ);

	///////// Read data from two queues.
	PCXYZ Points(ComPcProcessSVC_impl::queue_PointsOfCapture.front());
	Eigen::Matrix4f Mats(ComPcProcessSVC_impl::queue_TransformData.front());

	///////// Use filter to ...
	std::cout << "Transform_PointCloud() Start to Filter!" << std::endl;
	Filters(Points.makeShared(), tmp);

	///////// Transform the pointcloud.
	std::cout << "Transform_PointCloud() Start to transformPointCloud()!" << std::endl;
	pcl::transformPointCloud(*tmp, *tmp2, Mats);

	///////// Pop the first element of queue.
	std::cout << "Transform_PointCloud() Start to PopQueue!" << std::endl;
	ComPcProcessSVC_impl::queue_PointsOfCapture.pop();
	ComPcProcessSVC_impl::queue_TransformData.pop();
	ComPcProcessSVC_impl::QueueMutex.unlock();

	///////// ICP
	std::cout << "Transform_PointCloud() Start to ICP!" << std::endl;
	if ((*ComPcProcessSVC_impl::PointsOfTable).size() != 0)
		ICP_Single(tmp2, ComPcProcessSVC_impl::PointsOfTable, tmp3);
	else
		tmp3 = tmp2;

	///////// Accumulate pointcloud.
	(*ComPcProcessSVC_impl::PointsOfTable) += (*tmp3);

	std::cout << "#################################################" << std::endl;
	std::cout << "#+++++LEAVING Transform_PointCloud() process++++#" << std::endl;
	std::cout << "#################################################" << std::endl;
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
//**Edit by Dong. (END)
//*******************************************************

/*
 RTC::ReturnCode_t RTC_PointCloudProcess::onAborting(RTC::UniqueId ec_id)
 {
 return RTC::RTC_OK;
 }
 */

RTC::ReturnCode_t RTC_PointCloudProcess::onError(RTC::UniqueId ec_id) {
std::cout << "####ERROR#### Get in onError!" << std::endl;
char a=0;
std::cin>>a;
	return RTC::RTC_OK;
}

/*
 RTC::ReturnCode_t RTC_PointCloudProcess::onReset(RTC::UniqueId ec_id)
 {
 return RTC::RTC_OK;
 }
 */

/*
 RTC::ReturnCode_t RTC_PointCloudProcess::onStateUpdate(RTC::UniqueId ec_id)
 {
 return RTC::RTC_OK;
 }
 */

/*
 RTC::ReturnCode_t RTC_PointCloudProcess::onRateChanged(RTC::UniqueId ec_id)
 {
 return RTC::RTC_OK;
 }
 */

extern "C" {

void RTC_PointCloudProcessInit(RTC::Manager* manager) {
	coil::Properties profile(rtc_pointcloudprocess_spec);
	manager->registerFactory(
			profile,
			RTC::Create<RTC_PointCloudProcess>,
			RTC::Delete<RTC_PointCloudProcess>);
}

}
;


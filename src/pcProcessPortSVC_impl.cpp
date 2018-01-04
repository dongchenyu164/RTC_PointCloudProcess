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


//*******************************************************
//**Edit by Dong.
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <thread>
#include "Functions.h"
char* ComPcProcessSVC_impl::PublicReturnStringPointer = new char[100];
PCXYZ_Ptr ComPcProcessSVC_impl::PointsOfTable = PCXYZ_Ptr(new PCXYZ);
ComPcProcessSVC_impl::PointCloudProcessMode ComPcProcessSVC_impl::SystemMode = ComPcProcessSVC_impl::Capture;
std::queue<PCXYZ_Ptr> ComPcProcessSVC_impl::queue_PointsOfCapture;//被捕获的点云的存放队列。
std::queue<Eigen::Matrix4f> ComPcProcessSVC_impl::queue_TransformData;//被捕获的点云的位姿矩阵。
std::mutex ComPcProcessSVC_impl::QueueMutex;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
//**Edit by Dong. (END)
//*******************************************************


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
char* Make_RTC_ReturnString(std::string Data)
{
//	//delete ComPcProcessSVC_impl::PublicReturnStringPointer;
//	char* tmp = new char[Data.size() + 1];
//	//ComPcProcessSVC_impl::PublicReturnStringPointer = tmp;
//	return tmp;
	std::cout << Data << std::endl;
	return CORBA::string_dup(Data.c_str());
}

char* ComPcProcessSVC_impl::Capture_PointClould(const ::ComPcProcess::Matrix4_4 TransformData)
{
	std::thread::id ThreadId = std::this_thread::get_id();
	std::cout << "###### GetCupInfo() Thread ID:" << ThreadId << std::endl;

	while (!m_pointCloud_inIn->isNew());
	/*if (m_pointCloud_inIn->isNew())*/
	{
//		if (!QueueMutex.try_lock())
//			std::cout << "Queue is processing!" << std::endl;
		while (!QueueMutex.try_lock());
		std::cout << "Start to capture!" << std::endl;

		// read new point cloud
		m_pointCloud_inIn->read();
		// ========== point cloud convert RTC -> PCL ==============
		pcl::PointCloud<pcl::PointXYZ> cloud;
		cvt_RTCpc_to_PCLpc(*m_pointCloud_in, cloud);
		// ========== cut near 0 (under 0.001) ====================
		pcl::PointCloud<pcl::PointXYZ> DataIn;
		cut_pointCloud_z(cloud, DataIn, 0.001);

		double tmpTransformData[4][4];
		for(int i = 0;i < 4;i++)
			for(int j = 0;j < 4;j++)
				tmpTransformData[i][j] = TransformData[i][j];

		ComPcProcessSVC_impl::queue_PointsOfCapture.push(DataIn.makeShared());
		ComPcProcessSVC_impl::queue_TransformData.push(MakeTransformMatrix(tmpTransformData));

		QueueMutex.unlock();

		return Make_RTC_ReturnString("Capture_PointClould() Success!");
	}
//	else
//		return Make_RTC_ReturnString("No new data!");
}

char* ComPcProcessSVC_impl::SwitchSysMode(const char* ModeStr)
{
	std::thread::id ThreadId = std::this_thread::get_id();
	std::cout << "###### GetCupInfo() Thread ID:" << ThreadId << std::endl;

	std::string tmpModeStr = std::string(ModeStr);
	ComPcProcessSVC_impl::PointCloudProcessMode tmpMode = ComPcProcessSVC_impl::Capture;
	if(tmpModeStr == "CaptureMode")
		tmpMode = ComPcProcessSVC_impl::Capture;
	else if(tmpModeStr == "ProcessMode")
		tmpMode = ComPcProcessSVC_impl::Process;
	else
		tmpMode = ComPcProcessSVC_impl::Capture;

	switch (ComPcProcessSVC_impl::SystemMode)
	{
		case ComPcProcessSVC_impl::Capture://当前模式
			switch (tmpMode)//目标模式
			{
				case ComPcProcessSVC_impl::Capture:
					return Make_RTC_ReturnString("Now CaptureMode. No need to switch!");
					break;
				case ComPcProcessSVC_impl::Process:
					if (!ComPcProcessSVC_impl::queue_PointsOfCapture.empty())
						return Make_RTC_ReturnString("Now CaptureMode. Processing......!");
					ComPcProcessSVC_impl::SystemMode = ComPcProcessSVC_impl::Process;
					break;
				default:
					break;
			}
			break;
		case ComPcProcessSVC_impl::Process:
			switch (tmpMode)//目标模式
			{
				case ComPcProcessSVC_impl::Capture:
					ComPcProcessSVC_impl::SystemMode = ComPcProcessSVC_impl::Capture;
					Clear_QueueAndPoints();
					return Make_RTC_ReturnString("Switch to Capture mode and clear Queue and GlobePointCloud successfully!");
					break;
				case ComPcProcessSVC_impl::Process:
					return Make_RTC_ReturnString("Now ProcessMode. No need to switch!");
					break;
				default:
					break;
			}
			break;
		default:
			break;
	}
	return Make_RTC_ReturnString("No Mode mathed!");
}

char* ComPcProcessSVC_impl::Clear_QueueAndPoints()
{
	std::thread::id ThreadId = std::this_thread::get_id();
	std::cout << "###### GetCupInfo() Thread ID:" << ThreadId << std::endl;

	if (!ComPcProcessSVC_impl::QueueMutex.try_lock())
		return Make_RTC_ReturnString("Fail Processing!");

	(*ComPcProcessSVC_impl::PointsOfTable).clear();

	while (!ComPcProcessSVC_impl::queue_PointsOfCapture.empty())
		ComPcProcessSVC_impl::queue_PointsOfCapture.pop();
	while (!ComPcProcessSVC_impl::queue_TransformData.empty())
		ComPcProcessSVC_impl::queue_TransformData.pop();

	ComPcProcessSVC_impl::QueueMutex.unlock();

	return Make_RTC_ReturnString("Clear_Queue_Points() Success!");
}

ComPcProcess::CupInfo_slice* ComPcProcessSVC_impl::GetCupInfo()
{
	std::thread::id ThreadId = std::this_thread::get_id();
	std::cout << "###### GetCupInfo() Thread ID:" << ThreadId << std::endl;

	ComPcProcess::CupInfo* tmp = new ComPcProcess::CupInfo[5];
	for(int i = 0;i < 5;i++)
		for(int j = 0;j < 7;j++)
			(*tmp)[i][j] = (CORBA::Double)(i * 3);

	return (*tmp);
}

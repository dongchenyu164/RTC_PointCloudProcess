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

//////// Globle variables
PCXYZ_Ptr ComPcProcessSVC_impl::PointsOfTable = PCXYZ_Ptr(new PCXYZ);
ComPcProcess::CupInfo* ComPcProcessSVC_impl::Result_CupInfo = new ComPcProcess::CupInfo[5];

ComPcProcessSVC_impl::PointCloudProcessMode ComPcProcessSVC_impl::SystemMode = ComPcProcessSVC_impl::Capture;

//////// Queue's variables
std::queue<PCXYZ> ComPcProcessSVC_impl::queue_PointsOfCapture;//被捕获的点云的存放队列。
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
char* Make_RTC_ReturnString(std::string Data, bool stdOut = false)
{
	if(stdOut)
		std::cout << Data << std::endl;
	return CORBA::string_dup(Data.c_str());
}

char* ComPcProcessSVC_impl::Capture_PointClould(const ::ComPcProcess::Matrix4_4 TransformData)
{
	while (!m_pointCloud_inIn->isNew());
	while (!QueueMutex.try_lock());

	std::cout << "Capture_PointClould() Start to capture!" << std::endl;

	/////// read new point cloud
	m_pointCloud_inIn->read();

	/////// point cloud convert RTC -> PCL
	pcl::PointCloud<pcl::PointXYZ> DataIn;
	int Points = cvt_RTCpc_to_PCLpc(*m_pointCloud_in, DataIn);
	if(Points <= 1000)
	{
		QueueMutex.unlock();
		return Make_RTC_ReturnString("Capture_PointClould() Failed! Number of points is less than 1000!", true);
	}

	/////// cut near 0 (under 0.001)
	pcl::PointCloud<pcl::PointXYZ> No;
	cut_pointCloud_z(DataIn, No, 0.001);

	//////// Convet Matrix4_4 to double[][]
	double tmpTransformData[4][4];
	for(int i = 0;i < 4;i++)
		for(int j = 0;j < 4;j++)
			tmpTransformData[i][j] = TransformData[i][j];

	ComPcProcessSVC_impl::queue_PointsOfCapture.push(DataIn);
	ComPcProcessSVC_impl::queue_TransformData.push(MakeTransformMatrix(tmpTransformData));
	QueueMutex.unlock();

	return Make_RTC_ReturnString("Capture_PointClould() Success!", true);
}

char* ComPcProcessSVC_impl::SwitchSysMode(const char* ModeStr)
{
	std::string tmpModeStr = std::string(ModeStr);
	ComPcProcessSVC_impl::PointCloudProcessMode tmpMode = ComPcProcessSVC_impl::Capture;

	//////// Translate the string to PointCloudProcessMode.
	if(tmpModeStr == "CaptureMode")
		tmpMode = ComPcProcessSVC_impl::Capture;
	else if(tmpModeStr == "ProcessMode")
		tmpMode = ComPcProcessSVC_impl::Process;
	else
		tmpMode = ComPcProcessSVC_impl::Capture;

	//////// switch to target mode
	switch (ComPcProcessSVC_impl::SystemMode)
	{
		case ComPcProcessSVC_impl::Capture:// Current mode
			switch (tmpMode)// Target mode
			{
				case ComPcProcessSVC_impl::Capture:// Target mode
					return Make_RTC_ReturnString("SwitchSysMode() Now CaptureMode. No need to switch!", true);
					break;
				case ComPcProcessSVC_impl::Process:// Target mode
					if (!ComPcProcessSVC_impl::queue_PointsOfCapture.empty())
						return Make_RTC_ReturnString("SwitchSysMode() Switch to Process mode!", true);
					ComPcProcessSVC_impl::SystemMode = ComPcProcessSVC_impl::Process;
					break;
				default:
					break;
			}
			break;
		case ComPcProcessSVC_impl::Process:
			switch (tmpMode)// Current mode
			{
				case ComPcProcessSVC_impl::Capture:// Target mode
					ComPcProcessSVC_impl::SystemMode = ComPcProcessSVC_impl::Capture;
					Clear_QueueAndPoints();
					return Make_RTC_ReturnString("SwitchSysMode() Switch to Capture mode and clear Queue and GlobePointCloud successfully!", true);
					break;
				case ComPcProcessSVC_impl::Process:// Target mode
					return Make_RTC_ReturnString("SwitchSysMode() Now ProcessMode. No need to switch!", true);
					break;
				default:
					break;
			}
			break;
		default:
			break;
	}

	return Make_RTC_ReturnString("No Mode mathed!", true);
}

char* ComPcProcessSVC_impl::Clear_QueueAndPoints()
{
	if (!ComPcProcessSVC_impl::QueueMutex.try_lock())
		return Make_RTC_ReturnString("Fail Processing!", true);

	(*ComPcProcessSVC_impl::PointsOfTable).clear();

	while (!ComPcProcessSVC_impl::queue_PointsOfCapture.empty())
		ComPcProcessSVC_impl::queue_PointsOfCapture.pop();
	while (!ComPcProcessSVC_impl::queue_TransformData.empty())
		ComPcProcessSVC_impl::queue_TransformData.pop();

	ComPcProcessSVC_impl::QueueMutex.unlock();

	return Make_RTC_ReturnString("Clear_Queue_Points() Success!", true);
}

ComPcProcess::CupInfo_slice* ComPcProcessSVC_impl::GetCupInfo()
{
	ComPcProcessSVC_impl::Result_CupInfo = new ComPcProcess::CupInfo[5];
	for(int i = 0;i < 5;i++)
		for(int j = 0;j < 7;j++)
			(*ComPcProcessSVC_impl::Result_CupInfo)[i][j] = (CORBA::Double)(i * 3);

	return (*ComPcProcessSVC_impl::Result_CupInfo);
}

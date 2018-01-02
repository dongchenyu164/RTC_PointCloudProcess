// -*-C++-*-
/*!
 * @file  pcProcessPortSVC_impl.h
 * @brief Service implementation header of pcProcessPort.idl
 *
 */

#include "InterfaceDataTypesSkel.h"
#include "BasicDataTypeSkel.h"
#include "ExtendedDataTypesSkel.h"

#include "pcProcessPortSkel.h"

#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>

//*******************************************************
//**Edit by Dong.
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include "Define.h"
#include <mutex>
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
//**Edit by Dong. (END)
//*******************************************************

//#include <rtm/Manager.h>
//#include <rtm/DataFlowComponentBase.h>
//#include <rtm/CorbaPort.h>

#ifndef PCPROCESSPORTSVC_IMPL_H
#define PCPROCESSPORTSVC_IMPL_H

/*!
 * @class ComPcProcessSVC_impl
 * Example class implementing IDL interface ComPcProcess
 */
class ComPcProcessSVC_impl: public virtual POA_ComPcProcess,
		public virtual PortableServer::RefCountServantBase {
private:
	// Make sure all instances are built on the heap by making the
	// destructor non-public
	//virtual ~ComPcProcessSVC_impl();

public:
	/*!
	 * @brief standard constructor
	 */
	ComPcProcessSVC_impl(
			RTC::PointCloud *m_p,
			RTC::InPort<RTC::PointCloud> *m_p_i);

	ComPcProcessSVC_impl();
	/*!
	 * @brief destructor
	 */
	virtual ~ComPcProcessSVC_impl();

	// attributes and operations
	RTC::PointCloud* get_pointCloud(::CORBA::Boolean& flag);
	::CORBA::Boolean save_pointCloud(const char* str);

//*******************************************************
//**Edit by Dong.
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

	char* Capture_PointClould(const ::ComPcProcess::Matrix4_4 TransformData);
	char* SwitchSysMode(const char* ModeStr);
	char* Clear_QueueAndPoints();
	ComPcProcess::CupInfo_slice* GetCupInfo();

private:
	enum PointCloudProcessMode{Capture, Process};
	//先是数据读入状态，接收【当前机械臂位姿】信号
	PointCloudProcessMode SystemMode = Capture;

	PCXYZ_Ptr PointsOfTable;//合成后的桌子的点云。
	std::queue<PCXYZ_Ptr> queue_PointsOfCapture;//被捕获的点云的存放队列。
	std::queue<Eigen::Matrix4f> queue_TransformData;//被捕获的点云的位姿矩阵。
	std::mutex QueueMutex;

	ComPcProcess::CupInfo_slice Result_CupInfo;

	void Transform_PointCloud();//应该在OnExecute函数内，在 拍摄 模式时，每周期调用。

public:

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
//**Edit by Dong. (END)
//*******************************************************

	// --------------------------------------------
	RTC::PointCloud *m_pointCloud_in;
	RTC::InPort<RTC::PointCloud> *m_pointCloud_inIn;

};

#endif // PCPROCESSPORTSVC_IMPL_H


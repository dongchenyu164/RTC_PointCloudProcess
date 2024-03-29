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

	enum PointCloudProcessMode{Capture, Process};
	static PointCloudProcessMode SystemMode;

	static PCXYZ_Ptr PointsOfTable;//合成后的桌子的点云。
	static ComPcProcess::CupInfo* Result_CupInfo;// Informations of thing on the table.
	static std::queue<PCXYZ> queue_PointsOfCapture;//被捕获的点云的存放队列。
	static std::queue<Eigen::Matrix4f> queue_TransformData;//被捕获的点云的位姿矩阵。
	static std::mutex QueueMutex;

private:

public:

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
//**Edit by Dong. (END)
//*******************************************************

	// --------------------------------------------
	RTC::PointCloud *m_pointCloud_in;
	RTC::InPort<RTC::PointCloud> *m_pointCloud_inIn;

};

#endif // PCPROCESSPORTSVC_IMPL_H


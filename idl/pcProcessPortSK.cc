// This file is generated by omniidl (C++ backend)- omniORB_4_1. Do not edit.

#include "pcProcessPort.hh"
#include <omniORB4/IOP_S.h>
#include <omniORB4/IOP_C.h>
#include <omniORB4/callDescriptor.h>
#include <omniORB4/callHandle.h>
#include <omniORB4/objTracker.h>


OMNI_USING_NAMESPACE(omni)

static const char* _0RL_library_version = omniORB_4_1;



ComPcProcess_ptr ComPcProcess_Helper::_nil() {
  return ::ComPcProcess::_nil();
}

::CORBA::Boolean ComPcProcess_Helper::is_nil(::ComPcProcess_ptr p) {
  return ::CORBA::is_nil(p);

}

void ComPcProcess_Helper::release(::ComPcProcess_ptr p) {
  ::CORBA::release(p);
}

void ComPcProcess_Helper::marshalObjRef(::ComPcProcess_ptr obj, cdrStream& s) {
  ::ComPcProcess::_marshalObjRef(obj, s);
}

ComPcProcess_ptr ComPcProcess_Helper::unmarshalObjRef(cdrStream& s) {
  return ::ComPcProcess::_unmarshalObjRef(s);
}

void ComPcProcess_Helper::duplicate(::ComPcProcess_ptr obj) {
  if( obj && !obj->_NP_is_nil() )  omni::duplicateObjRef(obj);
}

ComPcProcess_ptr
ComPcProcess::_duplicate(::ComPcProcess_ptr obj)
{
  if( obj && !obj->_NP_is_nil() )  omni::duplicateObjRef(obj);
  return obj;
}

ComPcProcess_ptr
ComPcProcess::_narrow(::CORBA::Object_ptr obj)
{
  if( !obj || obj->_NP_is_nil() || obj->_NP_is_pseudo() ) return _nil();
  _ptr_type e = (_ptr_type) obj->_PR_getobj()->_realNarrow(_PD_repoId);
  return e ? e : _nil();
}


ComPcProcess_ptr
ComPcProcess::_unchecked_narrow(::CORBA::Object_ptr obj)
{
  if( !obj || obj->_NP_is_nil() || obj->_NP_is_pseudo() ) return _nil();
  _ptr_type e = (_ptr_type) obj->_PR_getobj()->_uncheckedNarrow(_PD_repoId);
  return e ? e : _nil();
}

ComPcProcess_ptr
ComPcProcess::_nil()
{
#ifdef OMNI_UNLOADABLE_STUBS
  static _objref_ComPcProcess _the_nil_obj;
  return &_the_nil_obj;
#else
  static _objref_ComPcProcess* _the_nil_ptr = 0;
  if( !_the_nil_ptr ) {
    omni::nilRefLock().lock();
    if( !_the_nil_ptr ) {
      _the_nil_ptr = new _objref_ComPcProcess;
      registerNilCorbaObject(_the_nil_ptr);
    }
    omni::nilRefLock().unlock();
  }
  return _the_nil_ptr;
#endif
}

const char* ComPcProcess::_PD_repoId = "IDL:ComPcProcess:1.0";


_objref_ComPcProcess::~_objref_ComPcProcess() {
  
}


_objref_ComPcProcess::_objref_ComPcProcess(omniIOR* ior, omniIdentity* id) :
   omniObjRef(::ComPcProcess::_PD_repoId, ior, id, 1)
   
   
{
  _PR_setobj(this);
}

void*
_objref_ComPcProcess::_ptrToObjRef(const char* id)
{
  if( id == ::ComPcProcess::_PD_repoId )
    return (::ComPcProcess_ptr) this;
  
  if( id == ::CORBA::Object::_PD_repoId )
    return (::CORBA::Object_ptr) this;

  if( omni::strMatch(id, ::ComPcProcess::_PD_repoId) )
    return (::ComPcProcess_ptr) this;
  
  if( omni::strMatch(id, ::CORBA::Object::_PD_repoId) )
    return (::CORBA::Object_ptr) this;

  return 0;
}

// Proxy call descriptor class. Mangled signature:
//  _cRTC_mPointCloud_o_cboolean
class _0RL_cd_a93046d6d953ea9d_00000000
  : public omniCallDescriptor
{
public:
  inline _0RL_cd_a93046d6d953ea9d_00000000(LocalCallFn lcfn,const char* op_,size_t oplen,_CORBA_Boolean upcall=0):
     omniCallDescriptor(lcfn, op_, oplen, 0, _user_exns, 0, upcall)
  {
    
  }
  
  
  void unmarshalReturnedValues(cdrStream&);
  void marshalReturnedValues(cdrStream&);
  
  
  static const char* const _user_exns[];

  ::CORBA::Boolean arg_0;
  RTC::PointCloud_var result;
};

void _0RL_cd_a93046d6d953ea9d_00000000::marshalReturnedValues(cdrStream& _n)
{
  (const RTC::PointCloud&) result >>= _n;
  _n.marshalBoolean(arg_0);

}

void _0RL_cd_a93046d6d953ea9d_00000000::unmarshalReturnedValues(cdrStream& _n)
{
  result = new RTC::PointCloud;
  (RTC::PointCloud&)result <<= _n;
  arg_0 = _n.unmarshalBoolean();

}

const char* const _0RL_cd_a93046d6d953ea9d_00000000::_user_exns[] = {
  0
};

// Local call call-back function.
static void
_0RL_lcfn_a93046d6d953ea9d_10000000(omniCallDescriptor* cd, omniServant* svnt)
{
  _0RL_cd_a93046d6d953ea9d_00000000* tcd = (_0RL_cd_a93046d6d953ea9d_00000000*)cd;
  _impl_ComPcProcess* impl = (_impl_ComPcProcess*) svnt->_ptrToInterface(ComPcProcess::_PD_repoId);
  tcd->result = impl->get_pointCloud(tcd->arg_0);


}

RTC::PointCloud* _objref_ComPcProcess::get_pointCloud(::CORBA::Boolean& flag)
{
  _0RL_cd_a93046d6d953ea9d_00000000 _call_desc(_0RL_lcfn_a93046d6d953ea9d_10000000, "get_pointCloud", 15);


  _invoke(_call_desc);
  flag = _call_desc.arg_0;
  return _call_desc.result._retn();


}
// Proxy call descriptor class. Mangled signature:
//  _cboolean_i_cstring
class _0RL_cd_a93046d6d953ea9d_20000000
  : public omniCallDescriptor
{
public:
  inline _0RL_cd_a93046d6d953ea9d_20000000(LocalCallFn lcfn,const char* op_,size_t oplen,_CORBA_Boolean upcall=0):
     omniCallDescriptor(lcfn, op_, oplen, 0, _user_exns, 0, upcall)
  {
    
  }
  
  void marshalArguments(cdrStream&);
  void unmarshalArguments(cdrStream&);

  void unmarshalReturnedValues(cdrStream&);
  void marshalReturnedValues(cdrStream&);
  
  
  static const char* const _user_exns[];

  ::CORBA::String_var arg_0_;
  const char* arg_0;
  ::CORBA::Boolean result;
};

void _0RL_cd_a93046d6d953ea9d_20000000::marshalArguments(cdrStream& _n)
{
  _n.marshalString(arg_0,0);

}

void _0RL_cd_a93046d6d953ea9d_20000000::unmarshalArguments(cdrStream& _n)
{
  arg_0_ = _n.unmarshalString(0);
  arg_0 = arg_0_.in();

}

void _0RL_cd_a93046d6d953ea9d_20000000::marshalReturnedValues(cdrStream& _n)
{
  _n.marshalBoolean(result);

}

void _0RL_cd_a93046d6d953ea9d_20000000::unmarshalReturnedValues(cdrStream& _n)
{
  result = _n.unmarshalBoolean();

}

const char* const _0RL_cd_a93046d6d953ea9d_20000000::_user_exns[] = {
  0
};

// Local call call-back function.
static void
_0RL_lcfn_a93046d6d953ea9d_30000000(omniCallDescriptor* cd, omniServant* svnt)
{
  _0RL_cd_a93046d6d953ea9d_20000000* tcd = (_0RL_cd_a93046d6d953ea9d_20000000*)cd;
  _impl_ComPcProcess* impl = (_impl_ComPcProcess*) svnt->_ptrToInterface(ComPcProcess::_PD_repoId);
  tcd->result = impl->save_pointCloud(tcd->arg_0);


}

::CORBA::Boolean _objref_ComPcProcess::save_pointCloud(const char* str)
{
  _0RL_cd_a93046d6d953ea9d_20000000 _call_desc(_0RL_lcfn_a93046d6d953ea9d_30000000, "save_pointCloud", 16);
  _call_desc.arg_0 = str;

  _invoke(_call_desc);
  return _call_desc.result;


}
// Proxy call descriptor class. Mangled signature:
//  _cstring_i_a4_a4_cdouble
class _0RL_cd_a93046d6d953ea9d_40000000
  : public omniCallDescriptor
{
public:
  inline _0RL_cd_a93046d6d953ea9d_40000000(LocalCallFn lcfn,const char* op_,size_t oplen,_CORBA_Boolean upcall=0):
     omniCallDescriptor(lcfn, op_, oplen, 0, _user_exns, 0, upcall)
  {
    
  }
  
  void marshalArguments(cdrStream&);
  void unmarshalArguments(cdrStream&);

  void unmarshalReturnedValues(cdrStream&);
  void marshalReturnedValues(cdrStream&);
  
  
  static const char* const _user_exns[];

  ComPcProcess::Matrix4_4 arg_0_;
  const ComPcProcess::Matrix4_4_slice* arg_0;
  ::CORBA::String_var result;
};

void _0RL_cd_a93046d6d953ea9d_40000000::marshalArguments(cdrStream& _n)
{
  
#ifndef OMNI_MIXED_ENDIAN_DOUBLE
  if (! _n.marshal_byte_swap()) {
    _n.put_octet_array((_CORBA_Octet*)((ComPcProcess::Matrix4_4_slice*)arg_0),128,omni::ALIGN_8);
  }
  else 
#endif
  {
    _n.declareArrayLength(omni::ALIGN_8, 128);
    for (_CORBA_ULong _0i0 = 0; _0i0 < 4; _0i0++){
      for (_CORBA_ULong _0i1 = 0; _0i1 < 4; _0i1++){
        arg_0[_0i0][_0i1] >>= _n;
      }
    }
  }

}

void _0RL_cd_a93046d6d953ea9d_40000000::unmarshalArguments(cdrStream& _n)
{
  _n.unmarshalArrayDouble((_CORBA_Double*)((ComPcProcess::Matrix4_4_slice*)arg_0_), 16);
  arg_0 = &arg_0_[0];

}

void _0RL_cd_a93046d6d953ea9d_40000000::marshalReturnedValues(cdrStream& _n)
{
  _n.marshalString(result,0);

}

void _0RL_cd_a93046d6d953ea9d_40000000::unmarshalReturnedValues(cdrStream& _n)
{
  result = _n.unmarshalString(0);

}

const char* const _0RL_cd_a93046d6d953ea9d_40000000::_user_exns[] = {
  0
};

// Local call call-back function.
static void
_0RL_lcfn_a93046d6d953ea9d_50000000(omniCallDescriptor* cd, omniServant* svnt)
{
  _0RL_cd_a93046d6d953ea9d_40000000* tcd = (_0RL_cd_a93046d6d953ea9d_40000000*)cd;
  _impl_ComPcProcess* impl = (_impl_ComPcProcess*) svnt->_ptrToInterface(ComPcProcess::_PD_repoId);
  tcd->result = impl->Capture_PointClould(tcd->arg_0);


}

char* _objref_ComPcProcess::Capture_PointClould(const ::ComPcProcess::Matrix4_4 TransformData)
{
  _0RL_cd_a93046d6d953ea9d_40000000 _call_desc(_0RL_lcfn_a93046d6d953ea9d_50000000, "Capture_PointClould", 20);
  _call_desc.arg_0 = &TransformData[0];

  _invoke(_call_desc);
  return _call_desc.result._retn();


}
// Proxy call descriptor class. Mangled signature:
//  _cstring
class _0RL_cd_a93046d6d953ea9d_60000000
  : public omniCallDescriptor
{
public:
  inline _0RL_cd_a93046d6d953ea9d_60000000(LocalCallFn lcfn,const char* op_,size_t oplen,_CORBA_Boolean upcall=0):
     omniCallDescriptor(lcfn, op_, oplen, 0, _user_exns, 0, upcall)
  {
    
  }
  
  
  void unmarshalReturnedValues(cdrStream&);
  void marshalReturnedValues(cdrStream&);
  
  
  static const char* const _user_exns[];

  ::CORBA::String_var result;
};

void _0RL_cd_a93046d6d953ea9d_60000000::marshalReturnedValues(cdrStream& _n)
{
  _n.marshalString(result,0);

}

void _0RL_cd_a93046d6d953ea9d_60000000::unmarshalReturnedValues(cdrStream& _n)
{
  result = _n.unmarshalString(0);

}

const char* const _0RL_cd_a93046d6d953ea9d_60000000::_user_exns[] = {
  0
};

// Local call call-back function.
static void
_0RL_lcfn_a93046d6d953ea9d_70000000(omniCallDescriptor* cd, omniServant* svnt)
{
  _0RL_cd_a93046d6d953ea9d_60000000* tcd = (_0RL_cd_a93046d6d953ea9d_60000000*)cd;
  _impl_ComPcProcess* impl = (_impl_ComPcProcess*) svnt->_ptrToInterface(ComPcProcess::_PD_repoId);
  tcd->result = impl->Clear_QueueAndPoints();


}

char* _objref_ComPcProcess::Clear_QueueAndPoints()
{
  _0RL_cd_a93046d6d953ea9d_60000000 _call_desc(_0RL_lcfn_a93046d6d953ea9d_70000000, "Clear_QueueAndPoints", 21);


  _invoke(_call_desc);
  return _call_desc.result._retn();


}
// Proxy call descriptor class. Mangled signature:
//  _cstring_i_cstring
class _0RL_cd_a93046d6d953ea9d_80000000
  : public omniCallDescriptor
{
public:
  inline _0RL_cd_a93046d6d953ea9d_80000000(LocalCallFn lcfn,const char* op_,size_t oplen,_CORBA_Boolean upcall=0):
     omniCallDescriptor(lcfn, op_, oplen, 0, _user_exns, 0, upcall)
  {
    
  }
  
  void marshalArguments(cdrStream&);
  void unmarshalArguments(cdrStream&);

  void unmarshalReturnedValues(cdrStream&);
  void marshalReturnedValues(cdrStream&);
  
  
  static const char* const _user_exns[];

  ::CORBA::String_var arg_0_;
  const char* arg_0;
  ::CORBA::String_var result;
};

void _0RL_cd_a93046d6d953ea9d_80000000::marshalArguments(cdrStream& _n)
{
  _n.marshalString(arg_0,0);

}

void _0RL_cd_a93046d6d953ea9d_80000000::unmarshalArguments(cdrStream& _n)
{
  arg_0_ = _n.unmarshalString(0);
  arg_0 = arg_0_.in();

}

void _0RL_cd_a93046d6d953ea9d_80000000::marshalReturnedValues(cdrStream& _n)
{
  _n.marshalString(result,0);

}

void _0RL_cd_a93046d6d953ea9d_80000000::unmarshalReturnedValues(cdrStream& _n)
{
  result = _n.unmarshalString(0);

}

const char* const _0RL_cd_a93046d6d953ea9d_80000000::_user_exns[] = {
  0
};

// Local call call-back function.
static void
_0RL_lcfn_a93046d6d953ea9d_90000000(omniCallDescriptor* cd, omniServant* svnt)
{
  _0RL_cd_a93046d6d953ea9d_80000000* tcd = (_0RL_cd_a93046d6d953ea9d_80000000*)cd;
  _impl_ComPcProcess* impl = (_impl_ComPcProcess*) svnt->_ptrToInterface(ComPcProcess::_PD_repoId);
  tcd->result = impl->SwitchSysMode(tcd->arg_0);


}

char* _objref_ComPcProcess::SwitchSysMode(const char* ModeStr)
{
  _0RL_cd_a93046d6d953ea9d_80000000 _call_desc(_0RL_lcfn_a93046d6d953ea9d_90000000, "SwitchSysMode", 14);
  _call_desc.arg_0 = ModeStr;

  _invoke(_call_desc);
  return _call_desc.result._retn();


}
_pof_ComPcProcess::~_pof_ComPcProcess() {}


omniObjRef*
_pof_ComPcProcess::newObjRef(omniIOR* ior, omniIdentity* id)
{
  return new ::_objref_ComPcProcess(ior, id);
}


::CORBA::Boolean
_pof_ComPcProcess::is_a(const char* id) const
{
  if( omni::ptrStrMatch(id, ::ComPcProcess::_PD_repoId) )
    return 1;
  
  return 0;
}

const _pof_ComPcProcess _the_pof_ComPcProcess;

_impl_ComPcProcess::~_impl_ComPcProcess() {}


::CORBA::Boolean
_impl_ComPcProcess::_dispatch(omniCallHandle& _handle)
{
  const char* op = _handle.operation_name();

  if( omni::strMatch(op, "get_pointCloud") ) {

    _0RL_cd_a93046d6d953ea9d_00000000 _call_desc(_0RL_lcfn_a93046d6d953ea9d_10000000, "get_pointCloud", 15, 1);
    
    _handle.upcall(this,_call_desc);
    return 1;
  }

  if( omni::strMatch(op, "save_pointCloud") ) {

    _0RL_cd_a93046d6d953ea9d_20000000 _call_desc(_0RL_lcfn_a93046d6d953ea9d_30000000, "save_pointCloud", 16, 1);
    
    _handle.upcall(this,_call_desc);
    return 1;
  }

  if( omni::strMatch(op, "Capture_PointClould") ) {

    _0RL_cd_a93046d6d953ea9d_40000000 _call_desc(_0RL_lcfn_a93046d6d953ea9d_50000000, "Capture_PointClould", 20, 1);
    
    _handle.upcall(this,_call_desc);
    return 1;
  }

  if( omni::strMatch(op, "Clear_QueueAndPoints") ) {

    _0RL_cd_a93046d6d953ea9d_60000000 _call_desc(_0RL_lcfn_a93046d6d953ea9d_70000000, "Clear_QueueAndPoints", 21, 1);
    
    _handle.upcall(this,_call_desc);
    return 1;
  }

  if( omni::strMatch(op, "SwitchSysMode") ) {

    _0RL_cd_a93046d6d953ea9d_80000000 _call_desc(_0RL_lcfn_a93046d6d953ea9d_90000000, "SwitchSysMode", 14, 1);
    
    _handle.upcall(this,_call_desc);
    return 1;
  }


  return 0;
}

void*
_impl_ComPcProcess::_ptrToInterface(const char* id)
{
  if( id == ::ComPcProcess::_PD_repoId )
    return (::_impl_ComPcProcess*) this;
  
  if( id == ::CORBA::Object::_PD_repoId )
    return (void*) 1;

  if( omni::strMatch(id, ::ComPcProcess::_PD_repoId) )
    return (::_impl_ComPcProcess*) this;
  
  if( omni::strMatch(id, ::CORBA::Object::_PD_repoId) )
    return (void*) 1;
  return 0;
}

const char*
_impl_ComPcProcess::_mostDerivedRepoId()
{
  return ::ComPcProcess::_PD_repoId;
}

POA_ComPcProcess::~POA_ComPcProcess() {}

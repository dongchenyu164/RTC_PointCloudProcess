// This file is generated by omniidl (C++ backend)- omniORB_4_1. Do not edit.
#ifndef __pcProcessPort_hh__
#define __pcProcessPort_hh__

#ifndef __CORBA_H_EXTERNAL_GUARD__
#include <omniORB4/CORBA.h>
#endif

#ifndef  USE_stub_in_nt_dll
# define USE_stub_in_nt_dll_NOT_DEFINED_pcProcessPort
#endif
#ifndef  USE_core_stub_in_nt_dll
# define USE_core_stub_in_nt_dll_NOT_DEFINED_pcProcessPort
#endif
#ifndef  USE_dyn_stub_in_nt_dll
# define USE_dyn_stub_in_nt_dll_NOT_DEFINED_pcProcessPort
#endif



#ifndef __BasicDataType_hh_EXTERNAL_GUARD__
#define __BasicDataType_hh_EXTERNAL_GUARD__
#include "BasicDataType.hh"
#endif
#ifndef __ExtendedDataTypes_hh_EXTERNAL_GUARD__
#define __ExtendedDataTypes_hh_EXTERNAL_GUARD__
#include "ExtendedDataTypes.hh"
#endif
#ifndef __InterfaceDataTypes_hh_EXTERNAL_GUARD__
#define __InterfaceDataTypes_hh_EXTERNAL_GUARD__
#include "InterfaceDataTypes.hh"
#endif



#ifdef USE_stub_in_nt_dll
# ifndef USE_core_stub_in_nt_dll
#  define USE_core_stub_in_nt_dll
# endif
# ifndef USE_dyn_stub_in_nt_dll
#  define USE_dyn_stub_in_nt_dll
# endif
#endif

#ifdef _core_attr
# error "A local CPP macro _core_attr has already been defined."
#else
# ifdef  USE_core_stub_in_nt_dll
#  define _core_attr _OMNIORB_NTDLL_IMPORT
# else
#  define _core_attr
# endif
#endif

#ifdef _dyn_attr
# error "A local CPP macro _dyn_attr has already been defined."
#else
# ifdef  USE_dyn_stub_in_nt_dll
#  define _dyn_attr _OMNIORB_NTDLL_IMPORT
# else
#  define _dyn_attr
# endif
#endif





#ifndef __ComPcProcess__
#define __ComPcProcess__

class ComPcProcess;
class _objref_ComPcProcess;
class _impl_ComPcProcess;

typedef _objref_ComPcProcess* ComPcProcess_ptr;
typedef ComPcProcess_ptr ComPcProcessRef;

class ComPcProcess_Helper {
public:
  typedef ComPcProcess_ptr _ptr_type;

  static _ptr_type _nil();
  static _CORBA_Boolean is_nil(_ptr_type);
  static void release(_ptr_type);
  static void duplicate(_ptr_type);
  static void marshalObjRef(_ptr_type, cdrStream&);
  static _ptr_type unmarshalObjRef(cdrStream&);
};

typedef _CORBA_ObjRef_Var<_objref_ComPcProcess, ComPcProcess_Helper> ComPcProcess_var;
typedef _CORBA_ObjRef_OUT_arg<_objref_ComPcProcess,ComPcProcess_Helper > ComPcProcess_out;

#endif

// interface ComPcProcess
class ComPcProcess {
public:
  // Declarations for this interface type.
  typedef ComPcProcess_ptr _ptr_type;
  typedef ComPcProcess_var _var_type;

  static _ptr_type _duplicate(_ptr_type);
  static _ptr_type _narrow(::CORBA::Object_ptr);
  static _ptr_type _unchecked_narrow(::CORBA::Object_ptr);
  
  static _ptr_type _nil();

  static inline void _marshalObjRef(_ptr_type, cdrStream&);

  static inline _ptr_type _unmarshalObjRef(cdrStream& s) {
    omniObjRef* o = omniObjRef::_unMarshal(_PD_repoId,s);
    if (o)
      return (_ptr_type) o->_ptrToObjRef(_PD_repoId);
    else
      return _nil();
  }

  static _core_attr const char* _PD_repoId;

  // Other IDL defined within this scope.
  static _dyn_attr const ::CORBA::TypeCode_ptr _tc_Matrix4_4;

  typedef ::CORBA::Double Matrix4_4[4][4];
  typedef ::CORBA::Double Matrix4_4_slice[4];

  static inline Matrix4_4_slice* Matrix4_4_alloc() {
    return new Matrix4_4_slice[4];
  }

  static inline Matrix4_4_slice* Matrix4_4_dup(const Matrix4_4_slice* _s) {
    if (!_s) return 0;
    Matrix4_4_slice* _data = Matrix4_4_alloc();
    if (_data) {
      for (_CORBA_ULong _0i0 = 0; _0i0 < 4; _0i0++){
        for (_CORBA_ULong _0i1 = 0; _0i1 < 4; _0i1++){
          
          _data[_0i0][_0i1] = _s[_0i0][_0i1];

        }
      }
  
    }
    return _data;
  }

  static inline void Matrix4_4_copy(Matrix4_4_slice* _to, const Matrix4_4_slice* _from){
    for (_CORBA_ULong _0i0 = 0; _0i0 < 4; _0i0++){
      for (_CORBA_ULong _0i1 = 0; _0i1 < 4; _0i1++){
        
        _to[_0i0][_0i1] = _from[_0i0][_0i1];

      }
    }
  
  }

  static inline void Matrix4_4_free(Matrix4_4_slice* _s) {
    delete [] _s;
  }

  class Matrix4_4_copyHelper {
  public:
    static inline Matrix4_4_slice* alloc() { return ::ComPcProcess::Matrix4_4_alloc(); }
    static inline Matrix4_4_slice* dup(const Matrix4_4_slice* p) { return ::ComPcProcess::Matrix4_4_dup(p); }
    static inline void free(Matrix4_4_slice* p) { ::ComPcProcess::Matrix4_4_free(p); }
  };

  typedef _CORBA_Array_Fix_Var<Matrix4_4_copyHelper,Matrix4_4_slice> Matrix4_4_var;
  typedef _CORBA_Array_Fix_Forany<Matrix4_4_copyHelper,Matrix4_4_slice> Matrix4_4_forany;

  typedef Matrix4_4_slice* Matrix4_4_out;


};

class _objref_ComPcProcess :
  public virtual ::CORBA::Object,
  public virtual omniObjRef
{
public:
  RTC::PointCloud* get_pointCloud(::CORBA::Boolean& flag);
  ::CORBA::Boolean save_pointCloud(const char* str);
  char* Capture_PointClould(const ::ComPcProcess::Matrix4_4 TransformData);
  char* Clear_QueueAndPoints();
  char* SwitchSysMode(const char* ModeStr);

  inline _objref_ComPcProcess()  { _PR_setobj(0); }  // nil
  _objref_ComPcProcess(omniIOR*, omniIdentity*);

protected:
  virtual ~_objref_ComPcProcess();

  
private:
  virtual void* _ptrToObjRef(const char*);

  _objref_ComPcProcess(const _objref_ComPcProcess&);
  _objref_ComPcProcess& operator = (const _objref_ComPcProcess&);
  // not implemented

  friend class ComPcProcess;
};

class _pof_ComPcProcess : public _OMNI_NS(proxyObjectFactory) {
public:
  inline _pof_ComPcProcess() : _OMNI_NS(proxyObjectFactory)(ComPcProcess::_PD_repoId) {}
  virtual ~_pof_ComPcProcess();

  virtual omniObjRef* newObjRef(omniIOR*,omniIdentity*);
  virtual _CORBA_Boolean is_a(const char*) const;
};

class _impl_ComPcProcess :
  public virtual omniServant
{
public:
  virtual ~_impl_ComPcProcess();

  virtual RTC::PointCloud* get_pointCloud(::CORBA::Boolean& flag) = 0;
  virtual ::CORBA::Boolean save_pointCloud(const char* str) = 0;
  virtual char* Capture_PointClould(const ::ComPcProcess::Matrix4_4 TransformData) = 0;
  virtual char* Clear_QueueAndPoints() = 0;
  virtual char* SwitchSysMode(const char* ModeStr) = 0;
  
public:  // Really protected, workaround for xlC
  virtual _CORBA_Boolean _dispatch(omniCallHandle&);

private:
  virtual void* _ptrToInterface(const char*);
  virtual const char* _mostDerivedRepoId();
  
};


_CORBA_GLOBAL_VAR _dyn_attr const ::CORBA::TypeCode_ptr _tc_ComPcProcess;



class POA_ComPcProcess :
  public virtual _impl_ComPcProcess,
  public virtual ::PortableServer::ServantBase
{
public:
  virtual ~POA_ComPcProcess();

  inline ::ComPcProcess_ptr _this() {
    return (::ComPcProcess_ptr) _do_this(::ComPcProcess::_PD_repoId);
  }
};







#undef _core_attr
#undef _dyn_attr

void operator<<=(::CORBA::Any& _a, const ComPcProcess::Matrix4_4_forany& _s);
_CORBA_Boolean operator>>=(const ::CORBA::Any& _a, ComPcProcess::Matrix4_4_forany& _s);

void operator<<=(::CORBA::Any& _a, ComPcProcess_ptr _s);
void operator<<=(::CORBA::Any& _a, ComPcProcess_ptr* _s);
_CORBA_Boolean operator>>=(const ::CORBA::Any& _a, ComPcProcess_ptr& _s);



inline void
ComPcProcess::_marshalObjRef(::ComPcProcess_ptr obj, cdrStream& s) {
  omniObjRef::_marshal(obj->_PR_getobj(),s);
}




#ifdef   USE_stub_in_nt_dll_NOT_DEFINED_pcProcessPort
# undef  USE_stub_in_nt_dll
# undef  USE_stub_in_nt_dll_NOT_DEFINED_pcProcessPort
#endif
#ifdef   USE_core_stub_in_nt_dll_NOT_DEFINED_pcProcessPort
# undef  USE_core_stub_in_nt_dll
# undef  USE_core_stub_in_nt_dll_NOT_DEFINED_pcProcessPort
#endif
#ifdef   USE_dyn_stub_in_nt_dll_NOT_DEFINED_pcProcessPort
# undef  USE_dyn_stub_in_nt_dll
# undef  USE_dyn_stub_in_nt_dll_NOT_DEFINED_pcProcessPort
#endif

#endif  // __pcProcessPort_hh__

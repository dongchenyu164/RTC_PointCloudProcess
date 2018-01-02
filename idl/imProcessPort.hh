// This file is generated by omniidl (C++ backend)- omniORB_4_1. Do not edit.
#ifndef __imProcessPort_hh__
#define __imProcessPort_hh__

#ifndef __CORBA_H_EXTERNAL_GUARD__
#include <omniORB4/CORBA.h>
#endif

#ifndef  USE_stub_in_nt_dll
# define USE_stub_in_nt_dll_NOT_DEFINED_imProcessPort
#endif
#ifndef  USE_core_stub_in_nt_dll
# define USE_core_stub_in_nt_dll_NOT_DEFINED_imProcessPort
#endif
#ifndef  USE_dyn_stub_in_nt_dll
# define USE_dyn_stub_in_nt_dll_NOT_DEFINED_imProcessPort
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





#ifndef __ComImProcess__
#define __ComImProcess__

class ComImProcess;
class _objref_ComImProcess;
class _impl_ComImProcess;

typedef _objref_ComImProcess* ComImProcess_ptr;
typedef ComImProcess_ptr ComImProcessRef;

class ComImProcess_Helper {
public:
  typedef ComImProcess_ptr _ptr_type;

  static _ptr_type _nil();
  static _CORBA_Boolean is_nil(_ptr_type);
  static void release(_ptr_type);
  static void duplicate(_ptr_type);
  static void marshalObjRef(_ptr_type, cdrStream&);
  static _ptr_type unmarshalObjRef(cdrStream&);
};

typedef _CORBA_ObjRef_Var<_objref_ComImProcess, ComImProcess_Helper> ComImProcess_var;
typedef _CORBA_ObjRef_OUT_arg<_objref_ComImProcess,ComImProcess_Helper > ComImProcess_out;

#endif

// interface ComImProcess
class ComImProcess {
public:
  // Declarations for this interface type.
  typedef ComImProcess_ptr _ptr_type;
  typedef ComImProcess_var _var_type;

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
  
};

class _objref_ComImProcess :
  public virtual ::CORBA::Object,
  public virtual omniObjRef
{
public:
  RTC::CameraImage* get_colorImage(::CORBA::Boolean& ref);
  RTC::CameraImage* get_irImage(::CORBA::Boolean& ref);
  RTC::CameraImage* get_depthImage(::CORBA::Boolean& ref);
  ::CORBA::Boolean save_colorImage(const char* str);
  ::CORBA::Boolean save_irImage(const char* str);
  ::CORBA::Boolean save_depthImage(const char* str);

  inline _objref_ComImProcess()  { _PR_setobj(0); }  // nil
  _objref_ComImProcess(omniIOR*, omniIdentity*);

protected:
  virtual ~_objref_ComImProcess();

  
private:
  virtual void* _ptrToObjRef(const char*);

  _objref_ComImProcess(const _objref_ComImProcess&);
  _objref_ComImProcess& operator = (const _objref_ComImProcess&);
  // not implemented

  friend class ComImProcess;
};

class _pof_ComImProcess : public _OMNI_NS(proxyObjectFactory) {
public:
  inline _pof_ComImProcess() : _OMNI_NS(proxyObjectFactory)(ComImProcess::_PD_repoId) {}
  virtual ~_pof_ComImProcess();

  virtual omniObjRef* newObjRef(omniIOR*,omniIdentity*);
  virtual _CORBA_Boolean is_a(const char*) const;
};

class _impl_ComImProcess :
  public virtual omniServant
{
public:
  virtual ~_impl_ComImProcess();

  virtual RTC::CameraImage* get_colorImage(::CORBA::Boolean& ref) = 0;
  virtual RTC::CameraImage* get_irImage(::CORBA::Boolean& ref) = 0;
  virtual RTC::CameraImage* get_depthImage(::CORBA::Boolean& ref) = 0;
  virtual ::CORBA::Boolean save_colorImage(const char* str) = 0;
  virtual ::CORBA::Boolean save_irImage(const char* str) = 0;
  virtual ::CORBA::Boolean save_depthImage(const char* str) = 0;
  
public:  // Really protected, workaround for xlC
  virtual _CORBA_Boolean _dispatch(omniCallHandle&);

private:
  virtual void* _ptrToInterface(const char*);
  virtual const char* _mostDerivedRepoId();
  
};


_CORBA_GLOBAL_VAR _dyn_attr const ::CORBA::TypeCode_ptr _tc_ComImProcess;



class POA_ComImProcess :
  public virtual _impl_ComImProcess,
  public virtual ::PortableServer::ServantBase
{
public:
  virtual ~POA_ComImProcess();

  inline ::ComImProcess_ptr _this() {
    return (::ComImProcess_ptr) _do_this(::ComImProcess::_PD_repoId);
  }
};







#undef _core_attr
#undef _dyn_attr

void operator<<=(::CORBA::Any& _a, ComImProcess_ptr _s);
void operator<<=(::CORBA::Any& _a, ComImProcess_ptr* _s);
_CORBA_Boolean operator>>=(const ::CORBA::Any& _a, ComImProcess_ptr& _s);



inline void
ComImProcess::_marshalObjRef(::ComImProcess_ptr obj, cdrStream& s) {
  omniObjRef::_marshal(obj->_PR_getobj(),s);
}




#ifdef   USE_stub_in_nt_dll_NOT_DEFINED_imProcessPort
# undef  USE_stub_in_nt_dll
# undef  USE_stub_in_nt_dll_NOT_DEFINED_imProcessPort
#endif
#ifdef   USE_core_stub_in_nt_dll_NOT_DEFINED_imProcessPort
# undef  USE_core_stub_in_nt_dll
# undef  USE_core_stub_in_nt_dll_NOT_DEFINED_imProcessPort
#endif
#ifdef   USE_dyn_stub_in_nt_dll_NOT_DEFINED_imProcessPort
# undef  USE_dyn_stub_in_nt_dll
# undef  USE_dyn_stub_in_nt_dll_NOT_DEFINED_imProcessPort
#endif

#endif  // __imProcessPort_hh__


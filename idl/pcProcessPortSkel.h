// -*- C++ -*-
/*!
 *
 * THIS FILE IS GENERATED AUTOMATICALLY!! DO NOT EDIT!!
 *
 * @file pcProcessPortSkel.h 
 * @brief pcProcessPort server skeleton header wrapper code
 * @date Wed May 24 16:53:11 2017 
 *
 */

#ifndef _PCPROCESSPORTSKEL_H
#define _PCPROCESSPORTSKEL_H



#include <rtm/config_rtc.h>
#undef PACKAGE_BUGREPORT
#undef PACKAGE_NAME
#undef PACKAGE_STRING
#undef PACKAGE_TARNAME
#undef PACKAGE_VERSION

#if   defined ORB_IS_TAO
#  include "pcProcessPortC.h"
#  include "pcProcessPortS.h"
#elif defined ORB_IS_OMNIORB
#  if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
#    undef USE_stub_in_nt_dll
#  endif
#  include "pcProcessPort.hh"
#elif defined ORB_IS_MICO
#  include "pcProcessPort.h"
#elif defined ORB_IS_ORBIT2
#  include "/pcProcessPort-cpp-stubs.h"
#  include "/pcProcessPort-cpp-skels.h"
#elif defined ORB_IS_RTORB
#  include "pcProcessPort.h"
#else
#  error "NO ORB defined"
#endif

#endif // _PCPROCESSPORTSKEL_H

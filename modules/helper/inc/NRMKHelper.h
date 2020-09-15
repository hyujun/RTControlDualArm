/* NRMKFoundation, Copyright 2013- Neuromeka. All rights reserved.
 *
 * This library is commercial and cannot be redistributed, and/or modified
 * WITHOUT ANY ALLOWANCE OR PERMISSION OF Neuromeka
 */
#pragma once

//
// Ensure that NRMKHELPER_DLL is default unless NRMKHELPER_STATIC is defined
//
#if defined(NRMKHELPER_STATIC)

#define NRMKHelper_API

#else


#if defined(_WIN32) && defined(_DLL)
	#if !defined(NRMKHELPER_DLL) // && !defined(NRMKHELPER_STATIC)
		#define NRMKHELPER_DLL
	#endif
#endif


//
// The following block is the standard way of creating macros which make exporting 
// from a DLL simpler. All files within this DLL are compiled with the Foundation_EXPORTS
// symbol defined on the command line. this symbol should not be defined on any project
// that uses this DLL. This way any other project whose source files include this file see 
// Foundation_API functions as being imported from a DLL, whereas this DLL sees symbols
// defined with this macro as being exported.
//
#if (defined(_WIN32) || defined(_WIN32_WCE)) && defined(NRMKHELPER_DLL)
	#if defined(NRMKHELPER_EXPORTS)
		#define NRMKHelper_API __declspec(dllexport)
	#else
		#define NRMKHelper_API __declspec(dllimport)	
	#endif
#endif

#if !defined(NRMKHelper_API)
	#define NRMKHelper_API
#endif

//
// Automatically link Foundation library.
//
#if defined(_MSC_VER)
	#if defined(NRMKHELPER_DLL)
		#if defined(_DEBUG)
			#define NRMKHELPER_LIB_SUFFIX "d.lib"
		#else
			#define NRMKHELPER_LIB_SUFFIX ".lib"
		#endif
	#else
		#if defined(_DEBUG)
			#define NRMKHELPER_LIB_SUFFIX "d.lib"
		#else
			#define NRMKHELPER_LIB_SUFFIX ".lib"
		#endif
	#endif

	#if !defined(NRMKHELPER_NO_AUTOMATIC_LIBS) && !defined(NRMKHELPER_EXPORTS)
		#pragma comment(lib, "NRMKHelper" NRMKHELPER_LIB_SUFFIX)
	#endif
#endif

#endif 
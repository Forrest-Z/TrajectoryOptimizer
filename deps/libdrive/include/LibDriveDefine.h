#ifndef LIB_DRIVE_DEFINE_H_ 
#define LIB_DRIVE_DEFINE_H_

#ifdef _WIN32
#include "messages.pb.h"
#include "chassis_messages.pb.h"
#else
#include "messages.pb.h"
#include "chassis_messages.pb.h"
#endif

#if defined (WIN32) || defined (_WIN64)   

#ifdef LIBDRIVE_EXPORTS
#define LIBDRIVE_API extern "C" __declspec(dllexport)
#else
#define LIBDRIVE_API extern "C" __declspec(dllimport)
#endif //LIBDRIVE_EXPORTS

#define CALL  __stdcall

typedef __int64 int64;

#elif defined __linux__					

#ifndef LIBDRIVE_API
#define LIBDRIVE_API extern "C"
#endif //LIBDRIVE_API

#define CALL	

typedef long long  int64;

#endif //defined (WIN32) || defined (_WIN64) 






#endif

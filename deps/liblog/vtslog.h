#pragma once
#include <string>

#if defined (WIN32) || defined (_WIN64)   

#ifdef VTSLOG_EXPORTS
#define VTSLOG_API extern "C" __declspec(dllexport)
#define LOG_API __declspec(dllexport)
#else
#define VTSLOG_API extern "C" __declspec(dllimport)
#define LOG_API __declspec(dllimport)
#endif //VTSLOG_API

#define CALL  __stdcall

#elif defined __linux__					

#ifndef VTSLOG_API
#define VTSLOG_API extern "C"
#define LOG_API 
#endif //VTSLOG_API

#define CALL	

#endif //defined (WIN32) || defined (_WIN64) 

namespace vts
{
namespace log
{
	enum LOG_LEVEL {
	LEVEL_INFO,
	LEVEL_WARN,
	LEVEL_ERROR,
	};
	enum UPLOAD_TYPE
{
	TCP,
		FTP,
	};

class logimpl;
class LOG_API vtslog {
	public:
		vtslog();
		~vtslog();
		 void  InitLog( const bool bstdout, const LOG_LEVEL stdoutLevel );
		 int  CrtFile( const std::string& casename );
		 int  ClsFile( const std::string& casename );
		 int  StartUpLoad( const UPLOAD_TYPE uploadtype, const std::string& addr, const std::string& usrpwd );
		 int  Log( LOG_LEVEL level, const std::string& filename, const int line,
			const int topicnum, const std::string topic[],
			const char* szFmt, ... );
	private:
		logimpl* impl_;


};
}
}


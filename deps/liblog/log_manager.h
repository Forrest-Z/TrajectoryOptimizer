#pragma once
#include "vtslog.h"

#define LOG_INFO(fmt, ...) LogInst->log_.Log(vts::log::LEVEL_INFO, __FILE__, __LINE__, 1,  LogInst->topic,  fmt, ##__VA_ARGS__ )
#define LOG_WARN(fmt, ...) LogInst->log_.Log(vts::log::LEVEL_WARN, __FILE__, __LINE__, 1,  LogInst->topic,  fmt, ##__VA_ARGS__ );
#define LOG_ERRO(fmt, ...) LogInst->log_.Log(vts::log::LEVEL_ERROR, __FILE__, __LINE__, 1,  LogInst->topic,  fmt, ##__VA_ARGS__ );


#define LOG_EVERY_N(num, n, fmt, ...) \
	if(num % n == 0){ \
		LOG_INFO(fmt,  ##__VA_ARGS__); \
	};  \

class log_manager
{
public:
	log_manager();
	virtual ~log_manager();
	static log_manager* GetInstance()
	{
		static log_manager instance;
		return &instance;
	}

	int init();

	vts::log::vtslog log_;

	std::string topic[1] = { "planning" };

};
#define LogInst log_manager::GetInstance() 

#include "log_manager.h"

log_manager::log_manager()
{}
log_manager::~log_manager()
{
	log_.ClsFile( "planning" );
}
int log_manager::init()
{
	log_.InitLog( true, vts::log::LEVEL_INFO );
	log_.CrtFile( "planning" );
	log_.StartUpLoad( vts::log::TCP, "10.0.104.86:10086", "" );

	return 0;
}

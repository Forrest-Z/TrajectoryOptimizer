#ifndef LIB_DRIVE_H_ 
#define LIB_DRIVE_H_

#include "LibDriveDefine.h"
namespace vts {
namespace drive {

struct TARGET_STATE
{
	float V;	
	float X;
	float Y;
	float Z;
	float Orientation_Z;
};

enum GetMessageMode
{
	kLastMessage     = 0,	
	kOneByOneMessage = 1,	
};

enum ReturnCode
{
	kOk						= 0,
	kErrEmptyChannelList	= -1,		
	kErrCannotFindChannel	= -2,			
	kErrChannelPtrNull		= -3,		
	kErrMessagePtrNull		= -4,		
	kErrMessagePtrError	    = -5,		
	kErrMessagePtrEmpty     = -6,		
	kErrMessageTypeUnknow   = -7,		
	kErrParseProtoError	    = -8,		
	kErrProtoSizeError		= -9,		
	kErrChannelNetError     = -10,   
	kErrReadJsonError       = -11,	
	kErrParseJsonError		= -12,	
	kErrPrepareTimeout		= -13,		
	kErrMessageFromSelf		= -14,		
	kErrActorIdEmpty		= -15,  
	kErrCaseIdError			= -16,		
	kErrBriefDataEmpty		= -17,		
	kErrSessionIdError		= -18,	
};

	LIBDRIVE_API int CALL VTS_Init( std::string & configPath );

	LIBDRIVE_API int CALL VTS_GetSimTime( int64 & simTime );

	LIBDRIVE_API int CALL VTS_Prepare( int timeout );

	LIBDRIVE_API int CALL VTS_GetSessionId( std::string & sessionId );

	LIBDRIVE_API int CALL VTS_GetBriefData( std::string & briefData );

	LIBDRIVE_API int CALL VTS_GetCaseName( std::string & caseName );

	LIBDRIVE_API int CALL VTS_GetCaseInfo( const std::string & caseName, TARGET_STATE & stTargetState, std::string & roleId );

	LIBDRIVE_API int CALL VTS_GetMapName( const std::string & anchorSegmentName, std::string & mapName );

	LIBDRIVE_API int CALL VTS_GetPrepare( vts::protocol::main::ActorPrepare & protoActorPrepare );

	LIBDRIVE_API int CALL VTS_PostPrepareResult( vts::protocol::main::ActorPrepareResult & protoActorPrepareResult );

	LIBDRIVE_API int CALL VTS_PostVehicleControl( vts::protocol::chassis::VehicleControl & protoVehicleControl );
	
	LIBDRIVE_API int CALL VTS_GetVehicleControl( vts::protocol::chassis::VehicleControl& protoVehicleControl, GetMessageMode mode, unsigned int & remainingNumber );

	LIBDRIVE_API int CALL VTS_PostVehicleFeedback( vts::protocol::chassis::VehicleFeedback & protoVehicleFeedback );

	LIBDRIVE_API int CALL VTS_GetVehicleFeedback( vts::protocol::chassis::VehicleFeedback & protoVehicleFeedback, GetMessageMode mode, unsigned int & remainingNumber );

	LIBDRIVE_API int CALL VTS_PostPubRole( vts::protocol::main::PubRole & protoPubRole );

	LIBDRIVE_API int CALL VTS_GetPubRole( vts::protocol::main::PubRole & protoPubRole, GetMessageMode mode, unsigned int & remainingNumber );

	LIBDRIVE_API int CALL VTS_GetNotify( vts::protocol::main::Notify & protoNotify );

	LIBDRIVE_API int CALL VTS_GetSessionInfo( vts::protocol::main::SessionInfo & protoSessionInfo );

	LIBDRIVE_API void CALL VTS_SetClientName( std::string & clientName );

	LIBDRIVE_API int CALL VTS_GetTryTest( vts::protocol::main::TryTest & protoTryTest );

	LIBDRIVE_API int CALL VTS_PostTryTest( vts::protocol::main::TryTest & protoTryTest );
}

}


#endif


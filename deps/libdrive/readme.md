---

---

------

# 																							LibDrive SDK开发手册

## 1. 内容简介

### 1.1 概述

LibDrive SDK是用户开发驾驶程序相关的二次开发包,主要包含的内容如下：

```
.h文件：
	proto message 相关：
		chassis_enums.pb.h
		chassis_messages.pb.h
		enums.pb.h
		fields.pb.h
		messages.pb.h
	sdk 接口相关
		LibDrive.h
		LibDriveDefine.h
```

```
Windows 64
.lib文件：
		LibDrive.lib
		libprotobuf.lib
		libprotoc.lib
.dll文件：
	sdk接口链接库：
		LibDrive.dll
	sdk依赖库：	
		vtslog.dll
		libcurl.dll
		zlibwapi.dll
		libssl-1_1-x64.dll
		libcrypto-1_1-x64.dll
		LibMulticastNetwork.dll
		boost_filesystem-vc141-mt-x64-1_73.dll

```

```
Linux
	libDrive.so
	libMulticastNetwork.so
	libvtslog.so
	libprotobuf.so
	libprotobuf.so.22
	libprotobuf.so.22.0.0
	libprotobuf-lite.so
	libprotobuf-lite.so.22
	libprotobuf-lite.so.22.0.0
	libprotoc.so
	libprotoc.so.22
	libprotoc.so.22.0.0
	libz.so
	libz.so.1
	libz.so.1.2.11
	libboost_filesystem.so
	libboost_filesystem.so.1.73.0
	libminizip.so
```

```python
.ini文件：
	common_config.ini内容如下：
		
	[common]				# 公共配置项
	mc_ip= 239.255.255.105	# 组播ip
	local_ip = 10.0.105.81	# 本地接收组播消息的网卡ip
	max_queue_size = 200	# 发送/接收队列的最大长度
	msg_size = 65536		# 发送/接收队列中每个内存块的长度
	drop_msg_strategy = 2   # 队列满之后消息丢弃规则 1:全丢，2丢弃最旧的一个

	channel_config_testee.ini内容如下：
    
    [pubrole]				# PubRole数据的通信channel
	mc_port= 10080			# 组播端口
	type=1					# 类型，1为只接收，2为只发送，3为接收发送									
    						# testee角色只接收pubrole数据
	local_port = 10080		# 本地接收组播消息的端口
	
	[vehiclecontrol]		# VehicleControl和VehicleFeedback数据的通信channel
	mc_port= 10081			# 组播端口	
	type=3					# 类型，1为只接收，2为只发送，3为接收发送	
    						# testee角色需发送vehiclecontrol并且接收vehiclefeedback
	local_port = 10081		# 本地接收组播消息的端口
	
	[prepare]				# ActorPrepare和ActorPrepareResult数据的通信channel
	mc_port= 10082			# 组播端口	
	type=3					# 类型，1为只接收，2为只发送，3为接收发送	
    						# testee角色需接收ActorPrepare和发ActorPrepareResult
	local_port = 10082		# 本地接收组播消息的端口
    
    [notify]				# Notify数据的通信channel
	mc_port= 10083			# 组播端口	
	type=1					# 类型，1为只接收，2为只发送，3为接收发送	
    						# testee角色只接收Notify数据
	local_port = 10083		# 本地接收组播消息的端口
	
	[heartbeat]				# 心跳组(必须设置的组)，daemon设置为1，其他客户端设置为2
	mc_port= 10084			# 组播端口
	type=2					# 类型，1为只接收，2为只发送，3为接收发送 #testee设置为2
	local_port = 10084		# 本地接收组播消息的端口	
		
```

### 1.2 系统要求

```javascript
Windows下的SDK:
	64位SDK
	Windows 10
Linux 下的SDK
	Ubuntu16.04
```

## 2. SDK接口调用的主要流程

![](https://gitee.com/windSeS/imgbed/raw/master/122.png)



## 3. SDK基本定义

### 3.1  数据结构定义

```c++
/*
 *Describe:		通过调用VTS_GetCaseInfo获取案例中的目标位置
 *Data: 		float V - 目标速度
 *Data:         float X - 目标点的x坐标
 *Data:         float Y - 目标点的y坐标
 *Data:			float Z - 目标点的z坐标
 */
typedef struct TARGET_STATE
{
	float V;	
	float X;
	float Y;
	float Z;
};
```

```c++
/*
 *Describe:		VTS_GetPubRole和VTS_GetVehicleFeedback可以选择获取消息的模式
 */
enum GetMessageMode
{
	kLastMessage     = 0,		///< 获取最新的消息
	kOneByOneMessage = 1,	    ///< 一个个的获取
};
```



```c++
/*
 *Describe:		sdk 接口的返回值
 */
enum ReturnCode
{
	kOk						= 0,
	kErrEmptyChannelList	= -1,			///< 没有channel
	kErrCannotFindChannel	= -2,			///< 无法找到对应的channel
	kErrChannelPtrNull		= -3,			///< channel ptr为空
	kErrMessagePtrNull		= -4,			///< 消息指针为null
	kErrMessagePtrError	    = -5,			///< 消息错误
	kErrMessagePtrEmpty     = -6,			///< 消息为空
	kErrMessageTypeUnknow   = -7,			///< 不是当前proto对应的类型的消息
	kErrParseProtoError	    = -8,			///< proto解析错误
	kErrProtoSizeError		= -9,			///< proto长度为0
	kErrChannelNetError     = -10,          ///< 对应网络库中的错误
	kErrReadJsonError       = -11,			///< read json文件错误
	kErrParseJsonError		= -12,			///< parse json文件错误
	kErrPrepareTimeout		= -13,			///< prepare超时
	kErrMessageFromSelf		= -14,			///< 消息来自于自身，需过滤		
	kErrActorIdEmpty		= -15,          ///< actor_id无效，
    										///< 先Prepare后GetCaseInfo
};
```

### 3.2  接口定义

```c++
/*
 *Describe:		SDK初始化,用于开启数据通信channel
 *Param: 		configPath [IN] - 如果configPath="";
 				则在执行程序目录下读取common_config.ini和channel_config_testee.ini; 
 				或者按如下的格式配置， 如：configPath="C:/config/"
 *Return Value: kOk - 成功
 *				kErrEmptyChannelList - 没有channel被开启
 */
int VTS_Init( std::string & configPath );
```

```c++
/*
 *Describe:		准备Test接口
 *Param: 		timeout [IN] - 获取准备消息的超时时间，
 				建议设置大一些，允许更多等待时间，单位是秒
 *Return Value: kOk - 成功
				kErrPrepareTimeout - 准备超时
 */
int VTS_Prepare( int timeout );
```



```C++
/*
 *Describe:		获取案例的目标点信息，在prepare成功之后获取
 *Param: 		caseNmae [IN]       - 下载到本地的案例 如：C:/case.json
 *Param:		stTargetState [OUT] - 目标点信息
 *Param:		roleId	[OUT]       - 获取case中的roleId
 *Return Value: kOk - 成功获取目标点信息
 *				kErrReadJsonError - 读取json文件错误
 *				kErrParseJsonError - 分析json文件错误 
 */
int VTS_GetCaseInfo( const std::string & caseName, TARGET_STATE & stTargetState, std::string & roleId );
```

```c++
/*
 *Describe:		从案例数据文件中获取地图名称
 *Param: 		anchorSegmentName [IN] - 下载到本地的案例数据 
 				如：C:/anchor_segment.json
 *Param:		mapName [OUT] - 地图名称，可以根据此地图名称对应去加载本地下载的地图
 *Return Value: kOk - 成功
 *				kErrReadJsonError  - 读取json文件错误
 *				kErrParseJsonError - 分析json文件错误
 */
int VTS_GetMapName( const std::string & anchorSegmentName, std::string & mapName );
```

```c++
/*
 *Describe:		获取仿真时间 可实时获取
 *Param: 		simTime [OUT] - 仿真时间戳
 *Return Value: kOk - 成功
 *				若还未启动测试，则返回的是系统时间戳
 */
int VTS_GetSimTime( int64 & simTime )
```

```c++
/*
 *Describe:		通过channel:prepare接收daemon发送的ActorPrepare，
 				如果调用了VTS_Prepare，就无须做此逻辑
 *Param: 		protoActorPrepare [OUT] - 接收到的ActorPrepare
 *Return Value: >= 0 - 成功获取到消息，以及当前channel中还剩余的消息数量
 *				kErrCannotFindChannel - 没有找到channel：prepare
 *				kErrChannelPtrNull    - 对应的channel指针为null
 *				kErrMessagePtrNull	  - 创建的消息指针为null
 *				kErrMessagePtrError   - 创建的消息指针错误
 *				kErrProtoSizeError    - 表示获取到的消息通过此proto反序列化失败
 *				kErrMessageTypeUnknow - 表示获取到的消息不是对应此消息类型的数据
 *				kErrMessageFromSelf	  - 表示获取到的消息是本进程发出的,在接口内过滤
 */
int VTS_GetPrepare( vts::protocol::main::ActorPrepare & protoActorPrepare );
```

```c++
/*
 *Describe:		通过channel:prepare发送ActorPrepareResult消息给daemon，
 				如果调用了VTS_Prepare，就无须做此逻辑
 *Param: 		protoActorPrepareResult [IN] - deamon以此判断testee已经准备好测试
 *Return Value: kOk - 发送成功
 *				kErrCannotFindChannel - 没有找到channel：prepare
 *				kErrChannelPtrNull    - 对应的channel指针为null
 *				kErrMessagePtrNull	  - 创建的消息指针为null
 *				kErrMessagePtrError   - 创建的消息指针错误
 *				kErrProtoSizeError    - proto的长度为0
 *				kErrChannelNetError   - 对应的channel中的网络问题
 */
int VTS_PostPrepareResult( vts::protocol::main::ActorPrepareResult & protoActorPrepareResult );
```

```c++
/*
 *Describe:		通过channel:vehiclecontrol发送VehicleControl给chassis
 *Param: 		protoVehicleControl [IN] - 填写控制信号的proto的数据发送给地盘
 *Return Value: kOk - 发送成功
 *				kErrCannotFindChannel - 没有找到channel：vehiclecontrol
 *				kErrChannelPtrNull    - 对应的channel指针为null
 *				kErrMessagePtrNull	  - 创建的消息指针为null
 *				kErrMessagePtrError   - 创建的消息指针错误
 *				kErrProtoSizeError    - proto的长度为0
 *				kErrChannelNetError   - 对应的channel中的网络问题
 */
int VTS_PostVehicleControl( vts::protocol::chassis::VehicleControl & protoVehicleControl );
```

```c++
/*
 *Describe:		通过channel:vehiclecontrol获取chassis发送的VehicleFeedback
 *Param: 		protoVehicleFeedback [OUT] - 获取控制信号的反馈信息
 *Param:		mode	[IN] - 获取消息模式
 *Return Value: >= 0 - 成功获取到消息，以及当前channel中还剩余的消息数量
 *				kErrCannotFindChannel - 没有找到channel：vehiclecontrol
 *				kErrChannelPtrNull    - 对应的channel指针为null
 *				kErrMessagePtrNull	  - 创建的消息指针为null
 *				kErrMessagePtrError   - 创建的消息指针错误
 *				kErrProtoSizeError    - 表示获取到的消息通过此proto反序列化失败
 *				kErrMessageTypeUnknow - 表示获取到的消息不是对应此消息类型的数据
 *				kErrMessageFromSelf	  - 表示获取到的消息是本进程发出的,在接口内过滤
 */
int VTS_GetVehicleFeedback( vts::protocol::chassis::VehicleFeedback & protoVehicleFeedback, GetMessageMode mode );
```

```c++
/*
 *Describe:		通过channel:pubrole获取chassis或daemon发送的PubRole
 *Param: 		protoPubRole [OUT] - chassis发送的车本体信息，或daemon发送的环境信息
  *Param:		mode	[IN] - 获取消息模式
 *Return Value: >= 0 - 成功获取到消息，以及当前channel中还剩余的消息数量
 *				kErrCannotFindChannel - 没有找到channel：pubrole
 *				kErrChannelPtrNull    - 对应的channel指针为null
 *				kErrMessagePtrNull	  - 创建的消息指针为null
 *				kErrMessagePtrError   - 创建的消息指针错误
 *				kErrProtoSizeError    - 表示获取到的消息通过此proto反序列化失败
 *				kErrMessageTypeUnknow - 表示获取到的消息不是对应此消息类型的数据
 *				kErrMessageFromSelf	  - 表示获取到的消息是本进程发出的,在接口内过滤
 */
int VTS_GetPubRole( vts::protocol::main::PubRole & protoPubRole, GetMessageMode mode );
```

```c++
/*
 *Describe:		通过channel:notify获取daemon发送的Notify
 *Param: 		protoNotify [OUT] - daemon发送的控制信息，如start,abort,finish等
 *Return Value: >= 0 - 成功获取到消息，以及当前channel中还剩余的消息数量
 *				kErrCannotFindChannel - 没有找到channel：notify
 *				kErrChannelPtrNull    - 对应的channel指针为null
 *				kErrMessagePtrNull	  - 创建的消息指针为null
 *				kErrMessagePtrError   - 创建的消息指针错误
 *				kErrProtoSizeError    - 表示获取到的消息通过此proto反序列化失败
 *				kErrMessageTypeUnknow - 表示获取到的消息不是对应此消息类型的数据
 *				kErrMessageFromSelf	  - 表示获取到的消息是本进程发出的,在接口内过滤
 */
int VTS_GetNotify( vts::protocol::main::Notify & protoNotify );
```

### 3.3  proto message描述

```c#
/*
 *Describe:		通过调用VTS_GetPrepare获取准备消息
 *Data: 		session_id   - 本次测试记录的唯一标识
 *Data:         actor_id     - 无需关注
 *Data:      	archive_info - 无需关注
 */
message ActorPrepare 
{
	string session_id        = 1;
	string actor_id          = 2;
	ArchiveInfo archive_info = 3;
}
```

```c#
/*
 *Describe:		通过调用VTS_PostPrepareResult发送准备好消息
 *Data: 		session_id   - ActorPrepare中的seesion_id
 *Data:         actor_id     - 无需关注
 *Data:      	result		 - 准备结果true/false
 *Data:			reason       - 如果result=false,说明原因
 */
message ActorPrepareResult 
{
	string session_id = 1;
	string actor_id   = 2;
	bool result       = 3;
	string reason     = 4;
}
```

```c#
/*
 *Describe:		通过调用VTS_PostVehicleControl发送控制信号给chassis
 *Data: 		header		   		 - 仿真测试消息头
 *Data:         steering_control     - 方向盘控制
 *Data:      	driving_control		 - 油门控制
 *Data:			brake_control        - 刹车控制
 *Data:			bcm					 - 车本体模型
 */
message VehicleControl
{
    SimHeader header 				   = 1;
    SteeringControl steering_control   = 2;
    DrivingControl driving_control     = 3;
    BrakeControl brake_control         = 4;
    BodyControlModule bcm              = 5;
};
```

```c#
/*
 *Describe:		通过调用VTS_GetVehicleFeedback获取chassis发送的控制信号反馈信息
 *Data: 		header   			 - 仿真测试消息头
 *Data:			error				 - 状态
 *Data:         steering_feedback    - 方向盘反馈
 *Data:      	driving_feedback	 - 油门反馈
 *Data:			brake_feedback       - 刹车反馈
 *Data:         gear_feedback		 - 挡位反馈
 *Data:			bcm_feedback		 - 车本体反馈
 */
message VehicleFeedback
{
    SimHeader header						= 1;
    VehicleError error 						= 2;
    SteeringFeedback steering_feedback		= 3;
    DrivingFeedback driving_feedback		= 4;
    BrakeFeedback brake_feedback			= 5;
    GearFeedback gear_feedback				= 6;
    BodyControlModuleFeedback bcm_feedback	= 7;
};
```

```c#
/*
 *Describe:		通过调用VTS_GetPubRole获取chassis发送的车本体数据信息，
 				或者daemon发送的周边环境信息
 *Data: 		header		   		 - 仿真测试消息头
 *Data:			session_id		     - 本次测试记录的唯一标识
 *Data:         s_roles			     - 单个交通参与者消息的集合
 *Data:      	c_roles				 - 无需关注
 */
message PubRole 
{
	SimHeader header 			  = 1;
	string session_id 			  = 2;
	repeated SingleRole s_roles   = 3;
	repeated CompoundRole c_roles = 4;
}
```

```c#
/*
 *Describe:		通过调用VTS_GetNotify获取daemon发送的测试控制消息
 *Data: 		header		   - 仿真测试消息头
 *Data:			session_id	   - 本次测试记录的唯一标识
 *Data:         role_id		   - 控制消息关联的交通参与者ID, single_role中的id, 可为空
 *Data:      	type		   - 测试控制信号类型：start，abort,finish等
 *Data:			s_param		   - 如果是abort/finish,可以查看原因
 *Data:			d_param		   - 无需关注
 */
message Notify 
{
	SimHeader header 		= 1;
	string session_id 		= 2;
	string role_id 			= 3;
	NotifyType type 		= 4;
	string s_param 			= 5;
	repeated double d_param = 6;
}
```

```c#
/*
 *Describe:		存档信息
 *Data: 		id		   	 - 文件ID
 *Data:			url		     - 下载链接
 *Data:         type		 - 存档类型
 *Data:      	modify_ts	 - 最新修改时间，单位:毫秒
 *Data:			meta_data	 - 存档内容描述
 */
message ArchiveInfo 
{
	string id 		 = 1;  
	string url 		 = 2;  
	ArchiveType type = 3;  
	int64 modify_ts  = 4;  
	string meta_data = 5; 
}
```

```c#
/*
 *Describe:		VehicleControl中的参数，方向盘控制
 *Data: 		target_steering_wheel_angle  - 设置此项 for AD test
 *Data:			actual_steering_wheel_angle  - 无需关注
 *Data:         target_steering_wheel_torque - 无需关注
 *Data:      	target_directive_wheel_angle - 无需关注
 */
message SteeringControl
{
    oneof cmd_value
    {
        float target_steering_wheel_angle = 1;//[-100， 100] 左正右负
        float actual_steering_wheel_angle = 2;//rad, for driving simulator
        float target_steering_wheel_torque= 3;//Nm, reserved
        float target_directive_wheel_angle= 4;//rad, reserved
    }
};
```

```c#
/*
 *Describe:		VehicleFeedback中的参数，方向盘反馈
 *Data: 		steering_wheel_angle         - 设置此项
 *Data:			steering_wheel_speed         - 无需关注
 *Data:         left_directive_wheel_angle   - 无需关注
 *Data:      	right_directive_wheel_angle  - 无需关注
 */
message SteeringFeedback
{
    float steering_wheel_angle 		  = 1;
    float steering_wheel_speed 		  = 2;
    float left_directive_wheel_angle  = 3;
    float right_directive_wheel_angle = 4;
}
```

```c#
/*
 *Describe:		VehicleControl中的参数，油门控制
 *Data: 		target_accelerator_pedal_position  - 目标油门踏板位置
 *Data:			actual_accelerator_pedal_position  - 无需关注
 *Data:			gear_control					   - 挡位控制
 */
message DrivingControl
{
    oneof cmd_value
    {
        float target_accelerator_pedal_position = 1;//0-100.0, for AD test
        float actual_accelerator_pedal_position = 2;//0-100.0
    }
    GearControl gear_control = 7;
}
```

```c#
/*
 *Describe:		VehicleFeedback中的参数，油门控制反馈
 *Data: 		driving_Mode  				- 驾驶模式
 *Data:			accelerator_pedal_position  - 油门踏板物理位置，
 											- 如果为智驾状态，该值为虚拟踏板位置
 *Data:			engine_rpm 					- 转速
 */
message DrivingFeedback
{
    DrivingMode driving_Mode 		 = 1;
    float accelerator_pedal_position = 2; 
    float engine_rpm 				 = 3;
};
```

```c#
/*
 *Describe:		DrivingControl中的参数，挡位控制
 *Data: 		gear_mode  			  - 挡位模式
 *Data:			target_gear_position  - 目标挡位
 *Data:			parking 			  - 手刹
 *Data:			clutch_pedal_angle 	  - 离合器踏板角度 无需关注
 */
message GearControl
{
    GearControlMode gear_mode  = 1;
    int32 target_gear_position = 2; //正值为前进挡，0为空挡，-5～-1为倒车档， -6为P档
    bool parking 			   = 3; //Hand brake state,0-release，1-up
    float clutch_pedal_angle   = 4;
};
```

```c#
/*
 *Describe:		VehicleFeedback中的参数，挡位控制反馈
 *Data: 		gear_mode  			   - 挡位模式
 *Data:			current_gear_position  - 当前挡位
 *Data:			hand_brake 			   - 手刹状态
 */
message GearFeedback
{
    GearControlMode gear_mode  	= 1;
    int32 current_gear_position	= 2;
    bool hand_brake 			= 3; //Hand brake state,0-release，1-up
};
```

```c#
/*
 *Describe:		VehicleControl中的参数，刹车控制
 *Data: 		target_brake_pedal_position  - 目标刹车踏板位置
 *Data:			actual_brake_pedal_position  - 实际刹车踏板位置
 */
message BrakeControl
{
    oneof cmd_value
    {
        float target_brake_pedal_position = 1;//for AD test
        float actual_brake_pedal_position = 2;//for driving simulator
    }
};
```

```c#
/*
 *Describe:		VehicleFeedback中的参数，刹车控制反馈
 *Data: 		driving_mode  		  - 驾驶模式
 *Data:			brake_pedal_position  - 刹车踏板位置
 */
message BrakeFeedback
{
    DrivingMode driving_mode   = 1;
    float brake_pedal_position = 2;//制动踏板物理位置，如果为智驾状态，为虚拟踏板位置
};
```

```c#
/*
 *Describe:		VehicleControl中的参数，车体控制模型
 *Data: 		left_turn_light   		- 左转向灯  -
 *Data:			right_turn_light  		- 右转向灯  -
 *Data:			low_beam 		  		- 近光灯
 *Data:			high_beam 		  		- 远光灯
 *Data:			front_fog_light   		- 前雾灯
 *Data:			rear_fog_light 	  		- 后雾灯
 *Data:			hazard_warning_light 	- 双闪 -
 *Data:			wiper 					- 雨刷
 *Data:			horn 					- 喇叭 -
 *Data:			auto_beam 				- 自动大灯
 *Data: 		auto_wiper 				- 自动雨刷
 *Data:			end_outline_marker_lamps- 后端示廓灯
 */
message BodyControlModule
{
    bool left_turn_light 		  = 1;	//0-关闭，1-开启，下同
    bool right_turn_light		  = 2;
    bool low_beam 				  = 3;
    bool high_beam 				  = 4;
    bool front_fog_light 		  = 5;
    bool rear_fog_light 		  = 6;
    bool hazard_warning_light 	  = 7;
    int32 wiper 				  = 8; //档位模式1-3
    bool horn 					  = 9;
    bool auto_beam 				  = 10;
    bool auto_wiper 			  = 11;
    bool end_outline_marker_lamps = 12;
};
```



```c#
/*
 *Describe:		VehicleFeedback中的参数，车体控制模型反馈
 *Data:			longitudinal_acceleration 	 	- 加速度
 *Data:			vehicle_speed 				 	- 车速
 *Data:			front_left_wheel_speed 		 	- 左前轮速度 
 *Data: 		fron_right_wheel_speed 		 	- 右前轮速度 
 *Data: 		rear_left_wheel_speed 		 	- 左后轮速度 
 *Data: 		rear_right_wheel_speed 		 	- 右后轮速度 
 *Data: 		left_turn_light_state  		 	- 左转向灯 
 *Data:			right_turn_light_state  	 	- 右转向灯
 *Data:			low_beam_state 				 	- 近光灯
 *Data:			high_beam_state    			 	- 远光灯
 *Data:			front_fog_light_state 		 	- 前雾灯
 *Data:			rear_fog_light_state 		 	- 后雾灯
 *Data:			hazard_warning_light_state 	 	- 双闪
 *Data:			wiper_state  				 	- 雨刷
 *Data:			horn_state 	 				 	- 喇叭
 *Data:			auto_beam_state 			 	- 自动大灯
 *Data: 		auto_wiper_state 			  	- 自动雨刷
 *Data:			brake_light_state 			    - 刹车灯
 *Data:			end_outline_marker_lamps_state 	- 后端示廓灯
 *Data:			stop_lamps_state 				-
 *Data:			reversing_lamps_state 			-
    
 *Data:			front_left_door_angle 			- 左前车门角度
 *Data:			front_right_door_angle 			- 右前车门角度
 *Data:			rear_left_door_angle 			- 左后车门角度
 *Data:			rear_right_door_angle 			- 右后车门角度
 *Data:			vehicle_boot_angle 				- 后备箱角度
 *Data:			vehicle_bonnet_angle			- 引擎盖角度
 */
message BodyControlModuleFeedback
{
    float longitudinal_acceleration	 	= 1;
    float vehicle_speed				 	= 2;
    float front_left_wheel_speed	 	= 3;
    float fron_right_wheel_speed	 	= 4;
    float rear_left_wheel_speed		 	= 5;
    float rear_right_wheel_speed	 	= 6;

    bool left_turn_light_state 		 	= 7;//0-关闭，1-开启，下同
    bool right_turn_light_state 	 	= 8;
    bool low_beam_state 			 	= 9;
    bool high_beam_state 			 	= 10;
    bool front_fog_light_state 		 	= 11;
    bool rear_fog_light_state 		 	= 12;
    bool hazard_warnning_light_state 	= 13;
    int32 wiper_state 				 	= 14;//档位模式1-3
    bool horn_state 				 	= 15;
    bool auto_beam_state 			 	= 16;
    bool auto_wiper_state 			 	= 17;
    bool brake_light_state 			    = 18;
    bool end_outline_marker_lamps_state = 19;
    bool stop_lamps_state 				= 20;
    bool reversing_lamps_state 			= 21;
    
    float front_left_door_angle         = 22;//0.0 - closed state, 下同
    float front_right_door_angle 		= 23;
    float rear_left_door_angle 			= 24;
    float rear_right_door_angle 		= 25;
    float vehicle_boot_angle 			= 26;//行李箱状态
    float vehicle_bonnet_angle 			= 27;//引擎盖状态

};
```

```c#
/*
 *Describe:		仿真测试消息头
 *Data: 		send_ts  - 消息发送的系统时间
 *Data:			sim_ts   - 消息发送的仿真时间
 *Data:			seq_no   - 消息序列号
 */
message SimHeader 
{
	int64 send_ts = 1;   // 单位:毫秒
	int64 sim_ts  = 2;   // 单位:毫秒
	int64 seq_no  = 3;   // 同类消息自增，从0开始，可用于判断是否丢包
}
```

```c#
/*
 *Describe:		交通参与者消息，如：车，人，信号灯等
 */
message SingleRole 
{
	string id 					  = 1;  // 交通参与者ID
	string name 				  = 2;  // 交通参与者名称
	RoleType type 				  = 3;  // 交通参与者类型，参考RoleType枚举
	string subtype 				  = 4;  // 交通参与者子类型，
   										// 见《通讯协议-角色状态数据取值说明.xlsx》
	ColliderBox box 			  = 5;  // 碰撞体信息
	RLSL rlsl 					  = 6;  // 碰撞体所在的道路位置信息,可为空
	Vector3f linear_speed 		  = 7;  // 线速度：相对于UTM坐标系，单位：米/秒
	Vector3f angular_speed 		  = 8;  // 角速度：相对于UTM坐标系，单位：弧度/秒
	Vector3f linear_acceleration  = 9;  // 线加速度
	Vector3f angular_acceleration = 10; // 角加速度
	repeated float f_status 	  = 11; // 浮点型状态集合，
    									// 见《通讯协议-角色状态数据取值说明.xlsx》
	repeated string s_status 	  = 12; // 字符串型状态集合，
    									// 见《通讯协议-角色状态数据取值说明.xlsx》
	int64 report_ts 			  = 13; // 上报时间，单位:毫秒
}
```

```c#
/*
 *Describe:		组合角色消息
 */
message CompoundRole 
{
	string id 						 = 1;  // 角色ID
	RoleType type 					 = 2;  // 角色类型
	string subtype 					 = 3;  // 角色子类型
	repeated SingleRole single_roles = 4;  // 子组件角色信息
}
```

```c#
/*
 *Describe:	AOI 信息
 */
message AOIInfo 
{
	string role_id   = 1;  // 
	float aoi_radius = 2;  // 单位:米
}
```

```c#
/*
 *Describe:	ColliderBox 中的参数，三维向量信息：双精度浮点型取值
 */
message Vector3d 
{
	double x = 1;  // x轴方向取值
	double y = 2;  // y轴方向取值
	double z = 3;  // z轴方向取值
}
```

```c#
/*
 *Describe:	SingleRole 中的参数，位置信息，另一种坐标系
 */
message RLSL 
{
	string road_id 	= 1;	// 道路id
	int32 lane_id 	= 2;	// 车道id
	double s 		= 3;  	// 单位:米
	double l 		= 4;  	// 单位:米
	double z 		= 5;  	// 单位:米
}
```

```c#
/*
 *Describe:	ColliderBox中的参数， 四元素姿态角
 */
message QuaternionF 
{
	float x = 1;	//
	float y = 2;	// 
	float z = 3;	// 
	float w = 4;	// 
}
```

```c#
/*
 *Describe:	ColliderBox中的参数，三维向量信息：单精度浮点型取值
 */
message Vector3f 
{
	float x = 1;  // x轴方向取值
	float y = 2;  // y轴方向取值
	float z = 3;  // z轴方向取值
}
```

```c#
/*
 *Describe:		碰撞体信息
 */
message ColliderBox 
{
	Vector3d bottom_center = 1;  // 碰撞体的底部中心点位置，单位：米
	Vector3f size          = 2;  // 碰撞体尺寸，单位：米
	QuaternionF rotation   = 3;  // 角度：单位：弧度
}
```

```c#
/*
 *Describe:		VehicleControl状态
 */
enum VehicleError
{
    OK				   = 0; 	// 正确
    VEHICLE_REGISTERED = 1;		// 车未注册
    TESTCASE_NOT_READY = 2;     // 案例未准备
    NO_MAP_FILE_FOUND  = 3;		// 找不到地图
}
```



```c#
/*
 *Describe:		test控制信号枚举
 */
enum NotifyType 
{
	NT_INVALID 				= 0;  //无效类型 无需关注
	NT_START_TEST 			= 1;  //开始测试 
	NT_ABORT_TEST 			= 2;  //中止测试
	NT_PAUSE_TEST 			= 3;  //暂停测试 无需关注
	NT_RESUME_TEST 			= 4;  //继续测试 无需关注
	NT_STEP_TEST 			= 5;  //单步测试 无需关注
	NT_FINISH_TEST 			= 6;  //完成测试
	NT_COLLIDE_ROLE 		= 7;  //交通参与者发生碰撞 无需关注
	NT_DESTROY_ROLE 		= 8;  //交通参与者消失 
	NT_CROSS_SOLID_LINE 	= 9;  //跨（压）实线 无需关注
	NT_CROSS_DOTTED_LINE 	= 10; //跨（压）虚线 无需关注
	NT_EXCEED_SPEED 		= 11; //超速 无需关注
	NT_DRIVE_REVERSED 		= 12; //逆行 无需关注
	NT_DEVIATE_DRIVING_AREA = 13; //偏离可行区域 无需关注
	NT_DRIVE_RED_LIGHT 		= 14; //闯红灯 无需关注
	NT_DRIVE_LOW_SPEED 		= 15; //低速行驶 无需关注
	NT_WRONG_TURN_SIGNAL 	= 16; //未正确开启转向灯 无需关注
	NT_SCORE_CHANGE 		= 17; //主控车分数变化 无需关注
	NT_ARRIVED_ROLE 		= 18; //主控车到达终点 无需关注
	NT_ROLLED 				= 19; //车辆翻滚 无需关注
	NT_RISK_COLLIDE 		= 20; //存在碰撞风险 无需关注
}
```

```c#
/*
 *Describe:		驾驶模式
 */
enum DrivingMode
{
    OFF 			   = 0; //智驾关闭状态(预留)
    MANUAL 			   = 1;       
    AUTONOMOUS_DRIVING = 2; //智驾激活状态
    FAULT 			   = 3; //异常状态
}
```

```c#
/*
 *Describe:		挡位模式
 */
enum GearControlMode
{
    AUTOMATIC_GEAR_MODE = 0; //自动挡
    MANUAL_GEAR_MODE    = 1; //手动挡模式
}
```

```c#
/*
 *Describe:		交通参与者类型	
 */
enum RoleType 
{
	RT_INVALID 			  = 0;
	RT_TREE 			  = 1; //树
	RT_GRASS		      = 2;
	RT_SHRUB 			  = 3;
	RT_LAMP 			  = 4;
	RT_OUTDOOR 			  = 5;
	RT_TRASHBIN 		  = 6;
	RT_HYDRANT 			  = 7;
	RT_DISTRIBUTIONBOX    = 8;
	RT_PAVEMENT 		  = 9;
	RT_MAILBOX 			  = 10;
	RT_UTILITYPOLE 		  = 11;
	RT_UTILITYTOWER	      = 12;
	RT_BUILDING 		  = 13;
	RT_WALL 			  = 14;
	RT_STATICSIGN 		  = 15;
	RT_DYNAMICSIGN 		  = 16;
	RT_TRAFFICLIGHT 	  = 17; //信号灯
	RT_DISPLAY 			  = 18;
	RT_BARRIERGATE 		  = 19;
	RT_BUMP 			  = 20;
	RT_CONE 			  = 21; //
	RT_BARRIER 			  = 22;
	RT_CAMERA 			  = 23;
	RT_VELOCIMETER 		  = 24;
	RT_FLASH 			  = 25;
	RT_ROCK 			  = 26;
	RT_HOLE 			  = 27;
	RT_GULLYHOLE 		  = 28;
	RT_MANHOLE 			  = 29;
	RT_COIL 			  = 30;
    RT_PUDDLE   		  = 31;
	RT_CRACK 			  = 32;
	RT_SWELL 			  = 33;
	RT_TRASH 			  = 34;
	RT_LEAVES 			  = 35;
	RT_OTHER 			  = 36;
	RT_MOTORVEHICLE 	  = 37; //机动车
	RT_EMERGENCY 		  = 38;
	RT_TRAIN 			  = 39;
	RT_COMBINATIONVEHICLE = 40;
	RT_NONMOTORVEHICLE 	  = 41; //非机动车
	RT_PEDESTRIAN 		  = 42; //行人
	RT_ANIMAL 			  = 43; //动物
	RT_CLOUD   			  = 44;
	RT_RAIN 			  = 45;
}
```

## 4. 示例代码

见：Sample_Drive.cpp


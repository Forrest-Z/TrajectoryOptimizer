/*****************************************************************
 * @file   c_map.h
 * @brief  基于 OpenDrive 的 HD-Map 数据接口，面向本次比赛的无人车对道路信息的查询和基本的路径规划需求.
 *
 * @author ZJLab VTS Mapper
 * @version V1.0
 * @date   Augest 2020
***********************************************************************/
#pragma once
#ifndef CMAP_H
#define CMAP_H

#ifdef _WIN32
#ifdef DLL_EXPORT
#define MAP_API __declspec(dllexport)
#else
#define MAP_API __declspec(dllimport)
#endif

#elif __linux__
#define MAP_API

#endif

#include <cstddef>
#include <stdint.h>
#include <cfloat>
#include <climits>
// --------------------------------- 预定义变量别名
typedef double CRoadS;			// (道路参考线轨迹坐标系下)从道路起始点开始沿道路走过的距离
typedef double CRouteS;			// 从规划路径起点到终点间在经过的各条道路上的CRoadS距离的累加之和

typedef char* CJunctionId;
typedef int32_t CObjectId;
typedef const char* CAnchorId;

typedef char* CRoadId;
typedef uint32_t CSectionIndex;
typedef int32_t CLocalLaneId;

// --------------------------------- 预定义枚举类型，部分类型返回值请参阅说明文档确认含义
typedef uint32_t ObjectType;		//  道路物体类型
typedef enum CLaneChangeType		//  车道允许变道类型
{
	kNoChange = 0,						// 仅直行
	kLeftChange = 1,					// 仅可左向变道
	kRightChange = 2,					// 仅可右向变道
	kBothChange = 4,					// 仅直行
	kUnknownChangeType = -1				// 仅直行
}CLaneChangeType;

typedef uint32_t CErrorCode;		 //  函数返回错误码类型


// --------------------------------- //  预定义结构体
typedef struct CLaneUId {			 //  车道全局Id（三个成员含义均可参考OpenDrive高精度地图格式定义）
	CRoadId road_id;					//  车道所在Road的全局Id
	CSectionIndex section_index;		//  车道所在LaneSection在该Road上的Index
	CLocalLaneId local_id;				//  车道的本地Id (在道路参考线左边则为正，右边则为负)
}CLaneUId;

typedef struct CXYZ {				 //  以空间绝对XYZ坐标系表示的位置(UTM，WGS84, 区域:北半球50R)，单位:m
	double x;							// UTM Easting
	double y;							// UTM Northing
	double z;							// 本次比赛中不考虑；
} CXYZ;

typedef struct CSLZ {				 //  以基于道路参考线的轨迹坐标系表示的位置，单位:m （轨迹坐标系定义请参考OpenDrive官方文档2.3节
									 //  或地图接口文档1.2节）
	CLaneUId lane_uid;					// Road->轨迹所属的道路，Section_index->该位置所在lanesection， local_id 该位置所在车道local_id
	CRoadS s;							// 从道路起始点沿轨迹方向走过的距离
	double l;							// 位置垂直到车道参考线上的距离(该位置在道路参考线左边则为正，右边则为负)
	double z;							// 本次比赛中不考虑；
} CSLZ;

typedef struct CAnchor {			 // 锚点结构体
	CAnchorId anchor_id;			 // 用户自定义锚点名字
	CSLZ slz;						 // 锚点SLZ坐标
} CAnchor;

typedef struct CLaneInfo {			 // 车道基本信息结构体
	CLaneUId lane_uid;				 // 车道全局id
	CRoadS  begin;					 // 车道起始点对应道路Road的S		
	CRoadS  end;					 // 车道起始点对应道路Road的S	
	CRoadS length;					 // 车道s方向的长度

}CLaneInfo;

//--------------------------------- 预定义用作返回数组的结构体
typedef struct CLaneUIdArray
{
	uint32_t length;
	CLaneUId* array;
}CLaneUIdArray;

typedef struct CXYZArray
{
	uint32_t length;
	CXYZ* array;
}CXYZArray;

typedef struct CSLZArray
{
	uint32_t length;
	CSLZ* array;
}CSLZArray;

typedef struct CAnchorArray
{
	uint32_t length;
	CAnchor* array;
}CAnchorArray;

//-------------------------------- 路径结构体
typedef struct CRoute				//用于存储路径规划算法返回的车道全局Id序列等
{
	CLaneUIdArray lane_uids;		// 车道全局Id序列
	CSLZ 		begin;				// 路径起始点SLZ坐标
	CSLZ 		end;				// 路径终点SLZ坐标
	CRouteS 	distance;			// 路径长度(仅考虑沿途道路s方向距离变化）
}CRoute;


// ------------------------------- 预定义空值(仅供便利赋予对象初值)
const CLaneUId		EmptyLaneUId = { NULL,INT_MAX,0 };
const CSLZ			EmptySLZ = { EmptyLaneUId, DBL_MIN, DBL_MAX };
const CAnchor		EmptyAnchor = { NULL, EmptySLZ };
const CLaneInfo		EmptyLaneInfo = { EmptyLaneUId, 0.0, 0.0, 0.0 };
const CSLZArray		EmptySLZArray = { 0, NULL };
const CXYZArray		EmptyXYZArray = { 0, NULL };
const CAnchorArray	EmptyAnchorArray = { 0, NULL };

#ifdef __cplusplus
extern "C" {
#endif

	//--------------------------- 地图接口初始化，加载地图文件
	MAP_API  void		 CMapInit();
	MAP_API  CErrorCode  CMapLoad(const char* file_path);


	//--------------------------- SL和XY两种坐标表示方式转换
	// 通过给定的xyz坐标找到其所在的道路，车道段和车道，计算出对应的SLZ坐标。若不在任何一条路上，则返回最近一条道路的Id，以及基于该条道路
	// 计算得到的SLZ坐标，同时若距离该最近的道路边界距离较大，则SectionIndex和LocalId会返回一个较大正/负值
	// hint: 用户给出一条置信度高的车道全局id， 则接口函数会优先搜索该id附近的道路，用于避免每次转换都做全局搜索（找不到则依然会做全局搜索）
	MAP_API  CErrorCode  CMapFindSLZ(CXYZ xyz, CLaneUId hint, CSLZ* slz);

	// 若不给hint则使用此函数
	MAP_API  CErrorCode  CMapFindSLZWithOutHInt(CXYZ xyz, CSLZ* slz);

	// 给定基于一条道路的SLZ，转换成XYZ绝对坐标
	MAP_API  CErrorCode  CMapCalcXYZ(CSLZ slz, CXYZ* xyz);

	// //--------------------------- 构造查询输入对象（仅供便利，亦可采用结构体直接赋初值）
	MAP_API CAnchor     CCreateAnchor(CAnchorId id, CSLZ pos);
	MAP_API CLaneUId	CCreateLaneUId(const char* road_id, CSectionIndex secton_index, CLocalLaneId local_id);
	MAP_API CSLZ		CCreateSLZ(CLaneUId lane_uid, double s, double l);


	// --------------------------- 规划路径相关接口
	// 操作锚点列表的函数，仅供辅助用户处理锚点列表
	MAP_API  CErrorCode  CAddAnchor(CAnchorArray* anchor_list, CAnchor anchor_to_add);
	MAP_API  CErrorCode  CDeleteAnchor(CAnchorArray* anchor_list, CAnchorId id_anchor_to_delete);
	MAP_API  CErrorCode  CFindAnchor(const char* pattern, CAnchorArray anchor_list, CAnchorArray* found_anchors);

	// 规划路径函数，输入应为有序的锚点列表
	MAP_API  CErrorCode  CMapPlanRoute(CAnchorArray anchor_list, CRoute* route);


	// --------------------------- 道路，车道，交叉口信息查询
	// 查询车道的基本信息
	MAP_API CErrorCode  CMapQueryLaneInfo(CLaneUId lane_uid, CLaneInfo* lane_info);

	// 查询车道上某一个位置的最大限速（单位km/h）
	MAP_API CErrorCode  CMapQueryLaneSpeedAt(CLaneUId lane_uid, CRoadS s, double* speed_limit);

	// 查询车道上某一个位置的允许变道情况
	MAP_API CErrorCode  CMapQueryLaneChangeTypeAt(CLaneUId lane_uid, CRoadS s, CLaneChangeType* lane_change_type);

	// 计算得到车道左右边界线，（注意 Sampling_spacing 具有上下界（10.0m ~ 0.001m）, 下同），按车道前进方向返回
	MAP_API CErrorCode  CMapQueryLaneBoundaries(CLaneUId lane_uid, double sampling_spacing, CSLZArray* left_boundary, CSLZArray* right_boundary);

	// 计算得到车道中心线，按车道前进方向返回
	MAP_API CErrorCode  CMapCalcLaneCenterLine(CLaneUId lane_uid, double sampling_spacing, CSLZArray* result);

	// 计算得到车道中心线中的某一段（s1~s2）中心线，按车道前进方向返回
	MAP_API CErrorCode  CMapCalcLaneCenterLineInterval(CLaneUId lane_uid, CRoadS s1, CRoadS s2, double sampling_spacing, CSLZArray* result);

	// 查看道路是否为属于一个Junction的内部道路
	MAP_API CErrorCode  CMapQueryRoadJunctionId(const char* road_id, CJunctionId* junction_id);

	// 查询指定Junction的边界点集（注意可能部分Junction并无边界点记录，则返回空值）
	MAP_API CErrorCode  CMapFindJunctionBoundary(CJunctionId junction_id, CXYZArray* junction_boundary);


	// --------------------------- 销毁对象，释放地图对象或者以上函数中用于传出值的指针等
	MAP_API void CMapDestory();
	MAP_API void CDestoryLaneUId(CLaneUId* cid);
	MAP_API void CDestorySLZ(CSLZ* cslz);
	MAP_API void CDestoryRoute(CRoute* croute);
	MAP_API void CDestorySLZArray(CSLZArray* cslz_array);
	MAP_API void CDestoryXYZArray(CXYZArray* cxyz_array);

	
#ifdef __cplusplus
}
#endif

#endif


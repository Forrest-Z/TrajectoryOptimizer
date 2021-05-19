/*****************************************************************
 * @file   map.h
 * @brief  An OpenDrive-based HD-Map Data Interface oriented to autonomous vehicle route planning.
 * @details This interface dedicats to provide autonomous vehicles' navigation module access to the necessary map data and also the 
 * 			basic route planning function. The ...
 * 			
 * @author Xingjiang Yu
 * @version V0.8
 * @date   April 2020
***********************************************************************/
#pragma once
#ifndef MAP_H
#define MAP_H

#include <functional>
#include <vector>
#include <string>
#include <cfloat>


#ifdef _WIN32
#ifdef CPP_DLL_EXPORT
#define MAP_API __declspec(dllexport)
#else
//#define MAP_API __declspec(dllimport)
#define MAP_API 
#endif

#elif __linux__
#define MAP_API

#endif

/**
 * @namespace zjlmap
 *
 * @brief All interfaces based on the ZJLab defined HDMap data.
 */
namespace zjlmap 
{

// ------------------------------------ Types redefined----------------------------------------------
typedef double RoadS;  // the distance between the begin of the road to a position on road along the reference line
typedef double RouteS; // the distance between the begin of the route to a position on route along the reference lines

typedef int32_t JunctionId;	  		// global id of a junction
typedef int32_t ObjectId;	  		// global id of an object
typedef const char* AnchorId; 		// global id of an anchor

typedef int32_t RoadId;		  		// global id of a road
typedef int32_t LocalLaneId;  		// local id of a lane on the Road  

/**
     * @struct LaneId 
     * @brief Parsed High-precision map format data in XML-tree-like structure.
     */
struct MAP_API LaneId
{
	RoadId road_id;		  // Global Id of Road
	int section_idx;
	LocalLaneId local_id; // Local id of lane on the Road
	LaneId();
	LaneId(RoadId ri, int si, LocalLaneId li);
	bool operator ==(LaneId op);

};
struct MAP_API LaneIdArray
{
	uint32_t length;
	LaneId* lane_id_array;
	LaneIdArray();
	
};

// ------------------------------------ Basic structures ---------------------------------------------------
/**
     * @struct XYZ 
     * @brief UTM XYZ coordinate of a certain position
     */
struct MAP_API XYZ
{

	double x; // UTM easting value
	double y; // UTM northing value (Map based on region in northern hemisphere)
	double z; // Absolute altitude value
	XYZ();
	XYZ(double xx, double yy, double zz);
};
/**
     * @struct XYZArray 
     * @brief Array of XYZ positions
     */
struct MAP_API XYZArray
{
	uint32_t length;
	XYZ* xyz_array;
	XYZ operator[](int index) const;
	XYZArray();
	
};
/**
     * @struct SLZ 
     * @brief Road reference line based  coordinate of a certain position 
     */
struct MAP_API SLZ
{
	LaneId lane_id; // Global lane id
	RoadS s;		// s value based on Road reference line //hzy. road_s??
	double l;		// l value offset from road reference line
	double z;		//	z value based on the reference line plane
	SLZ();
	SLZ(LaneId id, double ss, double ll, double zz);
};
/**
     * @struct SLZArray 
     * @brief Array of SLZ positions
     */
struct MAP_API SLZArray
{
	uint32_t length;
	SLZ* slz_array;
	SLZ operator[](int index) const;
	SLZArray();
	SLZArray(unsigned int size);
	
};


/**
     *  @struct Anchor, point on the map with special meaning
     *  @brief Enumerate possible running status of a function 
     */
struct MAP_API Anchor
{
	AnchorId id; // a string represents the anchor
	SLZ slz;	 // position of the anchor
	Anchor();
	
};
/**
     * @struct AnchorArray 
     * @brief Array of Anchors
     */
struct MAP_API AnchorArray
{
	uint32_t length;
	Anchor* anchor_array;
	AnchorArray();
	Anchor operator[](int index) const;
	
};

// TracePoint provides a better output from centerline calculation
struct MAP_API TracePoint {
	double x;
	double y;
	double z;
	LaneId lane_id;
	double s;
	double l;
	double hdg;
	double curv;
	double curv_deriv;
	TracePoint() = default;
	TracePoint(double xx, double yy, double zz, LaneId ldld, double ss, double ll, double hh, double cc, double cdcd);
};
// ------------------------------------ Lane related structures / enumration ----------------------------------------------

/**
     * @enum LaneChangeType 
     * @brief enumrate the possible lane change options at a position on a lane
    */
typedef enum
{
	kNoChange 			= 0, // can only go straight along this lane 
	kLeftChange 		= 1,
	kRightChange 		= 2,
	kBothChange 		= 4,
	kUnknownChangeType 	= -1

} LaneChangeType;

/**
     * @enum LaneTurnType 
     * @brief enumrate the possible lane turning options of a lane, compositions can be refelected by bits 
    */
//typedef enum // NOTE: Standard OpenDrives do not have Turn Type attributes
//{
//	kUnkonwnTurnType	= 0x0,	
//	kForward			= 0x1,
//	kLeftTurn 			= 0x2,
//	kRightTurn 			= 0x4,
//	kUTurn 				= 0x8
//
//} LaneTurnType;

/**
 * @struct LaneInfo
 * @brief Necessary information of a lane oriented to navigation
 * 
 */
struct MAP_API LaneInfo
{
	LaneId id; 		// The global id of the lane		
	RoadS  begin;	// start s of the lane on its belonged road			
	RoadS  end;		// end s of the lane on its belonged road	
	RoadS length;	//
	//LaneTurnType lane_turn_type; // NOTE: Currently TurnType feature is not included in our map, hence all the turn_type you
	//							 // read will be "kUnkonwnTurnType"
	LaneInfo();
};

struct MAP_API LaneLinkage {

	std::vector<LaneId> predecessor_lanes;
	std::vector<LaneId> successor_lanes;

	LaneId left_neighbor;
	LaneId right_neighbor;

	LaneLinkage() = default;
};
// ------------------------------------ Object related structures / enumration ----------------------------------------------
/**
     *  @enum ObjectType //TODO: Defined in the reference documentation 
     *  @brief Enumerate the route planning related opendrive objects
     */
typedef const char* ObjectType;
struct MAP_API ObjectTypeArray
{
	uint32_t length;
	SLZ* object_type_array;
};

/**
	 *  @struct object listed on the forward route
	 *  @brief a light-weighted traffic object information structure used for quick search 
	 */
struct MAP_API ObjectInRoute
{
	ObjectId id;	 // Object id
	ObjectType type; // Object Type
	RouteS distance; // distance to current position along the route
	SLZArray corner_points;
	double height;
	ObjectInRoute();  
};
struct MAP_API ObjectInRouteArray
{
	uint32_t length;
	ObjectInRoute* object_in_route_array;
	ObjectInRouteArray();
};

struct MAP_API ObjectOnRoad {
	ObjectId id;
	double s;
	double l;
	ObjectType type; // Object Type
	XYZArray corner_points;
	double height;
	ObjectOnRoad();
};

// ------------------------------------ Route related structures ----------------------------------------------
/**
	 * @struct Route
     * @brief The fundemental route structure stores the result of route planning method in the form of a list of lane id
     */
struct MAP_API Route
{
	LaneIdArray route; 
	SLZ 		begin;
	SLZ 		end;
	RouteS 		length;
	std::vector<LaneId> lane_id_vec;
	Route(); // TODO: 需要给予定长的初始化支持？
	void arr2vec();
};

struct MAP_API SampledLine
{
	SLZArray centerline; 
	SampledLine();
};

// ------------------------------------ ErrorCode Enumration ----------------------------------------------
/**
		 *  @enum ErrorCode
		 *  @brief Enumerate possible running status of the methods 
		 */
enum ErrorCode
{
	kOK =					0,	
	kFileReadingError =		1, // 文件路径错误或者xml格式错误
	kLaneLocalIdInvalid =	2,
	kMapNotReady =			3,

	kAnchorNotCreated =		4,
	kAnchorsNotEnough =		5,	   // 给出锚点少于2个
	kAnchorInvalid =		6,
	kAnchorRelocated =		7,	   // 锚点重定位，当用户所指位置无锚点而自动分配给nearest point时返回
	kAnchorsTooClose =		8,
	kRoadSInvalid =			9,	   //
	kRouteSInvalid =		10,	   //

	kPositionNotInMapBound =	11,	   //
	kUnknownType =				12,	   //
	kLoadedMapExists =			13,	   //	
	kInvaildPosition =			14, 	   //
	kPositionsTooClose =		15,	   //
	kTooEarlyToChange =			16,      //
	kTooLateToChange =			17,
	kIlegalLaneChange =			18, //
	kPositionNotInAnyJunction = 19,
	kPositionNotOnRoad =		20,
	kPositionNotOnLane =		21,
	kPositionReversed =			22,
	kDidntGetCoordinate =		23,
	kSpeedRecordNotFound =		24,
	kXYZNotOnRoads =			25,
	kAnchorListEmpty =			26,
	kAnchorNoMatches =			27,
	kJunctionIdInvalid =		28,
	kRoadIdInvalid =			29,
	kObjectNotFound =			30,
	kRoutingNotInitialized	 =	31,
	kLaneSectionIndexInvalid =	32,
	kNoTrafficLightFound	 =	33,
	kRouteNotFound			 =	34,
	kNotInMapBound			 =  35,
	kSpeedSInvalid			 =	36,

	kAnchodIdNotFound		 =	37,
	kAnchorAlreadyExistsInList= 38,
	kMapNotInited			 =  39,
	kIndexOverflow			 =  40,
	kLaneUidInvalid			 =  41,
	kJunctionBoundaryInvalid =  42,

	kInputValueInvalid		 =  43
};


//------------------------------------------------------ Const Values ----------------------------------------------
const LaneId EmptyLandId = LaneId();
const XYZ EmptyXYZ = XYZ(DBL_MAX, DBL_MAX, DBL_MAX);
const SLZ EmptySLZ = SLZ();
const Anchor EmptyAnchor = Anchor();

const LaneInfo EmptyLaneInfo = LaneInfo();
const ObjectInRoute EmptyObjectInRoute = ObjectInRoute();
const Route EmptyRoute = Route();
const SampledLine EmptySampledRoute = SampledLine();

const LaneIdArray EmptyLaneIdArray = LaneIdArray();
const XYZArray EmptyXYZArray = XYZArray();
const SLZArray EmptySLZArray = SLZArray();
const AnchorArray EmptyAnchorArray = AnchorArray();
const ObjectInRouteArray EmptyObjectInRouteArray = ObjectInRouteArray();

//------------------------------------------------------ LaneChangePolicy type function pointer---------------------
/**
 * @brief The decleration of the function pointer of the lane change policy
 * 
 */
MAP_API typedef SLZ (*LaneChangePolicy) (const SLZ slz, const LaneId lane_id);


//-------------------------------------------------------Interface Class --------------------------------------------
/**
	 *  @class Map
	 *  @brief Surface class of the route planning oriented OpenDrive HDMap Interface
	 *  @details 
	 */
class MAP_API Map
{
public: // TODO:
	/**
		     * @brief Constructor of Map class
		     */
	Map();
	/**
			 * @brief Destorier of Map class
			 */
	~Map();

	// ------------------------------------ Map data loading/unloading methods ----------------------------------------------
	/**
		     * @brief load one map from local file. 
			 * 		  If there already has map loaded in the Object, replace it.
			 * 		 
			 * 
			 * @param file_path: path of map data file
			 * @param handle: handle of the loaded map
		     * @return ErrorCode： 0：success, otherwise: failed
		     */
	ErrorCode load(const char *file_path, int& handle);

	/**
		     * @brief delete loaded map from Memory
		     * @return ErrorCode： 0：success, otherwise: failed
		     */
	ErrorCode unload(int& handle);

	// ------------------------------------ Position / S related methods ----------------------------------------------
	
	/**
		     * @brief transform an UTM-based XYZ coordinate to Road-Reference-Line-based SLZ coordinate
		     * @param xyz  XYZ coordinate system
			 * @param slz  SLZ coordinate system
			 * @param hint  lane_id as hint information for the searching
		     * @return  SLZ coordinate 
			 * 			or the SLZ coordinate of the nearest point on a road
			 * 			if not found return SLZ_EMPTY()
			 * 			
		     */
	SLZ find_slz(const XYZ &xyz,  double radius, const LaneId& hint) const; // 

	/**
		     * @brief transform an Road-Reference-Line-based SLZ coordinate to UTM-based XYZ coordinate 
		     * @param slz  SLZ coordinate system
		
		     * @return XYZ coordinate system:
			 * 		   If slz is empty, return XYZ_EMPTY()	
		     */
	XYZ xyz(const SLZ &slz) const;

	// ------------------------------------ Anchor related methods ----------------------------------------------
	/**
		     * @brief for users to manually create an anchor by specifing the anchor's id and position 
		     * @param id  proposed anchor's id 
		     * @param pos  proposed anchor's position in SLZ coordinate system
		     * @return the created anchor object, return ANCHOR_EMPTY() if no anchor was created 
		     */
	Anchor create_anchor(const AnchorId &id, const SLZ &pos) const;

	/**
		     * @brief for users to manually add an anchor to an existed anchor list 
		     * @param anchor anchor to be added 
		     * @return true:success, false: failed
		     */
	static ErrorCode add_anchor(const Anchor &anchor,  AnchorArray &anchor_list);

	/**
		     * @brief for users to manually delete an anchor from an existed anchor list 
		     * @param id id of the anchor to be deleted 
		     * @return true:success, false: failed
		     */
	static ErrorCode delete_anchor(const AnchorId &id,  AnchorArray &anchor_list);

	/**
		     * @brief for users to find the anchor with certain pattern in the anchor list 
			 * @param pattern pattern of the desired anchor's string id, could be writtern in regular expression
			 * @param anchor_list the anchor list to be searched in
			 * @param anchor the urged anchor
			 * @return true:success, false: failed
			*/
	static ErrorCode find_anchor(const char* &pattern, const AnchorArray &anchor_list, AnchorArray& found_anchors);

	// ------------------------------------ Lane related ----------------------------------------------
	/**
			 * @brief find the lane with specified id and read information 
			 * @param id 
			 * @return lane informations listed in Lane Structure, 
			 * 		   if Empty return LANE_INFO_EMPTY()
			 */
	LaneInfo query_lane_info(const LaneId &id) const;

	/**
			 * @brief query the speed limitation at a certain position on lane
			 * 
			 * @param id 
			 * @param s 
			 * @return double: speed_limit. If not found return DEL_MAX;
			 */
	ErrorCode query_lane_speed_at(const LaneId &id, const RoadS &s, double &speed_limit) const;

	/**
			 * @brief query the permitted lane change type at a certain position on lane
			 * 
			 * @param id 
			 * @param s 
			 * @param lane_change_type 
			 * @return LaneChangeType, if not found return kUnknownChangeType
			 */
	LaneChangeType query_lane_change_type_at(const LaneId &id, const RoadS &s) const;
	
	/**
		     * @brief query the left and right boundary of the specified lane
			 * @param lane_id  the global id of the lane to be queried
			 * @param left_boundary  the sample vertices' SLZ coordinate list of the left boundary
			 * @param right_boundary  the sample vertices' SLZ coordinate list of the right boundary
			 * @return ErrorCode
			*/
	ErrorCode query_lane_boundaries(const LaneId &lane_id, double sampling_spacing, SLZArray &left_boundary, SLZArray &right_boundary) const;

	//ErrorCode query_road_mark_between(const LaneId& id_1, const LaneId& id_2, ) const;
	/**
			 * @brief calculate the center line in a specified interval on the lane
			 * @param id id of the current lane
			 * @param s1 start point of the interval
			 * @param s2 end point of the interval
			 * @param sampling_spacing spacing between sampling points 
			 * @param centerline the center line in the specified interval on the current lane
			 * @return ErrorCode
			 */
	ErrorCode calc_lane_center_line(const LaneId &id, const RoadS &s1, const RoadS &s2, double sampling_spacing,
							  SLZArray &centerline) const;


	



	//  ------------------------------------ Route related methods ----------------------------------------------
	
	/**
	 * @brief plan the route from given start/end/way points based on an abstracted Opendrive Map by AStar algorithm.
	 * 
	 * @param start_anchor 
	 * @param way_point_list 
	 * @param end_anchor 
	 * @param route 
	 * @return ErrorCode 
	 */
	ErrorCode plan_route(const Anchor &start_anchor, 
						 const AnchorArray &way_point_list, 
						 const Anchor &end_anchor, 
						 Route &route) const;
	///**
	//	    * @brief return a list of sampling points of a user-specified interval on the given route
	//		* 
	//	    * @param route the route which the sample route segment based
	//	    * @param start_route_s the start RouteS of current Sample Route Segment on the route
	//		* @param length the proposed length of the sample route segment
	//		* @param default_lane_change_policy for getting the lane change trace without any user input
	//		* \@param turning_radius the turning radius given by the vehicle state
	//		* @param sample_route_segment the result route segment sampling point list to be passed out
	//	    * @return ErrorCode: 0 success; otherwise error occurs;
	//	    */
	ErrorCode sample_route(const Route &route,
										const RouteS &start_route_s, 
										const RouteS &length,
										SampledLine &sampled_route,
										LaneChangePolicy lane_change_policy
										) const;
	

	
	// ------------------------------------ Junction related methods ----------------------------------------------
	
	/**
			 * @brief calclate whether the given point is inside a specified junction or not
			 * 
			 * @param junction_id 
			 * @param pos 
			 * @return true: the point is inside the junction (including the situation point on the boundary),  
			 * 		   false: not inside the junction.
			 */
	bool is_inside_junction(const JunctionId &junction_id, const SLZ &pos) const;

	/**
		     * @brief find the junction's physical outline points if they exist.
			 * @param junction_id 
			 * @param junction_boundary XYZ vertices series in vehicle-heading-wise direction, begin with the nearest
			 * 							junction boundary point
			 * @return ErrorCode:0 successfully found the jucntion boundary; otherwise error occurs;
			*/
	ErrorCode find_junction_boundary(const JunctionId &junction_id, XYZArray &junction_boundary) const;

	// ------------------------------------ Object related methods ----------------------------------------------

	/**
		    * @brief return a list of objects sorted by the (forward) distance to the vehicle on the located route
		    * @param route 
		    * @param start_route_s 
		    * @param length 
		    * @param object_type_filter 
		    * @param object_list 
			* @return ErrorCode: 0 success; otherwise error occurs;
			*/
	ErrorCode find_objects_on_route(const Route &route, const RouteS &start_route_s, const RouteS &length, const ObjectTypeArray &object_type_filter,
									ObjectInRouteArray &object_list) const;

	

	ErrorCode find_objects_on_road(const RoadId& road, std::vector<ObjectOnRoad>& objects) const; 
	// ----------------------------------- Case related methods ----------------------------------------------

	///**
	// * @brief get the heading angle of the lane at the given position on lane
	//		* @param id lane's global id
	//	    * @param pos the given point of the querying 
	//	    * @param angle the angle between the lane's heading angle at pos and the east direction
	//		* @return ErrorCode: 0 success; otherwise error occurs;
	// */
	ErrorCode calc_road_heading_angle(const SLZ& pos, double& angle) const;

	ErrorCode calc_road_end_heading_angle(const LaneId& id, double& angle) const;

	ErrorCode query_lane_width_at(const SLZ& pos, double& width) const;

	ErrorCode query_road_width_at(const SLZ& pos, double& width) const;

	ErrorCode query_road_mark_types_between(const SLZ& pos1, const SLZ& pos2, char** &road_mark_types, unsigned int& length_) const;

	ErrorCode find_slz_with_hint(const XYZ& xyz, const LaneId& hint, SLZ& slz)const;
	ErrorCode calc_slz_with_road_id(const XYZ& xyz, const RoadId& road_id, SLZ& slz)const;
	ErrorCode find_slz_global(const XYZ& xyz, SLZ& slz)const;
	
	ErrorCode is_road_in_junction(const RoadId& road_id, JunctionId& junction_id) const;

	ErrorCode query_traffic_lights_in_road(const RoadId& road_id, std::vector<ObjectId>& object_ids) const;
	
	// check the boundaries of every lane in the map
	ErrorCode get_all_boundary_points(std::vector<XYZ>& plist);

	// check the distance from a position along the route to the end of the route
	double calc_dist_to_end_anchor(const XYZ& xyz, const std::vector<LaneId>& lane_uids, double start_s, double end_s)const;
	// check if the given point is inside the junction 
	bool is_inside_junction(const JunctionId& junction_id, const XYZ& pos) const;
	
	//
	ErrorCode is_inside_junction(const LaneId& hint, const XYZ& pos, JunctionId& junction_id) const;

	//
	ErrorCode query_lane_linkage(const LaneId& lane_uid, LaneLinkage& lane_linkage) const;

	//
	bool is_point_on_road(const RoadId road_id, const XYZ& xyz) const;

	// Override
	ErrorCode query_lane_boundaries(const LaneId& lane_id, const RoadS& s1, const RoadS& s2, double sampling_spacing, SLZArray& left_boundary, SLZArray& right_boundary) const;

	//
	ErrorCode calc_lane_center_line_curv(const LaneId& id, const RoadS& s1, const RoadS& s2, double sampling_spacing, std::vector<TracePoint>& centerline)const;
	
	ErrorCode get_all_laneids(std::vector<LaneId>& all_laneids ) const;
	//----------------------------------------------Auxiliary
	//bool is_road_id_valid(RoadId road_id);
	bool is_lane_id_valid(LaneId lane_id) const;
	bool is_slz_valid(SLZ slz) const;

	ErrorCode query_lane_road_mark(const LaneId& lane_id, double s,std::pair<std::string, std::string>& road_mark_pair) const;

	ErrorCode calc_spiral_points(const TracePoint& start_point, const TracePoint& end_point, double length, double sampling_spacing, std::vector<TracePoint>& result) const;

	bool is_point_around_road(const RoadId& road_id, double threshold, const XYZ& xyz, SLZ& slz) const;
private:
	class MapImpl;
	MapImpl* map_impl_ap_;

	/**
		     * @brief Copy Operation (just decleared, not implemented)
		     */
	Map(const Map &);
	/**
			 * @brief Copy Operator (just decleared, not implemented)
			 */
	Map &operator=(const Map &);



};

} // namespace zjlmap



#endif // !MAP_H_

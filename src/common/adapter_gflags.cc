#include "adapter_gflags.h"

DEFINE_string(map_dir, "./deps/map/", "global route map directory");
DEFINE_string(map_name, "AITownReconstructed_V0103_200518.xodr", "global route map name");
DEFINE_string(cfg_dir, "./cfg/", "config files directory");
DEFINE_string(case_dir, "./case/", "case files directory");

DEFINE_double(change_lane_ds, 15.0, "S distance from lane changing start point to lane chaning end point in frenet (SL) coordinate");
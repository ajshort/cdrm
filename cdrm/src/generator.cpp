#include "cdrm/cdrm.h"
#include "cdrm/generator.h"
#include "cdrm/voxelise.h"

#include <cdrm_msgs/GenerateCdrmGoal.h>
#include <ompl/base/PlannerData.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ros/console.h>


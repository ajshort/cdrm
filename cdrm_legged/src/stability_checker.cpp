#include <cdrm_legged/stability_checker.h>

#include <cdrm_legged/leg_model.h>
#include <cdrm_legged/planning_context.h>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <ros/console.h>

namespace cdrm_legged
{
StabilityChecker::StabilityChecker(const PlanningContext *context) : context_(context)
{
  const auto &robot_model = context->getRobotModel();

  for (const auto &link_name : robot_model->getLinkModelNames())
  {
    auto inertial = robot_model->getURDF()->getLink(link_name)->inertial;

    if (!inertial || inertial->mass == 0)
      continue;

    const auto &p = inertial->origin.position;
    const auto &r = inertial->origin.rotation;

    link_mass_[link_name] = inertial->mass;
    link_mass_origins_[link_name] = Eigen::Translation3d(p.x, p.y, p.z) * Eigen::Quaterniond(r.w, r.x, r.y, r.z);

    robot_mass_ += inertial->mass;
  }
}

StabilityChecker::~StabilityChecker()
{
}

Eigen::Vector3d StabilityChecker::getCom(const moveit::core::RobotState &state) const
{
  if (robot_mass_ == 0)
    return state.getJointTransform(context_->getBodyJoint()).translation();

  Eigen::Vector3d com = Eigen::Vector3d::Zero();

  for (const auto &link : link_mass_)
    com += link.second * (state.getGlobalLinkTransform(link.first) * link_mass_origins_.at(link.first)).translation();

  return com / robot_mass_;
}

using vector_Vector2d = std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>;

// TODO include license for https://wrf.ecse.rpi.edu//Research/Short_Notes/pnpoly.html
static bool isPointInConvexPolygon(const Eigen::Vector2d &p, const vector_Vector2d &vertices)
{
  int c = 0;

  for (std::size_t i = 0, j = vertices.size() - 1; i < vertices.size(); j = i++)
  {
    if (((vertices[i].y() > p.y()) != (vertices[j].y() > p.y())) &&
        (p.x() < (vertices[j].x() - vertices[i].x()) * (p.y() - vertices[i].y()) / (vertices[j].y() - vertices[i].y()) + vertices[i].x()))
      c = !c;
  }

  return c;
}

bool SupportPolygonStability::isStable(const moveit::core::RobotState &state, const Contacts &contacts) const
{
  static const Eigen::Vector3d gravity = {0, 0, -9.81};

  const auto &legs = context_->getLegModels();
  const Eigen::Vector3d com = getCom(state);

  // Check if the COM is over the convex hull formed by valid contacts (where gravity is within the friction cone).
  vector_Vector2d hull;

  for (const auto &contact : contacts.contacts_)
  {
    // TODO currently the friction cone check is disabled.
    // if (c.in_contact_ && std::acos(n.dot(-gravity) / (n.norm() * gravity.norm())) <= std::atan(c.mu_))
    //   hull.push_back(c.position_.head(2));

    hull.push_back(state.getGlobalLinkTransform(legs[contact].tip_link_).translation().head(2));
  }

  return isPointInConvexPolygon(com.head(2), hull);
}
}

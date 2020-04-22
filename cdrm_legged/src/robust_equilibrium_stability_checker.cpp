#include "legplan/robust_equilibrium_stability_checker.h"

#include <robust-equilibrium-lib/static_equilibrium.hh>

using robust_equilibrium::StaticEquilibrium;

namespace legplan
{
RobustEquilibriumStabilityChecker::RobustEquilibriumStabilityChecker(const PlanningContext *context)
  : StabilityChecker(context)
  , m_equilibrium(new StaticEquilibrium("LegPlan", getRobotMass(), 4, robust_equilibrium::SOLVER_LP_QPOASES))
{
}

bool RobustEquilibriumStabilityChecker::isStable(const moveit::core::RobotState &state, const Contacts &contacts) const
{
  Eigen::MatrixX3d points;
  Eigen::MatrixX3d normals;

  for (int i = 0; i < contacts.contacts_.size(); ++i) {
    normals.col(i) = contacts.normals_[i];
  }

  if (!m_equilibrium->setNewContacts(points, normals, 1.0, robust_equilibrium::STATIC_EQUILIBRIUM_ALGORITHM_LP))
    return false;

  bool equilibrium;
  m_equilibrium->checkRobustEquilibrium(getCom(state), equilibrium);

  return equilibrium;
}
}

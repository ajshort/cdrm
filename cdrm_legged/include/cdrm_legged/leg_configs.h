#pragma once

#include <eigen_stl_containers/eigen_stl_vector_container.h>

#include <set>
#include <vector>

namespace cdrm_legged
{
/**
 * Contains whether each CDRM configuration is collision free or a contact.
 */
struct LegConfigs
{
  /**
   * Valid CDRM configuration indices.
   */
  std::vector<unsigned int> contacts_;

  /**
   * Surface normals for each contact.
   */
  EigenSTL::vector_Vector3d normals_;

  /**
   * For each contact, a list of free configurations that can be reached through the CDRM.
   */
  std::vector<std::set<unsigned int>> reachable_;
};
}

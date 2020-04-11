#pragma once

#include <cdrm/cdrm.h>

namespace cdrm_welding
{
/**
 * A combination of CDRMs for welding.
 */
class WeldingCdrm
{
public:
  cdrm::Cdrm nozzle_cdrm_;
  cdrm::Cdrm tool_cdrm_;
  cdrm::Cdrm robot_cdrm_;

  bool load(const std::string &filename);
  bool save(const std::string &filename) const;
};
}

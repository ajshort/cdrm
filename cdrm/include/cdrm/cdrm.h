#pragma once

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>

#include <Eigen/Core>

#include <array>
#include <map>

namespace cdrm
{
struct VertexData
{
  Eigen::VectorXd q_;
};

/**
 * A C-space roadmap of configuration vertices and edges.
 */
using Roadmap = boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS, VertexData>;

/**
 * The main Contact Dynamic Roadmap (CDRM) data structure.
 */
class Cdrm
{
public:
  using Key = std::array<std::int16_t, 3>;
  using Vertex = boost::graph_traits<Roadmap>::vertex_descriptor;
  using Edge = boost::graph_traits<Roadmap>::edge_descriptor;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Cdrm(double resolution = 0.01);

  bool save(const std::string &filename) const;
  bool load(const std::string &filename);

  double resolution_;
  Roadmap roadmap_;

  std::multimap<Key, Vertex> colliding_vertices_;
  std::multimap<Key, Edge> colliding_edges_;
  std::multimap<Key, Vertex> contacts_;

  Eigen::Vector3d workspace_min_ = Eigen::Vector3d::Constant(std::numeric_limits<double>::max());
  Eigen::Vector3d workspace_max_ = Eigen::Vector3d::Constant(std::numeric_limits<double>::lowest());

  double min_contact_distance_ = std::numeric_limits<double>::max();
  double max_contact_distance_ = 0;
};
}

#pragma once

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>

#include <Eigen/Core>

namespace cdrm
{
/**
 * A vertex in the C-space roadmap with a robot configuration.
 */
struct Vertex
{
  Eigen::VectorXd q_;
};

/**
 * An edge in the C-space roadmap with an associated cost.
 */
struct Edge
{
  double cost_;
};

/**
 * A C-space roadmap of configuration vertices and costed edges.
 */
using Roadmap = boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS, Vertex, Edge>;

using VertexDescriptor = boost::graph_traits<Roadmap>::vertex_descriptor;
using EdgeDescriptor = boost::graph_traits<Roadmap>::edge_descriptor;
}

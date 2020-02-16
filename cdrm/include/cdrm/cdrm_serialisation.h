#pragma once

#include <cdrm/cdrm.h>

#include <boost/graph/adj_list_serialize.hpp>
#include <boost/graph/iteration_macros.hpp>
#include <boost/serialization/array.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/vector.hpp>

namespace boost
{
namespace serialization
{
template <class Archive>
void serialize(Archive &ar, cdrm::Cdrm &cdrm, const unsigned int version)
{
  ar & cdrm.resolution_;
  ar & cdrm.roadmap_;

  ar & cdrm.colliding_vertices_;
  ar & cdrm.colliding_edges_;
  ar & cdrm.contacts_;

  ar & cdrm.workspace_min_;
  ar & cdrm.workspace_max_;

  ar & cdrm.min_contact_distance_;
  ar & cdrm.max_contact_distance_;
}

template <class Archive>
void serialize(Archive &ar, cdrm::Vertex &v, const unsigned int version)
{
  ar & v.q_;
}

template <class Archive>
void serialize(Archive &ar, cdrm::Edge &e, const unsigned int version)
{
  ar & e.cost_;
}

template <class Archive>
void serialize(Archive &ar, Eigen::VectorXd &v, const unsigned int version)
{
  auto size = v.size();
  ar & size;

  v.resize(size);
  ar & boost::serialization::make_array(v.data(), v.size());
}

template <class Archive>
void serialize(Archive &ar, Eigen::Vector3d &v, const unsigned int version)
{
  ar & boost::serialization::make_array(v.data(), v.size());
}

template <class Archive>
void serialize(Archive &ar, cdrm::EdgeDescriptor &ed, const unsigned int version)
{
  ar & ed;
}
}
}

#pragma once

#include <cdrm/cdrm.h>

#include <boost/graph/adj_list_serialize.hpp>
#include <boost/graph/iteration_macros.hpp>
#include <boost/serialization/array.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/split_free.hpp>
#include <boost/serialization/vector.hpp>

namespace boost
{
namespace serialization
{
template <class Archive>
void save(Archive &ar, const cdrm::Cdrm &cdrm, const unsigned int version)
{
  ar << cdrm.resolution_;
  ar << cdrm.roadmap_;

  ar << cdrm.colliding_vertices_;

  std::map<cdrm::VertexDescriptor, int> indices;
  int index = 0;

  BGL_FORALL_VERTICES_T(vertex, cdrm.roadmap_, cdrm::Roadmap)
    indices[vertex] = index++;

  int colliding_edge_size = cdrm.colliding_edges_.size();
  ar << colliding_edge_size;

  for (const auto &it : cdrm.colliding_edges_)
  {
    ar << it.first;
    ar << indices[source(it.second, cdrm.roadmap_)];
    ar << indices[target(it.second, cdrm.roadmap_)];
  }

  ar << cdrm.contacts_;

  ar << cdrm.aabb_;

  ar << cdrm.workspace_min_;
  ar << cdrm.workspace_max_;

  ar << cdrm.min_contact_distance_;
  ar << cdrm.max_contact_distance_;
}

template <class Archive>
void load(Archive &ar, cdrm::Cdrm &cdrm, const unsigned int version)
{
  ar >> cdrm.resolution_;
  ar >> cdrm.roadmap_;

  ar >> cdrm.colliding_vertices_;

  std::map<cdrm::VertexDescriptor, int> indices;
  int index = 0;

  BGL_FORALL_VERTICES_T(vertex, cdrm.roadmap_, cdrm::Roadmap)
    indices[vertex] = index++;

  int colliding_edge_size;
  ar >> colliding_edge_size;

  for (unsigned int i = 0; i < colliding_edge_size; ++i)
  {
    cdrm::Key key;
    int source_index, target_index;
    ar >> key >> source_index >> target_index;

    auto source = indices[source_index];
    auto target = indices[target_index];
    auto edge_descriptor = boost::edge(source, target, cdrm.roadmap_).first;

    cdrm.colliding_edges_.insert(std::make_pair(key, edge_descriptor));
  }

  ar >> cdrm.contacts_;

  ar >> cdrm.aabb_;

  ar >> cdrm.workspace_min_;
  ar >> cdrm.workspace_max_;

  ar >> cdrm.min_contact_distance_;
  ar >> cdrm.max_contact_distance_;
}

template <class Archive>
void serialize(Archive &ar, cdrm::Cdrm &cdrm, const unsigned int version)
{
  split_free(ar, cdrm, version);
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
  ar & make_array(v.data(), v.size());
}

template <class Archive>
void serialize(Archive &ar, Eigen::Vector3d &v, const unsigned int version)
{
  ar & make_array(v.data(), v.size());
}

template <class Archive>
void serialize(Archive &ar, cdrm::EdgeDescriptor &ed, const unsigned int version)
{
  ar & ed;
}

template <typename Archive>
void save(Archive &ar, const Eigen::AlignedBox3d &aabb, unsigned int version)
{
  Eigen::Vector3d min = aabb.min();
  Eigen::Vector3d max = aabb.max();
  ar << min << max;
}

template <typename Archive>
void load(Archive &ar, Eigen::AlignedBox3d &aabb, unsigned int version)
{
  Eigen::Vector3d min, max;
  ar >> min >> max;
  aabb = Eigen::AlignedBox3d(min, max);
}

template <typename Archive>
void serialize(Archive &ar, Eigen::AlignedBox3d &aabb, const unsigned int version)
{
  split_free(ar, aabb, version);
}
}
}

#include "cdrm/cdrm.h"
#include "cdrm/voxelise.h"

#include <geometric_shapes/shapes.h>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_state/robot_state.h>
#include <ros/console.h>

namespace cdrm
{
// This code is based on "Fast 3D Triangle-Box Overlap Testing" by Tomas Akenine-Moller, taken from
// http://fileadmin.cs.lth.se/cs/Personal/Tomas_Akenine-Moller/code/

#define AXISTEST_X01(a, b, fa, fb) \
  p0 = a*tri.col(0).y() - b*tri.col(0).z(); \
  p2 = a*tri.col(2).y() - b*tri.col(2).z(); \
  if(p0<p2) {min=p0; max=p2;} else {min=p2; max=p0;} \
  rad = (fa + fb) * half_size; \
  if(min>rad || max<-rad) return false;

#define AXISTEST_X2(a, b, fa, fb) \
  p0 = a*tri.col(0).y() - b*tri.col(0).z(); \
  p1 = a*tri.col(1).y() - b*tri.col(1).z(); \
  if(p0<p1) {min=p0; max=p1;} else {min=p1; max=p0;} \
  rad = (fa + fb) * half_size; \
  if(min>rad || max<-rad) return false;

#define AXISTEST_Y02(a, b, fa, fb) \
  p0 = -a*tri.col(0).x() + b*tri.col(0).z(); \
  p2 = -a*tri.col(2).x() + b*tri.col(2).z(); \
  if(p0<p2) {min=p0; max=p2;} else {min=p2; max=p0;} \
  rad = (fa + fb) * half_size; \
  if(min>rad || max<-rad) return false;

#define AXISTEST_Y1(a, b, fa, fb) \
  p0 = -a*tri.col(0).x() + b*tri.col(0).z(); \
  p1 = -a*tri.col(1).x() + b*tri.col(1).z(); \
  if(p0<p1) {min=p0; max=p1;} else {min=p1; max=p0;} \
  rad = (fa + fb) * half_size; \
  if(min>rad || max<-rad) return false;

#define AXISTEST_Z12(a, b, fa, fb) \
  p1 = a*tri.col(1).x() - b*tri.col(1).y(); \
  p2 = a*tri.col(2).x() - b*tri.col(2).y(); \
  if(p2<p1) {min=p2; max=p1;} else {min=p1; max=p2;} \
  rad = (fa + fb) * half_size; \
  if(min>rad || max<-rad) return false;

#define AXISTEST_Z0(a, b, fa, fb) \
  p0 = a*tri.col(0).x() - b*tri.col(0).y(); \
  p1 = a*tri.col(1).x() - b*tri.col(1).y(); \
  if(p0<p1) {min=p0; max=p1;} else {min=p1; max=p0;} \
  rad = (fa + fb) * half_size; \
  if(min>rad || max<-rad) return false;


static bool isPlaneCubeOverlap(const Eigen::Vector3d &normal, const Eigen::Vector3d &v, double half_size)
{
  Eigen::Vector3d min, max;

  for (int i = 0; i < 3; ++i)
  {
    if (normal(i) > 0)
    {
      min(i) = -half_size - v(i);
      max(i) = half_size - v(i);
    }
    else
    {
      min(i) = half_size - v(i);
      max(i) = -half_size - v(i);
    }
  }

  if (normal.dot(min) > 0)
    return false;

  if (normal.dot(max) >= 0)
    return true;

  return false;
}

static bool isTriangleCubeIntersect(Eigen::Matrix3d tri, const Eigen::Vector3d &center, double size)
{
  const double half_size = size / 2;

  // Move the center to origin.
  tri.colwise() -= center;

  // Get the edges.
  const Eigen::Vector3d e0 = tri.col(1) - tri.col(0);
  const Eigen::Vector3d e1 = tri.col(2) - tri.col(1);
  const Eigen::Vector3d e2 = tri.col(0) - tri.col(2);

  // Perform axis tests.
  Eigen::Vector3d fe;
  double min, max;
  double p0, p1, p2;
  double rad;

  fe = e0.cwiseAbs();
  AXISTEST_X01(e0.z(), e0.y(), fe.z(), fe.y());
  AXISTEST_Y02(e0.z(), e0.x(), fe.z(), fe.x());
  AXISTEST_Z12(e0.y(), e0.x(), fe.y(), fe.x());

  fe = e1.cwiseAbs();
  AXISTEST_X01(e1.z(), e1.y(), fe.z(), fe.y());
  AXISTEST_Y02(e1.z(), e1.x(), fe.z(), fe.x());
  AXISTEST_Z0(e1.y(), e1.x(), fe.y(), fe.x());

  fe = e2.cwiseAbs();
  AXISTEST_X2(e2.z(), e2.y(), fe.z(), fe.y());
  AXISTEST_Y1(e2.z(), e2.x(), fe.z(), fe.x());
  AXISTEST_Z12(e2.y(), e2.x(), fe.y(), fe.x());

  // Check AABB.
  if (tri.minCoeff() > half_size || tri.maxCoeff() < -half_size)
    return false;

  // // Check plane box overlap.
  const Eigen::Vector3d normal = e0.cross(e1);

  if (!isPlaneCubeOverlap(normal, tri.col(0), half_size))
    return false;

  return true;
}

#undef AXISTEST_X01
#undef AXISTEST_X2
#undef AXISTEST_Y02
#undef AXISTEST_Y1
#undef AXISTEST_Z12
#undef AXISTEST_Z0

void voxelise(const shapes::Mesh &mesh, double resolution, const VoxelCallback &callback, const Eigen::Isometry3d &tf,
              const Eigen::Vector3d *min_bound, const Eigen::Vector3d *max_bound)
{
  using VectorMap = Eigen::Map<Eigen::Vector3d>;

  std::set<Key> seen;

  for (unsigned int triangle = 0; triangle < mesh.triangle_count; ++triangle)
  {
    const unsigned int offset = 3 * triangle;

    // Create the transformed triangle.
    Eigen::Matrix3d tri;
    tri.col(0) = tf * VectorMap(mesh.vertices + 3 * mesh.triangles[offset]);
    tri.col(1) = tf * VectorMap(mesh.vertices + 3 * mesh.triangles[offset + 1]);
    tri.col(2) = tf * VectorMap(mesh.vertices + 3 * mesh.triangles[offset + 2]);

    // Get the min and max corners of the voxels.
    Eigen::Vector3i sizes;

    Eigen::Vector3d min = tri.rowwise().minCoeff();
    Eigen::Vector3d max = tri.rowwise().maxCoeff();

    if (min_bound)
    {
      if ((max.array() - min_bound->array() < 0).all())
        continue;

      min = min.cwiseMax(*min_bound);
      max = max.cwiseMax(*min_bound);
    }

    if (max_bound)
    {
      if ((min.array() - max_bound->array() > 0).all())
        continue;

      min = min.cwiseMin(*max_bound);
      max = max.cwiseMin(*max_bound);
    }

    // Clamp to the bounds.
    for (unsigned int i = 0; i < 3; ++i)
    {
      min(i) = std::floor(min(i) / resolution) * resolution;
      max(i) = std::ceil(max(i) / resolution) * resolution;

      sizes(i) = static_cast<int>(std::ceil((max(i) - min(i)) / resolution));
    }

    // Iterate each cell and check if it collides with the triangle.
    for (int x = 0; x <= sizes.x(); ++x)
    {
      for (int y = 0; y <= sizes.y(); ++y)
      {
        for (int z = 0; z <= sizes.z(); ++z)
        {
          const Eigen::Vector3d centre = min + resolution * Eigen::Vector3d(x, y, z);

          if (isTriangleCubeIntersect(tri, centre, resolution))
            callback(centre, VectorMap(mesh.triangle_normals + 3 * triangle));
        }
      }
    }
  }
}

void voxelise(const moveit::core::RobotState &state, const std::vector<const moveit::core::LinkModel *> &links,
              double resolution, const VoxelCallback &callback, const Eigen::Isometry3d &tf,
              const Eigen::Vector3d *min_bound, const Eigen::Vector3d *max_bound)
{
  for (const auto *link : links)
  {
    const Eigen::Isometry3d &link_tf = state.getGlobalLinkTransform(link);

    const auto &shapes = link->getShapes();
    const auto &shape_tfs = link->getCollisionOriginTransforms();

    for (std::size_t i = 0; i < shapes.size(); ++i)
    {
      const auto *shape = shapes[i].get();

      if (shape->type == shapes::MESH)
      {
        const auto *mesh = static_cast<const shapes::Mesh *>(shape);
        voxelise(*mesh, resolution, callback, tf * link_tf * shape_tfs[i], min_bound, max_bound);
      }
    }
  }
}
}

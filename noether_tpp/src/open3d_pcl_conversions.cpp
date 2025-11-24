/*
 This is utility to that provides converter to convert open3d::geometry::PointCloud and pcl::PointCloud<T>.

// Point Type (pcl::PointXYZ, pcl::PointNormal, pcl::PointXYZRGB, pcl::PointXYZRGBA, pcl::PointXYZRGBNormal)
typedef pcl::PointXYZRGB PointType;

// Convert Point Cloud To Open3D From PCL
std::shared_ptr<open3d::geometry::PointCloud> open3d_cloud = convert::toOpen3D<PointType>( pcl_cloud );

// Convert Point Cloud To PCL From Open3D
pcl::PointCloud<PointType>::Ptr pcl_cloud = convert::toPCL<PointType>( open3d_cloud );

Copyright (c) 2020 Tsukasa Sugiura <t.sugiura0204@gmail.com>
Licensed under the MIT license.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <noether_tpp/open3d_pcl_conversions.h>

// STD
#include <algorithm>
#include <cstdint>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Open3D
#include <open3d/geometry/PointCloud.h>

// Eigen
#include <Eigen/Core>  // IWYU pragma: keep

constexpr double kNormal{ 1.0 / 255.0 };

namespace noether
{
template <>
pcl::PointCloud<pcl::PointXYZ> toPCL<pcl::PointXYZ>(const open3d::geometry::PointCloud& open3d_cloud)
{
  const uint32_t size = open3d_cloud.points_.size();

  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  pcl_cloud.width = size;
  pcl_cloud.height = 1;
  pcl_cloud.is_dense = false;
  pcl_cloud.points.resize(size);

#pragma omp parallel for
  for (uint32_t i = 0; i < size; i++)
  {
    pcl_cloud.points[i].getVector3fMap() = open3d_cloud.points_[i].cast<float>();
  }

  return pcl_cloud;
};

template <>
pcl::PointCloud<pcl::PointNormal> toPCL<pcl::PointNormal>(const open3d::geometry::PointCloud& open3d_cloud)
{
  const uint32_t size = open3d_cloud.points_.size();
  const Eigen::Vector3f zero = Eigen::Vector3f::Zero();

  pcl::PointCloud<pcl::PointNormal> pcl_cloud;
  pcl_cloud.width = size;
  pcl_cloud.height = 1;
  pcl_cloud.is_dense = false;
  pcl_cloud.points.resize(size);

#pragma omp parallel for
  for (uint32_t i = 0; i < size; i++)
  {
    pcl_cloud.points[i].getVector3fMap() = open3d_cloud.points_[i].cast<float>();
    const Eigen::Vector3f normal = (open3d_cloud.HasNormals()) ? open3d_cloud.normals_[i].cast<float>() : zero;
    std::copy(normal.begin(), normal.end(), pcl_cloud.points[i].normal);  // NOLINT
  }

  return pcl_cloud;
};

template <>
pcl::PointCloud<pcl::PointXYZRGB> toPCL<pcl::PointXYZRGB>(const open3d::geometry::PointCloud& open3d_cloud)
{
  const uint32_t size = open3d_cloud.points_.size();

  pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;
  pcl_cloud.width = size;
  pcl_cloud.height = 1;
  pcl_cloud.is_dense = false;
  pcl_cloud.points.resize(size);

  if (open3d_cloud.HasColors())
  {
#pragma omp parallel for
    for (uint32_t i = 0; i < size; i++)
    {
      pcl_cloud.points[i].getVector3fMap() = open3d_cloud.points_[i].cast<float>();
      const auto color = (open3d_cloud.colors_[i] * 255.0).cast<uint32_t>();
      uint32_t rgb = color[0] << 16 | color[1] << 8 | color[2];
      pcl_cloud.points[i].rgb = *reinterpret_cast<float*>(&rgb);  // NOLINT
    }

    return pcl_cloud;
  }

#pragma omp parallel for
  for (uint32_t i = 0; i < size; i++)
  {
    pcl_cloud.points[i].getVector3fMap() = open3d_cloud.points_[i].cast<float>();
    uint32_t rgb = 0x000000;
    pcl_cloud.points[i].rgb = *reinterpret_cast<float*>(&rgb);  // NOLINT
  }

  return pcl_cloud;
};

template <>
pcl::PointCloud<pcl::PointXYZRGBA> toPCL<pcl::PointXYZRGBA>(const open3d::geometry::PointCloud& open3d_cloud)
{
  const uint32_t size = open3d_cloud.points_.size();

  pcl::PointCloud<pcl::PointXYZRGBA> pcl_cloud;
  pcl_cloud.width = size;
  pcl_cloud.height = 1;
  pcl_cloud.is_dense = false;
  pcl_cloud.points.resize(size);

  if (open3d_cloud.HasColors())
  {
#pragma omp parallel for
    for (uint32_t i = 0; i < size; i++)
    {
      pcl_cloud.points[i].getVector3fMap() = open3d_cloud.points_[i].cast<float>();
      const auto color = (open3d_cloud.colors_[i] * 255.0).cast<uint32_t>();
      pcl_cloud.points[i].rgba = 0xff000000 | color[0] << 16 | color[1] << 8 | color[2];
    }

    return pcl_cloud;
  }

#pragma omp parallel for
  for (uint32_t i = 0; i < size; i++)
  {
    pcl_cloud.points[i].getVector3fMap() = open3d_cloud.points_[i].cast<float>();
    pcl_cloud.points[i].rgba = 0xff000000;
  }

  return pcl_cloud;
};

template <>
pcl::PointCloud<pcl::PointXYZRGBNormal> toPCL<pcl::PointXYZRGBNormal>(const open3d::geometry::PointCloud& open3d_cloud)
{
  const uint32_t size = open3d_cloud.points_.size();
  const Eigen::Vector3f zero = Eigen::Vector3f::Zero();

  pcl::PointCloud<pcl::PointXYZRGBNormal> pcl_cloud;
  pcl_cloud.width = size;
  pcl_cloud.height = 1;
  pcl_cloud.is_dense = false;
  pcl_cloud.points.resize(size);

#pragma omp parallel for
  for (uint32_t i = 0; i < size; i++)
  {
    pcl_cloud.points[i].getVector3fMap() = open3d_cloud.points_[i].cast<float>();
    const auto color = (open3d_cloud.colors_[i] * 255.0).cast<uint32_t>();
    uint32_t rgb = (open3d_cloud.HasColors()) ? color[0] << 16 | color[1] << 8 | color[2] : 0x000000;
    pcl_cloud.points[i].rgb = *reinterpret_cast<float*>(&rgb);  // NOLINT
    const Eigen::Vector3f normal = (open3d_cloud.HasNormals()) ? open3d_cloud.normals_[i].cast<float>() : zero;
    std::copy(normal.begin(), normal.end(), pcl_cloud.points[i].normal);  // NOLINT
  }

  return pcl_cloud;
};

template <>
open3d::geometry::PointCloud toOpen3D<pcl::PointXYZ>(const pcl::PointCloud<pcl::PointXYZ>& pcl_cloud)
{
  const uint32_t size = pcl_cloud.size();

  open3d::geometry::PointCloud open3d_cloud;
  open3d_cloud.points_.resize(size);

#pragma omp parallel for
  for (uint32_t i = 0; i < size; i++)
  {
    open3d_cloud.points_[i] = pcl_cloud.points[i].getVector3fMap().cast<double>();
  }

  return open3d_cloud;
};

template <>
open3d::geometry::PointCloud toOpen3D<pcl::PointNormal>(const pcl::PointCloud<pcl::PointNormal>& pcl_cloud)
{
  const uint32_t size = pcl_cloud.size();

  open3d::geometry::PointCloud open3d_cloud;
  open3d_cloud.points_.resize(size);
  open3d_cloud.normals_.resize(size);

#pragma omp parallel for
  for (uint32_t i = 0; i < size; i++)
  {
    open3d_cloud.points_[i] = pcl_cloud.points[i].getVector3fMap().cast<double>();
    open3d_cloud.normals_[i] = pcl_cloud.points[i].getNormalVector3fMap().cast<double>();
  }

  return open3d_cloud;
};

template <>
open3d::geometry::PointCloud toOpen3D<pcl::PointXYZRGB>(const pcl::PointCloud<pcl::PointXYZRGB>& pcl_cloud)
{
  const uint32_t size = pcl_cloud.size();

  open3d::geometry::PointCloud open3d_cloud;
  open3d_cloud.points_.resize(size);
  open3d_cloud.colors_.resize(size);

#pragma omp parallel for
  for (uint32_t i = 0; i < size; i++)
  {
    open3d_cloud.points_[i] = pcl_cloud.points[i].getVector3fMap().cast<double>();
    const uint32_t color = *reinterpret_cast<const uint32_t*>(&pcl_cloud.points[i].rgb);  // NOLINT
    open3d_cloud.colors_[i] =
        Eigen::Vector3d((color >> 16) & 0x0000ff, (color >> 8) & 0x0000ff, color & 0x0000ff) * kNormal;
  }

  return open3d_cloud;
};

template <>
open3d::geometry::PointCloud toOpen3D<pcl::PointXYZRGBA>(const pcl::PointCloud<pcl::PointXYZRGBA>& pcl_cloud)
{
  const uint32_t size = pcl_cloud.size();

  open3d::geometry::PointCloud open3d_cloud;
  open3d_cloud.points_.resize(size);
  open3d_cloud.colors_.resize(size);

#pragma omp parallel for
  for (uint32_t i = 0; i < size; i++)
  {
    open3d_cloud.points_[i] = pcl_cloud.points[i].getVector3fMap().cast<double>();
    const uint32_t color = pcl_cloud.points[i].rgba;
    open3d_cloud.colors_[i] =
        Eigen::Vector3d((color >> 16) & 0x000000ff, (color >> 8) & 0x000000ff, color & 0x000000ff) * kNormal;
  }

  return open3d_cloud;
};

template <>
open3d::geometry::PointCloud toOpen3D<pcl::PointXYZRGBNormal>(const pcl::PointCloud<pcl::PointXYZRGBNormal>& pcl_cloud)
{
  const uint32_t size = pcl_cloud.size();

  open3d::geometry::PointCloud open3d_cloud;
  open3d_cloud.points_.resize(size);
  open3d_cloud.normals_.resize(size);
  open3d_cloud.colors_.resize(size);

#pragma omp parallel for
  for (uint32_t i = 0; i < size; i++)
  {
    open3d_cloud.points_[i] = pcl_cloud.points[i].getVector3fMap().cast<double>();
    open3d_cloud.normals_[i] = pcl_cloud.points[i].getNormalVector3fMap().cast<double>();
    const uint32_t color = *reinterpret_cast<const uint32_t*>(&pcl_cloud.points[i].rgb);  // NOLINT
    open3d_cloud.colors_[i] =
        Eigen::Vector3d((color >> 16) & 0x0000ff, (color >> 8) & 0x0000ff, color & 0x0000ff) * kNormal;
  }

  return open3d_cloud;
};
}  // namespace noether

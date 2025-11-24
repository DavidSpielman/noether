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

#pragma once

#define NOMINMAX

// Open3D
#include <open3d/geometry/PointCloud.h>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace noether
{
template <typename T>
pcl::PointCloud<T> toPCL(const open3d::geometry::PointCloud& /*open3d_cloud*/)
{
  return pcl::PointCloud<T>();
};

template <>
pcl::PointCloud<pcl::PointXYZ> toPCL<pcl::PointXYZ>(const open3d::geometry::PointCloud& open3d_cloud);

template <>
pcl::PointCloud<pcl::PointNormal> toPCL<pcl::PointNormal>(const open3d::geometry::PointCloud& open3d_cloud);

template <>
pcl::PointCloud<pcl::PointXYZRGB> toPCL<pcl::PointXYZRGB>(const open3d::geometry::PointCloud& open3d_cloud);

template <>
pcl::PointCloud<pcl::PointXYZRGBA> toPCL<pcl::PointXYZRGBA>(const open3d::geometry::PointCloud& open3d_cloud);

template <>
pcl::PointCloud<pcl::PointXYZRGBNormal> toPCL<pcl::PointXYZRGBNormal>(const open3d::geometry::PointCloud& open3d_cloud);

template <typename T>
open3d::geometry::PointCloud toOpen3D(const pcl::PointCloud<T>& /*pcl_cloud*/)
{
  return open3d::geometry::PointCloud();
};

template <>
open3d::geometry::PointCloud toOpen3D<pcl::PointXYZ>(const pcl::PointCloud<pcl::PointXYZ>& pcl_cloud);

template <>
open3d::geometry::PointCloud toOpen3D<pcl::PointNormal>(const pcl::PointCloud<pcl::PointNormal>& pcl_cloud);

template <>
open3d::geometry::PointCloud toOpen3D<pcl::PointXYZRGB>(const pcl::PointCloud<pcl::PointXYZRGB>& pcl_cloud);

template <>
open3d::geometry::PointCloud toOpen3D<pcl::PointXYZRGBA>(const pcl::PointCloud<pcl::PointXYZRGBA>& pcl_cloud);

template <>
open3d::geometry::PointCloud toOpen3D<pcl::PointXYZRGBNormal>(const pcl::PointCloud<pcl::PointXYZRGBNormal>& pcl_cloud);
}  // namespace noether

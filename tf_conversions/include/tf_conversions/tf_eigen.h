/*
 * Copyright (c) 2009-2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// Author: Adam Leeper (and others)

#ifndef CONVERSIONS_TF_EIGEN_H
#define CONVERSIONS_TF_EIGEN_H

#include "tf_ros/transform_datatypes.h"
#include "Eigen/Core"
#include "Eigen/Geometry"

namespace tf_ros {

/// Converts a tf_ros Matrix3x3 into an Eigen Quaternion
void matrixTFToEigen(const tf_ros::Matrix3x3 &t, Eigen::Matrix3d &e);

/// Converts an Eigen Quaternion into a tf_ros Matrix3x3
void matrixEigenToTF(const Eigen::Matrix3d &e, tf_ros::Matrix3x3 &t);

/// Converts a tf_ros Pose into an Eigen Affine3d
void poseTFToEigen(const tf_ros::Pose &t, Eigen::Affine3d &e);

/// Converts a tf_ros Pose into an Eigen Isometry3d
void poseTFToEigen(const tf_ros::Pose &t, Eigen::Isometry3d &e);

/// Converts an Eigen Affine3d into a tf_ros Transform
void poseEigenToTF(const Eigen::Affine3d &e, tf_ros::Pose &t);

/// Converts an Eigen Isometry3d into a tf_ros Transform
void poseEigenToTF(const Eigen::Isometry3d &e, tf_ros::Pose &t);

/// Converts a tf_ros Quaternion into an Eigen Quaternion
void quaternionTFToEigen(const tf_ros::Quaternion &t, Eigen::Quaterniond &e);

/// Converts an Eigen Quaternion into a tf_ros Quaternion
void quaternionEigenToTF(const Eigen::Quaterniond &e, tf_ros::Quaternion &t);

/// Converts a tf_ros Transform into an Eigen Affine3d
void transformTFToEigen(const tf_ros::Transform &t, Eigen::Affine3d &e);

/// Converts a tf_ros Transform into an Eigen Isometry3d
void transformTFToEigen(const tf_ros::Transform &t, Eigen::Isometry3d &e);

/// Converts an Eigen Affine3d into a tf_ros Transform
void transformEigenToTF(const Eigen::Affine3d &e, tf_ros::Transform &t);

/// Converts an Eigen Isometry3d into a tf_ros Transform
void transformEigenToTF(const Eigen::Isometry3d &e, tf_ros::Transform &t);

/// Converts a tf_ros Vector3 into an Eigen Vector3d
void vectorTFToEigen(const tf_ros::Vector3 &t, Eigen::Vector3d &e);

/// Converts an Eigen Vector3d into a tf_ros Vector3
void vectorEigenToTF(const Eigen::Vector3d &e, tf_ros::Vector3 &t);

} // namespace tf_ros

#endif

//
// Copyright (c) 2014 CNRS
// Authors: Steve Tonneau (steve.tonneau@laas.fr)
//
// This file is part of hpp-rbprm.
// hpp-rbprm is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-rbprm is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-core  If not, see
// <http://www.gnu.org/licenses/>.

#ifndef HPP_IK_DEFINITIONS_HH
#define HPP_IK_DEFINITIONS_HH


#include <hpp/ik/config.hh>

#include <hpp/pinocchio/frame.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/core/config-projector.hh>

#include <Eigen/Core>

namespace hpp {

namespace ik {

typedef Eigen::Matrix <double, Eigen::Dynamic, 1> vector_t;
typedef Eigen::Vector3d vector3_t;
typedef Eigen::Quaterniond quat_t;


enum HPP_IK_DLLAPI IkConstraintType
{
    MAINTAIN_3D = 0,
    MAINTAIN_6D,
    MAINTAIN_COM,
    TARGET_3D,
    TARGET_6D,
    TARGET_POS_NORM,
    TARGET_COM
};

struct HPP_IK_DLLAPI IkHelper{

    IkHelper(hpp::pinocchio::DevicePtr_t device, const double tolerance = 1e-5)
    : device_(device)
    , proj_(core::ConfigProjector::create(device, "proj", tolerance, 40 ))
    {
        device_->controlComputation(pinocchio::Computation_t(device_->computationFlag()
                                        | pinocchio::JOINT_POSITION | pinocchio::JACOBIAN | pinocchio::COM));
    }

    IkHelper(){}
    ~IkHelper(){}

    hpp::pinocchio::DevicePtr_t device_;
    core::ConfigProjectorPtr_t proj_;
};


struct HPP_IK_DLLAPI FrameMarker{

    FrameMarker(pinocchio::Frame& frame):
        frame_(frame),
        offset_(vector3_t::Zero()),
        normal_(vector3_t::Zero())
    {}

    FrameMarker(pinocchio::Frame& frame, const vector3_t& offset, const vector3_t& normal):
        frame_(frame),
        offset_(offset),
        normal_(normal)
    {}

    ~FrameMarker(){}


    pinocchio::Frame frame_;
    vector3_t offset_;
    vector3_t normal_;
};

} // namespace hpp
} // namespace ik
#endif // HPP_IK_DEFINITIONS_HH

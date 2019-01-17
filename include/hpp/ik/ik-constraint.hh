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

#ifndef HPP_IK_CONSTRAINT_HH
#define HPP_IK_CONSTRAINT_HH


#include <hpp/ik/config.hh>
#include <hpp/ik/definitions.hh>

#include <hpp/pinocchio/frame.hh>
#include <hpp/core/config-projector.hh>

#include <Eigen/Core>

namespace hpp {

namespace ik {


template<MaintainConstraintType E>
void HPP_IK_DLLAPI AddMaintainConstraint
(IkHelper& ikHelper, const FrameMarker& frame);

void HPP_IK_DLLAPI AddMaintainCOMConstraint(IkHelper& ikHelper);

void HPP_IK_DLLAPI AddConstraint
(IkHelper& ikHelper, const pinocchio::Frame& frame, const vector3_t& target);

void HPP_IK_DLLAPI AddConstraint
(IkHelper& ikHelper, const FrameMarker& frame,const vector3_t& posTarget, const vector3_t& normTarget);

void HPP_IK_DLLAPI AddConstraint
(IkHelper& ikHelper, const FrameMarker& frameMarker, const vector3_t& posTarget, const quat_t& quatTarget);

void HPP_IK_DLLAPI AddCOMConstraint (IkHelper& ikHelper, const vector3_t& target);


void HPP_IK_DLLAPI AddPosConstraint  (IkHelper& ikHelper, const pinocchio::Frame &frame, const vector3_t    & positionTarget);
void HPP_IK_DLLAPI AddRotConstraint  (IkHelper& ikHelper, const pinocchio::Frame& frameMarker, const fcl::Matrix3f& rotationTarget);


template<>
inline void HPP_IK_DLLAPI AddMaintainConstraint<MAINTAIN_3D>
(IkHelper& ikHelper, const FrameMarker& frame)
{
    hpp::ik::AddPosConstraint(ikHelper,frame.frame_,frame.frame_.currentTransformation().translation());
}

template<>
inline void HPP_IK_DLLAPI AddMaintainConstraint<MAINTAIN_6D>
(IkHelper& ikHelper, const FrameMarker& frame)
{
    hpp::ik::AddPosConstraint(ikHelper,frame.frame_,frame.frame_.currentTransformation().translation());
    hpp::ik::AddRotConstraint(ikHelper,frame.frame_,frame.frame_.currentTransformation().rotation());
}


} // namespace hpp
} // namespace ik
#endif // HPP_IK_CONSTRAINT_HH

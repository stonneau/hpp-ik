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
#include <hpp/ik/hidden.hh>
#include <hpp/ik/definitions.hh>

#include <hpp/pinocchio/frame.hh>
#include <hpp/core/config-projector.hh>

#include <Eigen/Core>

namespace hpp {

namespace ik {


void AddPosConstraint  (IkHelper& ikHelper, const FrameMarker& frameMarker, const vector3_t    & positionTarget);
void AddRotConstraint  (IkHelper& ikHelper, const FrameMarker& frameMarker, const fcl::Matrix3f& rotationTarget);
void AddCOMConstraint  (IkHelper& ikHelper, const vector3_t  & positionTarget);

fcl::Transform3f computeProjectionMatrix(const FrameMarker& frameMarker, const pos_norm& positionNormal);


template<>
inline void HPP_IK_DLLAPI AddConstraint<MAINTAIN_3D>
(IkHelper& ikHelper, const FrameMarker& frame)
{
    hpp::ik::AddPosConstraint(ikHelper,frame,frame.frame_.currentTransformation().translation());
}

template<>
inline void HPP_IK_DLLAPI AddConstraint<MAINTAIN_6D>
(IkHelper& ikHelper, const FrameMarker& frame)
{
    hpp::ik::AddPosConstraint(ikHelper,frame,frame.frame_.currentTransformation().translation());
    hpp::ik::AddRotConstraint(ikHelper,frame,frame.frame_.currentTransformation().rotation());
}

template<>
inline void HPP_IK_DLLAPI AddConstraint<MAINTAIN_COM>(IkHelper& ikHelper)
{
    hpp::ik::AddCOMConstraint(ikHelper, ikHelper.device_->positionCenterOfMass());
}

template<>
inline void HPP_IK_DLLAPI AddConstraint<TARGET_3D>
(IkHelper& ikHelper, const FrameMarker& frame, const vector3_t& target)
{
    hpp::ik::AddPosConstraint(ikHelper, frame, target);
}

template<>
inline void HPP_IK_DLLAPI AddConstraint<TARGET_POS_NORM>
(IkHelper& ikHelper, const FrameMarker& frame, const pos_norm& positionNormal)
{
    fcl::Transform3f pM = hpp::ik::computeProjectionMatrix(frame,positionNormal);
    hpp::ik::AddPosConstraint(ikHelper, frame, pM.getTranslation());
    hpp::ik::AddRotConstraint(ikHelper, frame, pM.getRotation());
}

template<>
inline void HPP_IK_DLLAPI AddConstraint<TARGET_6D>
(IkHelper& ikHelper, const FrameMarker& frame, const pos_quat& target)
{
    hpp::ik::AddPosConstraint(ikHelper, frame, target.head<3>());
    hpp::ik::AddRotConstraint(ikHelper, frame, Eigen::Quaterniond(target.tail<4>()).toRotationMatrix());
}

template<>
inline void HPP_IK_DLLAPI AddConstraint<TARGET_COM>
(IkHelper& ikHelper, const vector3_t& target)
{
    hpp::ik::AddCOMConstraint(ikHelper, target);
}

} // namespace hpp
} // namespace ik
#endif // HPP_IK_CONSTRAINT_HH

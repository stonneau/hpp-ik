#include "hpp/ik/ik-constraint.hh"
#include <hpp/pinocchio/joint.hh>
#include <hpp/core/config-projector.hh>
#include <hpp/constraints/relative-com.hh>
#include <hpp/constraints/symbolic-calculus.hh>
#include <hpp/constraints/symbolic-function.hh>
#include <hpp/constraints/generic-transformation.hh>

#include <vector>

namespace hpp {

namespace ik {

const double epsilon = 10e-4;

std::vector<bool> setRotationConstraints()
{
    std::vector<bool> res;
    for(std::size_t i =0; i <3; ++i)
        res.push_back(true);
    return res;
}

std::vector<bool> setTranslationConstraints()
{
    std::vector<bool> res;
    for(std::size_t i =0; i <3; ++i)
        res.push_back(true);
    return res;
}



Eigen::Matrix3d GetRotationMatrix(const Eigen::Vector3d &from, const Eigen::Vector3d &to)
{
    Eigen::Quaterniond quat; quat.setFromTwoVectors(from, to);
    return quat.toRotationMatrix();
}

void AddPosConstraint(IkHelper& ikHelper, const pinocchio::Frame& effectorFrame, const vector3_t& positionTarget)
{
    pinocchio::JointPtr_t effectorJoint = effectorFrame.joint();
    pinocchio::Transform3f localFrame(1), globalFrame(1);
    globalFrame.translation(positionTarget);
    ikHelper.proj_->add(constraints::Implicit::create (constraints::Position::create("",ikHelper.device_,
                                                                           effectorJoint,
                                                                           effectorFrame.pinocchio().placement * localFrame,
                                                                           globalFrame,
                                                                           setTranslationConstraints())));
}

void AddRotConstraint(IkHelper& ikHelper, const pinocchio::Frame& effectorFrame, const fcl::Matrix3f& rotationTarget)
{
    pinocchio::JointPtr_t effectorJoint = effectorFrame.joint();
    pinocchio::Transform3f rotation(1);
    rotation.rotation(rotationTarget * effectorFrame.pinocchio().placement.rotation().transpose());
    ikHelper.proj_->add(constraints::Implicit::create (constraints::Orientation::create("",ikHelper.device_,
                                                                                  effectorJoint,
                                                                                  rotation,
                                                                                  setRotationConstraints())));
}

typedef constraints::PointCom PointCom;
typedef constraints::CalculusBaseAbstract<PointCom::ValueType_t, PointCom::JacobianType_t> s_t;
typedef constraints::SymbolicFunction<s_t> PointComFunction;
typedef constraints::SymbolicFunction<s_t>::Ptr_t PointComFunctionPtr_t;

fcl::Transform3f computeProjectionMatrix(const FrameMarker& frameMarker,
                                         const vector3_t& position, const vector3_t& normal)
{
    const fcl::Vec3f z = frameMarker.frame_.currentTransformation().rotation() * frameMarker.normal_;
    const fcl::Matrix3f alignRotation = GetRotationMatrix(z,normal);
    hppDout(notice,"alignRotation : \n"<<alignRotation);
    const fcl::Matrix3f rotation = alignRotation * frameMarker.frame_.currentTransformation().rotation();
    hppDout(notice,"rotation : \n"<<rotation);
    fcl::Vec3f posOffset = position - rotation * frameMarker.offset_;
    posOffset = posOffset + normal * epsilon;
    return fcl::Transform3f(rotation,posOffset);
}

void AddConstraint(IkHelper& ikHelper, const pinocchio::Frame& frame, const vector3_t& target)
{
    hpp::ik::AddPosConstraint(ikHelper, frame, target);
}

void AddConstraint
(IkHelper& ikHelper, const FrameMarker& frame, const vector3_t& posTarget, const vector3_t& normTarget)
{
    fcl::Transform3f pM = hpp::ik::computeProjectionMatrix(frame,posTarget, normTarget);
    hpp::ik::AddPosConstraint(ikHelper, frame.frame_, pM.getTranslation());
    hpp::ik::AddRotConstraint(ikHelper, frame.frame_, pM.getRotation());
}

void AddConstraint
(IkHelper& ikHelper, const FrameMarker& frameMarker, const vector3_t& posTarget, const quat_t& quatTarget)
{
    fcl::Matrix3f rotation = quatTarget.toRotationMatrix();
    fcl::Vec3f posOffset = posTarget - rotation * frameMarker.offset_;
    hpp::ik::AddPosConstraint(ikHelper, frameMarker.frame_, posOffset);
    hpp::ik::AddRotConstraint(ikHelper, frameMarker.frame_, rotation);
}


void AddCOMConstraint (IkHelper& ikHelper, const vector3_t& positionTarget)
{
    pinocchio::DevicePtr_t device = ikHelper.device_;
    pinocchio::CenterOfMassComputationPtr_t comComp = pinocchio::CenterOfMassComputation::create (device);
    comComp->add (device->rootJoint());
    comComp->compute ();
    PointComFunctionPtr_t comFunc = PointComFunction::create ("COM-constraint",
        device, PointCom::create (comComp));
    constraints::ComparisonTypes_t equals (3, constraints::Equality);
    constraints::ImplicitPtr_t comEq = constraints::Implicit::create(comFunc, equals);
    ikHelper.proj_->add(comEq);
    ikHelper.proj_->rightHandSide(comEq,positionTarget);
}

void AddMaintainCOMConstraint(IkHelper& ikHelper)
{
    hpp::ik::AddCOMConstraint(ikHelper, ikHelper.device_->positionCenterOfMass());
}



} // namespace hpp
} // namespace ik

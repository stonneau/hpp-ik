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
    Eigen::Vector3d u, v, uXv;
    Eigen::Vector3d  a;
    u = from; u.normalize();
    v = to  ; v.normalize();
    uXv = u.cross(v);
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    double sinTheta = uXv.norm();
    if (sinTheta < std::numeric_limits<double>::epsilon()) // angle is 0
    {
        return I;
    }
    else
    {
        double cosTheta = u.dot(v);
        a = uXv / sinTheta;
        Eigen::Matrix3d Iaaa;
        Iaaa(0,1) = 0     ; Iaaa(0,1) = -a[2]; Iaaa(0,2) =  a[1]; //  0  -z   y
        Iaaa(1,0) =  a[2] ; Iaaa(1,0) = 0    ; Iaaa(1,2) = -a[0]; //  z   0  -x
        Iaaa(2,0) = -a[1] ; Iaaa(2,1) = a[0] ; Iaaa(2,2) =  0; // -y   x   0

        return I * cosTheta + sinTheta * Iaaa + (1 - cosTheta) * (a*a.transpose());
    }
}

void AddPosConstraint(IkHelper& ikHelper, const FrameMarker& frameMarker, const vector3_t& positionTarget)
{
    const pinocchio::Frame& effectorFrame = frameMarker.frame_;
    pinocchio::JointPtr_t effectorJoint = effectorFrame.joint();
    pinocchio::Transform3f localFrame(1), globalFrame(1);
    globalFrame.translation(positionTarget);
    ikHelper.proj_->add(constraints::Implicit::create (constraints::Position::create("",ikHelper.device_,
                                                                           effectorJoint,
                                                                           effectorFrame.pinocchio().placement * localFrame,
                                                                           globalFrame,
                                                                           setTranslationConstraints())));
}

void AddRotConstraint(IkHelper& ikHelper, const FrameMarker& frameMarker, const fcl::Matrix3f& rotationTarget)
{
    const pinocchio::Frame& effectorFrame = frameMarker.frame_;
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

void AddCOMConstraint  (IkHelper& ikHelper, const vector3_t  & positionTarget)
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

fcl::Transform3f computeProjectionMatrix(const FrameMarker& frameMarker,
                                         const pos_norm& positionNormal)
{
    fcl::Vec3f position(positionNormal.head<3>());
    fcl::Vec3f normal(positionNormal.tail<3>());
    // device alwas assumed to be updated
    /*ikHelper.device_->currentConfiguration(configuration);
    ikHelper.device_->computeForwardKinematics();*/
    // the normal is given by the normal of the contacted object
    const fcl::Vec3f z = frameMarker.frame_.currentTransformation().rotation() * frameMarker.normal_;
    const fcl::Matrix3f alignRotation = GetRotationMatrix(z,normal);
    hppDout(notice,"alignRotation : \n"<<alignRotation);
    const fcl::Matrix3f rotation = alignRotation * frameMarker.frame_.currentTransformation().rotation();
    hppDout(notice,"rotation : \n"<<rotation);
    fcl::Vec3f posOffset = position - rotation * frameMarker.offset_;
    posOffset = posOffset + normal * epsilon;
    return fcl::Transform3f(rotation,posOffset);
}

} // namespace hpp
} // namespace ik

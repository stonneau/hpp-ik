// Copyright (C) 2018 LAAS-CNRS
// Author: Pierre Fernbach
//
// This file is part of the hpp-rbprm.
//
// hpp-rbprm is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// test-hpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with hpp-core.  If not, see <http://www.gnu.org/licenses/>.


#define BOOST_TEST_MODULE test-hpp-ik
#include <boost/test/included/unit_test.hpp>
#include <boost/test/unit_test.hpp>

//#include <hpp/ik/definitions.hh>
//#include <hpp/ik/ik-constraint.hh>
#include <hpp/pinocchio/device.hh>

#include "test-tools.hh"

#include <vector>


using namespace hpp;
using namespace ik;


static const double epsilon = 1e-4;

BOOST_AUTO_TEST_SUITE( hpp_ik )

BOOST_AUTO_TEST_CASE (create_frame_markers)
{
    pinocchio::DevicePtr_t hyq  = loadHyQ();
    std::vector<FrameMarker> hyqFrames = HyQFrames(hyq);

    pinocchio::DevicePtr_t hrp2 = loadHRP2();
    std::vector<FrameMarker> hrp2Frames = HRP2Frames(hrp2);
}

BOOST_AUTO_TEST_CASE (create_ikhelper)
{
    pinocchio::DevicePtr_t hyq  = loadHyQ();
    std::vector<FrameMarker> hyqFrames = HyQFrames(hyq);
    IkHelper ikHelper(hyq);
}
BOOST_AUTO_TEST_CASE (test_3D_target_constraint)
{
    pinocchio::DevicePtr_t hyq  = loadHyQ();
    std::vector<FrameMarker> hyqFrames = HyQFrames(hyq);
    IkHelper ikHelper(hyq);

    //get current position
    const FrameMarker& fm1 = hyqFrames[0];
    fcl::Vec3f cPos  = fm1.frame_.currentTransformation().translation();
    Eigen::Quaterniond quat(fm1.frame_.currentTransformation().rotation());
    fcl::Vec3f targetPos = cPos + fcl::Vec3f(0.1,0.5,0.03);
    AddConstraint(ikHelper,fm1.frame_,targetPos);
    pinocchio::Configuration_t configuration = hyq->currentConfiguration();
    BOOST_CHECK_EQUAL(ikHelper.proj_->apply(configuration), true);
    hyq->currentConfiguration(configuration); hyq->computeForwardKinematics();
    BOOST_CHECK((fm1.frame_.currentTransformation().translation() - targetPos).norm() < epsilon);
    // checking that rotation changed
    Eigen::Quaterniond quatproj(fm1.frame_.currentTransformation().rotation());
    BOOST_CHECK(quatproj.angularDistance(quat) > epsilon);
}

BOOST_AUTO_TEST_CASE (test_pos_normal_constraint)
{
    pinocchio::DevicePtr_t hyq  = loadHyQ();
    std::vector<FrameMarker> hyqFrames = HyQFrames(hyq);
    IkHelper ikHelper(hyq,1e-6);

    //get current position
    const FrameMarker& fm1 = hyqFrames[0];
    fcl::Vec3f cPos  = fm1.frame_.currentTransformation().translation();
    Eigen::Quaterniond quat(fm1.frame_.currentTransformation().rotation());
    fcl::Vec3f targetPos = cPos + fcl::Vec3f(0.1,0.0,0.03);
    fcl::Vec3f normal(1,0.,0.); normal.normalize();
    AddConstraint(ikHelper,fm1,targetPos, normal);
    pinocchio::Configuration_t configuration = hyq->currentConfiguration();
    BOOST_CHECK_EQUAL(ikHelper.proj_->apply(configuration), true);
    hyq->currentConfiguration(configuration); hyq->computeForwardKinematics();

    fcl::Vec3f offsetPos(targetPos); offsetPos[0]+=fm1.offset_.norm();
    BOOST_CHECK((fm1.frame_.currentTransformation().translation() - offsetPos).norm() < 100*epsilon);
}

BOOST_AUTO_TEST_CASE (test_6D_target_constraint)
{
    pinocchio::DevicePtr_t hyq  = loadHyQ();
    std::vector<FrameMarker> hyqFrames = HyQFrames(hyq);
    IkHelper ikHelper(hyq);

    //get current position
    const FrameMarker& fm1 = hyqFrames[0];
    fcl::Vec3f cPos  = fm1.frame_.currentTransformation().translation();
    fcl::Vec3f targetPos = cPos + fcl::Vec3f(0.1,0.5,0.03);;
    Eigen::Quaterniond quat(fm1.frame_.currentTransformation().rotation());
    AddConstraint(ikHelper,fm1,targetPos, quat);
    pinocchio::Configuration_t configuration = hyq->currentConfiguration();
    BOOST_CHECK_EQUAL(ikHelper.proj_->apply(configuration), true);
    hyq->currentConfiguration(configuration); hyq->computeForwardKinematics();
    fcl::Vec3f offsetPos(targetPos); offsetPos[0]+=fm1.offset_.norm();
    BOOST_CHECK(std::abs((fm1.frame_.currentTransformation().translation() - targetPos).norm() -
                 fm1.offset_.norm())< 100*epsilon);
    Eigen::Quaterniond quatproj(fm1.frame_.currentTransformation().rotation());
    BOOST_CHECK(quatproj.angularDistance(quat) < epsilon);
}

BOOST_AUTO_TEST_CASE (test_3D_maintain_constraint)
{
    pinocchio::DevicePtr_t hyq  = loadHyQ();
    std::vector<FrameMarker> hyqFrames = HyQFrames(hyq);
    IkHelper ikHelper(hyq);

    //get current position
    const FrameMarker& fm1 = hyqFrames[0];
    const FrameMarker& fm2 = hyqFrames[1];
    fcl::Vec3f cPos  = fm1.frame_.currentTransformation().translation();
    fcl::Vec3f cPos2 = fm2.frame_.currentTransformation().translation();
    fcl::Vec3f targetPos = cPos + fcl::Vec3f(0.,0.,0.03);
    AddConstraint(ikHelper,fm1.frame_,targetPos);
    pinocchio::Configuration_t configuration = hyq->currentConfiguration();
    pinocchio::Configuration_t targetconfiguration = configuration;
    BOOST_CHECK_EQUAL(ikHelper.proj_->apply(targetconfiguration), true);
    hyq->currentConfiguration(targetconfiguration); hyq->computeForwardKinematics();
    BOOST_CHECK((fm1.frame_.currentTransformation().translation() - targetPos).norm() < epsilon);
    BOOST_CHECK((fm2.frame_.currentTransformation().translation() - cPos2).norm() > epsilon);

    // adding maintain constraint
    hyq->currentConfiguration(configuration); hyq->computeForwardKinematics();
    AddMaintainConstraint<MAINTAIN_3D>(ikHelper,fm2);
    BOOST_CHECK_EQUAL(ikHelper.proj_->apply(targetconfiguration), true);
    hyq->currentConfiguration(targetconfiguration); hyq->computeForwardKinematics();
    BOOST_CHECK((fm1.frame_.currentTransformation().translation() - targetPos).norm() < epsilon);
    BOOST_CHECK((fm2.frame_.currentTransformation().translation() - cPos2).norm() < epsilon);
}

BOOST_AUTO_TEST_CASE (test_6D_maintain_constraint)
{
    pinocchio::DevicePtr_t hyq  = loadHyQ();
    std::vector<FrameMarker> hyqFrames = HyQFrames(hyq);
    IkHelper ikHelper(hyq);

    //get current position
    const FrameMarker& fm1 = hyqFrames[0];
    const FrameMarker& fm2 = hyqFrames[1];
    fcl::Vec3f cPos  = fm1.frame_.currentTransformation().translation();
    fcl::Vec3f cPos2 = fm2.frame_.currentTransformation().translation();
    Eigen::Quaterniond quat2(fm2.frame_.currentTransformation().rotation());
    fcl::Vec3f targetPos = cPos + fcl::Vec3f(0.1,0.5,0.03);
    AddConstraint(ikHelper,fm1.frame_,targetPos);
    pinocchio::Configuration_t configuration = hyq->currentConfiguration();
    pinocchio::Configuration_t targetconfiguration = configuration;
    BOOST_CHECK_EQUAL(ikHelper.proj_->apply(targetconfiguration), true);
    hyq->currentConfiguration(targetconfiguration); hyq->computeForwardKinematics();
    BOOST_CHECK((fm1.frame_.currentTransformation().translation() - targetPos).norm() < epsilon);
    BOOST_CHECK((fm2.frame_.currentTransformation().translation() - cPos2).norm() > epsilon);
    Eigen::Quaterniond quatproj(fm2.frame_.currentTransformation().rotation());
    BOOST_CHECK(quatproj.angularDistance(quat2) > epsilon);

    // adding maintain constraint
    hyq->currentConfiguration(configuration); hyq->computeForwardKinematics();
    AddMaintainConstraint<MAINTAIN_6D>(ikHelper,fm2);
    BOOST_CHECK_EQUAL(ikHelper.proj_->apply(targetconfiguration), true);
    hyq->currentConfiguration(targetconfiguration); hyq->computeForwardKinematics();
    BOOST_CHECK((fm1.frame_.currentTransformation().translation() - targetPos).norm() < epsilon);
    BOOST_CHECK((fm2.frame_.currentTransformation().translation() - cPos2).norm() < epsilon);
    quatproj = (fm2.frame_.currentTransformation().rotation());
    BOOST_CHECK(quatproj.angularDistance(quat2) < epsilon);
}

BOOST_AUTO_TEST_CASE (test_COM_target_constraint)
{
    pinocchio::DevicePtr_t hyq  = loadHyQ();
    IkHelper ikHelper(hyq);

    fcl::Vec3f targetPos(0.,0.,1.);
    AddCOMConstraint(ikHelper,targetPos);
    pinocchio::Configuration_t configuration = hyq->currentConfiguration();
    pinocchio::Configuration_t targetconfiguration = configuration;
    BOOST_CHECK_EQUAL(ikHelper.proj_->apply(targetconfiguration), true);
    hyq->currentConfiguration(targetconfiguration); hyq->computeForwardKinematics();
    BOOST_CHECK((hyq->positionCenterOfMass() - targetPos).norm() < epsilon);
}

BOOST_AUTO_TEST_CASE (test_COM_Maintain)
{
    pinocchio::DevicePtr_t hyq  = loadHyQ();
    std::vector<FrameMarker> hyqFrames = HyQFrames(hyq);
    IkHelper ikHelper(hyq);

    //get current position
    const FrameMarker& fm1 = hyqFrames[0];
    fcl::Vec3f cPos  = fm1.frame_.currentTransformation().translation();
    Eigen::Quaterniond quat(fm1.frame_.currentTransformation().rotation());
    fcl::Vec3f targetPos = cPos + fcl::Vec3f(0.1,0.,0.03);
    fcl::Vec3f currentCom = hyq->positionCenterOfMass();
    AddConstraint(ikHelper,fm1.frame_,targetPos);
    AddMaintainCOMConstraint(ikHelper);
    pinocchio::Configuration_t configuration = hyq->currentConfiguration();
    BOOST_CHECK_EQUAL(ikHelper.proj_->apply(configuration), true);
    hyq->currentConfiguration(configuration); hyq->computeForwardKinematics();
    BOOST_CHECK((hyq->positionCenterOfMass() - currentCom).norm() < epsilon);
}

BOOST_AUTO_TEST_SUITE_END()


// Copyright (C) 2014 LAAS-CNRS
// Author: Steve Tonneau
//
// This file is part of the hpp-rbprm.
//
// hpp-core is free software: you can redistribute it and/or modify
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

#include <boost/assign.hpp>
#include <hpp/pinocchio/device.hh>
#include <pinocchio/parsers/urdf.hpp>
#include <hpp/pinocchio/urdf/util.hh>
#include <hpp/ik/definitions.hh>
#include <hpp/ik/ik-constraint.hh>


namespace
{
        hpp::pinocchio::DevicePtr_t loadHyQ(){
        const std::string robotName("hyq");
        const std::string rootJointType ("freeflyer");
        const std::string packageName ("hyq_description");
        const std::string modelName ("hyq");
        const std::string urdfSuffix ("");
        const std::string srdfSuffix ("");

        hpp::pinocchio::DevicePtr_t device = hpp::pinocchio::Device::create (robotName);
        //hpp::pinocchio:: (device,rootJointType, packageName, modelName, urdfSuffix,srdfSuffix);


        hpp::pinocchio::urdf::loadRobotModel(device, rootJointType, packageName, modelName,
        urdfSuffix, srdfSuffix);

        device->rootJoint()->lowerBound(0, -2);
        device->rootJoint()->lowerBound(1, -1);
        device->rootJoint()->lowerBound(2, 0.3);
        device->rootJoint()->upperBound(0,  5);
        device->rootJoint()->upperBound(1,  1);
        device->rootJoint()->upperBound(2,  4);

        hpp::pinocchio::Configuration_t q_ref(device->configSize());
        q_ref<<-2.0,
                0.0,
                0.6838277139631803,
                0.0,
                0.0,
                0.0,
                1.0,
                0.14279812395541294,
                0.934392553166556,
                -0.9968239786882757,
                -0.06521258938340457,
                -0.8831796268418511,
                1.150049183494211,
                -0.06927610020154493,
                0.9507443168724581,
                -0.8739975339028809,
                0.03995660287873871,
                -0.9577096766517215,
                0.93846028213260710;
       device->controlComputation(hpp::pinocchio::Computation_t(device->computationFlag()
                                        | hpp::pinocchio::JOINT_POSITION | hpp::pinocchio::JACOBIAN | hpp::pinocchio::COM));
       device->currentConfiguration(q_ref);
       device->computeForwardKinematics();
       return device;
    }

    std::vector<hpp::ik::FrameMarker> HyQFrames(hpp::pinocchio::DevicePtr_t hyq)
    {
        std::vector<hpp::ik::FrameMarker> res;
        fcl::Vec3f legOffset( 0,-0.021,0);
        fcl::Vec3f legNormal(0,1,0);

        hpp::pinocchio::Frame rffoot = hyq->getFrameByName("rf_foot_joint");
        hpp::ik::FrameMarker rffootfMarker(rffoot, legOffset, legNormal);

        hpp::pinocchio::Frame lffoot = hyq->getFrameByName("lf_foot_joint");
        hpp::ik::FrameMarker lffootMarker(lffoot, legOffset, legNormal);

        hpp::pinocchio::Frame rhfoot = hyq->getFrameByName("rh_foot_joint");
        hpp::ik::FrameMarker rhfootMarker(rhfoot, legOffset, legNormal);

        hpp::pinocchio::Frame lhfoot = hyq->getFrameByName("lh_foot_joint");
        hpp::ik::FrameMarker lhfootMarker(lhfoot, legOffset, legNormal);

        res.push_back(rffootfMarker);
        res.push_back(lffootMarker);
        res.push_back(rhfootMarker);
        res.push_back(lhfootMarker);
        return res;
    }

    hpp::pinocchio::DevicePtr_t loadHRP2(){
        const std::string robotName("hrp2_14");
        const std::string rootJointType ("freeflyer");
        const std::string packageName ("hrp2_14_description");
        const std::string modelName ("hrp2_14");
        const std::string urdfSuffix ("_reduced");
        const std::string srdfSuffix ("");

        hpp::pinocchio::DevicePtr_t device = hpp::pinocchio::Device::create (robotName);
        hpp::pinocchio::urdf::loadRobotModel(device, rootJointType, packageName, modelName,
        urdfSuffix, srdfSuffix);
        device->rootJoint()->lowerBound(0, -2);
        device->rootJoint()->lowerBound(1, -2);
        device->rootJoint()->lowerBound(2, 0.4);
        device->rootJoint()->upperBound(0,  2);
        device->rootJoint()->upperBound(1,  2);
        device->rootJoint()->upperBound(2,  1.2);

        device->setDimensionExtraConfigSpace(6); // used by kinodynamic methods
        for(std::size_t i = 0 ; i < 6 ; ++i){
          device->extraConfigSpace().lower(i)=-5;
          device->extraConfigSpace().upper(i)=5;
        }

        hpp::pinocchio::Configuration_t q_ref(device->configSize());
        q_ref<<0.1, -0.82, 0.648702, 0.0 , 0.0, 0.0, 1.0,0.0, 0.0, 0.0, 0.0,0.261799388,  0.174532925, 0.0, -0.523598776, 0.0, 0.0, 0.17,0.261799388, -0.174532925, 0.0, -0.523598776, 0.0, 0.0, 0.17,0.0, 0.0, -0.453785606, 0.872664626, -0.41887902, 0.0,0.0, 0.0, -0.453785606, 0.872664626, -0.41887902, 0.0,0,0,0,0,0,0;

        device->controlComputation(hpp::pinocchio::Computation_t(device->computationFlag()
                                         | hpp::pinocchio::JOINT_POSITION | hpp::pinocchio::JACOBIAN | hpp::pinocchio::COM));
        device->currentConfiguration(q_ref);
        device->computeForwardKinematics();
        return device;

    }

    std::vector<hpp::ik::FrameMarker> HRP2Frames(hpp::pinocchio::DevicePtr_t hrp2)
    {
        std::vector<hpp::ik::FrameMarker> res;

        fcl::Vec3f rLegOffset( 0,0,-0.105);
        fcl::Vec3f rLegNormal(0,0,1);
        hpp::pinocchio::Frame rffoot = hrp2->getFrameByName("RLEG_JOINT5");
        hpp::ik::FrameMarker rffootfMarker(rffoot, rLegOffset, rLegNormal);


        fcl::Vec3f lLegOffset(0,0,-0.105);
        fcl::Vec3f lLegNormal(0,0,1);
        hpp::pinocchio::Frame lffoot = hrp2->getFrameByName("LLEG_JOINT5");
        hpp::ik::FrameMarker lffootMarker(lffoot, lLegOffset, lLegNormal);


        fcl::Vec3f rarmOffset(0,0,-0.1);
        fcl::Vec3f rarmNormal(0,0,1);
        hpp::pinocchio::Frame rhfoot = hrp2->getFrameByName("RARM_JOINT5");
        hpp::ik::FrameMarker rhfootMarker(rhfoot, rarmOffset, rarmNormal);


        res.push_back(rffootfMarker);
        res.push_back(lffootMarker);
        res.push_back(rhfootMarker);
        return res;
    }
} // namespace




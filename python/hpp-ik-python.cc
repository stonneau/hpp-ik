#include "hpp/ik/definitions.hh"
#include "hpp/ik/ik-constraint.hh"
#include "hpp/pinocchio/urdf/util.hh"
#include "hpp/pinocchio/frame.hh"


#include <vector>

#include <eigenpy/memory.hpp>
#include <eigenpy/eigenpy.hpp>

#include <boost/python.hpp>

/*** TEMPLATE SPECIALIZATION FOR PYTHON ****/
typedef hpp::ik::vector_t  vector_t;
typedef hpp::ik::vector3_t vector3_t;
typedef hpp::ik::quat_t quat_t;


/*** TEMPLATE SPECIALIZATION FOR PYTHON ****/

EIGENPY_DEFINE_STRUCT_ALLOCATOR_SPECIALIZATION(vector_t)
EIGENPY_DEFINE_STRUCT_ALLOCATOR_SPECIALIZATION(vector3_t)
EIGENPY_DEFINE_STRUCT_ALLOCATOR_SPECIALIZATION(quat_t)

using namespace hpp::ik;

namespace hppy_ik
{
using namespace boost::python;

IkHelper* wrapIkHelperConstructor(const std::string& robotName, const std::string& rootJointType,
                                  const std::string& packageName, const std::string& modelName,
                                  const std::string& urdfSuffix, const std::string& srdfSuffix)
{
    hpp::pinocchio::DevicePtr_t device = hpp::pinocchio::Device::create (robotName);
    hpp::pinocchio::urdf::loadRobotModel(device, rootJointType, packageName, modelName,
    urdfSuffix, srdfSuffix);
    return new IkHelper(device);
}

void clearProjector(IkHelper& helper)
{
    helper.proj_ = hpp::core::ConfigProjector::create(helper.device_, "proj",
                                                      helper.proj_->errorThreshold(), 40 );
}

void setConfig(IkHelper& helper, vector_t config )
{
    helper.device_->currentConfiguration(config);
    helper.device_->computeForwardKinematics();
}

vector_t getConfig(IkHelper& helper)
{
    return helper.device_->currentConfiguration();
}


bool project(IkHelper& helper)
{
    hpp::pinocchio::Configuration_t  conf = helper.device_->currentConfiguration();
    if (!helper.proj_->apply(conf))
        return false;
    setConfig(helper, conf);
    return true;
}


vector_t pos(FrameMarker& frameMarker)
{
    return frameMarker.frame_.currentTransformation().translation();
}

quat_t rot(FrameMarker& frameMarker)
{
    return quat_t(frameMarker.frame_.currentTransformation().rotation());
}


FrameMarker* warpFrameConstructor1(IkHelper& helper, const std::string& frameName)
{
    hpp::pinocchio::Frame frame = helper.device_->getFrameByName(frameName);
    return new FrameMarker(frame);
}

void add3DConstraint(IkHelper& helper, const FrameMarker& frame, const vector3_t& pos)
{
    AddConstraint(helper,frame.frame_,pos);
}

BOOST_PYTHON_MODULE(hppy_ik)
{
    /** BEGIN eigenpy init**/
    eigenpy::enableEigenPy();

    eigenpy::enableEigenPySpecific<vector3_t,vector3_t>();
    eigenpy::enableEigenPySpecific<vector_t,vector_t>();
    //eigenpy::exposeAngleAxis();
    //eigenpy::exposeQuaternion();

    /** END eigenpy init**/
    /** BEGIN struct**/

    class_<IkHelper>("IkHelper", no_init)
            .def("__init__", make_constructor(&wrapIkHelperConstructor))
            .def("clear", &clearProjector);

    class_<FrameMarker>("FrameMarker", no_init)
            .def("__init__", make_constructor(&warpFrameConstructor1))
            .def("pos", &pos)
            .def("rot", &rot);

    ;
    /** END struct**/

    /** methods **/
    def("target3D",  &add3DConstraint);
    def("setConfig", &setConfig);
    def("getConfig", &getConfig);
    def("project"  , &project);
    def("clearProjector"  , &clearProjector);

    /** BEGIN enum types **/
    enum_<MaintainConstraintType>("MaintainConstraintType")
            .value("MAINTAIN_3D"    , MAINTAIN_3D)
            .value("MAINTAIN_6D"    , MAINTAIN_6D)
            .export_values();
    /** END enum types **/
}
} // namespace hppy_ik


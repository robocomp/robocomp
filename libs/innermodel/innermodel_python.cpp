// http://www.boost.org/doc/libs/1_53_0/libs/python/doc/tutorial/doc/html/python/exposing.html

#include <boost/python.hpp>

#include <innermodel/innermodel.h>
#include <osgviewer/osgview.h>

using namespace RMat;
using namespace boost::python;


BOOST_PYTHON_MODULE(librobocomp_innermodel)
{
	void (InnerModel::*updateTransformValues)(std::string transformId, float tx, float ty, float tz, float rx, float ry, float rz, std::string parentId) = &InnerModel::updateTransformValuesS;
	void (InnerModel::*updateTransformValuesV)(std::string transformId, QVec v, std::string parentId) = &InnerModel::updateTransformValuesS;
	QVec (InnerModel::*transform)(const std::string & destId, const QVec &origVec, const std::string &origId) = &InnerModel::transformS;

	class_<InnerModel>("InnerModel", init<>())
	  .def(init<std::string>())
	  .def(init<InnerModel &>())
	  .def("updateTransformValues", updateTransformValues,
	    (
	    arg("transformId"),
	    arg("tx"), arg("ty"), arg("tz"),
	    arg("rx"), arg("ry"), arg("rz"),
	    arg("parentId")=""
	    )
	   )
	  .def("updateTransformValuesV", updateTransformValuesV,
	    (
	      arg("transformId"),
	      arg("v"),
	      arg("parentId")=""
	    )
	  )
	  .def("cleanupTables", &InnerModel::cleanupTables)
	  .def("jacobian", &InnerModel::jacobianSPython)
	  .def("transform", transform)
	  .def("getTransformationMatrix", &InnerModel::getTransformationMatrixS)
	  .def("getParentIdentifier", &InnerModel::getParentIdentifierS)

	  .def("getJoint", &InnerModel::getJointRef, boost::python::return_value_policy<boost::python::reference_existing_object>())
	;


	/*
	class_<InnerModelJoint>("InnerModelJoint",
	 init<
	  QString,
	  float, float, float,
	  float, float, float,
	  float, float, float,
	  float, float, float,
	  float, float,
	  uint32_t, std::string, float, InnerModelTransform * >
	 (
	  arg("id_"),
	  arg("lx_"), arg("ly_"), arg("lz_"),
	  arg("hx_"), arg("hy_"), arg("hz_"),
	  arg("tx_"), arg("ty_"), arg("tz_"),
	  arg("rx_"), arg("ry_"), arg("rz_"),
	  arg("min_") = -INFINITY, arg("max_") = INFINITY,
	  arg("port_") = 0, arg("axis") = "z", arg("home_") = 0, arg("parent_") = NULL
	 ))
*/
	class_<InnerModelJoint>("InnerModelJoint", init<>())
	 .def("getAngle", &InnerModelJoint::getAngle)
	 .def("unitaryAxis", &InnerModelJoint::unitaryAxis)
    	 .def("setAngle", &InnerModelJoint::setAngle,
    	     (
    	         arg("angle"),
    	         arg("force")=false
    	     )
    	 )
	;



}

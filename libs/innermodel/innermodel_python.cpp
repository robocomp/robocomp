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
    	 .def("transform", transform)
    	 .def("getTransformationMatrix", &InnerModel::getTransformationMatrixS)
    	 .def("getParentIdentifier", &InnerModel::getParentIdentifierS)
	;
}

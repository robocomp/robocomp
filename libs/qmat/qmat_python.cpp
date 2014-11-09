// http://www.boost.org/doc/libs/1_53_0/libs/python/doc/tutorial/doc/html/python/exposing.html

#include <boost/python.hpp>

#include <qmat/qvec.h>
#include <qmat/qmat.h>

using namespace boost::python;
using namespace RMat;


BOOST_PYTHON_MODULE(librobocomp_qmat)
{

	class_<QVec>("QVec", init<>()) 
	 .def(init<int32_t>())
	 .def(init<int32_t, float>())
	 .def(init<boost::python::list>())
	 .def("__setitem__", &QVec::setItem)
	 .def("__getitem__", &QVec::getItem)
	 .def(self - self)
	 .def(self + self)
	 .def(self * int())
	 .def("printvector", &QVec::prints)
	 .def("vec3", &QVec::vec3)
    ;

	class_<QMat>("QMat", init<>()) 
	 .def(init<int32_t, int32_t>())
	 .def(init<int32_t, int32_t, float>())
	 .def("__setitem__", &QMat::setItemV)
	 .def("__getitem__", &QMat::getItemV)
	 .def("set", &QMat::setItem)
	 .def("get", &QMat::getItem)
	 .def(self - self)
	 .def(self + self)
	 .def(self * int())
	 .def("printmatrix", &QMat::prints)
    ;
	
}



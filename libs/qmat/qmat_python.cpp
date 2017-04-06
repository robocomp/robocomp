// http://www.boost.org/doc/libs/1_53_0/libs/python/doc/tutorial/doc/html/python/exposing.html

#include <boost/python.hpp>

#include <qmat/qvec.h>
#include <qmat/qmat.h>
#include <qmat/qrtmat.h>

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
	 .def(self += self)
	 .def(self -= self)
	 .def(self * int())
// 	 .def(self * class_<QMat>)
	 .def("printvector", &QVec::prints)
	 .def("set", &QMat::setItem)
	 .def("get", &QMat::getItem)
	 .def("vec1", &QVec::vec1)
	 .def("vec2", &QVec::vec2)
	 .def("vec3", &QVec::vec3)
	 .def("scalarDivision", &QVec::scalarDivision)
	 .def("scalarMultiplication", &QVec::scalarMultiplication)
	 .def("pointProduct", &QVec::pointProduct)
	 .def("normalize", &QVec::normalize)
	 .def("crossProduct", &QVec::crossProduct)
	 .def("equals", &QVec::equals)
	 .def("toHomogeneousCoordinates", &QVec::toHomogeneousCoordinates)
	 .def("fromHomogeneousCoordinates", &QVec::fromHomogeneousCoordinates)
	 .def("norm2", &QVec::norm2)
	 .def("max", &QVec::maxP)
	 .def("min", &QVec::minP)
	 .def("maxAbs", &QVec::maxAbs)
	 .def("minAbs", &QVec::minAbs)
	 .def("externProduct", &QVec::externProduct)
	 .def("crossProductMatrix", &QVec::crossProductMatrix)
	 .def("toRowMatrix", &QVec::toRowMatrix)
	 .def("toColumnMatrix", &QVec::toColumnMatrix)
	 .def("gaussianVector", &QVec::gaussianVector)
	 .def("uniformVector", &QVec::uniformVector)
	 .def("gaussianSamples", &QVec::gaussianSamples)
	 .def("dotProduct", &QVec::dotProduct)
// 	 .def("mean", &QVec::mean)
// 	 .def("variance", &QVec::variance)
	 .def("len", &QVec::size)
	 .def("size", &QVec::size)
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
	 .def(self += self)
	 .def(self -= self)
	 .def(self * self)
	 .def(self / self)
	 .def(self * int())
	 .def("printmatrix", &QMat::prints)
	 .def("invert", &QMat::invert)
	 .def("nRows", &QMat::nRows)
	 .def("nCols", &QMat::nCols)
	 .def("transpose", &QMat::transpose)
	 .def("t", &QMat::t)
	 .def("determinant", &QMat::determinant)
	 .def("trace", &QMat::trace)
	 .def("extractAnglesR", &QMat::extractAnglesR)
	 .def("extractAnglesR_min", &QMat::extractAnglesR_min)
	 .def("sqrt", &QMat::sqrt)
	 .def("cholesky", &QMat::cholesky)
	 .def("eigenValsVectors", &QMat::eigenValsVectors)
	 .def("getCol", &QMat::getCol)
	 .def("getRow", &QMat::getRow)

    ;

	class_<RTMat>("RTMat", init<>())
	 .def(init< optional<bool, bool, bool> >())
	 .def(init< float, float, float, float, float, float, optional<bool, bool, bool> >())

	 .def("__setitem__", &RTMat::setItemV)
	 .def("__getitem__", &RTMat::getItemV)
	 .def("set", &RTMat::setItem)
	 .def("get", &RTMat::getItem)
	 .def(self - self)
	 .def(self + self)
	 .def(self += self)
	 .def(self -= self)
	 .def(self * self)
	 .def(self / self)
	 // a method was here
	 .def("printmatrix", &RTMat::prints)
	 .def("invert", &RTMat::invert)
	 .def("nRows", &RTMat::nRows)
	 .def("nCols", &RTMat::nCols)
	 .def("transpose", &RTMat::transpose)
	 .def("t", &RTMat::t)
	 .def("determinant", &RTMat::determinant)
	 .def("trace", &RTMat::trace)
	 .def("extractAnglesR", &RTMat::extractAnglesR)
	 .def("extractAnglesR_min", &RTMat::extractAnglesR_min)
	 .def("sqrt", &RTMat::sqrt)
	 .def("cholesky", &RTMat::cholesky)
	 .def("eigenValsVectors", &RTMat::eigenValsVectors)
	 .def("getCol", &RTMat::getCol)
	 .def("getRow", &RTMat::getRow)
    ;

}

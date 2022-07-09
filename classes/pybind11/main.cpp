/////////////////////////////////////////////////
// Code example of how to call Python from C++
///////////////////////////////////////////////
#include <iostream>
#include <pybind11/embed.h> 
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <cppitertools/range.hpp>
#include <cppitertools/enumerate.hpp>

namespace py = pybind11;
using namespace pybind11::literals; // to bring in the `_a` literal

int main() 
{
	py::scoped_interpreter guard{}; // start the interpreter and keep it alive

	py::module np = py::module::import("numpy");
	py::object plt = py::module::import("matplotlib.pyplot");
	// create a spline object
	py::object app_spline = py::module::import("scipy.interpolate").attr("splprep");
	// create a spline evaluator
	py::object get_spline = py::module::import("scipy.interpolate").attr("splev");

	// create a numpy array
	py::array_t<float> python_x = np.attr("arange")(0, 2.f*M_PI + M_PI/4.f, 2.f*M_PI/8.f);
	
	// create an Eigen::VectorXf and fill it from an std::vector
	std::vector<float> kk;
	for(auto i : iter::range(0.0, 2.f*M_PI + M_PI/4.f, 2.f*M_PI/8.f))
		kk.push_back(i);
	Eigen::VectorXf x(kk.size());
	for(auto [l,i]: iter::enumerate(kk))
		x[l] = i;
	
	// call functions form numpy passing directly the Eigen::VectorXf
	py::array_t<float> y = np.attr("sin")(x);
	
	// create a python tuple
	py::tuple t = py::make_tuple(1, 2, 3);
	py::tuple values = py::make_tuple(x, y);
	// cal the LSQUnivariateSpline from numpy.interpolate and get a callable object
	//py::object res = app_spline(x, y, t);
	py::tuple spline = app_spline(values);
	 
	// call functions form numpy`
	py::object xnew = np.attr("arange")(0, 2.f*M_PI, M_PI/50.f);
	// call the spline object with new data
	//py::object ynew = res(xnew);
	py::tuple out = get_spline(xnew, spline[0]);	
	//print through python
	py::print(out[0]);
	//print in c++
	for(auto &e: out[0])
		std::cout << e << std::endl;
	// draw
	plt.attr("plot")(x, y, "x", out[0], out[1], x, y, "b");
	plt.attr("show")();
	
}

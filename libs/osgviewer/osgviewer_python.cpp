// http://www.boost.org/doc/libs/1_53_0/libs/python/doc/tutorial/doc/html/python/exposing.html

#include <boost/python.hpp>

#include <osgviewer/osgview.h>

using namespace RMat;
using namespace boost::python;



BOOST_PYTHON_MODULE(librobocomp_osgviewer)
{
	class_<OsgView>("OsgView", init<QWidget *>());
}

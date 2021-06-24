#include <boost/python.hpp>
#include <boost/python/list.hpp>
#include <boost/python/extract.hpp>
#include <boost/python/numpy.hpp>


#include <erquy.hpp>

using namespace boost::python;

BOOST_PYTHON_MODULE(erquy_py) {
	// Py_Initialize();
	// boost::python::numpy::initialize();

	class_<erquy::World>("World", init<>())
		.def("loadUrdf", &erquy::World::loadUrdf)
		// .def("setState", &erquy::World::setState)
		.def("integrate", &erquy::World::integrate)
	;
}
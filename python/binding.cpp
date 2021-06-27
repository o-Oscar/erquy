#include "pinocchio/fwd.hpp"
#include "eigenpy/eigenpy.hpp"

#include <boost/python.hpp>
#include <boost/python/list.hpp>
#include <boost/python/extract.hpp>
#include <boost/python/numpy.hpp>


#include <erquy.hpp>

using namespace boost::python;

BOOST_PYTHON_MODULE(erquy_py) {
	// Py_Initialize();
	boost::python::numpy::initialize();
	eigenpy::enableEigenPy();

	class_<erquy::World>("World")
		.def("loadUrdf", &erquy::World::loadUrdf)

		// the almighty integrate step
		.def("integrate", &erquy::World::integrate)

		// --- getters and setters ---
		.def("getGeneralizedCoordinateDim", &erquy::World::getGeneralizedCoordinateDim)
		.def("getGeneralizedVelocityDim", &erquy::World::getGeneralizedVelocityDim)
		.def("getDOF", &erquy::World::getGeneralizedVelocityDim)
		.def("nq", &erquy::World::getGeneralizedCoordinateDim)
		.def("nv", &erquy::World::getGeneralizedVelocityDim)


		.def("setGeneralizedCoordinate", &erquy::World::setGeneralizedCoordinate)
		.def("getGeneralizedCoordinate", &erquy::World::getGeneralizedCoordinate)

		.def("setGeneralizedVelocity", &erquy::World::setGeneralizedVelocity)
		.def("getGeneralizedVelocity", &erquy::World::getGeneralizedVelocity)
		
		.def("setState", &erquy::World::setState)
		.def("getState", &erquy::World::getState)

		.def("setTimeStep", &erquy::World::setTimeStep)
		.def("getTimeStep", &erquy::World::getTimeStep)

		.def("setERP", &erquy::World::setERP)
		.def("getERP", &erquy::World::getERP)

		.def("setGravity", &erquy::World::setGravity)
		.def("getGravity", &erquy::World::getGravity)

		.def("getJacobians", range(&erquy::World::getJacB, &erquy::World::getJacE))
		.def("getM", &erquy::World::getM)
	;
}
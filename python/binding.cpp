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

		.def("enablePd", &erquy::World::enablePd)
		.def("setPdGains", &erquy::World::setPdGains)
		.def("setPdTarget", &erquy::World::setPdTarget)
		.def("getPdForce", &erquy::World::getPdForce)

		.def("setGeneralizedTorque", &erquy::World::setGeneralizedTorque)


		.def("getContactInfos", &erquy::World::getContactInfos)

		.def("getFrameNames", range(&erquy::World::getFrameNamesBegin, &erquy::World::getFrameNamesEnd))
		.def("getFrameIdxByName", &erquy::World::getFrameIdxByName)
		.def("getJointNames", range(&erquy::World::getJointNamesBegin, &erquy::World::getJointNamesEnd))
		.def("getJointIdxByName", &erquy::World::getJointIdxByName)

		.def("setMaterialPairProp", &erquy::World::setMaterialPairProp)

		.def("getFramePosition", &erquy::World::getFramePosition)
		.def("getFrameOrientation", &erquy::World::getFrameOrientation)
		.def("getFrameVelocity", &erquy::World::getFrameVelocity)
		.def("getFrameAngularVelocity", &erquy::World::getFrameAngularVelocity)

		.def_readonly("solver", &erquy::World::solver)
	;

	class_<erquy::PgsSolver>("PgsSolver")
		.def("set_solver_params", &erquy::PgsSolver::set_solver_params)
	;
}

#include "erquy.hpp"

using namespace erquy;

World::World () {

}

void World::loadUrdf (std::string urdf_path, std::string meshes_path) {
	pinocchio::urdf::buildModel(urdf_path, model_);
	data_ = pinocchio::Data(model_);
	
	pinocchio::urdf::buildGeom(model_, urdf_path, pinocchio::COLLISION, geom_model_, "");
	geom_model_.addAllCollisionPairs();
	geom_data_ = pinocchio::GeometryData (geom_model_);
	std::cout << "Loading the model " << std::endl;
}

// --- getters and setters ---

void World::setState (Eigen::VectorXd q, Eigen::VectorXd u) {
	q_ = q;
	u_ = u;
}
boost::python::tuple World::getState () {
	return boost::python::make_tuple(q_, u_);
}

void World::setTimeStep (real timeStep) {
	timeStep_ = timeStep;
}
real World::getTimeStep () {
	return timeStep_;
}

void World::setERP (real erp) {
	erp_ = erp;
}
real World::getERP () {
	return erp_;
}

void World::setGravity (Eigen::Vector3d gravity) {
	model_.gravity.linear() = gravity;
}
Eigen::Vector3d World::getGravity () {
	return model_.gravity.linear();
}


namespace py = boost::python;

template<class T>
py::list std_vector_to_py_list(const std::vector<T>& v)
{
    py::object get_iter = py::iterator<std::vector<T> >();
    py::object iter = get_iter(v);
    py::list l(iter);
    return l;
}

// DEPRECIATED (brocken)
boost::python::tuple World::getStepInfo () {
	if (all_jac_.size() > 0) {
		return boost::python::make_tuple(all_jac_[0], all_lamb_[0]);
	} else {
		return boost::python::make_tuple();
	}
	// return boost::python::make_tuple(std_vector_to_py_list<Eigen::MatrixXd>(all_jac_), std_vector_to_py_list<Eigen::Vector3d>(all_lamb_));
}


#include <iostream>
/*
#include "pinocchio/parsers/sample-models.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
*/
#include "erquy.hpp"

erquy::World::World () : model_() {

}

void erquy::World::loadUrdf (std::string urdf_path, std::string meshes_path) {
	// pinocchio::urdf::buildModel(urdf_path, model_);
	// data_ = pinocchio::Data(model_);
	
	// pinocchio::urdf::buildGeom(model_, urdf_path, pinocchio::COLLISION, geom_model_, "");
	// geom_model_.addAllCollisionPairs();
	// geom_data_ = pinocchio::GeometryData (geom_model_);
	std::cout << "Loading the model " << std::endl;
}
/*
void erquy::World::setState (Eigen::VectorXd q, Eigen::VectorXd u) {
	q_ = q;
	u_ = u;
}
*/
void erquy::World::integrate () {
	std::cout << "coucou" << std::endl;
}
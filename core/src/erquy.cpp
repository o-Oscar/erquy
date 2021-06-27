
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

	gravity_ << 0, 0, 0;
	model_.gravity.linear() << 0, 0, 0;
}

Eigen::Matrix3d get_skew_from_vector(Eigen::Vector3d dpos) {
	Eigen::Matrix3d to_return;
	to_return << 0, -dpos(2), dpos(1),
				dpos(2), 0, -dpos(0),
				-dpos(1), dpos(0), 0;
	return to_return;
}

void World::integrate () {

	all_jac_.clear();
	all_lamb_.clear();
	
	pinocchio::forwardKinematics(model_, data_, q_, u_); // update every positions and velocities
	pinocchio::updateFramePlacements(model_, data_);

	b = pinocchio::rnea(model_, data_, q_, u_, Eigen::VectorXd::Zero(model_.nv)); // computes the torques needed to track zero acceleration
	M = pinocchio::crba(model_, data_, q_); // computes the mass matrix
	// fucking pinocchio
	M.triangularView<Eigen::StrictlyLower>() = M.transpose().triangularView<Eigen::StrictlyLower>();
	/*
	b = Eigen::VectorXd::Zero(model_.nv); // computes the torques needed to track zero acceleration
	M = Eigen::MatrixXd::Identity(model_.nv, model_.nv); // computes the mass matrix*/
	// std::cout << M << std::endl;

	// calculate acceleration due to gravity
	ag = Eigen::VectorXd::Zero(model_.nv);
	// for (auto const & joint : model_.joints) {
	
	for (int i(0); i < model_.njoints; i++) {
		if (model_.joints[i].nq() == 7 && model_.joints[i].nv() == 6) {
			ag.segment<3>(model_.joints[i].idx_v()) = data_.oMi[i].rotation().transpose() * gravity_;
		}
	}/*
	for (int i(0); i < model_.nframes; i++) {
		std::cout << model_.frames[i] << std::endl;
	}*/
	
	// finding all collisions
	pinocchio::computeCollisions(model_, data_, geom_model_, geom_data_, q_);

	int fid0; // id of the frames that are potentially in collision
	int fid1;

	for (int k=0; k < geom_model_.collisionPairs.size(); k++) {
		const pinocchio::CollisionPair & cp = geom_model_.collisionPairs[k];
		const hpp::fcl::CollisionResult & cr = geom_data_.collisionResults[k];

		fid0 = geom_model_.geometryObjects[cp.first].parentFrame; 
		fid1 = geom_model_.geometryObjects[cp.second].parentFrame; 
		
		for (int l(0); l < cr.numContacts(); l++) {
			hpp::fcl::Contact c = cr.getContact(l);

			// computing the jacobian for this contact
			Eigen::MatrixXd fjac0 = Eigen::MatrixXd::Zero(6, model_.nv);
			Eigen::MatrixXd fjac1 = Eigen::MatrixXd::Zero(6, model_.nv);
			pinocchio::computeFrameJacobian(model_, data_, q_, fid0, pinocchio::LOCAL_WORLD_ALIGNED, fjac0);
			pinocchio::computeFrameJacobian(model_, data_, q_, fid1, pinocchio::LOCAL_WORLD_ALIGNED, fjac1);

			Eigen::Vector3d dpos0 = data_.oMf[fid0].translation() - c.pos; // = pos in world - pos of frame
			Eigen::Vector3d dpos1 = data_.oMf[fid1].translation() - c.pos; // = pos in world - pos of frame

			Eigen::Matrix3d dpos_hat0 = get_skew_from_vector(dpos0);
			Eigen::Matrix3d dpos_hat1 = get_skew_from_vector(dpos1);
			

			Eigen::MatrixXd res_jac = fjac0.topRows(3) + dpos_hat0 * fjac0.bottomRows(3)
									- fjac1.topRows(3) - dpos_hat1 * fjac1.bottomRows(3);
			
			Eigen::Quaterniond normal_rot = Eigen::Quaterniond::FromTwoVectors(c.normal, -Eigen::Vector3d::UnitZ());
			res_jac = normal_rot.normalized().toRotationMatrix() * res_jac;
			
			all_jac_.push_back(res_jac);
			all_lamb_.push_back(Eigen::Vector3d::Zero(3));
		}
	}
	
	
	// Solves for the contact forces that satisfy every constrains
	Eigen::MatrixXd M_inv = M.inverse();
	PgsSolver::solve (u_ + timeStep_ * ag - M_inv * timeStep_ * b, M_inv, all_jac_, all_lamb_);
	

	// computes the forward dynamics
	Eigen::VectorXd full_tau =  - timeStep_ * b;
	
	for (int i(0); i < all_jac_.size(); i++) {
		full_tau += all_jac_[i].transpose() * all_lamb_[i];
	}
	// std::cout << "full_tau" << std::endl;
	// std::cout << full_tau.transpose() << std::endl;
	u_ += timeStep_ * ag + M.colPivHouseholderQr().solve(full_tau);
	q_ = pinocchio::integrate(model_, q_, u_ * timeStep_);
}

// --- getters and setters ---

int World::getGeneralizedCoordinateDim (){
	return model_.nq;
}
int World::getGeneralizedVelocityDim (){
	return model_.nv;
}

void World::setGeneralizedCoordinate (Eigen::VectorXd q) {
	q_ = q;
}
Eigen::VectorXd World::getGeneralizedCoordinate () {
	return q_;
}

void World::setGeneralizedVelocity (Eigen::VectorXd u) {
	u_ = u;
}
Eigen::VectorXd World::getGeneralizedVelocity () {
	return u_;
}

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
	// model_.gravity.linear() = gravity;
	gravity_ = gravity;
}
Eigen::Vector3d World::getGravity () {
	// return model_.gravity.linear();
	return gravity_;
}

std::vector<Eigen::MatrixXd>::iterator World::getJacB ()
{
	return all_jac_.begin();
}
std::vector<Eigen::MatrixXd>::iterator World::getJacE ()
{
	return all_jac_.end();
}

Eigen::MatrixXd World::getM () {
	return M;
}
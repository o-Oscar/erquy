
#include "erquy.hpp"

using namespace erquy;

World::World () {
	material_pair_props_[default_contact_pair_] = .2;
}

Eigen::Matrix3d get_skew_from_vector(Eigen::Vector3d dpos) {
	Eigen::Matrix3d to_return;
	to_return << 0, -dpos(2), dpos(1),
				dpos(2), 0, -dpos(0),
				-dpos(1), dpos(0), 0;
	return to_return;
}

// main method

void World::integrate () {

	pinocchio::computeJointJacobians(model_, data_, q_);

	all_jac_.clear();
	all_lamb_.clear();
	
	// finding all collisions
	pinocchio::computeCollisions(model_, data_, geom_model_, geom_data_, q_);

	int fid0; // id of the frames that are potentially in collision
	int fid1;
	int jid0; // id of the joints that are potentially in collision
	int jid1;

	n_contact_ = 0;
	for (int k=0; k < geom_model_.collisionPairs.size(); k++) {
		n_contact_ += geom_data_.collisionResults[k].numContacts();
	}

	contact_joint_id_ = Eigen::MatrixXi::Zero(n_contact_, 2);
	full_jac_ = Eigen::MatrixXd::Zero(3*n_contact_, model_.nv);
	// full_lamb_ = Eigen::VectorXd::Zero(3*n_contact_);
	
	if (full_lamb_.rows() != 3*n_contact_) { // warm-starts the algorithm with the previous solution to speed up the resolution
		full_lamb_ = Eigen::VectorXd::Zero(3*n_contact_);
	}
	


	
	all_mu_.clear();

	int contact_id = 0;
	for (int k=0; k < geom_model_.collisionPairs.size(); k++) {
		for (int l(0); l < geom_data_.collisionResults[k].numContacts(); l++) {
			hpp::fcl::Contact c = geom_data_.collisionResults[k].getContact(l);

			fid0 = geom_model_.geometryObjects[geom_model_.collisionPairs[k].first].parentFrame; 
			fid1 = geom_model_.geometryObjects[geom_model_.collisionPairs[k].second].parentFrame; 
			jid0 = model_.frames[fid0].parent;
			jid1 = model_.frames[fid1].parent;
			contact_joint_id_(contact_id, 0) = jid0;
			contact_joint_id_(contact_id, 1) = jid1;
			std::pair<int, int> p = std::make_pair(jid0, jid1);
			if (material_pair_props_.find(p) != material_pair_props_.end()) {
				all_mu_.push_back(material_pair_props_[p]);
			}
			else {
				all_mu_.push_back(material_pair_props_[default_contact_pair_]);
			}
			
			
			// computing the jacobian for this contact
			Eigen::MatrixXd fjac0 = Eigen::MatrixXd::Zero(6, model_.nv);
			Eigen::MatrixXd fjac1 = Eigen::MatrixXd::Zero(6, model_.nv);
			pinocchio::getJointJacobian(model_, data_, jid0, pinocchio::WORLD, fjac0);
			pinocchio::getJointJacobian(model_, data_, jid1, pinocchio::WORLD, fjac1);

			Eigen::Matrix3d cpos_hat = get_skew_from_vector(- c.pos);

			Eigen::MatrixXd res_jac = fjac0.topRows(3) + cpos_hat * fjac0.bottomRows(3)
									- fjac1.topRows(3) - cpos_hat * fjac1.bottomRows(3);
			
			Eigen::Quaterniond normal_rot = Eigen::Quaterniond::FromTwoVectors(c.normal, -Eigen::Vector3d::UnitZ());
			full_jac_.block(contact_id*3, 0, 3, model_.nv) = normal_rot.normalized().toRotationMatrix() * res_jac;
			contact_id++;
		}
	}
	
	// main pinocchio algorithm
	h_ = pinocchio::rnea(model_, data_, q_, u_, Eigen::VectorXd::Zero(model_.nv)); // computes the torques needed to track zero acceleration
	pinocchio::crba(model_, data_, q_); // computes the mass matrix
	data_.M.triangularView<Eigen::StrictlyLower>() = data_.M.transpose().triangularView<Eigen::StrictlyLower>(); // fucking pinocchio
	
	// calculate acceleration due to gravity
	ag_ = Eigen::VectorXd::Zero(model_.nv);
	for (int i(0); i < model_.njoints; i++) {
		if (model_.joints[i].nq() == 7 && model_.joints[i].nv() == 6) {
			ag_.segment<3>(model_.joints[i].idx_v()) = data_.oMi[i].rotation().transpose() * gravity_;
		}
	}

	// computes the pysical quantities for easy integration
	dq_ = Eigen::VectorXd::Zero(model_.nv);
	pinocchio::difference(model_, q_targ_, q_, dq_);
	tau_star_ =  data_.M * (u_ + timeStep_ * ag_) + timeStep_ * (- h_ + tau_ - dq_.cwiseProduct(kp_) + u_targ_.cwiseProduct(kd_)) ;
	M_bar_ = data_.M + (timeStep_ * timeStep_ * kp_ + timeStep_ * kd_).asDiagonal().toDenseMatrix();
	M_bar_inv_ = M_bar_.inverse();
	
	// Solves for the contact forces that satisfy every constrains
	solver.solve (M_bar_inv_ * tau_star_, M_bar_inv_, n_contact_, full_jac_, full_lamb_, all_mu_);
	
	// computes the forward dynamics
	u_ = M_bar_inv_ * (tau_star_ + full_jac_.transpose() * full_lamb_);
	q_ = pinocchio::integrate(model_, q_, u_ * timeStep_);
	pinocchio::forwardKinematics(model_, data_, q_, u_);
}




// --- getters and setters ---

void World::loadUrdf (std::string urdf_path, std::string meshes_path) {
	pinocchio::urdf::buildModel(urdf_path, model_);
	data_ = pinocchio::Data(model_);
	
	pinocchio::urdf::buildGeom(model_, urdf_path, pinocchio::COLLISION, geom_model_, "");
	geom_model_.addAllCollisionPairs();
	geom_data_ = pinocchio::GeometryData (geom_model_);
	// std::cout << "Loading the model : " << urdf_path << std::endl;

	gravity_ << 0, 0, 0;
	model_.gravity.linear() << 0, 0, 0;

	kp_ = Eigen::VectorXd::Zero(model_.nv);
	kd_ = Eigen::VectorXd::Zero(model_.nv);
	tau_ = Eigen::VectorXd::Zero(model_.nv);

	frame_names_.clear();
	for (auto frame : model_.frames) {
		frame_names_.push_back(frame.name);
	}
}


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
	pinocchio::forwardKinematics(model_, data_, q_, u_);
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
	gravity_ = gravity;
}
Eigen::Vector3d World::getGravity () {
	return gravity_;
}

void World::enablePd (bool enable_pd) {
	enable_pd_ = enable_pd;
}

void World::setPdGains (Eigen::VectorXd kp, Eigen::VectorXd kd) {
	kp_ = kp;
	kd_ = kd;
}

void World::setPdTarget (Eigen::VectorXd q_targ, Eigen::VectorXd u_targ) {
	q_targ_ = q_targ;
	u_targ_ = u_targ;
}

Eigen::MatrixXd World::getPdForce () {
	dq_ = Eigen::VectorXd::Zero(model_.nv);
	pinocchio::difference(model_, q_targ_, q_, dq_);
	return tau_ - dq_.cwiseProduct(kp_) - (u_-u_targ_).cwiseProduct(kd_);
}

void World::setGeneralizedTorque (Eigen::VectorXd tau) {
	tau_ = tau;
}

boost::python::tuple World::getContactInfos () {
	Eigen::MatrixXd contact_forces =  full_lamb_ / timeStep_;
	return boost::python::make_tuple(n_contact_, contact_joint_id_, contact_forces);
}

void World::setMaterialPairProp (int first_idx, int second_idx, real mu) {
	std::pair<int, int> p = std::make_pair(first_idx, second_idx);
	material_pair_props_[p] = mu;
}

std::vector<std::string>::iterator World::getJointNamesBegin()
{
	return model_.names.begin();
}
std::vector<std::string>::iterator World::getJointNamesEnd()
{
	return model_.names.end();
}

int World::getJointIdxByName (std::string name) {
	for (int i(0); i < model_.njoints; i++) {
		if (model_.names[i] == name) {
			return i;
		}
	}
	return -1;
}

std::vector<std::string>::iterator World::getFrameNamesBegin()
{
	return frame_names_.begin();
}
std::vector<std::string>::iterator World::getFrameNamesEnd()
{
	return frame_names_.end();
}

int World::getFrameIdxByName (std::string name) {
	for (int i(0); i < model_.nframes; i++) {
		if (model_.frames[i].name == name) {
			return i;
		}
	}
	return -1;
}
Eigen::Vector3d World::getFramePosition (int idx) {
	return pinocchio::updateFramePlacement(model_, data_, idx).translation();
}
Eigen::Matrix3d World::getFrameOrientation (int idx) {
	return pinocchio::updateFramePlacement(model_, data_, idx).rotation();
}
Eigen::Vector3d World::getFrameVelocity (int idx) {
	return pinocchio::getFrameVelocity(model_, data_, idx, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED).linear();
}
Eigen::Vector3d World::getFrameAngularVelocity (int idx) {
	return pinocchio::getFrameVelocity(model_, data_, idx, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED).angular();
}
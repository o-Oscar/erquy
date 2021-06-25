
#include "erquy.hpp"

using namespace erquy;

Eigen::Matrix3d get_skew_from_vector(Eigen::Vector3d dpos) {
	Eigen::Matrix3d to_return;
	to_return << 0, -dpos(2), dpos(1),
				dpos(2), 0, -dpos(0),
				-dpos(1), dpos(0), 0;
	return to_return;
}

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

void World::integrate () {
	Eigen::VectorXd a0 = Eigen::VectorXd::Zero(model_.nv);
	Eigen::VectorXd v0 = Eigen::VectorXd::Zero(model_.nv);
	Eigen::VectorXd b;
	Eigen::MatrixXd M;

	Eigen::VectorXd full_tau;


	
	// Ma = -b
	{
		
		pinocchio::forwardKinematics(model_, data_, q_, u_); // update every positions and velocities
		pinocchio::updateFramePlacements(model_, data_);

		b = pinocchio::rnea(model_, data_, q_, u_, a0); // computes the torques needed to track zero acceleration. Gravity is already included.
		M = pinocchio::crba(model_, data_, q_); // computes the mass matrix
		
		all_jac_.clear();
		all_lamb_.clear();
		
		// finding all collisions
		pinocchio::computeCollisions(model_, data_, geom_model_, geom_data_, q_);

		for (size_t k=0; k < geom_model_.collisionPairs.size(); k++) {
			const pinocchio::CollisionPair & cp = geom_model_.collisionPairs[k];

			int fid0 = geom_model_.geometryObjects[cp.first].parentFrame; 
			int fid1 = geom_model_.geometryObjects[cp.second].parentFrame; 
			
			const hpp::fcl::CollisionResult & cr = geom_data_.collisionResults[k];
			for (int l(0); l < cr.numContacts(); l++) {
				hpp::fcl::Contact c = cr.getContact(l);

				// computing the jacobian for this contact
				Eigen::MatrixXd fjac0 = Eigen::MatrixXd::Zero(6, model_.nv);
				Eigen::MatrixXd fjac1 = Eigen::MatrixXd::Zero(6, model_.nv);
				pinocchio::computeFrameJacobian(model_, data_, q_, fid0, pinocchio::LOCAL_WORLD_ALIGNED, fjac0);
				pinocchio::computeFrameJacobian(model_, data_, q_, fid1, pinocchio::LOCAL_WORLD_ALIGNED, fjac1);

				Eigen::Vector3d dpos0 = data_.oMf[fid0].translation() - c.pos; // = pos in world - pos of frame
				Eigen::Matrix3d dpos_hat0 = get_skew_from_vector(dpos0);
				
				Eigen::Vector3d dpos1 = data_.oMf[fid1].translation() - c.pos; // = pos in world - pos of frame
				Eigen::Matrix3d dpos_hat1 = get_skew_from_vector(dpos1);
				

				Eigen::MatrixXd res_jac = fjac0.topRows(3) + dpos_hat0 * fjac0.bottomRows(3)
										- fjac1.topRows(3) - dpos_hat1 * fjac1.bottomRows(3);

				Eigen::Quaterniond normal_rot = Eigen::Quaterniond::FromTwoVectors(c.normal, -Eigen::Vector3d::UnitZ());
				res_jac = normal_rot.normalized().toRotationMatrix() * res_jac;
				
				all_jac_.push_back(res_jac);
				all_lamb_.push_back(Eigen::Vector3d::Zero(3));
			}
		}
		
		
		//computes the forces at the contact points for now : PGS with only one iteration
		Eigen::MatrixXd M_inv = M.inverse();
		for (int i(0); i < all_jac_.size(); i++) {
			// calc Mi
			Eigen::MatrixXd Mi_inv = all_jac_[i] * M_inv * all_jac_[i].transpose();
			Eigen::Vector3d ci = Eigen::Vector3d::Zero(3);
			ci = all_jac_[i] * u_ + all_jac_[i] * M_inv * timeStep_ * (-b); // + all other contacts (Ji * M_inv * Jk.T * lamb_k)
			all_lamb_[i] = -Mi_inv.inverse() * ci;
			
			// this line enforces frictionnless contact and lets us check that the jacobian is rotated the right amount
			// all_lamb_[i][0] = 0; all_lamb_[i][1] = 0;
			
		}
		

		// computes the dynamics
		full_tau = -timeStep_ * b;
		
		for (int i(0); i < all_jac_.size(); i++) {
			full_tau += all_jac_[i].transpose() * all_lamb_[i];
		}
		u_ += M.colPivHouseholderQr().solve(full_tau);
		q_ = pinocchio::integrate(model_, q_, u_ * timeStep_);
	}
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
#include "erquy.hpp"

using namespace erquy;


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

	b = pinocchio::rnea(model_, data_, q_, u_, Eigen::VectorXd::Zero(model_.nv)); // computes the torques needed to track zero acceleration. Gravity is already included.
	M = pinocchio::crba(model_, data_, q_); // computes the mass matrix
	
	
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
	std::vector<Eigen::MatrixXd> all_Mi;
	for (int i(0); i < all_jac_.size(); i++) {
		all_Mi.push_back((all_jac_[i] * M_inv * all_jac_[i].transpose()).inverse());
	}
	for (int pgs_step(0); pgs_step < 5; pgs_step++) {
		// std::cout << "All contacts forces :" << pgs_step << std::endl;
		for (int i(0); i < all_jac_.size(); i++) {
			Eigen::VectorXd temp_sum = u_ +  M_inv * timeStep_ * (-b);
			for (int k(0); k < all_jac_.size(); k++) {
				if (k != i) {
					temp_sum += M_inv * all_jac_[k].transpose() * all_lamb_[k];
				}
			}
			Eigen::Vector3d ci = all_jac_[i] * temp_sum;
			all_lamb_[i] = -all_Mi[i] * ci;

			// this line enforces frictionnless contact and lets us check that the jacobian is rotated the right amount
			// all_lamb_[i][0] = 0; all_lamb_[i][1] = 0;
			// std::cout << all_lamb_[i].transpose() << std::endl;
		}
	}
	

	// computes the forward dynamics
	Eigen::VectorXd full_tau = -timeStep_ * b;
	
	for (int i(0); i < all_jac_.size(); i++) {
		full_tau += all_jac_[i].transpose() * all_lamb_[i];
	}
	u_ += M.colPivHouseholderQr().solve(full_tau);
	q_ = pinocchio::integrate(model_, q_, u_ * timeStep_);
}

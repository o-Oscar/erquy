
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/frames-derivatives.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/geometry.hpp"

#include <iostream>

#include "erquy.hpp"

#include <Eigen/Dense>

/*
TODO : 
chercher les points de contact
Si on en trouve un, on s'arrange pour mettre la vitesse à zéro avec une force en plus
Simuler un timestep avec la force calculée précédement
*/

Eigen::Matrix3d get_skew_from_vector(Eigen::Vector3d dpos) {
	Eigen::Matrix3d to_return;
	to_return << 0, -dpos(2), dpos(1),
				dpos(2), 0, -dpos(0),
				-dpos(1), dpos(0), 0;
	return to_return;
}

int main(int argc, char ** argv)
{
	wave();
	// You should change here to set up your own URDF file or just pass it as an argument of this example.
	const std::string urdf_filename = (argc<=1) ? std::string("/example-robot-data/robots/ur_description/urdf/ur5_robot.urdf") : argv[1];

	// Load the urdf model
	Model model;
	model.gravity.linear() << 0., 0., -9.80665;
	// pinocchio::urdf::buildModel(urdf_filename, pinocchio::JointModelFreeFlyer(), model);
	pinocchio::urdf::buildModel(urdf_filename, model);
	std::cout << "model name: " << model.name << std::endl;

	GeometryModel geom_model;
	pinocchio::urdf::buildGeom(model, urdf_filename, pinocchio::COLLISION, geom_model, "");
	geom_model.addAllCollisionPairs();
	GeometryData geom_data(geom_model);
	
	for (auto const& f : model.frames) {
		std::cout << f.name << std::endl;
	}
	
	// Create data required by the algorithms
	Data data(model);


	// Sample a random configuration
	std::cout << "model.nq = " << model.nq << std::endl;

	Eigen::VectorXd q(model.nq);
	q << 0, 0, 1.5, 0, 0, 0, 1;
	Eigen::VectorXd u(model.nv);
	u << 0, 0, 0, 0, 0, 0;



	Eigen::VectorXd a0 = Eigen::VectorXd::Zero(model.nv);
	Eigen::VectorXd v0 = Eigen::VectorXd::Zero(model.nv);
	Eigen::VectorXd b;
	Eigen::MatrixXd M;

	std::cout << b << std::endl;
	std::cout << M << std::endl;

	float_t dt = 0.01;
	
	Eigen::VectorXd full_tau;


	
	// Ma = -b
	for (int i(0); i < 40; i++) {
		std::cout << "Step :" << std::endl;
		
		pinocchio::forwardKinematics(model, data, q, u); // update every positions and velocities
		pinocchio::updateFramePlacements(model, data);

		b = pinocchio::rnea(model, data, q, u, a0); // computes the torques needed to track zero acceleration. Gravity is already included.
		M = pinocchio::crba(model, data, q); // computes the mass matrix
		
		// finding all collisions
		pinocchio::computeCollisions(model, data, geom_model, geom_data, q);

		std::vector<Eigen::MatrixXd> all_jac;
		std::vector<Eigen::Vector3d> all_lamb;
		for (size_t k=0; k < geom_model.collisionPairs.size(); k++) {
			const CollisionPair & cp = geom_model.collisionPairs[k];

			int fid0 = geom_model.geometryObjects[cp.first].parentFrame; 
			int fid1 = geom_model.geometryObjects[cp.second].parentFrame; 
			
			const hpp::fcl::CollisionResult & cr = geom_data.collisionResults[k];
			for (int l(0); l < cr.numContacts(); l++) {
				hpp::fcl::Contact c = cr.getContact(l);

				
				Eigen::MatrixXd fjac0 = Eigen::MatrixXd::Zero(6, model.nv); // Eigen::MatrixXd(6, model.nv);
				Eigen::MatrixXd fjac1 = Eigen::MatrixXd::Zero(6, model.nv); // Eigen::MatrixXd(6, model.nv);
				pinocchio::computeFrameJacobian(model, data, q, fid0, pinocchio::WORLD, fjac0);
				pinocchio::computeFrameJacobian(model, data, q, fid1, pinocchio::WORLD, fjac1);

				Eigen::Vector3d dpos0 = c.pos - data.oMf[fid0].translation(); // = pos in world - pos of frame
				// dpos0 << 1.5, 0, 0;
				Eigen::Matrix3d dpos_hat0 = get_skew_from_vector(dpos0);
				
				Eigen::Vector3d dpos1 = c.pos - data.oMf[fid1].translation(); // = pos in world - pos of frame
				// dpos1 << 1.5, 0, 0;
				Eigen::Matrix3d dpos_hat1 = get_skew_from_vector(dpos1);
				

				Eigen::MatrixXd res_jac = fjac0.topRows(3) + dpos_hat0 * fjac0.bottomRows(3)
										- fjac1.topRows(3) - dpos_hat1 * fjac1.bottomRows(3);
				
				all_jac.push_back(res_jac);
				all_lamb.push_back(Eigen::Vector3d::Zero(3));
			}
		}
		

		//computes the forces at the contact points
		Eigen::MatrixXd M_inv = M.inverse();
		for (int i(0); i < all_jac.size(); i++) {
			// calc Mi
			Eigen::MatrixXd Mi_inv = all_jac[i] * M_inv * all_jac[i].transpose();
			Eigen::Vector3d ci = Eigen::Vector3d::Zero(3);
			ci = all_jac[i] * u + all_jac[i] * M_inv * dt * (-b); // + all other contacts (Ji * M_inv * Jk.T * lamb_k)
			all_lamb[i] = -Mi_inv.inverse() * ci;
		}
		
		// computes the dynamics
		// a = M.colPivHouseholderQr().solve(-b); // + sum (J_i.T * lamb_i)
		full_tau = -dt * b;
		for (int i(0); i < all_jac.size(); i++) {
			full_tau += all_jac[i].transpose() * all_lamb[i];
		}
		u += M.colPivHouseholderQr().solve(full_tau);
		q = pinocchio::integrate(model, q, u * dt);

		std::cout << "ball_pos" << std::endl << data.oMf[4].translation().transpose() << std::endl;
		std::cout << "ball_vel" << std::endl << pinocchio::getFrameVelocity(model, data, 4, pinocchio::ReferenceFrame::LOCAL) << std::endl;
		// std::cout << "u" << std::endl << u.transpose() << std::endl;
		
	}



	/*
	// Perform the forward kinematics over the kinematic tree
	forwardKinematics(model,data,q);
	// Print out the placement of each joint of the kinematic tree
	for(JointIndex joint_id = 0; joint_id < (JointIndex)model.njoints; ++joint_id)
	std::cout << model.names[joint_id] << ": "
				<< "\ntranslation : " << std::setprecision(2)
				<< data.oMi[joint_id].translation().transpose()
				<< "\nrotation : " << data.oMi[joint_id].rotation().transpose()
				<< std::endl;
	*/
}
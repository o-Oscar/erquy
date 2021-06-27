#include "solver.hpp"
#include <iostream>

using namespace erquy;

void PgsSolver::step (	const Eigen::VectorXd & zero_velocity,
				const Eigen::MatrixXd & M_inv,
				const std::vector<Eigen::MatrixXd> & all_Mi,
				const std::vector<Eigen::MatrixXd> & all_jac, 
				std::vector<Eigen::Vector3d> & all_lamb)
{
	for (int i(0); i < all_jac.size(); i++) {
		Eigen::VectorXd temp_sum = zero_velocity; // u_ +  M_inv * timeStep_ * (-b);
		for (int k(0); k < all_jac.size(); k++) {
			if (k != i) {
				temp_sum += M_inv * all_jac[k].transpose() * all_lamb[k];
			}
		}
		Eigen::Vector3d ci = all_jac[i] * temp_sum;
		// all_lamb[i] = -all_Mi[i].inverse() * ci; //  carefull : if the contact is locked in some direction, the inverse does not exist

		// these line enforces frictionnless contact and lets us check that the jacobian is rotated the right amount
		all_lamb[i][2] = -ci(2) / all_Mi[i](2,2);
		all_lamb[i][0] = 0; all_lamb[i][1] = 0;
	}
}

void PgsSolver::solve (const Eigen::VectorXd & zero_velocity,
				const Eigen::MatrixXd & M_inv,
				const std::vector<Eigen::MatrixXd> & all_jac, 
				std::vector<Eigen::Vector3d> & all_lamb)
{
	// std::cout << "Solving... ";
	std::vector<Eigen::MatrixXd> all_Mi;
	for (int i(0); i < all_jac.size(); i++) {
		all_Mi.push_back((all_jac[i] * M_inv * all_jac[i].transpose()));
		// std::cout << (all_jac[i] * M_inv * all_jac[i].transpose()) << std::endl;
	}
	for (int pgs_step(0); pgs_step < 10; pgs_step++) {
		step (zero_velocity, M_inv, all_Mi, all_jac, all_lamb);
	}
	for (int i(0); i < all_jac.size(); i++) {
		all_lamb[i][0] = 0; all_lamb[i][1] = 0;
	}
	/*
	std::cout << "Resulting forces : " << std::endl;
	for (int i(0); i < all_jac.size(); i++) {
		std::cout << all_lamb[i].transpose() << std::endl;
	}*/

}
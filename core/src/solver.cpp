#include "solver.hpp"
#include <iostream>
#include <math.h>

using namespace erquy;

double sgn(double val) {
    return (double(0) < val) - (val < double(0));
}

double calc_lamb_z (	const Eigen::Vector3d & ci,
				const Eigen::Matrix3d & Mi_inv,
				const double & r,
				const double & theta)
{
	return (- ci[2] - Mi_inv(2, 0) * r * cos (theta) - Mi_inv(2, 1) * r * sin (theta)) / Mi_inv(2,2);
}

double calc_r (const Eigen::Vector3d & ci,
				const Eigen::Matrix3d & Mi_inv,
				const double & theta)
{
	return - ci[2] / (Mi_inv(2,2)/mu + Mi_inv(2,0) * cos(theta) + Mi_inv(2,1) * sin(theta));
}

Eigen::Vector3d calc_dh2 (const Eigen::Vector3d & lamb) {
	Eigen::Vector3d dh2;
	dh2 << 2*lamb[0], 2*lamb[1], -2*mu*mu*lamb[2];
	return dh2;
}

Eigen::Vector3d calc_nu_chap (	const Eigen::Vector3d & lamb,
								const Eigen::Matrix3d & Mi_inv)
{
	Eigen::Vector3d dh1, dh2;
	dh1 = Mi_inv.row(2);
	dh2 << 2*lamb[0], 2*lamb[1], -2*mu*mu*lamb[2];
	return dh1.cross(dh2);
}

double calc_dE_dtheta (	const Eigen::Vector3d & ci,
						const Eigen::Matrix3d & Mi_inv,
						const Eigen::Vector3d & lamb)
{
	return (Mi_inv * lamb + ci).transpose() * calc_nu_chap(lamb, Mi_inv);
}


// goal : finding the lambda on the 
void PgsSolver::step_slipping_contact (	const Eigen::Vector3d & ci,
										const Eigen::Matrix3d & Mi_inv,
										Eigen::Vector3d & lamb_out) {
	// initialisation of the bisection
	Eigen::Vector3d lamb_v0 = -Mi_inv.inverse() * ci;
	double theta = atan2 (lamb_v0[1], lamb_v0[0]);
	double r = calc_r(ci, Mi_inv, theta);
	double lamb_z = calc_lamb_z(ci, Mi_inv, r, theta);
	Eigen::Vector3d lamb;
	lamb << r * cos(theta), r * sin(theta), lamb_z;
	double D0 = sgn(calc_dE_dtheta(ci, Mi_inv, lamb));
	double alpha = -0.1 * D0;
	double thetap;
	Eigen::Vector3d lambp;
	// std::cout << "lamb_v0 : " << lamb_v0.transpose() << std::endl;
	// std::cout << "lamb_0 : " << lamb.transpose() << std::endl;
	// std::cout << "delta_lamb_0 : " << (lamb - lamb_v0).transpose() << std::endl;
	// std::cout << "dh2_0 : " << calc_dh2(lamb).transpose() << std::endl;
	// std::cout << "theta_0 : " << theta << std::endl;
	// std::cout << "r_0 : " << r << std::endl;
	// std::cout << "first_check : " << calc_dh2(lamb).transpose() * (lamb - lamb_v0) << std::endl;
	for (int i(0); i < 10; i++) {
		thetap = theta;
		lambp = lamb;
		theta = theta + alpha;
		r = calc_r(ci, Mi_inv, theta);
		lamb_z = calc_lamb_z(ci, Mi_inv, r, theta);
		lamb << r * cos(theta), r * sin(theta), lamb_z;
		if (calc_dh2(lamb).transpose() * (lamb - lamb_v0) > 0 || r < 0) {
			alpha *= 0.5;
			// std::cout << "Bad theta : " << theta << std::endl;
			theta = thetap;
		}else {
			if (calc_dE_dtheta(ci, Mi_inv, lamb) * D0 > 0) {
				alpha *= 1.;
				// std::cout << "Still looking" << std::endl;
			}else {
				// std::cout << "Trying to exit" << std::endl;
				break;
			}
		}
	}

	// actual bisection
	double thetad;
	Eigen::Vector3d lambd;
	// while ((lamb-lambp).squaredNorm() > 1e-4 ) { // gets stuck for some reason even if the theta are very similar
	while (abs(theta - thetap) > 1e-6) {
		thetad = .5 * (theta + thetap);
		r = calc_r(ci, Mi_inv, theta);
		lamb_z = calc_lamb_z(ci, Mi_inv, r, theta);
		lambd << r * cos(theta), r * sin(theta), lamb_z;
		if (calc_dE_dtheta(ci, Mi_inv, lambd) * D0 > 0) {
			thetap = thetad;
			lambp = lambd;
		}else {
			theta = thetad;
			lamb = lambd;
		}
		// std::cout << "squared norm : " << (lamb-lambp).squaredNorm() << std::endl;
		// std::cout << "thetas : " << theta << " : " << thetap << std::endl;
	}
	lamb_out = lamb;
}

double PgsSolver::step (	const Eigen::VectorXd & zero_velocity,
				const Eigen::MatrixXd & M_inv,
				const std::vector<Eigen::MatrixXd> & all_Mi_inv,
				const std::vector<Eigen::MatrixXd> & all_jac, 
				std::vector<Eigen::Vector3d> & all_lamb)
{
	std::vector<int> id_arr;
	for (int i(0); i < all_jac.size(); i++) {
		id_arr.push_back(i);
	}
	// random_shuffle(id_arr.begin(), id_arr.end());

	std::vector<Eigen::Vector3d> new_all_lamb;
	for (int id(0), i; id < all_jac.size(); id++) {
		i = id_arr[id];

		Eigen::VectorXd temp_sum = zero_velocity; // u_ +  M_inv * timeStep_ * (-b);
		for (int k(0); k < all_jac.size(); k++) {
			if (k != i) {
				temp_sum += M_inv * all_jac[k].transpose() * all_lamb[k];
			}
		}
		Eigen::Vector3d ci = all_jac[i] * temp_sum;
		
		Eigen::Vector3d lamb = Eigen::Vector3d::Zero();
		
		// using a small diagonal value to handle the case of degenerate M matrices (not great code, not backed up by any theory)
		Eigen::Matrix3d M_inv = all_Mi_inv[i];
		if (M_inv.determinant() < 1e-12) {
			M_inv += 1e-4 * Eigen::Matrix3d::Identity(3, 3);
		}

		// opening contact
		if (ci(2) > 0) {
			// std::cout << "opening contact" << std::endl;
			// do nothing : the force should be zero
		} else
		// sticking contact
		if (mu * mu * all_lamb[i][2] * all_lamb[i][2] > all_lamb[i][0] * all_lamb[i][0] + all_lamb[i][1] * all_lamb[i][1]) {
			
			// std::cout << "sticking contact" << std::endl;
			lamb = -M_inv.inverse() * ci;
		} else
		// slipping contact
		{
			// std::cout << "slipping contact" << std::endl;
			// std::cout << all_lamb[i].transpose() << std::endl;
			// std::cout << mu * mu * all_lamb[i][2] * all_lamb[i][2] << " : " << all_lamb[i][0] * all_lamb[i][0] + all_lamb[i][1] * all_lamb[i][1] << std::endl;
			// step_slipping_contact(ci, all_Mi_inv[i], lamb);
			step_slipping_contact(ci, M_inv, lamb);
		}
		// lamb = -(all_Mi_inv[i] + 1e-9 * Eigen::Matrix3d::Identity(3, 3)).inverse() * ci;


		// all_lamb[i] = -all_Mi[i].inverse() * ci; //  carefull : if the contact is locked in some direction, the inverse does not exist
		// these line enforces frictionnless contact and lets us check that the jacobian is rotated the right amount
		// lamb[2] = -ci(2) / all_Mi_inv[i](2,2);
		// lamb[0] = 0; lamb[1] = 0;

		new_all_lamb.push_back(lamb);
	}

	// copy of the new lambda
	double alpha = 0.7;
	double residual = 0;
	for (int i(0); i < all_jac.size(); i++) {
		residual += (new_all_lamb[i] - all_lamb[i]).squaredNorm();
		all_lamb[i] += alpha * (new_all_lamb[i] - all_lamb[i]);
	}
	return residual;
}

void PgsSolver::solve (const Eigen::VectorXd & zero_velocity,
				const Eigen::MatrixXd & M_inv,
				const std::vector<Eigen::MatrixXd> & all_jac, 
				std::vector<Eigen::Vector3d> & all_lamb)
{
	
	std::vector<Eigen::MatrixXd> all_Mi;
	for (int i(0); i < all_jac.size(); i++) {
		all_Mi.push_back((all_jac[i] * M_inv * all_jac[i].transpose()));
	}
	
	// TODO : iterate until max number of step or physical violation is very small
	double residual = 42;
	for (int pgs_step(0); pgs_step < 10 && residual > 1e-6; pgs_step++) {
		// std::cout << "step " << pgs_step << std::endl;
		residual = step (zero_velocity, M_inv, all_Mi, all_jac, all_lamb);
		// std::cout << "residual : " << residual << std::endl;
	}


	/*
	for (int i(0); i < all_jac.size(); i++) {
		all_lamb[i][0] = 0; all_lamb[i][1] = 0;
	}*/

	// std::cout << "Resulting forces : " << std::endl;
	// for (int i(0); i < all_jac.size(); i++) {
	// 	std::cout << all_lamb[i].transpose() << std::endl;
	// }

}
#include "solver.hpp"
#include <iostream>
#include <math.h>

using namespace erquy;

void PgsSolver::set_solver_params (double betha1, double betha2, double betha3) {
	betha1_ = betha1;
	betha2_ = betha2;
	betha3_ = betha3;
}

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
				const double & theta,
				real & mu)
{
	return - ci[2] / (Mi_inv(2,2)/mu + Mi_inv(2,0) * cos(theta) + Mi_inv(2,1) * sin(theta));
}

Eigen::Vector3d calc_dh2 (const Eigen::Vector3d & lamb, real & mu) {
	Eigen::Vector3d dh2;
	dh2 << 2*lamb[0], 2*lamb[1], -2*mu*mu*lamb[2];
	return dh2;
}

Eigen::Vector3d calc_nu_chap (	const Eigen::Vector3d & lamb,
								const Eigen::Matrix3d & Mi_inv,
								real & mu)
{
	Eigen::Vector3d dh1, dh2;
	dh1 = Mi_inv.row(2);
	dh2 << 2*lamb[0], 2*lamb[1], -2*mu*mu*lamb[2];
	return dh1.cross(dh2);
}

double calc_dE_dtheta (	const Eigen::Vector3d & ci,
						const Eigen::Matrix3d & Mi_inv,
						const Eigen::Vector3d & lamb,
						real & mu)
{
	return (Mi_inv * lamb + ci).transpose() * calc_nu_chap(lamb, Mi_inv, mu);
}


void PgsSolver::step_slipping_contact (	const Eigen::Vector3d & ci,
										const Eigen::Matrix3d & Mi_inv,
										Eigen::Vector3d & lamb_out,
										real & mu) {
	// initialisation of the bisection
	Eigen::Vector3d lamb_v0 = -Mi_inv.inverse() * ci;
	double theta = atan2 (lamb_v0[1], lamb_v0[0]);
	double r = calc_r(ci, Mi_inv, theta, mu);
	double lamb_z = calc_lamb_z(ci, Mi_inv, r, theta);
	Eigen::Vector3d lamb;
	lamb << r * cos(theta), r * sin(theta), lamb_z;
	double D0 = sgn(calc_dE_dtheta(ci, Mi_inv, lamb, mu));
	double alpha = -betha1_ * D0;
	double thetap;
	Eigen::Vector3d lambp;
	int i;
	for (i = 0; i < 300; i++) {
		thetap = theta;
		lambp = lamb;
		theta = theta + alpha;
		r = calc_r(ci, Mi_inv, theta, mu);
		lamb_z = calc_lamb_z(ci, Mi_inv, r, theta);
		lamb << r * cos(theta), r * sin(theta), lamb_z;
		if (calc_dh2(lamb, mu).transpose() * (lamb - lamb_v0) > 0 || r < 0) {
			// std::cout << "Bad theta : " << theta << std::endl;
			alpha *= betha2_;
			theta = thetap;
		}else {
			if (calc_dE_dtheta(ci, Mi_inv, lamb, mu) * D0 > 0) {
				// std::cout << "Still looking" << std::endl;
				alpha *= betha3_;
			}else {
				// std::cout << "Trying to exit " << i << std::endl;
				break;
			}
		}
	}
	// std::cout << "exited : " << i << std::endl;

	// actual bisection
	double thetad;
	Eigen::Vector3d lambd;
	// while ((lamb-lambp).squaredNorm() > 1e-4 ) { // gets stuck for some reason even if the theta are very similar
	i = 0;
	while (abs(theta - thetap) > 1e-6) { // 1e-6
		thetad = .5 * (theta + thetap);
		r = calc_r(ci, Mi_inv, theta, mu);
		lamb_z = calc_lamb_z(ci, Mi_inv, r, theta);
		lambd << r * cos(theta), r * sin(theta), lamb_z;
		if (calc_dE_dtheta(ci, Mi_inv, lambd, mu) * D0 > 0) {
			thetap = thetad;
			lambp = lambd;
		}else {
			theta = thetad;
			lamb = lambd;
		}
		i+=1;
	}
	// std::cout << "exited : " << i << std::endl;
	lamb_out = lamb;
}

void PgsSolver::step_sticking_contact (const Eigen::Vector3d & ci,
										const Eigen::Matrix3d & Mi_inv,
										Eigen::Vector3d & lamb_out) {
	lamb_out = -Mi_inv.inverse() * ci;
}

double PgsSolver::step (	const int & n_contact,
							const Eigen::MatrixXd & D,
							const Eigen::VectorXd & c,
							const std::vector<Eigen::Matrix3d> & all_Mi_inv,
							const std::vector<Eigen::Matrix3d> & all_Mi,
							Eigen::VectorXd & full_lamb,
							const std::vector<real> & all_mu)
{
	// shuffling the order in which the forces are processed to improve stability
	std::vector<int> id_arr;
	for (int i(0); i < n_contact; i++) {
		id_arr.push_back(i);
	}
	random_shuffle(id_arr.begin(), id_arr.end());

	double residual = 0;
	for (int id(0), i; id < n_contact; id++) {
		i = id_arr[id];

		Eigen::Vector3d ci = c.segment<3>(i*3) + D.block(3*i, 0, 3, 3*n_contact) * full_lamb;
		Eigen::Vector3d lamb_v0 = -all_Mi[i] * ci;
		Eigen::Vector3d lamb = Eigen::Vector3d::Zero();
		
		real mu = all_mu[i];

		if (ci(2) > 0) { // opening contact
			// nothing.
		} else if (mu * mu * lamb_v0[2] * lamb_v0[2] > lamb_v0[0] * lamb_v0[0] + lamb_v0[1] * lamb_v0[1]) {
			lamb = lamb_v0;
		} else // slipping contact
		{
			step_slipping_contact(ci, all_Mi_inv[i], lamb, mu);
		}

		residual += (lamb - full_lamb.segment<3>(3*i)).squaredNorm();
		full_lamb.segment<3>(3*i) += alpha_ * (lamb - full_lamb.segment<3>(3*i));
	}

	return residual;
}

void PgsSolver::solve (const Eigen::VectorXd & zero_velocity,
				const Eigen::MatrixXd & M_inv,
				int n_contact,
				const Eigen::MatrixXd & full_jac,
				Eigen::VectorXd & full_lamb,
				const std::vector<real> & all_mu)
{
	// calculer la matrice de delassus approche na??ve
	Eigen::MatrixXd D = full_jac * M_inv * full_jac.transpose();
	// calculer la matrice de delassus, probablement mieux mais en vrai on s'en fout pk M_inv doit ??tre calcul?? dans tous les cas.
	// Eigen::MatrixXd D = full_jac * M.colPivHouseholderQr().solve(full_jac.transpose());

	// calculer la vitesse sans force dans toutes les jacobiennes
	Eigen::VectorXd c = full_jac * zero_velocity;

	// calculer les inverses des matrices de masse apparentes aux contacts
	std::vector<Eigen::Matrix3d> all_Mi;
	std::vector<Eigen::Matrix3d> all_Mi_inv;
	for (int i(0); i < n_contact; i++) {
		all_Mi_inv.push_back(D.block<3,3>(i*3, i*3));
		if (D.block<3,3>(i*3, i*3).determinant() < 1e-12) {
			all_Mi.push_back((all_Mi_inv[i] + 1e-4 * Eigen::Matrix3d::Identity(3, 3)).inverse());
		} else {
			all_Mi.push_back(all_Mi_inv[i].inverse());
		}
		D.block<3,3>(i*3, i*3) = Eigen::Matrix3d::Zero();
	}

	residual_ = 42;
	alpha_ = 1;
	for (pgs_step_ = 0; pgs_step_ < 100 && residual_ > 1e-6; pgs_step_++) {
		residual_  = step(n_contact, D, c, all_Mi_inv, all_Mi, full_lamb, all_mu);
		if (alpha_ > 0.7)
			alpha_ *= 0.99;
	}
	// std::cout << "pgs_step : " << pgs_step_ << std::endl;
}
#include <Eigen/Core>
#include <Eigen/Geometry> 

#include <vector>
#include <bits/stdc++.h>


namespace erquy {

	typedef double real;


	class PgsSolver {
		public:
			void solve (const Eigen::VectorXd & zero_velocity,
						const Eigen::MatrixXd & M_inv,
						int n_contact,
						const Eigen::MatrixXd & full_jac, 
						Eigen::VectorXd & full_lamb,
						const std::vector<real> & all_mu);

			double step (	const int & n_contact,
							const Eigen::MatrixXd & D,
							const Eigen::VectorXd & c,
							const std::vector<Eigen::Matrix3d> & all_Mi_inv,
							const std::vector<Eigen::Matrix3d> & all_Mi,
							Eigen::VectorXd & full_lamb,
							const std::vector<real> & all_mu);

			void step_slipping_contact (const Eigen::Vector3d & ci,
										const Eigen::Matrix3d & Mi_inv,
										Eigen::Vector3d & lamb,
										real & mu);
			void step_sticking_contact (const Eigen::Vector3d & ci,
										const Eigen::Matrix3d & Mi_inv,
										Eigen::Vector3d & lamb_out);
			
			void set_solver_params (double betha1, double betha2, double betha3);


		private:
			double alpha_ = 1.;
			double betha1_ = 0.3;
			double betha2_ = 0.6;
			double betha3_ = 1.;

			double pgs_step_;
			double residual_;

			std::vector<Eigen::MatrixXd> all_Mi_inv;
			std::vector<Eigen::MatrixXd> all_JiM_inv;
			std::vector<Eigen::MatrixXd> allJkT_lamb;
	};

}
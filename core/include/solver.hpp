
#include <Eigen/Core>
#include <Eigen/Geometry> 

#include <vector>
#include <bits/stdc++.h>

#define mu 0.5

namespace erquy {


	class PgsSolver {
		public:
		static void solve (	const Eigen::VectorXd & zero_velocity,
							const Eigen::MatrixXd & M_inv,
							const std::vector<Eigen::MatrixXd> & all_jac, 
							std::vector<Eigen::Vector3d> & all_lamb);

		static double step (	const Eigen::VectorXd & zero_velocity,
								const Eigen::MatrixXd & M_inv,
								const std::vector<Eigen::MatrixXd> & all_Mi,
								const std::vector<Eigen::MatrixXd> & all_jac, 
								std::vector<Eigen::Vector3d> & all_lamb);

		static void step_slipping_contact (	const Eigen::Vector3d & ci,
											const Eigen::Matrix3d & Mi_inv,
											Eigen::Vector3d & lamb);
	};

}
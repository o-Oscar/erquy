
#include <Eigen/Core>
#include <Eigen/Geometry> 

#include <vector>

namespace erquy {


	class PgsSolver {
		public:
		static void solve (	const Eigen::VectorXd & zero_velocity,
							const Eigen::MatrixXd & M_inv,
							const std::vector<Eigen::MatrixXd> & all_jac, 
							std::vector<Eigen::Vector3d> & all_lamb);

		static void step (	const Eigen::VectorXd & zero_velocity,
							const Eigen::MatrixXd & M_inv,
							const std::vector<Eigen::MatrixXd> & all_Mi,
							const std::vector<Eigen::MatrixXd> & all_jac, 
							std::vector<Eigen::Vector3d> & all_lamb);
	};

}
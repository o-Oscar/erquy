#include "pinocchio/fwd.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/frames-derivatives.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/geometry.hpp"
#include "pinocchio/algorithm/aba.hpp"


#include "pinocchio/multibody/fcl.hpp"

#include <Eigen/Geometry> 

#include <boost/python.hpp>

#include <iostream>

#include "solver.hpp"

namespace erquy {

	typedef double real;

	class World {
		public:	
			World ();
			void loadUrdf(std::string urdf_path, std::string meshes_path);

			// the almighty integrate step
			void integrate();

			// --- getters and setters ---
			int getGeneralizedCoordinateDim ();
			int getGeneralizedVelocityDim ();

			void setGeneralizedCoordinate (Eigen::VectorXd q);
			Eigen::VectorXd getGeneralizedCoordinate ();

			void setGeneralizedVelocity (Eigen::VectorXd u);
			Eigen::VectorXd getGeneralizedVelocity ();

			void setState (Eigen::VectorXd q, Eigen::VectorXd u);
			boost::python::tuple getState ();

			void setTimeStep (real timeStep);
			real getTimeStep();

			void setERP (real erp);
			real getERP ();

			void setGravity (Eigen::Vector3d gravity);
			Eigen::Vector3d getGravity ();

			void enablePd (bool enable_pd);
			void setPdGains (Eigen::VectorXd kp, Eigen::VectorXd kd);
			void setPdTarget (Eigen::VectorXd q_targ, Eigen::VectorXd u_targ);
			Eigen::MatrixXd getPdForce ();

			void setGeneralizedTorque (Eigen::VectorXd tau);
			void setMaxTorque (Eigen::VectorXd tau_max);

			boost::python::tuple getContactInfos ();
			Eigen::MatrixXd computeDistances ();
			void setMaterialPairProp (int first_idx, int second_idx, real mu);

			std::vector<std::string>::iterator getJointNamesBegin ();
			std::vector<std::string>::iterator getJointNamesEnd ();
			int getJointIdxByName (std::string name);
		
			std::vector<std::string>::iterator getFrameNamesBegin ();
			std::vector<std::string>::iterator getFrameNamesEnd ();
			int getFrameIdxByName (std::string name);
			Eigen::Vector3d getFramePosition (int idx);
			Eigen::Matrix3d getFrameOrientation (int idx);
			Eigen::Vector3d getFrameVelocity (int idx);
			Eigen::Vector3d getFrameAngularVelocity (int idx);
			
			// std::vector<Eigen::MatrixXd>::iterator getLambB ();
			// std::vector<Eigen::MatrixXd>::iterator getLambE ();

			PgsSolver solver;

		private:

			pinocchio::Model model_;
			pinocchio::Data data_;

			pinocchio::GeometryModel geom_model_;
			pinocchio::GeometryData geom_data_;

			bool enable_pd_ = false;
			Eigen::VectorXd q_;
			Eigen::VectorXd u_;
			Eigen::VectorXd kp_;
			Eigen::VectorXd kd_;
			Eigen::VectorXd cur_kp_;
			Eigen::VectorXd cur_kd_;

			Eigen::VectorXd tau_;
			Eigen::VectorXd tau_max_;
			Eigen::VectorXd cur_pd_tau_;

			Eigen::VectorXd q_targ_;
			Eigen::VectorXd u_targ_;

			// contact data
			std::map<std::pair<int, int>, real> material_pair_props_;
			std::pair<int, int> default_contact_pair_ = std::make_pair(-1, -1);
			std::vector<real> all_mu_;

			int n_contact_ = 0;
			Eigen::MatrixXi contact_joint_id_;
			Eigen::MatrixXd full_jac_;
			Eigen::VectorXd full_lamb_;

			real timeStep_ = 0.01;
			real erp_ = 0.003; // Error reduction parameter : time constant for resolving penetration
			Eigen::Vector3d gravity_;

			// --- step realted ---
			Eigen::VectorXd h_;
			Eigen::VectorXd ag_;
			Eigen::VectorXd dq_;
			Eigen::VectorXd tau_star_;
			Eigen::MatrixXd M_bar_;
			Eigen::MatrixXd M_bar_inv_;
		
			std::vector<Eigen::MatrixXd> all_jac_;
			std::vector<Eigen::Vector3d> all_lamb_;

			// --- misc data ---
			std::vector<std::string> frame_names_;

	};
}
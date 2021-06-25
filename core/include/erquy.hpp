#include "pinocchio/fwd.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/frames-derivatives.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/geometry.hpp"

#include "pinocchio/multibody/fcl.hpp"

#include <Eigen/Geometry> 

#include <boost/python.hpp>

#include <iostream>

namespace erquy {

	typedef double real;

	class World {
		public:	
			World ();
			void loadUrdf(std::string urdf_path, std::string meshes_path);

			// the almighty integrate step
			void integrate();

			// --- getters and setters ---
			void setState (Eigen::VectorXd q, Eigen::VectorXd u);
			boost::python::tuple getState ();

			void setTimeStep (real timeStep);
			real getTimeStep();

			void setERP (real erp);
			real getERP ();

			void setGravity (Eigen::Vector3d gravity);
			Eigen::Vector3d getGravity ();

			boost::python::tuple getStepInfo ();

		private:

			pinocchio::Model model_;
			pinocchio::Data data_;

			pinocchio::GeometryModel geom_model_;
			pinocchio::GeometryData geom_data_;

			Eigen::VectorXd q_;
			Eigen::VectorXd u_;

			real timeStep_ = 0.01;
			real erp_ = 0.003; // Error reduction parameter : time constant for resolving penetration

			std::vector<Eigen::MatrixXd> all_jac_;
			std::vector<Eigen::Vector3d> all_lamb_;
		
	};
}
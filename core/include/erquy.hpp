#include "pinocchio/fwd.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/frames-derivatives.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/geometry.hpp"

#include <iostream>

namespace erquy {
	class World {
		public:	
			World ();
			void loadUrdf(std::string urdf_path, std::string meshes_path);
			// void setState (Eigen::VectorXd q, Eigen::VectorXd u);
			void integrate();
		private:

			pinocchio::Model model_;
			// pinocchio::Data data_;

			// pinocchio::GeometryModel geom_model_;
			// pinocchio::GeometryData geom_data_;

			Eigen::VectorXd q_;
			Eigen::VectorXd u_;

	};
}
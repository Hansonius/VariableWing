/**
* \brief Class to define everything goes on inside the wingbox
* @author Josh Hansen
*/

#ifndef WINGBOX
#define WINGBOX

#include <vector>
#include "linear_spring.h"
#include "dashpot.h"
#include "torsion_spring.h"
#include "geometric_math.h"

class wingbox
{
	public:

		wingbox();

		void SetLinearSprings(std::vector<linear_spring> linear_springs);
		linear_spring* GetLinearSpring(int linear_spring_index);

		void SetDashpots(std::vector<dashpot> dashpots);
		dashpot* GetDashpot(int dashpot_index);

		void SetTorsionSprings(std::vector<torsion_spring> torsion_springs);
		torsion_spring* GetTorsionSpring(int torsion_spring_index);


		// Set where the spring is attach to on the wall
		void SetLinearSpringWallPosition(std::vector<double> position, int linear_spring_index);
		std::vector<double> GetLinearSpringWallPosition(int linear_spring_index);

		void SetDashpotPosition(std::vector<double> position, int dashpot_index);
		std::vector<double> GetDashpotPosition(int dashpot_index);

		void SetTorsionSpringPosition(std::vector<double> position, int torsion_spring_index);
		std::vector<double> GetTorsionSpringPosition(int torsion_spring_index);

		// Set the direction in which the axis of the spring points
		void SetLinearSpringOrientation(std::vector<double> orientation, int linear_spring_index);
		std::vector<double> GetLinearSpringOrientation(int linear_spring_index);

		// Set the direction in which the axis of the dashpot points
		void SetDashpotOrientation(std::vector<double> orientation, int dashpot_index);
		std::vector<double> GetDashpotOrientation(int dashpot_index);

		// Set the axis of which the moment of the torsion spring act on
		void SetTorsionSpringOrientation(std::vector<double> orientation, int torsion_spring_index);
		std::vector<double> GetTorsionSpringOrientation(int torsion_spring_index);
		

		void SetMomentPoint(std::vector<double> rotation_point);

		// overkill??
		void SetRotationAxis(std::vector<double> rotation_axis);
		std::vector<double> GetRotationAxis();
		
		std::vector<double> GetTotalMoment();
		std::vector<std::vector<double>> GetMomentOfInertia();


		void AdvanceTime(double timestep, double delta_phi);
		
		bool CheckFailure();

		void Setup();

		// Variable to check the state of failure in the system
		bool fail_check;


	private:
		std::vector<linear_spring> m_lin_springs;
		std::vector<dashpot> m_dashpots;
		std::vector<torsion_spring> m_tor_springs;

		std::vector<std::vector<double>> m_ls_wall_positions;
		std::vector<std::vector<double>> m_ls_wing_positions;

		std::vector<std::vector<double>> m_dp_positions;
		std::vector<std::vector<double>> m_ts_positions;

		std::vector<std::vector<double>> m_ls_axes;
		std::vector<std::vector<double>> m_dp_axes;
		std::vector<std::vector<double>> m_ts_axes;



		/*
		std::vector<double> m_ls_ext_forces;
		std::vector<double> m_ls_velocities;

		std::vector<double> m_dp_ext_forces;

		std::vector<double> m_ts_angular_velocities;
		std::vector<double> m_ts_ext_moments;
		*/


		std::vector<double> m_rotation_point;
		std::vector<double> m_rotation_axis;

		int m_ls_count;
		int m_dp_count;
		int m_ts_count;


		double m_phi;


		std::vector<std::vector<double>> m_moment_of_inertia;
		std::vector<double> m_wb_total_moment;

		geometric_math math;

		/*
		void UpdateExternalForces();
		void UpdateExternalMoments();
		*/

		void UpdateLinearSpring(int spring_index, double dt);
		void UpdateDashpot(int dashpot_index, double dt, double delta_phi);
		void UpdateTorsionSpring(int spring_index, double dt);
		


		void UpdateMomentOfInertia();
		void UpdateWingboxMoment();
};

#endif

/**
* \brief Class to define a variable wing as a combination of a lifting line and a wingbox controls object
* @author Josh Hansen
*/

#ifndef VARIABLE_WING
#define VARIABLE_WING

#include "wingbox.h"
#include "lifting_line.h"
#include <vector>

class variable_wing
{
	public:
		variable_wing();
		void SetLiftingLine(lifting_line LL);
		void SetWingbox(wingbox wb);

		void SetInitialRootChordAngle(double rc_angle);
		double GetInitialRootChordAngle();
		double GetRootChordAngle();

		void SetMomentPoint(std::vector<double> moment_point);

		void SetFuselagePlane(std::vector<double> plane_normal, std::vector<double> plane_point);

		void SetFreestreamVelocity(std::vector<double> freestream);


		std::vector<double> GetPhi();
		std::vector<double> GetPhiDot();
		std::vector<double> GetPhi2Dot();

		int GetActiveAirstationCount();

		void Setup();
		void AdvanceTime(double timestep);
		void Solve();

		bool fail_check;

	private:
		geometric_math math;
		lifting_line m_LL;
		wingbox m_wb;


		double m_init_rc_angle;
		double m_delta_phi;

		std::vector<double> m_phi;
		std::vector<double> m_phi_dot;
		std::vector<double> m_phi_2_dot;


		std::vector<double> m_fuselage_plane;
		std::vector<bool> m_as_activation; // vector to keep track of whether or not the airstations should be turned on or off

		std::vector<double> m_moment_point;
		std::vector<double> m_net_moment;
		std::vector<std::vector<double>> m_total_moment_of_inertia;


		
		std::vector<double> m_freestream;



		void UpdateRCAngle(double dt);
		void UpdateAirstations();
		void CheckAirstationActivation();
		void UpdateAirstationActivation();


		void CalcVelocities();


};



#endif

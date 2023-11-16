/**
* \brief Class to define dashpots
* @author Josh Hansen
*/

#ifndef DASHPOT
#define DASHPOT

class dashpot
{
	public:
		dashpot();
		void SetMass(double mass);
		void SetDampingCoeff(double damping_coeff);
		void SetVelocity(double velocity);
		void SetForce(double force);
		void SetMaxForce(double max_force);

		double GetMass();
		double GetDampingCoeff();
		double GetVelocity();
		double GetForce();
		double GetMaxForce();
		
		bool CheckFailure();

		void Setup();

	private:
		double m_mass;
		double m_damping_coeff;
		double m_velocity;
		double m_force;
		double m_max_force;

		bool m_used_set_force;
};

#endif
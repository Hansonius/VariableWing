/**
* \brief Class to make linear springs
* @author Josh Hansen
*/

#include "material.h"

#ifndef LINEAR_SPRING
#define LINEAR_SPRING

struct material_properties
{
	double shear_modulus;
	double density;
};

class linear_spring
{
	public:
		linear_spring();

		void SetWireDiameter(double wire_diameter);
		void SetCoilDiameter(double coil_diameter);
		void SetRelaxedPitch(double pitch);
		void SetTotalCoilCount(int total_coil_count);
		void SetActiveCoilCount(int active_coil_count);
		void SetMaterial(material spring_material);

		void SetMass(double spring_mass);
		void SetSpringConstant(double spring_constant);
		void SetForce(double force);
		void SetDisplacement(double displacement);
		double GetDisplacement();

		void SetMinimumLength(double minimum_length);
		void SetMaximumLength(double maximum_length);

		void SetUnstretchedLength(double unstretched_length);
		double GetUnstretchedLength();

		double GetLength();

		double GetMass();
		double GetSpringConstant();
		double GetForce();
		double GetSpringEnergy();

		bool CheckFailure();

		void Setup();

	private:
		double m_displacement;
		double m_wire_diameter;
		double m_coil_diameter;
		double m_relaxed_pitch;
		int m_total_coil_count;
		int m_active_coil_count;
		material m_material;
		double m_mass;
		double m_spring_constant;
		double m_force;
		double m_spring_energy;
		double m_wire_length;

		double m_minimum_length;
		double m_maximum_length;

		double m_unstretched_length;
		double m_length;

		void CalcMass();
		void CalcSpringConstant();
		void CalcMinimumLength();
		void CalcMaximumLength();
		void CalcUnstretchedLength();

};
#endif
/**
* \brief Class to make torsion springs
* @author Josh Hansen
*/

#ifndef TORSIONSPRING_H
#define TORSIONSPRING_H

#include "material.h"

class torsion_spring
{
	public:
		torsion_spring();
		void SetWireDiameter(double wire_diameter);
		void SetCoilDiameter(double coil_diameter);
		void SetTotalCoilCount(int total_coil_count);
		void SetActiveCoilCount(int active_coil_count);
		void SetMaterial(material mat);

		void SetMass(double spring_mass);
		void SetSpringConstant(double spring_constant);
		void SetDisplacement(double displacement);
		
		void SetRelaxedArmAngle(double relaxed_angle);
		double GetRelaxedArmAngle();


		void SetMaxAngle(double max_angle);
		void SetMinAngle(double min_angle);
		double GetMaxAngle();
		double GetMinAngle();

		double GetMass();
		double GetSpringConstant();
		double GetDisplacement();

		double GetMoment();
		double GetSpringEnergy();


		void SetArm1Length(double arm_1_len);
		void SetArm2Length(double arm_2_len);
		double GetArm1Length();
		double GetArm2Length();

		double GetStress();

		void SetMomentOfInertia(double moment_of_interia);
		double GetMomentOfInertia();

		bool CheckFailure();


		void Setup();



	private:
		double m_displacement;
		double m_wire_diameter;
		double m_coil_diameter;
		int m_total_coil_count;
		int m_active_coil_count;
		material m_material;
		double m_mass;
		double m_spring_constant;
		double m_moment;
		double m_spring_energy;
		double m_wire_length;
		
		double m_relaxed_angle;
		double m_min_angle;
		double m_max_angle;

		double m_arm1_len;
		double m_arm2_len;

		double m_moment_of_inertia;

		// Bending correction factor
		double m_kb;

		// Spring Index
		double m_c; 

		double m_stress;

		void CalcMass();
		void CalcSpringConstant();
		void CalcWireLength();
		void CalcMomentOfInertia();


		// Save these for later
		//void CalcMinAngle();
		//void CalcMaxAngle();

		void CalcSpringIndex();
		void CalcStressCorrection();
};
#endif
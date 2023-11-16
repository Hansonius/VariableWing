#include "torsion_spring.h"
#include "material.h"
#include <cmath>

#include <iostream>

torsion_spring::torsion_spring()
{
	m_mass = -1.0;
	m_wire_length = -1.0;
	m_spring_constant = -1.0;
	m_kb = -1.0;
	m_c = -1.0;
	m_arm1_len = 0.0;
	m_arm2_len = 0.0;
	m_displacement = 0.0;
	m_moment_of_inertia = -1.0;

	m_active_coil_count = 0;
	m_total_coil_count = 0;
	m_coil_diameter = 0.0;
	m_wire_diameter = 0.0;

	m_arm1_len = 0.0;
	m_arm2_len = 0.0;

	m_max_angle = INFINITY;
	m_min_angle = -INFINITY;

	m_relaxed_angle = 0.0;
}


void torsion_spring::SetWireDiameter(double wire_diameter)
{
	m_wire_diameter = wire_diameter;
}

void torsion_spring::SetCoilDiameter(double coil_diameter)
{
	m_coil_diameter = coil_diameter;
}

void torsion_spring::SetTotalCoilCount(int total_coil_count)
{
	m_total_coil_count = total_coil_count;
}

void torsion_spring::SetActiveCoilCount(int active_coil_count)
{
	m_active_coil_count = active_coil_count;
}

void torsion_spring::SetMaterial(material spring_material)
{
	m_material = spring_material;
}

void torsion_spring::SetMass(double mass)
{
	m_mass = mass;
}

void torsion_spring::SetSpringConstant(double spring_constant)
{
	m_spring_constant = spring_constant;
}

void torsion_spring::SetDisplacement(double displacement)
{
	m_displacement = displacement;
}

void torsion_spring::SetRelaxedArmAngle(double relaxed_angle)
{
	m_relaxed_angle = relaxed_angle;
}

double torsion_spring::GetRelaxedArmAngle()
{
	return m_relaxed_angle;
}

void torsion_spring::SetMaxAngle(double max_angle)
{
	m_max_angle = max_angle;
}

void torsion_spring::SetMinAngle(double min_angle)
{
	m_min_angle = min_angle;
}

double torsion_spring::GetMaxAngle()
{
	return m_max_angle;
}

double torsion_spring::GetMinAngle()
{
	return m_min_angle;
}

double torsion_spring::GetMass()
{
	if (m_mass < 0)
	{
		CalcMass();
	}
	return m_mass;
}

void torsion_spring::CalcMass()
{
	if (m_wire_length < 0.0)
	{
		CalcWireLength();
	}

	m_mass = m_material.GetDensity() * m_wire_length;
}

void torsion_spring::CalcWireLength()
{
	// Arm lengths + coil length
	m_wire_length = m_arm1_len + m_arm2_len + acos(-1) * m_wire_diameter * m_total_coil_count;
}

double torsion_spring::GetSpringConstant()
{
	if (m_spring_constant < 0.0)
	{
		CalcSpringConstant();
	}

	return m_spring_constant;
}

void torsion_spring::CalcSpringConstant()
{
	// k_T = E * pi * d^4 / (64 * pi * D * N + 1/3 * (a1 + a2))
	m_spring_constant = (m_material.GetYoungsModulus() * acos(-1) * pow(m_wire_diameter, 4)) / (64 * (acos(-1) * m_coil_diameter * double(m_active_coil_count) + (m_arm1_len + m_arm2_len) / 3.0));
}

double torsion_spring::GetDisplacement()
{
	return m_displacement;
}

double torsion_spring::GetMoment()
{
	m_moment = GetSpringConstant() * m_displacement;
	return m_moment;
}

double torsion_spring::GetSpringEnergy()
{
	m_spring_energy = 0.5 * GetSpringConstant() * pow(m_displacement, 2);
	return m_spring_energy;
}

void torsion_spring::SetArm1Length(double arm_1_len)
{
	m_arm1_len = arm_1_len;
}

void torsion_spring::SetArm2Length(double arm_2_len)
{
	m_arm2_len = arm_2_len;
}

double torsion_spring::GetArm1Length()
{
	return m_arm1_len;
}

double torsion_spring::GetArm2Length()
{
	return m_arm2_len;
}

double torsion_spring::GetStress()
{
	if (m_kb < 0.0)
	{
		CalcStressCorrection();
	}

	m_stress = m_kb * 32.0 / (acos(-1) * pow(m_wire_diameter, 3));
	return m_stress;
}


void torsion_spring::SetMomentOfInertia(double moment_of_inertia)
{
	m_moment_of_inertia = moment_of_inertia;
}


double torsion_spring::GetMomentOfInertia()
{
	return m_moment_of_inertia;
}








void torsion_spring::CalcStressCorrection()
{
	if (m_c < 0.0)
	{
		CalcSpringIndex();
	}

	// kb = (4c^2 - c- 1) / (4c * (c-1))
	m_kb = (4.0 * pow(m_c, 2) - m_c - 1.0) / (4.0 * m_c * (m_c - 1.0));
}

void torsion_spring::CalcSpringIndex()
{
	m_c = m_coil_diameter / m_wire_diameter;
}

void torsion_spring::CalcMomentOfInertia()
{
	// Calculate the moment of inertia for the spring as if it were a hollow cylinder
	
	// Inner diameter is the coil diameter - the wire diameter, and outer diameter is the coil diameter + the wire diameter
	double id = m_coil_diameter - m_wire_diameter;
	double od = m_coil_diameter + m_wire_diameter;

	// Moment of inertia for a hollow cylinder is 1/2 * M * (inner_rad^2 + outer_rad^2) = 1/8 * M * (inner_diam^2 + outer_diam^2)
	m_moment_of_inertia = 1.0 / 8.0 * m_mass * (pow(id, 2) + pow(od, 2));
}




bool torsion_spring::CheckFailure()
{
	if (GetStress() > m_material.GetYieldStrength())
	{
		return true;
	}

	if (m_relaxed_angle + m_displacement < GetMinAngle())
	{
		return true;
	}

	if (m_relaxed_angle + m_displacement > GetMaxAngle())
	{
		return true;
	}

	return false;
}



void torsion_spring::Setup()
{
	if (m_mass < 0)
	{
		CalcMass();
	}

	if (m_spring_constant <= 0.0)
	{
		CalcSpringConstant();
	}

	if (m_wire_length <= 0.0)
	{
		CalcWireLength();
	}

	if (m_kb <= 0.0)
	{
		CalcStressCorrection();
	}

	if (m_moment_of_inertia <= 0.0)
	{
		CalcMomentOfInertia();
	}
}
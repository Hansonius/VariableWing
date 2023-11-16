#include "linear_spring.h"
#include "material.h"
#include <iostream>

using namespace std;

linear_spring::linear_spring()
{
	m_displacement = 0.0;
	m_mass = -1.0;
	m_spring_constant = -1.0;
	m_minimum_length = -1.0;
	m_maximum_length = -1.0;
	m_unstretched_length = -1.0;
}

void linear_spring::SetWireDiameter(double wire_diameter)
{
	m_wire_diameter = wire_diameter;
}

void linear_spring::SetCoilDiameter(double coil_diameter)
{
	m_coil_diameter = coil_diameter;
}

void linear_spring::SetRelaxedPitch(double relaxed_pitch)
{
	m_relaxed_pitch = relaxed_pitch;
}

void linear_spring::SetTotalCoilCount(int total_coil_count)
{
	m_total_coil_count = total_coil_count;
}

void linear_spring::SetActiveCoilCount(int active_coil_count)
{
	m_active_coil_count = active_coil_count;
}

void linear_spring::SetMaterial(material spring_material)
{
	m_material = spring_material;
}

void linear_spring::SetForce(double force)
{
	m_force = force;
}

void linear_spring::SetDisplacement(double displacement)
{
	m_displacement = displacement;
}

double linear_spring::GetDisplacement()
{
	return m_displacement;
}

void linear_spring::SetMass(double mass)
{
	if (mass < 0)
	{
		cout << endl << "Unacceptable value for mass. Please provide a positive value." << endl;
	}
	m_mass = mass;
}

double linear_spring::GetMass()
{
	return m_mass;
}

// Calculates the mass of the spring
void linear_spring::CalcMass()
{
	double density = m_material.GetDensity();
	m_wire_length = m_total_coil_count * sqrt(pow(acos(-1) * m_coil_diameter, 2) + pow(m_relaxed_pitch, 2));
	m_mass = density * m_wire_length * (acos(-1) / 4.0) * pow(m_wire_diameter, 2);
}

void linear_spring::SetSpringConstant(double spring_constant)
{
	if (spring_constant < 0)
	{
		cout << endl << "Unacceptable value for spring constant. Please provide a positive value." << endl;
	}
	m_spring_constant = spring_constant;
}

double linear_spring::GetSpringConstant()
{
	// If spring constant hasn't been set, then find it here
	if (m_spring_constant < 0)
	{
		CalcSpringConstant();
	}

	return m_spring_constant;
}

void linear_spring::CalcSpringConstant()
{
	double shear_mod = m_material.GetShearModulus();
	m_spring_constant = shear_mod * pow(m_wire_diameter, 4) / (8.0 * pow(m_coil_diameter, 3) * m_active_coil_count);
}

double linear_spring::GetLength()
{
	return m_unstretched_length + m_displacement;
}


double linear_spring::GetForce()
{
	m_force = m_spring_constant * m_displacement;
	return m_force;
}

double linear_spring::GetSpringEnergy()
{
	m_spring_energy = 0.5 * m_spring_constant * pow(m_displacement, 2);
	return m_spring_energy;
}

void linear_spring::SetMinimumLength(double minimum_length)
{
	m_minimum_length = minimum_length;
}

void linear_spring::SetMaximumLength(double maximum_length)
{
	m_maximum_length = maximum_length;
}

bool linear_spring::CheckFailure()
{
	bool fail_check = false;

	m_length = m_unstretched_length + m_displacement;

	// Key things that can fail are going under the minimum length, going over the maximum length, and exceeding the maximum shear stress
	if (m_length < m_minimum_length)
	{
		m_length = m_minimum_length;
		cout << endl << "Spring was overcompressed." << endl;
		fail_check = true;
	}

	if (m_length > m_maximum_length)
	{
		m_length = m_maximum_length;
		cout << endl << "Spring was overextended and failed." << endl;
		fail_check = true;
	}

	// Maximum shear that is occuring within the spring. Happens at the inner portion of the wire material
	double max_shear = 4.0 * m_force / (acos(-1) * pow(m_wire_diameter, 2)) * (2.0 * m_coil_diameter / m_wire_diameter + 1.0);
	if (m_material.GetShearStength() < max_shear)
	{
		cout << endl << "Spring exceeded maximum allowable shear stresses." << endl;
		fail_check = true;
	}

	return fail_check;
}

void linear_spring::CalcMinimumLength()
{
	// Minimum length would be where all of the active coils come together and there is no more room for compression
	m_minimum_length = m_active_coil_count * m_wire_diameter;
}

void linear_spring::CalcMaximumLength()
{
	// Maximum length is a little bit harder to pin down, but for now we'll just use this
	m_maximum_length = 2.0 * m_unstretched_length;
}

void linear_spring::SetUnstretchedLength(double unstretched_length)
{
	m_unstretched_length = unstretched_length;
}

double linear_spring::GetUnstretchedLength()
{
	return m_unstretched_length;
}

void linear_spring::CalcUnstretchedLength()
{
	m_unstretched_length = m_relaxed_pitch * m_active_coil_count;
}


void linear_spring::Setup()
{
	if (m_mass < 0)
	{
		CalcMass();
	}

	if (m_minimum_length < 0)
	{
		CalcMinimumLength();
	}

	if (m_maximum_length < 0)
	{
		CalcMaximumLength();
	}

	if (m_unstretched_length < 0)
	{
		CalcUnstretchedLength();
	}
}
#include "dashpot.h"


dashpot::dashpot()
{
	m_mass = 0.0;
	m_damping_coeff = 0.0;
	m_force = 0.0;
	m_velocity = 0.0;
	m_max_force = 10000000000000000000000000000.0;
	m_used_set_force = false;
}

void dashpot::SetMass(double mass)
{
	m_mass = mass;
}

void dashpot::SetDampingCoeff(double damping_coeff)
{
	m_damping_coeff = damping_coeff;
}

void dashpot::SetVelocity(double velocity)
{
	m_velocity = velocity;
}

void dashpot::SetForce(double force)
{
	m_force = force;
	m_used_set_force = true;
}

void dashpot::SetMaxForce(double max_force)
{
	m_max_force = max_force;
}



double dashpot::GetMass()
{
	return m_mass;
}

double dashpot::GetDampingCoeff()
{
	return m_damping_coeff;
}

double dashpot::GetVelocity()
{
	return m_velocity;
}

double dashpot::GetForce()
{
	if (!m_used_set_force)
	{
		m_force = m_damping_coeff * m_velocity;
	}
	
	return m_force;
}

double dashpot::GetMaxForce()
{
	return m_max_force;
}

bool dashpot::CheckFailure()
{
	if (m_force > m_max_force)
	{
		return true;
	}

	return false;
}

void dashpot::Setup()
{
	// Nothing here right now
	return;
}
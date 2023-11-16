#include "airstation.h"
#include "aero_model.h"
#include "geometric_math.h"

#include <cmath>
#include <iostream>

using namespace std;


airstation::airstation()
{
	m_local_aero_model = 0;
	m_reverse_flow = false;
	m_loads_airstation_frame.resize(3);
	m_geometric_angle = 0.0;
}


// Get and Set functions for each of the variables needed
void airstation::SetRho(double rho)
{
	m_rho = rho;
}


double airstation::GetRho()
{
	return m_rho;
}


void airstation::SetChord(double chord)
{
	 m_chord = chord;
}


double airstation::GetChord()
{
	return m_chord;
}



void airstation::SetSoS(double speed_of_sound)
{
	m_speed_of_sound = speed_of_sound;
}


double airstation::GetSoS()
{
	return m_speed_of_sound;
}


void airstation::SetGeometricAngle(double geometric_AoA)
{
	m_geometric_angle = geometric_AoA;
}


double airstation::GetGeometricAngle()
{
	return m_geometric_angle;
}


void airstation::SetNormalVelocity(double normal_velocity)
{
	m_normal_velocity = normal_velocity;
}


double airstation::GetNormalVelocity()
{
	return m_normal_velocity;
}


void airstation::SetTangentVelocity(double tangent_velocity)
{
	m_tangent_velocity = tangent_velocity;
}


double airstation::GetTangentVelocity()
{
	return m_tangent_velocity;
}




void airstation::ComputePhi()
{

	double tangent_vel = m_tangent_velocity;
	double perpendicular_vel = -m_normal_velocity;  // perpendicular velocity is defined as positive down


	m_phi = atan(perpendicular_vel / tangent_vel);

	//cout << "Phi: " << m_phi << endl;

	if (tangent_vel < 0)
	{
		m_reverse_flow = true;
	}

	else
	{
		m_reverse_flow = false;
	}
}


double airstation::GetPhi()
{
	return m_phi;
}



// Computations done for getting certain kinematic characteristics of the airstation
double airstation::Compute_AoA()
{
	ComputePhi();

	m_AoA = m_geometric_angle - m_phi;

	return m_AoA;
}


// Get functions for the variables that we just computed
double airstation::GetAoA()
{
	return m_AoA;
}


double airstation::GetVelMag()
{
	geometric_math math;
	// Take the second norm of the velocity vector
	m_vel_mag = math.VectorNorm({ m_tangent_velocity, m_normal_velocity }, 2);

	return m_vel_mag;
}


double airstation::GetMach()
{
	m_Mach = m_vel_mag / m_speed_of_sound;
	return m_Mach;
}


vector<double> airstation::loads_airstation_frame_calc()
{
	m_loads_airstation_frame = { 0, 0, 0 };

	// Total lift on airstation
	m_loads_airstation_frame[0] = 0.5 * m_aero_coeff[0] * m_rho * pow(m_vel_mag, 2) * m_ref_area;

	// Total drag on airstation
	m_loads_airstation_frame[1] = 0.5 * m_aero_coeff[1] * m_rho * pow(m_vel_mag, 2) * m_ref_area;

	// Total pitching moment on airstation
	m_loads_airstation_frame[2] = 0.5 * m_aero_coeff[2] * m_rho * pow(m_vel_mag, 2) * m_ref_area * m_chord;

	return m_loads_airstation_frame;
}



double airstation::GetLift()
{
	return m_loads_airstation_frame[0];
}


double airstation::GetDrag()
{
	return m_loads_airstation_frame[1];
}


double airstation::GetPitchingMoment()
{
	return m_loads_airstation_frame[2];
}




vector<double> airstation::GetLoadsAirstationFrame()
{
	return m_loads_airstation_frame;
}






// Need to have something to call the aero coefficients
void airstation::SetAeroModel(const aero_model* aero)
{
	// Grabs the aerodynamic coefficients found by the function in the aero_model class
	m_local_aero_model = aero;
}


void airstation::SetAeroCoeffs(double Cl, double Cd, double Cm)
{	
	m_aero_coeff = { Cl, Cd, Cm };
}


void airstation::ComputeAeroCoeffs()
{
	// If there is no aero model set, then don't use aerodynamics
	if (m_local_aero_model != 0)
	{
		m_aero_coeff = m_local_aero_model->GetAeroCoeff(m_AoA, m_reverse_flow);
	}
}


double airstation::GetCl()
{
	return m_aero_coeff[0];
}


double airstation::GetCd()
{
	return m_aero_coeff[1];
}


double airstation::GetCm()
{
	return m_aero_coeff[2];
}




void airstation::SetReferenceArea(double ref_area)
{
	m_ref_area = ref_area;
}

double airstation::GetReferenceArea()
{
	return m_ref_area;
}

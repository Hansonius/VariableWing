#include "aero_model.h"
#include <iostream>


using namespace std;




// Defining Set and Get functions for each of the constants being used for calculation of the forces and moments


// Variables for coefficient of lift
void aero_model::SetLiftslope(double liftslope)
{
	m_liftslope = liftslope;
}

double aero_model::GetLiftslope()
{
	return m_liftslope;
}


// Variables for coefficient of drag (2nd order fit)
void aero_model::SetCd0(double cd0)
{
	m_cd0 = cd0;
}

double aero_model::GetCd0()
{
	return m_cd0;
}

void aero_model::SetCd1(double cd1)
{
	m_cd1 = cd1;
}

double aero_model::GetCd1()
{
	return m_cd1;
}

void aero_model::SetCd2(double cd2)
{
	m_cd2 = cd2;
}

double aero_model::GetCd2()
{
	return m_cd2;
}


// Variables for coefficient of pitching moment
void aero_model::SetCm0(double cm0)
{
	m_cm0 = cm0;
}

double aero_model::GetCm0()
{
	return m_cm0;
}


// Variables used in all
void aero_model::SetAlpha0(double alpha0)
{
	m_alpha0 = alpha0;
}

double aero_model::GetAlpha0()
{
	return m_alpha0;
}






// Defining the Get functions for all of the calculated values found for variables of this class
vector<double> aero_model::GetAeroCoeff(double AoA, bool reverse_flow) const
{

	// This is the angle of attack accounting for reverse flow marked as an angle from 0 to 2*pi. Not used this current linearized model, but it would be for c81 tables or other models
	double AoA_360 = AoA + reverse_flow * acos(-1);


	vector<double> aero_coeff = { 0, 0, 0 };
	
	// Parameter for determinging the sign of the coefficients
	double db_sign = 1.0;

	// Need to do this so all the coefficients are made negative which allow for accurate computation of the forces and moments
	if (reverse_flow == true)
	{
		db_sign = -1.0;
	}

	// Lift coefficient
	aero_coeff[0] = db_sign * m_liftslope * (AoA - (1 - reverse_flow) * m_alpha0);                                                                      // If flow is reversed, do not use alpha0 in calculations


	// Drag coefficient -> does this need to be elaborated on?
	aero_coeff[1] = db_sign * ( m_cd0 + m_cd1 * (AoA - (1 - reverse_flow) * m_alpha0) + m_cd2 * pow((AoA - (1 - reverse_flow) * m_alpha0), 2) );        // If flow is reversed, do not use alpha0 in calculations
	


	// Pitching Moment coefficient
	aero_coeff[2] = m_cm0 + reverse_flow * 0.5 * (aero_coeff[0] * cos(AoA) + aero_coeff[1] * sin(AoA));                                                // If flow is reversed, include extra moment terms from quarter chord change of location




	return aero_coeff;
}




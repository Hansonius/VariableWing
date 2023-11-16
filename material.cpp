#include "material.h"
#include <iostream>

using namespace std;

material::material()
{
	m_material_name = "unnamed_material";
	m_G = -1.0;
	m_E = -1.0;
	m_nu = -2.0; // can range from -1.0 to 0.5
	m_alpha = 0.0;
	m_rho = 0.0;
	m_sigma_y = 0.0;
	m_tau_max = 0.0;
}


void material::SetMaterialName(string material_name)
{
	m_material_name = material_name;
}


string material::GetMaterialName()
{
	return m_material_name;
}


void material::SetShearModulus(double shear_modulus)
{
	m_G = shear_modulus;
}


double material::GetShearModulus()
{
	if (m_G < 0.0)
	{
		CalcEGnu(m_E, -1.0, m_nu);
	}

	return m_G;
}


void material::SetYoungsModulus(double youngs_modulus)
{
	m_E = youngs_modulus;
}

double material::GetYoungsModulus() 
{
	if (m_E < 0.0)
	{
		CalcEGnu(-1.0, m_G, m_nu);
	}

	return m_E;
}

void material::SetCTE(double coefficient_thermal_expansion)
{
	m_alpha = coefficient_thermal_expansion;
}


double material::GetCTE()
{
	return m_alpha;
}


void material::SetDensity(double density)
{
	m_rho = density;
}


double material::GetDensity()
{
	return m_rho;
}


void material::SetPoissonRatio(double poisson_ratio)
{
	if ((poisson_ratio >= -1.0) & (poisson_ratio <= 0.5))
	{
		m_nu = poisson_ratio;
	}

	else
	{
		bool okay_input = false;
		while (!okay_input)
		{
			cout << "Out of bounds for possible Poisson Ratio. Please input number between range of -1.0 and 0.5." << endl;
			cout << "Poisson Ratio: ";
			cin >> m_nu;
			cout << endl;

			if ((poisson_ratio >= -1.0) & (poisson_ratio <= 0.5))
			{
				okay_input = true;
			}

			else
			{
				cout << "Inappropriate input." << endl;
			}
		}
	}
}


double material::GetPoissonRatio()
{
	if (m_nu < -1.0)
	{
		CalcEGnu(m_E, m_G, -1.0);
	}

	return m_nu;
}


void material::SetYieldStrength(double yield_strength)
{
	m_sigma_y = yield_strength;
}


double material::GetYieldStrength()
{
	if (m_sigma_y < 0.0)
	{
		CalcStressMax(-1.0, m_tau_max);
	}
	return m_sigma_y;
}


void material::SetShearStrength(double tau_max)
{
	m_tau_max = tau_max;
}


double material::GetShearStength()
{
	if (m_tau_max < 0.0)
	{
		CalcStressMax(m_sigma_y, -1.0);
	}
	return m_tau_max;
}





void material::CalcEGnu(double E = -1.0, double G = -1.0, double nu = -1.0)
{
	if (E < 0)
	{
		E = G * (2.0 * (1.0 + nu));
		m_E = E;
	}

	else if (G < 0)
	{
		G = E / (2.0 * (1.0 + nu));
		m_G = G;
	}

	else
	{
		nu = E / (2.0 * G) - 1.0;
		m_nu = nu;
	}
}

void material::CalcStressMax(double sigma_y = -1.0, double tau_max = -1.0)
{
	if (sigma_y < 0)
	{
		sigma_y = 2.0 * tau_max;
		m_sigma_y = sigma_y;
	}

	else if (tau_max < 0)
	{
		tau_max = sigma_y / 2.0;
		m_tau_max = tau_max;
	}

	else
	{
		cout << endl << "Input error to CalcStressMax(). Both values provided were positive and so no action was taken." << endl;
	}
}

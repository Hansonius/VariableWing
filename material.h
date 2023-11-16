/**
* \brief Class to define material properties
* @author Josh Hansen
*/

#ifndef MATERIAL
#define MATERIAL

#include <string>

class material
{
	public:
		material();

		void SetMaterialName(std::string material_name);
		std::string GetMaterialName();

		void SetShearModulus(double G);
		double GetShearModulus();

		void SetYoungsModulus(double E);
		double GetYoungsModulus();

		void SetPoissonRatio(double nu);
		double GetPoissonRatio();

		void SetDensity(double rho);
		double GetDensity();

		void SetCTE(double alpha); // CTE - Coefficient of Linear Thermal Expansion
		double GetCTE();

		void SetYieldStrength(double sigma_y);
		double GetYieldStrength();

		void SetShearStrength(double tau_max);
		double GetShearStength();

	private:
		std::string m_material_name;

		double m_G;
		double m_E;
		double m_nu;
		double m_rho;
		double m_alpha;
		double m_sigma_y;
		double m_tau_max;

		void CalcEGnu(double E, double G, double nu);
		void CalcStressMax(double sigma_y, double tau_max);
};


#endif
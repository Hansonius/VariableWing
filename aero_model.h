/**
 * @brief aero_model.h contains code for creating an aerodynamics model
 *
 * This class stores the relevant data for an aerodynamics model of interest and calculates the coefficients of lift, drag, and pitching moment for a given angle of attack
 *
 * @author Josh Hansen
 * @author Nicolas Reveles
 *
 */


#ifndef AEROMODEL_H
#define AEROMODEL_H

#include <vector>

class aero_model
{
	public:

		// Variables used for getting all coefficients
		/**
		* \brief Sets the angle of attack at which there is 0 net lift
		* \param Expects units of radians
		*/
		void SetAlpha0(double alpha0);


		/**
		* \brief Gets the angle of attack at which there is 0 net lift
		* \return Returns Alpha0 in units of radians
		*/
		double GetAlpha0();


		/**
		* \brief Sets the liftslope or \f$C_l_\alpha\f$ value for a certain airfoil cross-section
		*
		* The lift coefficient is found using a first order polynomial of the form: \f$ C_l = C_l_\alpha * (\alpha - \alpha_0) \f$
		* Note that if there is reverse flow, the alpha0 offset term is not included
		*
		* \param liftslope Expects units of 1 / radians
		*/
		void SetLiftslope(double liftslope);
		
		
		/**
		* \brief Gets the Cl_alpha value for the aerodynamics model
		* \return Returns Cl_alpha in units of 1 / radians
		*/
		double GetLiftslope();


		// Variables used for getting coefficient of drag
		/**
		* \brief Sets the \f$C_d_0\f$ value for the geometry of interest
		*
		* The drag coefficient is found using a second order polynomial of the form: \f$ C_d = C_d_0 + C_d_1*(\alpha - \alpha_0) + C_d_2*(\alpha - \alpha_0)^2 \f$
		* Note that if there is reverse flow, the alpha0 offset terms are not included
		*
		* \param cd0 Expects unitless measure
		*/
		void SetCd0(double cd0);
		
		
		/**
		* \brief Gets the coefficient of drag value at 0 degree angle of attack
		* \return Returns \f$C_d_0\f$ as unitless measure
		*/
		double GetCd0();

		/**
		* \brief Sets the first order term drag coefficient
		*
		* The drag coefficient is found using a second order polynomial of the form: \f$ C_d = C_d_0 + C_d_1*(\alpha - \alpha_0) + C_d_2*(\alpha - \alpha_0)^2 \f$
		* Note that if there is reverse flow, the alpha0 offset terms are not included
 		*
		* \param cd1 Expects units of 1 / radians
		*/
		void SetCd1(double cd1);
		
		
		/**
		* \brief Gets the first order term drag coefficient
		* \return Returns \f$C_d_1\f$ in units of 1 / radians
		*/
		double GetCd1();


		/**
		* \brief Sets the second order term drag coefficient
		*
		* The drag coefficient is found using a second order polynomial of the form: \f$ C_d = C_d_0 + C_d_1*(\alpha - \alpha_0) + C_d_2*(\alpha - \alpha_0)^2 \f$
		* Note that if there is reverse flow, the alpha0 offset terms are not included
		*
		* \param cd2 Expects units of 1 / radians^2
		*/
		void SetCd2(double cd2);
		
		
		/**
		* \brief Sets the second order term drag coefficient
		* \return Returns \f$C_d_2\f$ in units of 1 / radians^2
		*/
		double GetCd2();


		// Variables used for getting coefficient of pitching moment
		/**
		* \brief Sets the coefficient of pitching moment at 0 degree angle of attack
		*
		* The coefficient of the pitching moment is modeled through a first order equation of the form: \f$ C_m = C_m_0\f$ when subjected to normal forward-flight conditions
		* The coefficient of the pitching moment is modeled through a first order equation of the form: \f$ C_m = C_m_0 + 0.5*(C_lcos(\alpha) + C_d*sin(\alpha))\f$ when subjected to reverse flow conditions
		*
		* \param cm0 Expects unitless measure
		*/
		void SetCm0(double cm0);
		
		
		/**
		* \brief Gets the coefficient of pitching moment at 0 degree angle of attack
		* \return Returns \f$C_m_0\f$ as a unitless measure
		*/
		double GetCm0();


		// Function to find forces and moments acting in the airstation and body frame
		/**
		* \brief Finds the coefficients of lift, drag, and pitching moment for the airstation
		* \param AoA Angle of attack in radians
		* \return <Coefficient of Lift, Coefficient of Drag, Coefficient of Pitching Moment>
		*/
		std::vector<double> GetAeroCoeff(double AoA, bool reverse_flow) const;

		
	


	private:
		// Airstation geometric parameters
		double m_liftslope;
		double m_cd0;
		double m_cd1;
		double m_cd2;
		double m_cm0;
		double m_alpha0;
		
};
#endif
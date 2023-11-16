#ifndef AIRSTATION_H
#define AIRSTATION_H

/**
 * @brief airstation.h contains code for calculating lift, drag, and pitching moment at an airstation
 *
 * Finds the aerodynamic forces acting at an airstation utilizing the aero_model class
 *
 * @author Josh Hansen
 * @author Nicolas Reveles
 *
 */

#include "aero_model.h"
#include <vector>
#include <cmath>


/**
\brief This class stores the data for an individual airstation and calculates the aerodynamic forces and moments acting on it
*/
class airstation
{
	public:
		airstation();

		// Get and Set Functions for each of the variables of interest

		/**
		* \brief Sets the freestream denisty of the air at the airstation
		* \param rho Expects units of mass/length^3
		*/
		void SetRho(double rho);
		
		
		/**
		* \brief Gets the freestream density of the air at the airstation
		* \return Returns the freestream density in units of mass / length^3
		*/
		double GetRho();
		

		/**
		* \brief Sets the chord length of the airstation
		* \param chord Expects units of length
		*/
		void SetChord(double chord);
		
		
		/**
		* \brief Gets the chord length of the airstation
		* \return Returns the chord length of the airstation in units of length
		*/
		double GetChord();


		/**
		* \brief Speed of sound at the airstation.
		* \param speed_of_sound Expects units of length/time
		*/
		void SetSoS(double speed_of_sound);
		
		
		/**
		* \brief Gets the speed of sound at the airstation
		* \return Returns the speed of sound at the airstation in units of length / time
		*/
		double GetSoS();


		/**
		* \brief This function computes the angle of attack (AoA) of the airstation 
		* \return The angle of attack of the airstation (in radians)
		*/
		double Compute_AoA();
		
		
		// Get functions for what is computed in this class
		/**
		* \brief Grabs the angle of attack for a certain airstation
		* \return Outputs the angle of attack for an airstatio in radians
		*/
		double GetAoA();


		/**
		* \brief Grabs the magnitude of the total velocity that airstation sees
		* \return Outputs the magnitude of the velocity with respect to the airin units of length/time
		*/
		double GetVelMag();


		/**
		* \brief Grabs the Mach number seen by the airstation
		* \return Outputs the unitless Mach number for the airstation
		*/
		double GetMach();


		/**
		* \brief Finds the lift, drag, and pitching moment for the airstation
		* \return <Lift, Drag, Pitching Moment>
		*/
		std::vector<double> loads_airstation_frame_calc();


		// Get functions for grabbing all the dimensional answers
		/**
		* \brief Grabs the aerodynamic loads of <Lift, Drag, Pitching Moment> for an airstation
		* \return Outputs a 3 element vector of <Lift, Drag, Pitching Moment> for an airstation
		*/
		std::vector<double> GetLoadsAirstationFrame();



		// Need to be able to set the aerodynamic coefficients directly
		
		/**
		* \brief Sets the aerodynamic coefficients of the Cl, Cd, and Cm for an airstation. Allows these parameters to be set directly instead of calculating them with an aerodynamics model
		* \param Cl Expects unitless measure
		* \param Cd Expects unitless measure
		* \param Cm Expects unitless measure
		*/
		void SetAeroCoeffs(double Cl, double Cd, double Cm);


		/**
		* \brief Sets up an aerodynamics model to use for finding the forces and moments acting on the airstation
		*
		* If the aero_model is set to NULL, the user is expected to set the aerodynamic coefficients directly 
		*
		* \param aero This is a constant pointer object of the aero_model class which should have parameters such as the cl0, cd0, cl_alpha terms predefined
		*/
		void SetAeroModel(const aero_model* aero);


		/**
		* \brief Computes the aerodynamic coefficients of the airstation using the aero_model and the AoA of the airstation
		*/
		void ComputeAeroCoeffs();


		/**
		* \brief Gets the coefficient of lift for the airstation
		* \return Returns the coefficient of lift for the airstation
		*/
		double GetCl();


		/**
		* \brief Gets the coefficient of drag for the airstation
		* \return Returns the coefficient of drag for the airstation
		*/
		double GetCd();


		/**
		* \brief Gets the coefficient of pitching moment for the airstation
		* \return Returns the coefficient of pitching moment for the airstation
		*/
		double GetCm();


		/**
		* \brief Sets the geometric component of the angle of attack for the airstation. This is defined as the angle between the chord of the airstation and the horizon
		* \param geometric_AoA Angle between chord line and rotor disk in radians
		*/
		void SetGeometricAngle(double geometric_AoA);
		

		/**
		* \brief Gets the geometric angle of the airstation
		* \return Returns geometric angle of airstation in radians
		*/
		double GetGeometricAngle();


		/**
		* \brief Sets the velocity in the normal direction for the airstation
		* \param normal_velocity Units of length / time
		*/
		void SetNormalVelocity(double normal_velocity);
		
		
		/**
		* \brief Sets the velocity in the tangent direction for the airstation
		* \param tangent_velocity Units of length / time
		*/
		void SetTangentVelocity(double tangent_velocity);


		/**
		* \brief Gets the normal velocity acting on the airstation
		* \return Normal velocity in units of length / time
		*/
		double GetNormalVelocity();
		
		
		/**
		* \brief Gets the tangent velocity acting on the airstation
		* \return Tangent velocity in units of length / time
		*/
		double GetTangentVelocity();


		/**
		* \brief Sets the reference area of the airstation
		* \param ref_area Units of length^2
		*/
		void SetReferenceArea(double ref_area);

		double GetReferenceArea();


		/**
		* \brief Gets the lift acting on the airstation
		* \return Lift in units of mass * length / time^2
		*/
		double GetLift();


		/**
		* \brief Gets the drag acting on the airstation
		* \return Drag in units of mass * length / time^2
		*/
		double GetDrag();


		/**
		* \brief Gets the pitching moment acting on the airstation
		* \return Pitching moment in units of mass * length^2 / time^2
		*/
		double GetPitchingMoment();


		/**
		* \brief Gets the flight path angle of the airstation. Defined as the inverse tan of perpendicular velocity / tangent velocity, where perpendicular velocity is negative normal velocity. See attached documentation for further clarification
		* \return Angle \f$\phi\f$ in units of radians
		*/
		double GetPhi();



	private:
		// Geometry
		double m_chord;


		// Translation/rotation variables
		double m_tangent_velocity;
		double m_normal_velocity;


		
		// Speed of Sound
		double m_speed_of_sound;


		// General variables we are storing for later use
		double m_geometric_angle;
		double m_spanwise_loc;
		double m_Mach;


		// Results of the functions
		std::vector<double> m_aero_coeff;
		std::vector<double> m_loads_airstation_frame;
		std::vector<double> m_forces_body_frame;
		std::vector<double> m_moments_body_frame;

		
		// Fluid constants
		double m_rho;
		std::vector<double> m_freestream_velocity;

		// Aero model object being passed in
		const aero_model* m_local_aero_model;


		// Key airstation terms
		double m_AoA;
		double m_vel_mag;


		/**
		* \brief Computes the angle between the horizon and the net velocity vector, phi
		*/
		void ComputePhi();
		double m_phi;


		// Boolean variable to check whether or not the airstation is seeing reverse velocity
		bool m_reverse_flow;


		double m_ref_area;
};

#endif

/**
 * @brief lifting_line.h contains code for finding the net forces and moments acting on a series of airstations
 *
 * This class stores the data for a certain lifting line object broken up into airstations. It calculates the net forces and moments 
 * about a certain reference point from this series of airstations. Allows for mutation of the position of the airstations and the 
 * directions of their loads to be compatible with structual dynamics solvers, but does not implement these effects itself.
 *
 * @author Josh Hansen
 * @author Nicolas Reveles
 *
 */


#ifndef LIFTINGLINE_H
#define LIFTINGLINE_H

#include "airstation.h"
#include "aero_model.h"
#include <vector>

#include <cstring>
#include <fstream>


class lifting_line
{
	public:
		/**
		* \brief Sets the number of airstations along the lifting line
		* \number-of_airstations Expects integrer number of airstations along the lifting line
		*/
		void SetNumberOfAirstations(int number_of_airstations);
		
		
		/**
		* \brief Gets the number of airstations along the lifting line
		* \return Returns a whole number of airstations that the lifting line is broken up into
		*/
		int GetNumberofAirstations();
	

		/**
		* \brief Specifies the point of interest to find the moments acting
		* \param moment_point Expects 3x1 vector input for certain coordinate in form of <X_coord, Y_coord, Z_coord>
		*/
		void SetMomentPoint(std::vector<double> moment_point);
		
		
		/**
		* \brief Gets the point about which to calculate moments 
		* \return Returns a 3x1 vector in units of length specifying the position of the point to take the moments about in the body frame axes
		*/
		std::vector<double> GetMomentPoint();

		
		/**
		* \brief Sets the aerodynamics model to be used along the lifting line
		* \param aero Constant pointer object of the aero_model class so that less data is stored overall when the same aerodynamics model is used at multiple airstations
		* \param airstation_index Index of the airstation at which this aerodynamics model should be applied. Index of 0 indicates the airstation closest to the root and the index increases as moving outwards to the tip
		*/
		void SetAirstationAerodynamicsModel(const aero_model* aero, int airstation_index);


		// Get functions for the forces and moments calculated
		/**
		* \brief Grabs the total forces acting on the lifting line
		* \return Outputs a 3x1 vector for the total forces acting on the lifting line in the body frame axes 
		*/
		std::vector<double> GetTotalForces();


		/**
		* \brief Grabs the total moments acting on the lifting line about a specified point
		* \return Outputs a 3x1 vector for the total moments acting on the lifting line about a specified point in the body frame axes
		*/
		std::vector<double> GetTotalMomentsAboutPoint();


		/**
		* \brief Gets an airstation at a certain index indicated along the lifting line
		* \param airstation_index Integer number ranging from 0 to the number of airstations along the lifting line - 1. Index of 0 indicates airstation closest to the root and larger indeces mean closer to the tip
		* \return Returns a pointer to an airstation at the airstation index indicated
		*/
		airstation* GetAirstation(int airstation_index);


		/**
		* \brief Calculates the total forces and moments acting along the lifting line by summing up all the components from airstations along the line
		*/
		void TotalForceMomentCalc();


		/**
		* \brief Sets the freestream density of the air
		* \param rho Units of mass / length^3
		*/
		void SetRhoInf(double rho);


		/**
		* \brief Sets the speed of sound in the region surrounding the lifting line
		* \param SoS Units of length / time
		*/
		void SetSoS(double SoS);



		/**
		* \brief Calculates the local wind for an airstation based on a given freestream velocity in the lifting line frame
		* \param local_wind 3x1 vector of the effective wind velocity in the lifting line frame with units of length / time
		* \param airstation_index Integer ranging between 0 and the number of airstations - 1. Airstations are labelled such that an index of 0 is closest to the root and then the indeces increase going towards the tip
		*/
		void SetAirstationFreestream(std::vector<double> local_wind, int airstation_index);


		/**
		* \brief Allows for the velocity of the airstation from actions such as rotation or aeroelastic effects to be set directly
		* \param airstation_velocity 3x1 of the velocity of the airstation due to effects such as rotation or aeroelasticity in the lifting line frame
		* \param airstation_index Integer ranging between 0 and the number of airstations - 1. Airstations are labelled such that an index of 0 is closest to the root and then the indeces increase going towards the tip
		*/
		void SetAirstationVelocity( std::vector<double> airstation_velocity, int airstation_index);


		/**
		* \brief Sets up all constant parameters of each airstations in a master function
		*/
		void SetupAirstations();


		/**
		* \brief Updates the resulting aerodynamics of each airstation as the lifting line sees different frestream velocities
		*/
		void UpdateAirstationsAerodynamics();


		// GEOMETRY FUNCTIONS
		/**
		* \brief Sets the airstation positions in the lifting line frame axes
		* X is in the spanwise direction of 0 degree sweep, Y is from trailing edge to leading edge at 0 degree geometric angle and sweep, and Z is positive upwards by the right hand rule. The origin is currently assumed to be shared with the body frame origin
		* \param airstation_positions Positions of the quarter chord of an airstation center in the lifting line frame of reference 
		*/
		void SetAirstationPositions(std::vector<std::vector<double>> airstation_positions);


		/**
		* \brief Gets the quarter chord position of an airstation in the lifting line frame
		* \param airstation_index Airstation index ranging from 0 to number of airstations - 1
		* \return 3x1 vector of the quarter chord position of an airstation in the lifting line frame
		*/
		std::vector<double> GetAirstationPosition(int airstation_index);


		/**
		* \brief Sets the airstation chordwise vectors in the lifting line axes
		* Note that this vector is untwisted so if the corresponding twist angle for the airstation is non-zero, this vector will be rotated by that angle
		* \param airstation_chordwise_vectors Nx3 array describing the chordwise direction of each airstation. Each row of the array is a 3D unit vector
		*/
		void SetAirstationChordwiseVectors(std::vector<std::vector<double> > airstation_chordwise_vectors);


		/**
		* \brief Gets the untwisted chordwise vector of an airstation in the lifting line frame
		* \param airstation_index Airstation index ranging from 0 to number of airstations - 1
		* \return 3x1 vector of the untiwsted chordwise vector in the lifting line frame
		*/
		std::vector<double> GetAirstationChordwiseVector(int airstation_index);


		/**
		* \brief Sets the airstation normal vectors in the lifting line axes
		* Note that this vector is untwisted so if the corresponding twist angle for the airstation is non-zero, this vector will be rotated by that angle
		* \param airstation_normal_vectors Nx3 array describing the normal direction of each airstation. Each row of the array is a 3D unit vector
		*/
		void SetAirstationNormalVectors(std::vector<std::vector<double> > airstation_normal_vectors);


		/**
		* \brief Gets the untwisted normal vector of an airstation in the lifting line frame
		* \param airstation_index Airstation index ranging from 0 to number of airstations - 1
		* \return 3x1 vector of the untwisted normal vector in the lifting line frame
		*/
		std::vector<double> GetAirstationNormalVector(int airstation_index);


		/**
		* \brief Sets the airstation twist angles in the airstation chordwise and normal axes
		* Twists the chordwise and normal vectors for the airstation by this angle about their cross product. Positive twist angle corresponds to increase in geometric angle for traditional setups
		* \param airstation_twist_angles Vector of length N with units of radians
		*/
		void SetAirstationTwistAngles(std::vector<double> airstation_twist_angles);


		/**
		* \brief Gets the twist angle at a certain airstation
		* \param airstation_index Airstation index ranging from 0 to number of airstations - 1
		* \return Twist angle in units of radians
		*/
		double GetAirstationTwistAngle(int airstation_index);


		/**
		* \brief Sets the chord lengths at each airstation 
		* \param airstation_chord_lengths Vector of length N with units of length
		*/
		void SetAirstationChordLengths(std::vector<double> airstation_chord_lengths);


		/**
		* \brief Gets the chord length at a certain airstation
		* \param airstation_index Airstation index ranging from 0 to number of airstations - 1
		* \return Chord length in units of length
		*/
		double GetAirstationChordLength(int airstation_index);


		/**
		* \brief Sets the chord length of the root of the lfting line
		* \param root_chord length Units of length
		*/
		void SetRootChordLength(double root_chord_length);


		/**
		* \brief Gets the root chord length
		* \return Root chord length in units of length
		*/
		double GetRootChordLength();


		/**
		* \brief Sets the chord length of the tip of the lifting line
		* \param tip_chord_length Units of length
		*/
		void SetTipChordLength(double tip_chord_length);


		/**
		* \brief Gets the tip chord length
		* \return Tip chord length in units of length
		*/
		double GetTipChordLength();

		/**
		* \brief Sets the quarter chord position of the root of the lifting line in the lifting line axes
		* \param root_quarter_chord_position 3D vector with units of distance
		*/
		void SetRootQuarterChordPosition(std::vector<double> root_quarter_chord_position);


		/**
		* \brief Gets the position of the quarter chord of the root in the lifting line axes
		* \return 3x1 vector of the position of the root quarter chord in units of length
		*/
		std::vector<double> GetRootQuarterChordPosition();


		/**
		* \brief Sets the quarter chord position of the tip of the lifting line in the lifting line axes
		* \param tip_quarter_chord_position 3D vector with units of distance
		*/
		void SetTipQuarterChordPosition(std::vector<double> tip_quarter_chord_position);


		/**
		* \brief Gets the position of the quarter chord of the tip in the lifting line axes
		* \return 3x1 vector of the position of the tip quarter chord in units of length
		*/
		std::vector<double> GetTipQuarterChordPosition();


		/**
		* \brief Sets the chordwise vector of the root of the lifting line in the lifting line axes
		* Note that this vector is untwisted so if the corresponding twist angle for the airstation is non-zero, this vector will be rotated by that angle
		* \param rc_chordwise_vec 3D unit vector
		*/
		void SetRootChordChordwiseVector(std::vector<double> rc_chordwise_vec);


		/**
		* \brief Gets the untwistedchordwise vector of the root chord in the lifting line axes
		* \return 3x1 vector of the untwisted chordwise vector of the root chord
		*/
		std::vector<double> GetRootChordChordwiseVector();


		/**
		* \brief Sets the normal vector of the root of the lifting line in the lifting line axes
		* Note that this vector is untwisted so if the corresponding twist angle for the airstation is non-zero, this vector will be rotated by that angle
		* \param rc_normal_vec 3D unit vector
		*/
		void SetRootChordNormalVector(std::vector<double> rc_normal_vec);


		/**
		* \brief Gets the untwisted normal vector of the root chord in the lifting line axes
		* \return 3x1 vector of the untwisted normal vector of the root chord
		*/
		std::vector<double> GetRootChordNormalVector();


		/**
		* \brief Sets the chordwise vector of the tip of the lifting line in the lifting line axes
		* Note that this vector is untwisted so if the corresponding twist angle for the airstation is non-zero, this vector will be rotated by that angle
		* \param tc_chordwise_vec 3D unit vector
		*/
		void SetTipChordChordwiseVector(std::vector<double> tc_chordwise_vec);


		/**
		* \brief Gets the untwisted chordwise vector of the tip chord in the lifting line axes
		* \return 3x1 vector of the untwisted chordwise vector of the tip chord
		*/
		std::vector<double> GetTipChordChordwiseVector();


		/**
		* \brief Sets the normal vector of the tip of the lifting line in the lifting line axes
		* Note that this vector is untwisted so if the corresponding twist angle for the airstation is non-zero, this vector will be rotated by that angle
		* \param tc_normal_vec 3D unit vector
		*/
		void SetTipChordNormalVector(std::vector<double> tc_normal_vec);


		/**
		* \brief Gets the untwisted normal vector of the tip chord in the lifting line axes
		* \return 3x1 vector of the untwisted normal vector of the tip chord
		*/
		std::vector<double> GetTipChordNormalVector();


		/**
		* \brief Sets the twist angle of the root chord of the lifting line
		* Twists the chordwise and normal vectors for the root by this angle about their cross product. Positive twist angle corresponds to increase in geometric angle for traditional setups
		* \param rc_twist Units of radians 
		*/
		void SetRootChordTwist(double rc_twist);


		/**
		* \brief Gets the twist angle of the root chord
		* \return Twist angle in units of radians
		*/
		double GetRootChordTwist();

		/**
		* \brief Sets the twist angle of the tip chord of the lifting line
		* Twists the chordwise and normal vectors for the tip by this angle about their cross product. Positive twist angle corresponds to increase in geometric angle for traditional setups
		* \param tc_twist Units of radians
		*/
		void SetTipChordTwist(double tc_twist);


		/**
		* \brief Gets the twist angle of the tip chord
		* \return Twist angle in units of radians
		*/
		double GetTipChordTwist();

		/**
		* \brief Gets the positions of the 4 corners of an airstation in the lifting line axes  
		* \param airstation_index Airstation index ranging from 0 to number of airstations - 1
		* \return Returns a 4 x 3 array where each row is a point in 3D space in the lifting line axes of the form:  < X position, Y position, Z position >. The first row corresponds to 'point A', the second to 'point B', etc., as described in the attached documentation
		*/
		std::vector<std::vector<double> > GetAirstationCorners(int airstation_index);


		/**
		* \brief Gets the twisted chordwise vector of a certain airstation
		* \param airstation_index Airstation index ranging from 0 to number of airstations - 1
		* \return 3x1 vector of the twisted chordwise vector at the specified airstation
		*/
		std::vector<double> GetTwistedChordwiseVector(int airstation_index);


		/**
		* \brief Gets the twisted normal vector of a certain airstation
		* \param airstation_index Airstation index ranging from 0 to number of airstations - 1
		* \return 3x1 vector of the twisted normal vector at the specified airstation
		*/
		std::vector<double> GetTwistedNormalVector(int airstation_index);


		/**
		* \brief Gets the total reference area of the lifting line
		* \return Total reference area in units of length^2
		*/
		double GetTotalReferenceArea();


		/**
		* \brief Master function that twists the airstation vectors, finds the reference area for each airstation, and then sets those areas to each airstation
		*/
		void GeometrySetup();


		/**
		* \brief Generates a file to help plot the airstations in 3D space. The file is formatted where each row is an airstation and the columns consist of the quarter chord, leading edge, and trailing edge positions
		* \param filename Desired filename of the output. Recommended formats: .txt, .dat 
		*/
		//void GenerateDataFile(std::string filename);


		/**
		* \brief Reads in a data file which specifies the geometry of a lifting line into a 2D array
		* See the provided sample input file: 'README_Sample_Input.txt' for further instruction of how these files are set up 
		* \param filename Filename of the input file. Recommended formats: .txt, .dat
		* \return 1 if a file is read in correctly, 0 if the operation failed
		*/
		int ReadDataFile(std::string filename);



		void SetMass(double mass);
		double GetMass();

		void SetCenterOfMassPosition(std::vector<double> center_of_mass_pos);
		std::vector<double> GetCenterOfMassPosition();

		void SetMomentOfInertia_CM(std::vector<std::vector<double>> moi_cm);
		std::vector<std::vector<double>> GetMomentOfInertia_CM();




	private:
		// Number of airstaions
		int m_number_of_airstations;

		// Creates a vector of airstation that are indexed by a certain number (i.e. what number airstation it is)
		std::vector<airstation> m_airstations;

		// Specified point about which moments act
		std::vector<double> m_moment_point;

		// Force and moments member variables
		std::vector<double> m_total_forces;
		std::vector<double> m_total_moments_about_point;


		// Parameters for defining the root chord
		double m_root_chord_length;
		double m_root_chord_twist;
		std::vector<double> m_root_quarter_chord_position;
		std::vector<double> m_root_chord_chordwise_vector;
		std::vector<double> m_root_chord_normal_vector;

		std::vector<double> m_root_chord_twisted_chordwise_vector;
		std::vector<double> m_root_chord_twisted_normal_vector;
		std::vector<double> m_RC_LE_pos;
		std::vector<double> m_RC_TE_pos;

		
		// Parameters for defining the tip chord
		double m_tip_chord_length;
		double m_tip_chord_twist;
		std::vector<double> m_tip_quarter_chord_position;
		std::vector<double> m_tip_chord_chordwise_vector;
		std::vector<double> m_tip_chord_normal_vector;

		std::vector<double> m_tip_chord_twisted_chordwise_vector;
		std::vector<double> m_tip_chord_twisted_normal_vector;
		std::vector<double> m_TC_LE_pos;
		std::vector<double> m_TC_TE_pos;

		std::vector<double> m_airstation_twist_angles;
		std::vector<double> m_airstation_chord_lengths;


		// Arrays for keeping track of the airstation freestreams (which can vary due to effects such as inflow for rotorcraft and should not be assumed to be uniform) and the airstation velocities (which are 0 for fixed lifting lines, but may vary for rotary systems)
		std::vector<std::vector<double> > m_airstation_freestreams;
		std::vector<std::vector<double> > m_airstation_velocities;

		double m_rho_inf;
		double m_SoS;

		//std::vector<double> m_airstation_spanwise_locations;

		std::vector<std::vector<double> > m_airstation_positions;

		std::vector<std::vector<double> > m_airstation_chordwise_vectors;

		std::vector<std::vector<double> > m_airstation_normal_vectors;


		std::vector<std::vector<double> > m_airstation_twisted_chordwise_vectors;
		std::vector<std::vector<double> > m_airstation_twisted_normal_vectors;




		// Functions for math of converting everything into the airstation frame and back out
		void ComputeAirstationNormalVelocities();
		void ComputeAirstationTangentVelocities();




		// Functions for getting accurate reference areas
		/**
		* \brief Applies the prescribed twist to the chordwise and normal vectors for each airstation as well as the root and tip chords
		*/
		void CalcTwistedVectors();


		/**
		* \brief Calculates the positions of the leading edges and trailing edges of each airstation based on the twist chordwise and normal vectors and the provided quarter chord positions for the aistations, root, and tip
		*/
		void CalcLEs_TEs();
		std::vector<std::vector<double> > m_LE_positions;
		std::vector<std::vector<double> > m_TE_positions;


		/**
		* \brief Calculates the positions of the 4 corners of each airstation using the mid points between leading edges and trailing edges of each airstation. See attached documentation for labelling of these points and how the m_airstation_corners variable is structured
		*/
		void CalcAirstationCorners();
		std::vector<std::vector<std::vector<double> > > m_airstation_corners; // Each airstation has 4 3D points describing where the edges of the airstations are


		/**
		* \brief Calculates and sets the reference area of each airstation as the sum of the area of two triangles on opposite corners of the airstation. See attached documentation for more details
		*/
		void CalcAirstationReferenceAreas();
		std::vector<double> m_airstation_reference_areas; // vector of the reference areas for each airstation
		double m_total_reference_area;


		/**
		* \brief Calculates updates to the airstation parameters such as aerodynamic coefficients based on changes to freestream velocity or some other non constant parameter
		*/
		void ComputeAirstationAerodynamicParameters();


		/**
		* \brief Sets the values from the data matrix generated from the input file into the other private variables so that they are easier to grab and utilize
		*/
		void SetValuesFromDataArray(std::vector<std::vector<double> > data_matrix);


		/**
		* \brief Sets the freestream density value, speed of sound, and the chord length of each airstation need for computation of the lift, drag, and pitching moment
		*/
		void SetAirstationParameters();


		/**
		* \brief Initializes the vector of airstations, the array of airstation freestreams, and the array of airstation velocities to have the correct size (i.e. m_number_of_airstations) 
		*/
		void InitializeInternalParameters();



		double m_mass;
		std::vector<double> m_cm_pos;
		std::vector<std::vector<double>> m_moi_cm;

};
#endif

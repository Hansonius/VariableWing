#include "aero_model.h"
#include "airstation.h"
#include "lifting_line.h"

#include "geometric_math.h"

#include <iostream>

#include <string>
#include <fstream>
#include <sstream>


using namespace std;

// Airstations setup
void lifting_line::SetNumberOfAirstations(int number_of_airstations)
{
	m_number_of_airstations = number_of_airstations;

	// Initialize parameters that need to have a size of m_number_of airstations
	InitializeInternalParameters();
}


int lifting_line::GetNumberofAirstations()
{
	return m_number_of_airstations;
}


void lifting_line::SetMomentPoint(vector<double> moment_point)
{
	m_moment_point = moment_point;
}


vector<double> lifting_line::GetMomentPoint()
{
	return m_moment_point;
}


airstation* lifting_line::GetAirstation(int airstation_index)
{
	return &(m_airstations[airstation_index]);
}


void lifting_line::SetAirstationAerodynamicsModel(const aero_model* aero, int airstation_index)
{
	m_airstations[airstation_index].SetAeroModel(aero);
}


// Constation values across all airstations
void lifting_line::SetRhoInf(double rho_inf)
{
	m_rho_inf = rho_inf;

	// Set this at the airstation level as it is needed for proper computation of lift/drag/pitching moment
	for (int i = 0; i < m_number_of_airstations; i++)
	{
		m_airstations[i].SetRho(m_rho_inf);
	}
}


void lifting_line::SetSoS(double SoS)
{
	m_SoS = SoS;

	// Set this at the airstation level as it is needed for proper computation of the aerodynamic coefficients which it uses for finding lift/drag/pitching moment
	for (int i = 0; i < m_number_of_airstations; i++)
	{
		m_airstations[i].SetSoS(m_SoS);
	}
}


// THIS IS NOT CONSTANT FOR ALL AIRSTATIONS AS THERE COULD BE INFLOW
void lifting_line::SetAirstationFreestream(vector<double> freestream, int airstation_index)
{
	m_airstation_freestreams[airstation_index] = freestream;
}


void lifting_line::SetAirstationVelocity(vector<double> airstation_velocity, int airstation_index)
{
	m_airstation_velocities[airstation_index] = airstation_velocity;
}


void lifting_line::InitializeInternalParameters()
{
	m_airstations.resize(m_number_of_airstations);
	m_airstation_freestreams.resize(m_number_of_airstations);
	m_airstation_velocities.resize(m_number_of_airstations);
}


// Computational functions for parameters which vary across the airstations
void lifting_line::ComputeAirstationNormalVelocities()
{
	geometric_math math;

	double total_norm_vel;
	vector<double> unit_norm_vec;

	for (int i = 0; i < m_number_of_airstations; i++)
	{
		// Ensure that the vector we are dotting to is a unit vector
		unit_norm_vec = math.UnitVector(m_airstation_normal_vectors[i]);

		// Comprised of the freestream and airstation velocity components
		total_norm_vel = math.DotProduct(m_airstation_freestreams[i], unit_norm_vec) + math.DotProduct(m_airstation_velocities[i], unit_norm_vec);

		m_airstations[i].SetNormalVelocity(total_norm_vel);
	}
}


void lifting_line::ComputeAirstationTangentVelocities()
{
	geometric_math math;

	double total_tan_vel;
	vector<double> unit_chordwise_vec;

	for (int i = 0; i < m_number_of_airstations; i++)
	{
		// Ensure that the vector we are dotting to is a unit vector
		unit_chordwise_vec = math.UnitVector(m_airstation_chordwise_vectors[i]);

		// The velocity of the airstation is going to be in the +y direction in the instance that it is rotating about some shaft and the freestream will be in the -y direction if there is wind against the lifting line. However, in this instance the negative y wind and the 
		// +y movement should be added together to get the effective velocity. To get this to happen, we need to subtract off the freestream term

		// Comprised of the freestream and airstation velocity components. Tangent velocity is in the opposite direction to the untwisted chordwise vector, and so we multiply the unit chordwise vector by -1 when dotting with the freestream
		total_tan_vel = -1.0 * math.DotProduct(m_airstation_freestreams[i], unit_chordwise_vec) + math.DotProduct(m_airstation_velocities[i], unit_chordwise_vec);

		m_airstations[i].SetTangentVelocity(total_tan_vel);
	}
}





void lifting_line::TotalForceMomentCalc()
{
	m_total_forces = { 0, 0, 0 };
	m_total_moments_about_point = { 0, 0, 0 };

	geometric_math math;

	double lift_mag;
	double drag_mag;
	double pitching_moment_mag;
	double phi;

	vector<double> unit_norm_vec;
	vector<double> unit_chordwise_vec;
	vector<double> unit_spanwise_vec;

	vector<double> lift_vec;
	vector<double> drag_vec;
	vector<double> pitching_moment_vec;

	vector<double> moment_arm;

	for (int i = 0; i < m_number_of_airstations; i++)
	{
		// First find the loads in the airstation frame and then grab them
		m_airstations[i].loads_airstation_frame_calc();
		lift_mag = m_airstations[i].GetLift();

		/*
		cout << "Tangent velocity: " << m_airstations[i].GetTangentVelocity() << endl;
		cout << "Normal velocity: " << m_airstations[i].GetNormalVelocity() << endl;
		cout << "Cl:" << m_airstations[i].GetCl() << endl;
		cout << "AoA:" << m_airstations[i].GetAoA() << endl;
		cout << "Phi:" << m_airstations[i].GetPhi() << endl;
		cout << "Geometric angle: " << m_airstations[i].GetGeometricAngle() << endl;
		cout << "Reference area:" << m_airstations[i].GetReferenceArea() << endl;
		cout << "Lift mag:" << lift_mag << endl;
		*/

		drag_mag = m_airstations[i].GetDrag();
		pitching_moment_mag = m_airstations[i].GetPitchingMoment();

		// All specified in the lifting line frame
		unit_norm_vec = math.UnitVector(m_airstation_normal_vectors[i]);
		unit_chordwise_vec = math.UnitVector(m_airstation_chordwise_vectors[i]);
		unit_spanwise_vec = math.UnitVector(math.CrossProduct3D(unit_chordwise_vec, unit_norm_vec)); // spanwise vec is the cross product between the chordwise and normal vectors

		// Grab phi from the airstation
		phi = m_airstations[i].GetPhi();

		// Lift is in the direction of the normal vector rotated by phi,  so add lift * (unit_norm_vec rotated by phi about the spanwise vec) to the total forces
		lift_vec = math.Scalar_x_Vector(lift_mag, math.RotateVector(unit_norm_vec, unit_spanwise_vec, phi));
		m_total_forces = math.SumVectors(m_total_forces, lift_vec);


		// Drag is in the opposite direction of the chordwise vector so add drag * -(unit_chordwise_vec rotated by phi about the spanwise vec) to the total forces
		drag_vec = math.Scalar_x_Vector(drag_mag, math.Scalar_x_Vector(-1, math.RotateVector(unit_chordwise_vec, unit_spanwise_vec, phi)));
		m_total_forces = math.SumVectors(m_total_forces, drag_vec);


		// Pitching moment is about the local spanwise vector so add pitching moment * unit_spanwise_vec
		pitching_moment_vec = math.Scalar_x_Vector(pitching_moment_mag, unit_spanwise_vec);
		m_total_moments_about_point = math.SumVectors(m_total_moments_about_point, pitching_moment_vec);


		// Now we need to add in the moments from the lift and drag
		// Moment arm is the airstation position - moment point (in the lifting line frame)
		moment_arm = math.SumVectors(m_airstation_positions[i], math.Scalar_x_Vector(-1, m_moment_point));


		// Moment is defined as r x F, so add lift and drag moments to total moments
		m_total_moments_about_point = math.SumVectors(m_total_moments_about_point, math.CrossProduct3D(moment_arm, lift_vec));

		m_total_moments_about_point = math.SumVectors(m_total_moments_about_point, math.CrossProduct3D(moment_arm, drag_vec));
	}

	// Reports total forces and moments in the lifting line frame
	//cout << endl << "Total Forces:    X: " << m_total_forces[0] << "    Y: " << m_total_forces[1] << "    Z: " << m_total_forces[2] << endl;
	//cout << "Total Moments:    X: " << m_total_moments_about_point[0] << "    Y: " << m_total_moments_about_point[1] << "    Z: " << m_total_moments_about_point[2] << endl << endl;
}






// Get functions for the calculated force and moment terms
vector<double> lifting_line::GetTotalForces()
{
	return m_total_forces;
}

vector<double> lifting_line::GetTotalMomentsAboutPoint()
{
	return m_total_moments_about_point;
}












// GEOMETRY SETUP
// Functions for root and tip chord definitions
void lifting_line::SetRootChordLength(double root_chord_length)
{
	m_root_chord_length = root_chord_length;
}


double lifting_line::GetRootChordLength()
{
	return m_root_chord_length;
}


void lifting_line::SetTipChordLength(double tip_chord_length)
{
	m_tip_chord_length = tip_chord_length;
}


double lifting_line::GetTipChordLength()
{
	return m_tip_chord_length;
}


void lifting_line::SetRootChordTwist(double root_chord_twist)
{
	m_root_chord_twist = root_chord_twist;
}


double lifting_line::GetRootChordTwist()
{
	return m_root_chord_twist;
}


void lifting_line::SetTipChordTwist(double tip_chord_twist)
{
	m_tip_chord_twist = tip_chord_twist;
}


double lifting_line::GetTipChordTwist()
{
	return m_tip_chord_twist;
}


void lifting_line::SetRootQuarterChordPosition(vector<double> root_quarter_chord_position)
{
	m_root_quarter_chord_position = root_quarter_chord_position;
}


vector<double> lifting_line::GetRootQuarterChordPosition()
{
	return m_root_quarter_chord_position;
}


void lifting_line::SetTipQuarterChordPosition(vector<double> tip_quarter_chord_position)
{
	m_tip_quarter_chord_position = tip_quarter_chord_position;
}


vector<double> lifting_line::GetTipQuarterChordPosition()
{
	return m_tip_quarter_chord_position;
}


void lifting_line::SetRootChordChordwiseVector(vector<double> rc_chordwise_vec)
{
	m_root_chord_chordwise_vector = rc_chordwise_vec;
}


vector<double> lifting_line::GetRootChordChordwiseVector()
{
	return m_root_chord_chordwise_vector;
}


void lifting_line::SetRootChordNormalVector(vector<double> rc_normal_vec)
{
	m_root_chord_normal_vector = rc_normal_vec;
}


vector<double> lifting_line::GetRootChordNormalVector()
{
	return m_root_chord_normal_vector;
}


void lifting_line::SetTipChordChordwiseVector(vector<double> tc_chordwise_vec)
{
	m_tip_chord_chordwise_vector = tc_chordwise_vec;
}


vector<double> lifting_line::GetTipChordChordwiseVector()
{
	return m_tip_chord_chordwise_vector;
}


void lifting_line::SetTipChordNormalVector(vector<double> tc_normal_vec)
{
	m_tip_chord_normal_vector = tc_normal_vec;
}


vector<double> lifting_line::GetTipChordNormalVector()
{
	return m_tip_chord_normal_vector;
}


// Functions for airstation geometric definitions
void lifting_line::SetAirstationPositions(vector<vector<double>> airstation_positions)
{
	m_airstation_positions.resize(m_number_of_airstations);
	for (int i = 0; i < m_number_of_airstations; i++)
	{
		m_airstation_positions[i] = airstation_positions[i];
	}
}


vector<double> lifting_line::GetAirstationPosition(int airstation_index)
{
	return m_airstation_positions[airstation_index];
}


void lifting_line::SetAirstationChordwiseVectors(vector<vector<double>> airstation_chordwise_vectors)
{
	m_airstation_chordwise_vectors.resize(m_number_of_airstations);
	for (int i = 0; i < m_number_of_airstations; i++)
	{
		m_airstation_chordwise_vectors[i] = airstation_chordwise_vectors[i];
	}
}


vector<double> lifting_line::GetAirstationChordwiseVector(int airstation_index)
{
	return m_airstation_chordwise_vectors[airstation_index];
}


void lifting_line::SetAirstationNormalVectors(vector<vector<double>> airstation_normal_vectors)
{
	m_airstation_normal_vectors.resize(m_number_of_airstations);
	for (int i = 0; i < m_number_of_airstations; i++)
	{
		m_airstation_normal_vectors[i] = airstation_normal_vectors[i];
	}
}


vector<double> lifting_line::GetAirstationNormalVector(int airstation_index)
{
	return m_airstation_normal_vectors[airstation_index];
}


void lifting_line::SetAirstationTwistAngles(vector<double> airstation_twist_angles)
{
	m_airstation_twist_angles.resize(m_number_of_airstations);
	for (int i = 0; i < m_number_of_airstations; i++)
	{
		m_airstation_twist_angles[i] = airstation_twist_angles[i];
	}
}


double lifting_line::GetAirstationTwistAngle(int airstation_index)
{
	return m_airstation_twist_angles[airstation_index];
}


void lifting_line::SetAirstationChordLengths(vector<double> chord_lengths)
{
	m_airstation_chord_lengths.resize(m_number_of_airstations);
	for (int i = 0; i < m_number_of_airstations; i++)
	{
		m_airstation_chord_lengths[i] = chord_lengths[i];
	}
}


double lifting_line::GetAirstationChordLength(int airstation_index)
{
	return m_airstation_chord_lengths[airstation_index];
}


void lifting_line::CalcTwistedVectors()
{
	m_airstation_twisted_chordwise_vectors.resize(m_number_of_airstations);
	m_airstation_twisted_normal_vectors.resize(m_number_of_airstations);

	vector<double> rotation_axis;
	double twist_angle;

	geometric_math math;

	// Untwist all the airstations
	for (int i = 0; i < m_number_of_airstations; i++)
	{
		rotation_axis = math.CrossProduct3D(m_airstation_chordwise_vectors[i], m_airstation_normal_vectors[i]); // rotate by theta about the cross product between the specified chordwise and normal vectors
		twist_angle = m_airstation_twist_angles[i];

		m_airstation_twisted_chordwise_vectors[i] = math.RotateVector(m_airstation_chordwise_vectors[i], rotation_axis, twist_angle);
		m_airstation_twisted_normal_vectors[i] = math.RotateVector(m_airstation_normal_vectors[i], rotation_axis, twist_angle);
	}

	// Untwist the root chord
	rotation_axis = math.CrossProduct3D(m_root_chord_chordwise_vector, m_root_chord_normal_vector);
	twist_angle = m_root_chord_twist;

	m_root_chord_twisted_chordwise_vector = math.RotateVector(m_root_chord_chordwise_vector, rotation_axis, twist_angle);
	m_root_chord_twisted_normal_vector = math.RotateVector(m_root_chord_normal_vector, rotation_axis, twist_angle);


	//Untwist the tip chord
	rotation_axis = math.CrossProduct3D(m_tip_chord_chordwise_vector, m_tip_chord_normal_vector);
	twist_angle = m_tip_chord_twist;

	m_tip_chord_twisted_chordwise_vector = math.RotateVector(m_tip_chord_chordwise_vector, rotation_axis, twist_angle);
	m_tip_chord_twisted_normal_vector = math.RotateVector(m_tip_chord_normal_vector, rotation_axis, twist_angle);
}


vector<double> lifting_line::GetTwistedChordwiseVector(int airstation_index)
{
	return m_airstation_twisted_chordwise_vectors[airstation_index];
}


vector<double> lifting_line::GetTwistedNormalVector(int airstation_index)
{
	return m_airstation_twisted_normal_vectors[airstation_index];
}


void lifting_line::CalcLEs_TEs()
{
	m_LE_positions.resize(m_number_of_airstations);
	m_TE_positions.resize(m_number_of_airstations);

	// Calculate the LEs and TEs for the airstations
	for (int i = 0; i < m_number_of_airstations; i++)
	{
		
		m_LE_positions[i].resize(m_airstation_positions[i].size());
		m_TE_positions[i].resize(m_airstation_positions[i].size());
		
		
		for (int j = 0; j < m_airstation_positions[i].size(); j++)
		{
			m_LE_positions[i][j] = m_airstation_positions[i][j] + 0.25 * m_airstation_twisted_chordwise_vectors[i][j] * m_airstation_chord_lengths[i];   // assumes that the airstations are located at the quarter chord position
			m_TE_positions[i][j] = m_airstation_positions[i][j] - 0.75 * m_airstation_twisted_chordwise_vectors[i][j] * m_airstation_chord_lengths[i];
		}

		//cout << "LE pos:    X: " << m_LE_positions[i][0] << "    Y: " << m_LE_positions[i][1] << "    Z: " << m_LE_positions[i][2] << endl;
	}


	// Calc the LEs and TEs for the root and tip chords
	// First, ensure that the vectors are of the right size
	m_RC_LE_pos.resize(m_root_quarter_chord_position.size());
	m_RC_TE_pos.resize(m_root_quarter_chord_position.size());

	m_TC_LE_pos.resize(m_tip_quarter_chord_position.size());
	m_TC_TE_pos.resize(m_tip_quarter_chord_position.size());

	for (int i = 0; i < m_root_quarter_chord_position.size(); i++)
	{
		m_RC_LE_pos[i] = m_root_quarter_chord_position[i] + 0.25 * m_root_chord_twisted_chordwise_vector[i] * m_root_chord_length;
		m_RC_TE_pos[i] = m_root_quarter_chord_position[i] - 0.75 * m_root_chord_twisted_chordwise_vector[i] * m_root_chord_length;

		m_TC_LE_pos[i] = m_tip_quarter_chord_position[i] + 0.25 * m_tip_chord_twisted_chordwise_vector[i] * m_tip_chord_length;
		m_TC_TE_pos[i] = m_tip_quarter_chord_position[i] - 0.75 * m_tip_chord_twisted_chordwise_vector[i] * m_tip_chord_length;
	}
}


void lifting_line::CalcAirstationCorners()
{
	geometric_math math;

	vector<double> point_A;
	vector<double> point_B;
	vector<double> point_C;
	vector<double> point_D;

	m_airstation_corners.resize(m_number_of_airstations);

	for (int i = 0; i < m_number_of_airstations; i++)
	{
		// Define the first airstation which incorporates the root chord
		if (i == 0)
		{
			point_A = m_RC_LE_pos;
			point_B = m_RC_TE_pos;
			
			point_C = math.MidPoint(m_TE_positions[i], m_TE_positions[i + 1]);
			point_D = math.MidPoint(m_LE_positions[i], m_LE_positions[i + 1]);
		}

		// Define the last airstation which incorporates the tip chord
		else if (i == m_number_of_airstations - 1)
		{
			point_A = point_D;
			point_B = point_C;

			point_C = m_TC_TE_pos;
			point_D = m_TC_LE_pos;
		}

		// Define all the other airstations in the middle
		else
		{
			point_A = point_D;
			point_B = point_C;

			point_C = math.MidPoint(m_TE_positions[i], m_TE_positions[i + 1]);
			point_D = math.MidPoint(m_LE_positions[i], m_LE_positions[i + 1]);
		}

		// Set the corners of each airstation
		m_airstation_corners[i] = { point_A, point_B, point_C, point_D };
	}
}


void lifting_line::CalcAirstationReferenceAreas()
{

	geometric_math math;

	m_airstation_reference_areas.resize(m_number_of_airstations);

	vector<double> point_A;
	vector<double> point_B;
	vector<double> point_C;
	vector<double> point_D;

	m_total_reference_area = 0.0;

	for (int i = 0; i < m_number_of_airstations; i++)
	{
		point_A = m_airstation_corners[i][0];
		point_B = m_airstation_corners[i][1];
		point_C = m_airstation_corners[i][2];
		point_D = m_airstation_corners[i][3];


		// Reference area is the sum of the ABC and ADC triangluar areas for each airstation. Note that the area is being broken up into two triangles as the 4 points defining the corners of the airstation do not necessarily need to be in the same plane
		m_airstation_reference_areas[i] = math.Triangle3DArea(point_A, point_B, point_C) + math.Triangle3DArea(point_A, point_D, point_C);

		// Set the reference area for the airstation
		m_airstations[i].SetReferenceArea(m_airstation_reference_areas[i]);

		// Just for keeping track of. May be useful for calculation of parameters when multiple lifting lines are involved or for things such as the aspect ratio
		m_total_reference_area += m_airstation_reference_areas[i];
	}
}


double lifting_line::GetTotalReferenceArea()
{
	return m_total_reference_area;
}


vector<vector<double>> lifting_line::GetAirstationCorners(int airstation_index)
{
	return m_airstation_corners[airstation_index];
}


void lifting_line::GeometrySetup()
{
	CalcTwistedVectors();
	CalcLEs_TEs();
	CalcAirstationCorners();
	CalcAirstationReferenceAreas();
}


// Not necessarily needed after reworking the MATLAB plotting function
/*
void lifting_line::GenerateDataFile(string filename)
{
	ofstream data_file(filename);


	data_file << "QC Position    |    LE Position    |    TE Position" << endl;

	// Root chord 
	data_file << m_root_quarter_chord_position[0] << " " << m_root_quarter_chord_position[1] << " " << m_root_quarter_chord_position[2] << " ";
	data_file << m_RC_LE_pos[0] << " " << m_RC_LE_pos[1] << " " << m_RC_LE_pos[2] << " ";
	data_file << m_RC_TE_pos[0] << " " << m_RC_TE_pos[1] << " " << m_RC_TE_pos[2] << endl;

	// Airstations
	for (int i = 0; i < m_number_of_airstations; i++)
	{
		data_file << m_airstation_positions[i][0] << " " << m_airstation_positions[i][1] << " " << m_airstation_positions[i][2] << " ";
		data_file << m_LE_positions[i][0] << " " << m_LE_positions[i][1] << " " << m_LE_positions[i][2] << " ";
		data_file << m_TE_positions[i][0] << " " << m_TE_positions[i][1] << " " << m_TE_positions[i][2] << endl;
	}

	// Tip chord
	data_file << m_tip_quarter_chord_position[0] << " " << m_tip_quarter_chord_position[1] << " " << m_tip_quarter_chord_position[2] << " ";
	data_file << m_TC_LE_pos[0] << " " << m_TC_LE_pos[1] << " " << m_TC_LE_pos[2] << " ";
	data_file << m_TC_TE_pos[0] << " " << m_TC_TE_pos[1] << " " << m_TC_TE_pos[2] << endl;


	data_file.close();
}
*/


int lifting_line::ReadDataFile(string filename)
{
	ifstream data_file(filename);
	
	
	if (!data_file)
	{
		cerr << "Failed to open the file." << endl;
		return 0;
	}
	

	vector<vector<double>> data_matrix;
	int rows;
	int cols;
	string line;

	bool dimensions_set = false;

	int current_row = 0;
	while (getline(data_file, line))
	{
		// If it is not a comment, read it in
		if (line[0] != '#')
		{
			// Check to see if the dimensions are set for the matrix
			if (dimensions_set == false)
			{
				istringstream iss(line);

				
				// Reads in the rows and columns and outputs an error message if it fails
				if (!(iss >> rows >> cols)) {
					cerr << "Failed to read matrix dimensions." << endl;
					return 0;
				}
				

				data_matrix.resize(rows, vector<double>(cols));
				dimensions_set = true;
			}

			// Read in the matrix at the current line
			else
			{
				istringstream iss(line);

				// Print out the line
				for (int j = 0; j < cols; j++)
				{
					// Reads in the current element and outputs an error message if it fails
					if (!(iss >> data_matrix[current_row][j]))
					{
						cerr << "Failed to read in the matrix at the indeces: [" << current_row << ", " << j << "]" << endl;
						return 0;
					}
				}
				current_row++;
			}
		}
	}

	data_file.close();

	// Set all the parameters from the matrix into the private variables here
	SetValuesFromDataArray(data_matrix);

	cout << "I read the matrix in correctly and have set the values for the root/tip chords, and the airstations." << endl;

	// Also set the number of airstations from reading this in so that it may be used later
	int num_as = rows - 2;
	SetNumberOfAirstations(num_as);

	return 1;
}


void lifting_line::SetValuesFromDataArray(vector<vector<double>> data_matrix)
{
	/*
	data_matrix is expected to be an (N+2) x 11 matrix where N is the number of airstations. First row is the root chord data, the last row the tip chord, and everything inbetween is the airstations
	The first 3 columns are the position, the next three the untwisted chordwise vector, the next three the untwisted normal vector, the 10 entry the twist about the axis normal to the plane specified
	by the chordwise and normal vectors, and the last entry is the chord length.
	*/

	// First make sure to allocate the dimensions of the airstation parameter matrices correctly
	int num_as = data_matrix.size() - 2;

	m_airstation_positions.resize(num_as);
	m_airstation_chordwise_vectors.resize(num_as);
	m_airstation_normal_vectors.resize(num_as);
	m_airstation_twist_angles.resize(num_as);
	m_airstation_chord_lengths.resize(num_as);


	// Now iterate over the data matrix to set everything
	for (int i = 0; i < data_matrix.size(); i++)
	{
		// Root chord
		if (i == 0)
		{
			// The the '+ X' term is not included when the second entry -> i.e. the indeces only go to '+ X - 1'. However, when a '+ X' term is in the first entry, then X is the first index 
			m_root_quarter_chord_position = vector<double>(data_matrix[i].begin(), data_matrix[i].begin() + 3);
			m_root_chord_chordwise_vector = vector<double>(data_matrix[i].begin() + 3, data_matrix[i].begin() + 6);
			m_root_chord_normal_vector = vector<double>(data_matrix[i].begin() + 6, data_matrix[i].begin() + 9);
			m_root_chord_twist = data_matrix[i][9];
			m_root_chord_length = data_matrix[i][10];
		}
		// Tip chord
		else if (i == data_matrix.size() - 1)
		{
			m_tip_quarter_chord_position = vector<double>(data_matrix[i].begin(), data_matrix[i].begin() + 3);
			m_tip_chord_chordwise_vector = vector<double>(data_matrix[i].begin() + 3, data_matrix[i].begin() + 6);
			m_tip_chord_normal_vector = vector<double>(data_matrix[i].begin() + 6, data_matrix[i].begin() + 9);
			m_tip_chord_twist = data_matrix[i][9];
			m_tip_chord_length = data_matrix[i][10];
		}
		// Airstations
		else
		{
			// Offset by index by 1 as the index of 0 in this loop is used for the root chord
			m_airstation_positions[i-1] = vector<double>(data_matrix[i].begin(), data_matrix[i].begin() + 3);
			m_airstation_chordwise_vectors[i-1] = vector<double>(data_matrix[i].begin() + 3, data_matrix[i].begin() + 6);
			m_airstation_normal_vectors[i-1] = vector<double>(data_matrix[i].begin() + 6, data_matrix[i].begin() + 9);
			m_airstation_twist_angles[i-1] = data_matrix[i][9];
			m_airstation_chord_lengths[i-1] = data_matrix[i][10];
		}
	}
}



// Overall setup functions
void lifting_line::ComputeAirstationAerodynamicParameters()
{
	for (int i = 0; i < m_number_of_airstations; i++)
	{
		m_airstations[i].GetVelMag();
		m_airstations[i].Compute_AoA();
		m_airstations[i].ComputeAeroCoeffs();
	}
}


void lifting_line::SetAirstationParameters()
{
	for (int i = 0; i < m_number_of_airstations; i++)
	{
		m_airstations[i].SetRho(m_rho_inf);
		m_airstations[i].SetSoS(m_SoS);
		m_airstations[i].SetChord(m_airstation_chord_lengths[i]);
	}
}

// ONLY USE AFTER THE VELOCITIES HAVE BEEN SET FOR EACH AIRSTATION
void lifting_line::SetupAirstations()
{
	SetAirstationParameters();
	ComputeAirstationNormalVelocities();
	ComputeAirstationTangentVelocities();
	ComputeAirstationAerodynamicParameters();
}


void lifting_line::UpdateAirstationsAerodynamics()
{
	ComputeAirstationNormalVelocities();
	ComputeAirstationTangentVelocities();
	ComputeAirstationAerodynamicParameters();
}





void lifting_line::SetMass(double mass)
{
	m_mass = mass;
}

double lifting_line::GetMass()
{
	return m_mass;
}

void lifting_line::SetCenterOfMassPosition(vector<double> cm_pos)
{
	m_cm_pos = cm_pos;
}

vector<double> lifting_line::GetCenterOfMassPosition()
{
	return m_cm_pos;
}

void lifting_line::SetMomentOfInertia_CM(vector<vector<double>> moi_cm)
{
	m_moi_cm = moi_cm;
}

vector<vector<double>> lifting_line::GetMomentOfInertia_CM()
{
	return m_moi_cm;
}


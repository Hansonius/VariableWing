#include "variable_wing.h"
#include "wingbox.h"
#include "lifting_line.h"

#include <iostream>

using namespace std;

variable_wing::variable_wing()
{
	m_phi_dot = { 0, 0, 0 };
	m_phi = { 0, 0, 0 };
	m_delta_phi = 0.0;

	fail_check = false;
}

void variable_wing::SetLiftingLine(lifting_line LL)
{
	m_LL = LL;
	m_as_activation = vector<bool>(m_LL.GetNumberofAirstations(), 1);
}

void variable_wing::SetWingbox(wingbox wb)
{
	m_wb = wb;
}

void variable_wing::SetInitialRootChordAngle(double rc_angle)
{
	m_init_rc_angle = rc_angle;
}

double variable_wing::GetInitialRootChordAngle()
{
	return m_init_rc_angle;
}

double variable_wing::GetRootChordAngle()
{
	// Hard codes Z axis!!!!!
	return m_phi[2];
}


void variable_wing::SetFuselagePlane(vector<double> plane_normal, vector<double> plane_point)
{
	m_fuselage_plane = math.PlaneFromNormalPoint(plane_normal, plane_point);
}


void variable_wing::SetMomentPoint(vector<double> moment_point)
{
	m_moment_point = moment_point;

	// Set the moment point for the lifting line and the wingbox
	m_LL.SetMomentPoint(m_moment_point);
	m_wb.SetMomentPoint(m_moment_point);


}


void variable_wing::AdvanceTime(double dt)
{
	// Advance the wingbox by a certain timestep
	// -> pass in the dt for the dashpots and the delta_phi for the springs?
	m_wb.AdvanceTime(dt, m_delta_phi);

	fail_check = m_wb.fail_check;

	//cout << "After advancing wb" << endl;

	// Update Root Anlge
	UpdateRCAngle(dt);

	// Advance the lifting line geometry checks
	UpdateLiftingLine();

	// Updates the airstation activation vector
	CheckAirstationActivation();

	// Update the freestream velocities at the airstations
	UpdateFlow();

	// Update the aerodynamics and get the new forces and moments
	m_LL.UpdateAirstationsAerodynamics();

	// Manually write the aerodynamic coefficients of the airstaitons that are off to have zero Cl, Cd, Cm
	UpdateAirstationActivation();

	m_LL.TotalForceMomentCalc();

	

	
}

void variable_wing::UpdateRCAngle(double dt)
{
	// M_net = I_tot * d^2 phi / dt^2
	// Should we just assume about the Z-axis only?

	// Net moment about the point
	//math.PrintMatrix({ m_LL.GetTotalMomentsAboutPoint() }, "LL Moment");
	//math.PrintMatrix({ m_wb.GetTotalMoment() }, "WB Moment");

	m_net_moment = math.SumVectors(m_LL.GetTotalMomentsAboutPoint(), m_wb.GetTotalMoment());
	//math.PrintMatrix({ m_net_moment }, "Net Moment");

	// Finding the moment of inertia of the lifting line about the moment point
	vector<double> moi_arm = math.SumVectors(m_LL.GetCenterOfMassPosition(), math.Scalar_x_Vector(-1.0, m_moment_point));
	vector<vector<double>> off_axis_moi = math.Scalar_x_Matrix(-m_LL.GetMass(), math.Matrix_x_Matrix(math.CrossProductEquivalent(moi_arm), math.CrossProductEquivalent(moi_arm)));
	
	

	//math.PrintMatrix(off_axis_moi, "Off Axis MOI");

	vector<vector<double>> LL_moi_0 = math.SumMatrices(m_LL.GetMomentOfInertia_CM(), off_axis_moi);
	
	m_total_moment_of_inertia = math.SumMatrices(LL_moi_0, m_wb.GetMomentOfInertia());

	//math.PrintMatrix(math.Invert(m_total_moment_of_inertia), "Inverse MOI");



	// Euler's method. Ewww!
	m_phi_2_dot = math.Matrix_x_Vector(math.Invert(m_total_moment_of_inertia), m_net_moment);

	//math.PrintMatrix({ m_phi_2_dot }, "phi 2 dot");

	m_phi_dot = math.SumVectors(m_phi_dot, math.Scalar_x_Vector(dt, m_phi_2_dot));

	m_phi = math.SumVectors(m_phi, math.Scalar_x_Vector(dt, m_phi_dot));


	// ******Used for the rotations of the lifting line and wingbox******
	// Really bad as it hard codes the Z axis
	//m_delta_phi = math.Scalar_x_Vector(dt, m_phi_dot)[2];
	
	m_delta_phi = math.DotProduct(math.Scalar_x_Vector(dt, m_phi_dot), m_wb.GetRotationAxis()); // instead try getting the angle this way, although the math could be wrong here?
	//cout << "Delta phi: " << m_delta_phi << endl;

	//math.PrintMatrix({ m_phi_dot }, "Phi_dot");
	//math.PrintMatrix({ m_phi }, "Phi");
}


void variable_wing::UpdateLiftingLine()
{
	vector<double> pos;
	vector<double> cw_vec;
	vector<double> norm_vec;
	vector<double> z_axis = { 0.0, 0.0, 1.0 };

	vector<vector<double>> as_pos(m_LL.GetNumberofAirstations(), vector<double>(3, 0.0));
	vector<vector<double>> as_cw_vecs(m_LL.GetNumberofAirstations(), vector<double>(3, 0.0));
	vector<vector<double>> as_norm_vecs(m_LL.GetNumberofAirstations(), vector<double>(3, 0.0));

	// Sweep back the airstations by a rotation of the displacement
	for (int i = 0; i < m_LL.GetNumberofAirstations(); i++)
	{
		pos = m_LL.GetAirstationPosition(i);
		cw_vec = m_LL.GetAirstationChordwiseVector(i);
		norm_vec = m_LL.GetAirstationNormalVector(i);
		
		// Not disp, but delta disp
		pos = math.RotateVector(pos, z_axis, m_delta_phi);
		cw_vec = math.RotateVector(cw_vec, z_axis, m_delta_phi);
		norm_vec = math.RotateVector(norm_vec, z_axis, m_delta_phi);

		as_pos[i] = pos;
		as_cw_vecs[i] = cw_vec;
		as_norm_vecs[i] = norm_vec;
	}

	m_LL.SetAirstationPositions(as_pos);
	m_LL.SetAirstationChordwiseVectors(as_cw_vecs);
	m_LL.SetAirstationNormalVectors(as_norm_vecs);



	// Find the center of mass again
	vector<double> center_of_mass_pos = math.RotateVector(m_LL.GetCenterOfMassPosition(), z_axis, m_delta_phi);
	m_LL.SetCenterOfMassPosition(center_of_mass_pos);

	// Rotate the moment of inertia matrix as well
	vector<vector<double>> rotation_matrix = math.EulerAngleRotationMatrix({ 0.0, 0.0, m_delta_phi }, "XYZ");
	//math.PrintMatrix(rotation_matrix, "Full Euler Angle Rotation Matrix");

	m_LL.SetMomentOfInertia_CM(math.Matrix_x_Matrix(rotation_matrix, m_LL.GetMomentOfInertia_CM()));


	//math.PrintMatrix(m_LL.GetMomentOfInertia_CM(), "Wing MOI");
	//math.PrintMatrix({ center_of_mass_pos }, "LL COM Pos");
}

void variable_wing::CheckAirstationActivation()
{
	// We check if the airstations are in the flow by looking if the airstation's position is on the left or right hand side of the plane 
	vector<double> pos;
	bool plane_check;
	for (int i = 0; i < m_LL.GetNumberofAirstations(); i++)
	{
		pos = m_LL.GetAirstationPosition(i);
		plane_check = math.CheckPointPlaneSide(m_fuselage_plane, pos);
		
		// Update whether or not the airstations will be active based on their position
		m_as_activation[i] = plane_check;
	}
}


void variable_wing::UpdateAirstationActivation()
{
	airstation* as;
	for (int i = 0; i < m_LL.GetNumberofAirstations(); i++)
	{
		if (m_as_activation[i] == 0)
		{
			as = m_LL.GetAirstation(i);
			as->SetAeroCoeffs(0.0, 0.0, 0.0);
		}
	}
}


void variable_wing::UpdateFlow()
{
	// Update the freestreams of each of the airstations
	for (int i = 0; i < m_LL.GetNumberofAirstations(); i++)
	{
		m_LL.SetAirstationFreestream(m_freestream, i);
	}
}



void variable_wing::SetFreestreamVelocity(vector<double> freestream)
{
	m_freestream = freestream;
}


void variable_wing::Setup()
{
	m_wb.Setup();

	m_LL.GeometrySetup();


	for (int i = 0; i < m_LL.GetNumberofAirstations(); i++)
	{
		m_LL.SetAirstationFreestream(m_freestream, i);
	}

	m_LL.SetupAirstations();

	m_LL.TotalForceMomentCalc();

	math.PrintMatrix({ m_LL.GetTotalForces() }, "Total Forces at t = 0");
}







vector<double> variable_wing::GetPhi()
{
	return m_phi;
}

vector<double> variable_wing::GetPhiDot()
{
	return m_phi_dot;
}

vector<double> variable_wing::GetPhi2Dot()
{
	return m_phi_2_dot;
}

int variable_wing::GetActiveAirstationCount()
{
	// Output how many airstations are activated at any one time
	int active_as_count = 0;
	for (int i = 0; i < m_LL.GetNumberofAirstations(); i++)
	{
		active_as_count += m_as_activation[i];
	}

	return active_as_count;
}


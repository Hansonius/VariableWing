#include "wingbox.h"
#include "linear_spring.h"
#include "dashpot.h"
#include "geometric_math.h"

#include <iostream>

using namespace std;

wingbox::wingbox()
{
	m_ls_wall_positions = { {0.0, 0.0, 0.0} };
	m_ls_wing_positions = { {0.0, 0.0, 0.0} };

	m_dp_positions = { {0.0, 0.0, 0.0} };
	m_ts_positions = { {0.0, 0.0, 0.0} };

	m_ls_axes = { {0.0, 0.0, 0.0} };
	m_dp_axes = { {0.0, 0.0, 0.0} };
	m_ts_axes = { {0.0, 0.0, 0.0} };

	fail_check = false;
}


void wingbox::SetLinearSprings(vector<linear_spring> linear_springs)
{
	m_lin_springs = linear_springs;
	m_ls_count = m_lin_springs.size();

	math.ResizeMatrix(m_ls_wall_positions, m_ls_count, 3);
	math.ResizeMatrix(m_ls_wing_positions, m_ls_count, 3);
	math.ResizeMatrix(m_ls_axes, m_ls_count, 3);
}

linear_spring* wingbox::GetLinearSpring(int linear_spring_index)
{
	return &m_lin_springs[linear_spring_index];
}

void wingbox::SetDashpots(vector<dashpot> dashpots)
{
	m_dashpots = dashpots;
	m_dp_count = m_dashpots.size();

	math.ResizeMatrix(m_dp_positions, m_dp_count, 3);
	math.ResizeMatrix(m_dp_axes, m_dp_count, 3);
}

dashpot* wingbox::GetDashpot(int dashpot_index)
{
	return &m_dashpots[dashpot_index];
}

void wingbox::SetTorsionSprings(vector<torsion_spring> torsion_springs)
{
	m_tor_springs = torsion_springs;
	m_ts_count = m_tor_springs.size();

	math.ResizeMatrix(m_ts_positions, m_ts_count, 3);
	math.ResizeMatrix(m_ts_axes, m_ts_count, 3);
}

torsion_spring* wingbox::GetTorsionSpring(int torsion_spring_index)
{
	return &m_tor_springs[torsion_spring_index];
}



void wingbox::SetLinearSpringWallPosition(vector<double> position, int linear_spring_index)
{
	m_ls_wall_positions[linear_spring_index] = position;
}

vector<double> wingbox::GetLinearSpringWallPosition(int linear_spring_index)
{
	return m_ls_wall_positions[linear_spring_index];
}

void wingbox::SetDashpotPosition(vector<double> position, int dashpot_index)
{
	m_dp_positions[dashpot_index] = position;
}

vector<double> wingbox::GetDashpotPosition(int dashpot_index)
{
	return m_dp_positions[dashpot_index];
}

void wingbox::SetTorsionSpringPosition(vector<double> position, int torsion_spring_index)
{
	m_ts_positions[torsion_spring_index] = position;
}

vector<double> wingbox::GetTorsionSpringPosition(int torsion_spring_index)
{
	return m_ts_positions[torsion_spring_index];
}

void wingbox::SetLinearSpringOrientation(vector<double> orientation, int linear_spring_index)
{
	m_ls_axes[linear_spring_index] = math.UnitVector(orientation);
}

vector<double> wingbox::GetLinearSpringOrientation(int linear_spring_index)
{
	return m_ls_axes[linear_spring_index];
}

void wingbox::SetDashpotOrientation(vector<double> orientation, int dashpot_index)
{
	m_dp_axes[dashpot_index] = math.UnitVector(orientation);
}

vector<double> wingbox::GetDashpotOrientation(int dashpot_index)
{
	return m_dp_axes[dashpot_index];
}

void wingbox::SetTorsionSpringOrientation(vector<double> orientation, int torsion_spring_index)
{
	m_ts_axes[torsion_spring_index] = math.UnitVector(orientation);
}

vector<double> wingbox::GetTorsionSpringOrientation(int torsion_spring_index)
{
	return m_ts_axes[torsion_spring_index];
}

void wingbox::SetMomentPoint(vector<double> rotation_point)
{
	m_rotation_point = rotation_point;
}


void wingbox::SetRotationAxis(vector<double> rotation_axis)
{
	m_rotation_axis = rotation_axis;
}

vector<double> wingbox::GetRotationAxis()
{
	return m_rotation_axis;
}



bool wingbox::CheckFailure()
{
	for (int i = 0; i < m_ls_count; i++)
	{
		if (m_lin_springs[i].CheckFailure())
		{
			cout << "Linear spring " << i << " failed." << endl;
			return true;
		}
	}

	for (int i = 0; i < m_dp_count; i++)
	{
		if (m_dashpots[i].CheckFailure())
		{
			cout << "Dashpot " << i << " failed." << endl;
			return true;
		}
	}

	for (int i = 0; i < m_ts_count; i++)
	{
		if (m_tor_springs[i].CheckFailure())
		{
			cout << "Torsion spring " << i << " failed." << endl;
			return true;
		}
	}
	return false;
}







void wingbox::UpdateLinearSpring(int ls_index, double delta_phi)
{
	// Find the new displacement for the spring
	linear_spring* ls = GetLinearSpring(ls_index);

	// Update the wing attachment point location
	vector<double> wing_attach_arm = math.SumVectors(m_ls_wing_positions[ls_index], math.Scalar_x_Vector(-1.0, m_rotation_point));
	vector<double> rotated_wing_arm = math.RotateVector(wing_attach_arm, m_rotation_axis, delta_phi);
	m_ls_wing_positions[ls_index] = math.SumVectors(m_rotation_point, rotated_wing_arm);

	// Update the spring displacement
	vector<double> spring_arm = math.SumVectors(m_ls_wing_positions[ls_index], math.Scalar_x_Vector(-1.0, m_ls_wall_positions[ls_index]));
	double spring_length = math.VectorNorm(spring_arm, 2);
	double new_disp = spring_length - ls->GetUnstretchedLength();
	ls->SetDisplacement(new_disp);
}


void wingbox::UpdateDashpot(int dp_index, double dt, double delta_phi)
{
	// Find the new velocity of the dashpot
	dashpot* dp = GetDashpot(dp_index);

	// If the dp is a distance R from the rotation point, then R * delta_phi gives the distance that's being travelled in a time dt
	// Seems at least partially wrong, but good enough for now. Also, doesn't account for the fact that the dashpot position can change
	vector<double> dp_arm = math.SumVectors(m_dp_positions[dp_index], math.Scalar_x_Vector(-1.0, m_rotation_point));

	// Find the effect that the rotation has on the damper
	vector<double> v_dir = math.CrossProduct3D(m_dp_axes[dp_index], dp_arm);

	v_dir = math.Scalar_x_Vector(1.0 / math.VectorNorm(v_dir, 2), v_dir);

	double r_mag = math.VectorNorm(dp_arm, 2);

	vector<double> vel = math.Scalar_x_Vector(r_mag * delta_phi / dt, v_dir);

	double new_vel = math.VectorNorm(vel, 2);


	// REALLY STUPID WAY TO DO THIS
	if (delta_phi > 0)
	{
		new_vel *= -1.0;
	}

	dp->SetVelocity(new_vel);
}


void wingbox::UpdateTorsionSpring(int ts_index, double delta_phi)
{
	// Update new angle
	torsion_spring* ts = GetTorsionSpring(ts_index);

	// Check how aligned the axis of the torsion spring and the rotation axis are
	double axis_sim = math.DotProduct(m_ts_axes[ts_index], m_rotation_axis);

	// Just hard code the displacement being perfectly in line with the axis of the spring for now
	double new_disp = (delta_phi - ts->GetRelaxedArmAngle()) + ts->GetDisplacement();

	ts->SetDisplacement(new_disp);
}


void wingbox::UpdateMomentOfInertia()
{
	vector<vector<double>> new_moi(3, vector<double>(3, 0.0));

	double mass;
	vector<double> pos(3, 0.0);
	vector<double> moment_arm(3, 0.0);

	vector<double> rot_point_term = math.Scalar_x_Vector(-1.0, m_rotation_point);
	vector<vector<double>> cpe;
	vector<vector<double>> r2_mat;

	// Treat every item within the wingbox as a point mass and find out where they are relative to the moment point
	// Parallel Axis theorem: J0 = Jcm - m(rx)^2 -> here just assume that Jcm is small
	
	vector<double> spring_arm;
	vector<double> ls_cm_pos;
	for (int i = 0; i < m_ls_count; i++)
	{
		mass = GetLinearSpring(i)->GetMass();
		pos = m_ls_wall_positions[i];

		// How to find the center of mass of the spring?
		spring_arm = math.SumVectors(m_ls_wing_positions[i], math.Scalar_x_Vector(-1.0, pos));
		ls_cm_pos = math.SumVectors(pos, math.Scalar_x_Vector(0.5, spring_arm));

		moment_arm = math.SumVectors(ls_cm_pos, rot_point_term);
		cpe = math.CrossProductEquivalent(moment_arm);
		r2_mat = math.Matrix_x_Matrix(cpe, cpe);
		new_moi = math.SumMatrices(new_moi, math.Scalar_x_Matrix(-mass, r2_mat));
	}

	for (int i = 0; i < m_dp_count; i++)
	{
		mass = GetDashpot(i)->GetMass();
		pos = m_dp_positions[i];

		moment_arm = math.SumVectors(pos, rot_point_term);

		cpe = math.CrossProductEquivalent(moment_arm);
		r2_mat = math.Matrix_x_Matrix(cpe, cpe);

		new_moi = math.SumMatrices(new_moi, math.Scalar_x_Matrix(-mass, r2_mat));
	}

	vector<vector<double>> Jcm;
	double ts_moi;
	for (int i = 0; i < m_ts_count; i++)
	{
		mass = GetTorsionSpring(i)->GetMass();
		pos = m_ts_positions[i];

		moment_arm = math.SumVectors(pos, rot_point_term);

		cpe = math.CrossProductEquivalent(moment_arm);
		r2_mat = math.Matrix_x_Matrix(cpe, cpe);

		ts_moi = GetTorsionSpring(i)->GetMomentOfInertia();
		Jcm = math.Scalar_x_Matrix(ts_moi, math.Diagonalize(m_ts_axes[i]));
		new_moi = math.SumMatrices(new_moi, Jcm);
		new_moi = math.SumMatrices(new_moi, math.Scalar_x_Matrix(-mass, r2_mat));
	}

	m_moment_of_inertia = new_moi;
}


void wingbox::UpdateWingboxMoment()
{
	// Find the moment from all the components in the wingbox about the rotation point
	double force_mag;

	vector<double> wb_total_moment(3, 0.0);

	vector<double> pos(3, 0.0);
	vector<double> moment_arm(3, 0.0);
	vector<double> rot_point_term = math.Scalar_x_Vector(-1.0, m_rotation_point);
	vector<double> moment(3, 0.0);
	vector<double> force_vec(3, 0.0);
	
	vector<double> ls_spring_arm;
	for (int i = 0; i < m_ls_count; i++)
	{
		force_mag = GetLinearSpring(i)->GetForce();

		//cout << "Spring disp: " << GetLinearSpring(i)->GetDisplacement() << endl;
		//cout << "Force mag: " << force_mag << endl;

		// How to find the center of mass of the spring?
		ls_spring_arm = math.SumVectors(m_ls_wing_positions[i], math.Scalar_x_Vector(-1.0, m_ls_wall_positions[i]));
		pos = math.SumVectors(m_ls_wall_positions[i], math.Scalar_x_Vector(0.5, ls_spring_arm));

		force_vec = math.Scalar_x_Vector(force_mag, m_ls_axes[i]);

		moment_arm = math.SumVectors(pos, rot_point_term);
		moment = math.CrossProduct3D(moment_arm, force_vec);

		wb_total_moment = math.SumVectors(wb_total_moment, moment);

		//math.PrintMatrix({ force_vec }, "LS Force");
		//math.PrintMatrix({ moment }, "LS Moment");
	}

	for (int i = 0; i < m_dp_count; i++)
	{
		pos = m_dp_positions[i];
		force_mag = GetDashpot(i)->GetForce();
		force_vec = math.Scalar_x_Vector(force_mag, m_dp_axes[i]);

		moment_arm = math.SumVectors(pos, rot_point_term);

		moment = math.CrossProduct3D(moment_arm, force_vec);

		wb_total_moment = math.SumVectors(wb_total_moment, moment);

		//math.PrintMatrix({ force_vec }, "DP Force");
		//math.PrintMatrix({ moment }, "DP Moment");
	}


	double moment_mag;
	for (int i = 0; i < m_ts_count; i++)
	{
		moment_mag = GetTorsionSpring(i)->GetMoment();
		moment = math.Scalar_x_Vector(moment_mag, m_ts_axes[i]);

		wb_total_moment = math.SumVectors(wb_total_moment, moment);

		//math.PrintMatrix({ moment }, "TS Moment");
	}

	

	m_wb_total_moment = wb_total_moment;
}



vector<double> wingbox::GetTotalMoment()
{
	return m_wb_total_moment;
}


vector<vector<double>> wingbox::GetMomentOfInertia()
{
	return m_moment_of_inertia;
}





void wingbox::Setup()
{
	for (int i = 0; i < m_ls_count; i++)
	{
		m_lin_springs[i].Setup();

		// Now make sure that we get the attachment points for the springs also
		m_ls_wing_positions[i] = math.SumVectors(m_ls_wall_positions[i], math.Scalar_x_Vector(m_lin_springs[i].GetLength(), m_ls_axes[i]));
	}

	for (int i = 0; i < m_dp_count; i++)
	{
		m_dashpots[i].Setup();
	}

	for (int i = 0; i < m_ts_count; i++)
	{
		m_tor_springs[i].Setup();
	}

	UpdateMomentOfInertia();
}




void wingbox::AdvanceTime(double dt, double delta_phi)
{
	for (int i = 0; i < m_ls_count; i++)
	{
		UpdateLinearSpring(i, delta_phi);
	}

	for (int i = 0; i < m_dp_count; i++)
	{
		UpdateDashpot(i, dt, delta_phi);
	}

	for (int i = 0; i < m_ts_count; i++)
	{
		UpdateTorsionSpring(i, delta_phi);
	}

	UpdateWingboxMoment();

	UpdateMomentOfInertia();

	fail_check = CheckFailure();
}
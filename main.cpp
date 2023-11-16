#include "lifting_line.h"
#include "linear_spring.h"
#include "torsion_spring.h"
#include "wingbox.h"
#include "variable_wing.h"
#include "dashpot.h"
#include "material.h"
#include "geometric_math.h"

#include <iostream>
#include <vector>
#include <cstring>


#include <fstream>

using namespace std;


void write_state(ofstream* file, double time, int active_as_count, vector<double> x, vector<double> x_dot)
{
	*file << time << " " << active_as_count << " ";

	for (int i = 0; i < x.size(); i++)
	{
		*file << x[i] << " ";
	}

	for (int i = 0; i < x_dot.size(); i++)
	{
		*file << x_dot[i] << " ";
	}

	*file << endl;
}



int main()
{
	geometric_math math;

	// Set up the characteristics of steel
	material steel;
	steel.SetDensity(8000.0); // kg/m^3
	steel.SetYoungsModulus(210000000000.0); // Pa
	steel.SetPoissonRatio(0.3);
	steel.SetYieldStrength(1000000000.0); // Pa
	steel.SetShearModulus(79300000000.0); // Pa
	steel.SetShearStrength(0.6 * steel.GetYieldStrength()); // 60% of the tensile strength


	// Set up a linear spring
	linear_spring ls0;
	ls0.SetMaterial(steel);
	ls0.SetActiveCoilCount(25);
	ls0.SetTotalCoilCount(25);
	ls0.SetWireDiameter(0.1); // 1 cm
	ls0.SetCoilDiameter(0.2); // 6ish" in meters
	ls0.SetMaximumLength(2.0); // m
	ls0.SetMinimumLength(0.0);
	ls0.SetRelaxedPitch(0.04); // m



	vector<double> ls0_pos = { 0.0, -0.5, 0.0 };
	vector<double> ls0_axis = { 1.0, 0.0, 0.0 };

	
	// Set up a damper
	dashpot dp0;
	dp0.SetMass(10); // kg
	dp0.SetMaxForce(10000000000000000); // N
	dp0.SetDampingCoeff(4500); // Ns/m

	vector<double> dp0_pos = { 0.0, -0.5, 0.0 };
	vector<double> dp0_axis = { 1.0, 0.0, 0.0 };



	// Set up a torsion spring
	torsion_spring ts0;
	ts0.SetMaterial(steel);
	ts0.SetMass(10);
	ts0.SetWireDiameter(0.02); // m
	ts0.SetCoilDiameter(0.2); // m
	ts0.SetActiveCoilCount(25);
	ts0.SetRelaxedArmAngle(0.0); // rad

	ts0.SetMinAngle(-acos(-1) / 2.0); // rad
	ts0.SetMaxAngle(acos(-1) / 2.0); // rad

	//ts0.SetSpringConstant(10.0) // Nm / rad

	//ts0.Setup();
	//cout << "TS0 spring k: " << ts0.GetSpringConstant() << endl;

	vector<double> ts0_pos = { 0.0, 0.0, 0.0 };
	vector<double> ts0_axis = { 0.0, 0.0, -1.0 };




	// Set up a wingbox
	wingbox wb;

	vector<linear_spring> lin_springs = { ls0 };
	vector<dashpot> dashpots = { dp0 };
	vector<torsion_spring> tor_springs = { ts0 };


	// Set up the linear springs
	wb.SetLinearSprings(lin_springs);
	wb.SetLinearSpringWallPosition(ls0_pos, 0);
	wb.SetLinearSpringOrientation(ls0_axis, 0);


	// Set up the dashpots
	wb.SetDashpots(dashpots);
	wb.SetDashpotPosition(dp0_pos, 0);
	wb.SetDashpotOrientation(dp0_axis, 0);


	// Set up the torsion springs
	wb.SetTorsionSprings(tor_springs);
	wb.SetTorsionSpringPosition(ts0_pos, 0);
	wb.SetTorsionSpringOrientation(ts0_axis, 0);


	// General wingbox setup
	wb.SetRotationAxis({ 0.0, 0.0, 1.0 });


	


	// Aerodynamic model setup
	//double cd0 = 0.006;
	double cd0 = 0.02;
	double cd1 = 0.0;
	double cd2 = 0.3;
	double cm0 = 0.01;
	double liftslope = 5.9; // units of 1/radian
	double alpha0 = math.Deg2Rad(-4.0);

	aero_model* aero_setup = new aero_model;
	aero_setup->SetCd0(cd0);
	aero_setup->SetCd1(cd1);
	aero_setup->SetCd2(cd2);
	aero_setup->SetCm0(cm0);
	aero_setup->SetLiftslope(liftslope);
	aero_setup->SetAlpha0(alpha0);

	const aero_model* aero = aero_setup;



	
	// Set up a lifting line
	lifting_line LL;

	// Reads in the geometry of the wing
	string filename = "LiftingLines/Knight_Hefner_100AS.txt";
	int check = LL.ReadDataFile(filename);

	for (int i = 0; i < LL.GetNumberofAirstations(); i++)
	{
		LL.SetAirstationAerodynamicsModel(aero, i);
	}

	//LL.GeometrySetup(); // Put this in the variable wing setup()

	double rho_inf = 1.225; // kg/m^3
	double speed_of_sound = 323; // m/s

	LL.SetRhoInf(rho_inf);
	LL.SetSoS(speed_of_sound);

	// Set mass properties of the lifting line
	// Put the center of mass at the mid airstation's quarter chord
	vector<double> mid_point = LL.GetAirstationPosition(LL.GetNumberofAirstations() / 2.0);
	LL.SetCenterOfMassPosition(mid_point);

	// Wild guess
	LL.SetMomentOfInertia_CM({ { 10.0, 0.0, 0.0 },
		                       { 0.0, 100.0, 0.0 }, 
		                       { 0.0, 0.0, 200.0 } });
	LL.SetMass(500); // kg
	





	// Set up the variable wing
	variable_wing vw;
	vw.SetLiftingLine(LL);
	vw.SetWingbox(wb);

	vw.SetInitialRootChordAngle(math.Deg2Rad(45.0));
	
	// Set up the fuselage plane
	vector<double> plane_normal = { 1.0, 0.0, 0.0 };
	vector<double> plane_point = { 0.2, 0.0, 0.0 };
	vw.SetFuselagePlane(plane_normal, plane_point);

	vector<double> moment_point = { 0.0, 0.0, 0.0 };
	vw.SetMomentPoint(moment_point);

	// Define this in the lifting line frame
	vector<double> freestream = { 0.0, -100.0, 0.0 }; // m/s
	vw.SetFreestreamVelocity(freestream);
	
	// Ensure that you have this before starting the simulation
	vw.Setup();

	





	// Set up the simulation
	double time = 0.0;
	double timestep = 0.1;
	double tmax = 1000;


	// Writing to a file
	ofstream output_file("Data/variable_wing_data_stiff.txt");

	if (!output_file.is_open())
	{
		cerr << "Error opening the file." << endl;
		return 1;
	}

	output_file << "System State in Time:\n";
	output_file << "Time | # of Active Airstaitons | Phi | Phi_Dot\n";
	vector<double> phi;
	vector<double> phi_dot;
	int active_as_count;

	bool failed = false;

	while (time <= tmax)
	{
		//cout << "T = " << time << "s" << endl;
		vw.AdvanceTime(timestep);

		// Writing to the file at each timestep
		active_as_count = vw.GetActiveAirstationCount();
		phi = vw.GetPhi();
		phi_dot = vw.GetPhiDot();
		write_state(&output_file, time, active_as_count, phi, phi_dot);

		time += timestep;

		failed = vw.fail_check;
		if (failed == true)
		{
			break;
		}
	}
	
	output_file.close();

	cout << "Wrote the data to the file properly." << endl;

	return 0;
}



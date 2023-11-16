#include "geometric_math.h"

#include <cmath>
#include <vector>

#include <iostream>
#include <string>

using namespace std;

//template <typename T>

void geometric_math::ResizeMatrix(vector<vector<double>> &mat, int row_count, int col_count)
{
	vector<vector<double>> new_mat(row_count, vector<double>(col_count));

	// Variables to keep track of where we are in the old matrix when putting stuff into the new matrix
	int current_row = 0;
	int current_col = 0;

	// Checks whether or not elements will be left out of the new matrix and tells the user if that will happen
	int total_element_count_old = 0;
	for (int i = 0; i < mat.size(); i++)
	{
		total_element_count_old += mat[i].size();
	}

	if (total_element_count_old > row_count * col_count)
	{
		cout << "Some elements of the old matrix will be left out in the new matrix." << endl;
	}
	else if (total_element_count_old < row_count * col_count)
	{
		cout << "The new matrix will contain some empty elements based on the desired dimensions." << endl;
	}


	for (int i = 0; i < row_count; i++)
	{
		for (int j = 0; j < col_count; j++)
		{
			if (current_row >= mat.size())
			{
				break;
			}

			// Check if the bounds have been exceeded for a certain row
			if (current_col >= mat[current_row].size())
			{
				current_col = 0;
				current_row++;
			}

			new_mat[i][j] = mat[current_row][current_col];

			// Update the column to pull from
			current_row++;
		}
	}

	// Assign the new built up matrix
	mat = new_mat;
}


double geometric_math::Triangle3DArea(vector<double> point_A, vector<double> point_B, vector<double> point_C)
{
	vector<double> AB;
	AB.resize(point_A.size());

	vector<double> AC;
	AC.resize(point_A.size());

	for (int i = 0; i < point_A.size(); i++)
	{
		AB[i] = point_B[i] - point_A[i];
		AC[i] = point_C[i] - point_A[i];
	}

	vector<double> ABxAC = CrossProduct3D(AB, AC);

	double mag = VectorNorm(ABxAC, 2);

	double area = 0.5 * mag;

	return area;
}


vector<double> geometric_math::CrossProduct3D(vector<double> vec1, vector<double> vec2)
{
	vector<double> cross_product;
	cross_product.resize(vec1.size());

	cross_product[0] = vec1[1] * vec2[2] - vec1[2] * vec2[1];
	cross_product[1] = -(vec1[0] * vec2[2] - vec1[2] * vec2[0]);
	cross_product[2] = vec1[0] * vec2[1] - vec1[1] * vec2[0];

	return cross_product;
}


double geometric_math::DotProduct(vector<double> vec1, vector<double> vec2)
{
	double dot_product = 0;

	for (int i = 0; i < vec1.size(); i++)
	{
		dot_product += vec1[i] * vec2[i];
	}

	return dot_product;
}


double geometric_math::VectorNorm(vector<double> vec, int norm)
{
	// As stated in the header file, an input of 0 for the norm will take the infinity norm
	if (norm == 0)
	{
		double max_val = 0;

		for (int i = 0; i < vec.size(); i++)
		{
			if (abs(vec[i]) > max_val)
			{
				max_val = abs(vec[i]);
			}
		}

		return max_val;
	}

	// Otherwise, return the p-norm
	else
	{
		double mag = 0;

		for (int i = 0; i < vec.size(); i++)
		{
			mag += abs(pow(vec[i], norm));
		}

		return pow(mag, 1.0 / double(norm));
	}
	
}


vector<double> geometric_math::RotateVector(vector<double> vec, vector<double> axis, double theta)
{
	// Make sure that the axis vector is a unit vector
	vector<double> k = axis;

	k = UnitVector(k);

	// Now find the rotated vector
	vector<double> rotated_vec;
	rotated_vec.resize(vec.size());


	vector<double> KxV = CrossProduct3D(k, vec);
	double k_dot_v = DotProduct(k, vec);

	double c_theta = cos(theta);
	double s_theta = sin(theta);

	for (int i = 0; i < vec.size(); i++)
	{
		// Rodrigues formula
		rotated_vec[i] = vec[i] * c_theta + KxV[i] * s_theta + k[i] * k_dot_v * (1 - c_theta);
	}

	return rotated_vec;
}


vector<double> geometric_math::MidPoint(vector<double> point_1, vector<double> point_2)
{
	vector<double> mid_point;
	mid_point.resize(point_1.size());

	for (int i = 0; i < mid_point.size(); i++)
	{
		mid_point[i] = 0.5 * (point_1[i] + point_2[i]);
	}

	return mid_point;
}


vector<double> geometric_math::Scalar_x_Vector(double scalar, vector<double> vec)
{
	vector<double> output_vec;
	output_vec.resize(vec.size());

	for (int i = 0; i < vec.size(); i++)
	{
		output_vec[i] = scalar * vec[i];
	}

	return output_vec;
}


vector<vector<double>> geometric_math::Scalar_x_Matrix(double scalar, vector<vector<double>> mat)
{
	int N = mat.size();
	int M = mat[0].size();

	vector<vector<double>> output_mat;
	output_mat.resize(N, vector<double>(M));

	for (int i = 0; i < N; i++)
	{
		output_mat[i] = Scalar_x_Vector(scalar, mat[i]);
	}

	return output_mat;
}


vector<double> geometric_math::UnitVector(vector<double> vec)
{
	vector<double> unit_vec;
	unit_vec.resize(vec.size());

	double mag = VectorNorm(vec, 2);

	for (int i = 0; i < vec.size(); i++)
	{
		unit_vec[i] = vec[i] / mag;
	}

	return unit_vec;
}


vector<vector<double>> geometric_math::TransposeMatrix(vector<vector<double>> mat)
{
	int N = mat.size();
	int M = mat[0].size();

	vector<vector<double>> output_mat;
	output_mat.resize(M, vector<double> (N));

	for (int i = 0; i < M; i++)
	{
		for (int j = 0; j < N; j++)
		{
			output_mat[i][j] = mat[j][i];
		}
	}
	return output_mat;
}


vector<double> geometric_math::Matrix_x_Vector(vector<vector<double>> mat, vector<double> vec)
{
	vector<double> output_vec;

	int M1 = mat[0].size();
	int M2 = vec.size();

	if (M1 != M2)
	{
		cout << endl << "The Matrix_x_Vector operation cannot be performed as the inputs are of incompatible sizes" << endl;
	}
	
	else
	{
		// For an NxM * Mx1 scenario, the output is of size N
		output_vec.resize(mat.size());

		for (int i = 0; i < output_vec.size(); i++)
		{
			output_vec[i] = DotProduct(mat[i], vec);
		}
	}

	return output_vec;
}



vector<vector<double>> geometric_math::Matrix_x_Matrix(vector<vector<double>> mat1, vector<vector<double>> mat2)
{
	// NxM * MxR -> NxR
	int N = mat1.size();

	int M1 = mat1[0].size();
	int M2 = mat2.size();

	int R = mat2[0].size();

	vector<vector<double>> output_mat;

	if (M1 != M2)
	{
		cout << endl << "The Matrix_x_Matrix operation cannot be performed as the inputs are of incompatible sizes" << endl;
	}

	else
	{
		output_mat.resize(N, vector<double> (R));

		// We can transpose the second matrix to take dot products for each term of the output
		vector<vector<double>> t_mat2 = TransposeMatrix(mat2);

		for (int i = 0; i < N; i++)
		{
			for (int j = 0; j < R; j++)
			{
				output_mat[i][j] = DotProduct(mat1[i], t_mat2[j]);
			}
		}
	}

	return output_mat;

}


vector<vector<double>> geometric_math::Vector_x_Vector(vector<double> vec1, vector<double> vec2)
{
	vector<vector<double>> output_mat;

	// Nx1 * 1*M gives NxM
	int N = vec1.size();
	int M = vec2.size();

	output_mat.resize(N, vector<double> (M));

	for (int i = 0; i < N; i++)
	{
		for (int j = 0; j < M; j++)
		{
			output_mat[i][j] = vec1[i] * vec2[j];
		}
	}

	return output_mat;
}



vector<double> geometric_math::SumVectors(vector<double> vec1, vector<double> vec2)
{
	vector<double> output_vec;

	if (vec1.size() != vec2.size())
	{
		cout << "The SumVectors operation cannot be performed as the inputs are of incompatible size" << endl;
	}

	else
	{
		output_vec.resize(vec1.size());

		for (int i = 0; i < vec1.size(); i++)
		{
			output_vec[i] = vec1[i] + vec2[i];
		}
	}

	return output_vec;
}



vector<vector<double>> geometric_math::SumMatrices(vector<vector<double>> mat1, vector<vector<double>> mat2)
{
	// Check if the matrices can be summed
	int N1 = mat1.size();
	int M1 = mat1[0].size();

	int N2 = mat2.size();
	int M2 = mat2[0].size();

	vector<vector<double>> output_mat(N1, vector<double>(M1, 0.0));

	if ((N1 != N2) || (M1 != M2))
	{
		cout << "The SumMatrices operation cannot be performed as the inputs are of incompatible size" << endl;
	}

	else
	{
		for (int i = 0; i < mat1.size(); i++)
		{
			output_mat[i] = SumVectors(mat1[i], mat2[i]);
		}
	}

	return output_mat;
}


vector<double> geometric_math::ProjectVectorOntoVector(vector<double> vec1, vector<double> vec2)
{
	// Projects vec1 onto vec2 (or u onto v)
	// Coefficient in from of v for the projection = u.v / mag(v)^2
	double coeff_v = DotProduct(vec1, vec2) / pow(VectorNorm(vec2, 2), 2);

	vector<double> projected_vector = Scalar_x_Vector(coeff_v, vec2);

	return projected_vector;
}


vector<double> geometric_math::ProjectVectorOntoPlane(vector<double> vec, vector<double> plane_normal)
{
	// Projects the vector onto the plane specified by the plane normal
	// This is given as proj(u, plane) = u - proj(u, n)

	// Find projection onto the plane normal
	vector<double> proj_u_n = ProjectVectorOntoVector(vec, plane_normal);

	// Add the vector to the negative of the projection onto the plane normal
	vector<double> proj_u_plane = SumVectors(vec, Scalar_x_Vector(-1, proj_u_n));

	return proj_u_plane;
}


double geometric_math::AngleBetweenVectors(vector<double> vec1, vector<double> vec2, vector<double> plane_normal)
{
	double mag_u = VectorNorm(vec1, 2);
	double mag_v = VectorNorm(vec2, 2);

	double u_dot_v = DotProduct(vec1, vec2);

	// This gives the unsigned angle between the two vectors
	double theta = acos(u_dot_v / (mag_u * mag_v));

	// To find the sign of the angle, we take the cross product of vec1 and vec2 to find the normal of the plane specified by the two vectors and then dot that to the normal vector specified by the user
	// If the dot product is positive then theta is positive, if the dot product is negative then it is negative
	
	// If the normal vector is specified and the vector is 3D then do the method mentioned above
	if ((!plane_normal.empty()) & (vec1.size() == 3))
	{
		vector<double> cross_u_v = CrossProduct3D(vec1, vec2);
		double sign_check = DotProduct(cross_u_v, plane_normal);

		if (sign_check < 0)
		{
			theta = -theta;
		}
	}

	// If the vectors are not 3D then this is not currently handled
	else if ((!plane_normal.empty()) & (vec1.size() != 3))
	{
		cout << "Failed to properly check sign. Positive value returned." << endl;
	}


	return theta;
}




vector<vector<double>> geometric_math::CrossProductEquivalent(vector<double> vec)
{
	vector<vector<double>> mat = { {0,        -vec[2],      vec[1]},
								   {vec[2],      0,        -vec[0]},
								   {-vec[1],     vec[0],      0} };

	return mat;
}


vector<vector<double>> geometric_math::Diagonalize(std::vector<double> vec)
{
	int N = vec.size();
	vector<vector<double>> output_mat(N, vector<double>(N, 0.0));

	for (int i = 0; i < N; i++)
	{
		output_mat[i][i] = vec[i];
	}

	return output_mat;
}


double geometric_math::Determinant(std::vector<std::vector<double>> mat)
{
	int n = mat.size();
	double det = 0;

	if (n == 1) 
	{
		return mat[0][0];
	}
	else if (n == 2) 
	{
		return (mat[0][0] * mat[1][1]) - (mat[1][0] * mat[0][1]);
	}

	else 
	{
		vector<vector<double>> submatrix(n - 1, vector<double>(n - 1, 0));

		for (int x = 0; x < n; x++) {
			int subi = 0;
			for (int i = 1; i < n; i++) {
				int subj = 0;
				for (int j = 0; j < n; j++) {
					if (j == x) {
						continue;
					}
					submatrix[subi][subj] = mat[i][j];
					subj++;
				}
				subi++;
			}
			det = det + (pow(-1, x) * mat[0][x] * Determinant(submatrix));
		}
	}

	return det;
}

vector<vector<double>> geometric_math::Invert(vector<vector<double>> mat)
{
	int N = mat.size();
	double det = Determinant(mat);
	if (det == 0) {
		cout << "Singular matrix, can't find its inverse." << endl;
		return vector<vector<double>> {{0.0}};
	}

	vector<vector<double>> cofactors(N, vector<double>(N, 0.0));

	for (int i = 0; i < N; i++)
	{
		for (int j = 0; j < N; j++)
		{
			cofactors[i][j] = Cofactor(mat, i, j);
		}
	}

	vector<vector<double>> adjoint = TransposeMatrix(cofactors);

	vector<vector<double>> inverse = Scalar_x_Matrix(1.0 / det, adjoint);

	return inverse;
}


double geometric_math::Cofactor(vector<vector<double>> mat, int row, int col)
{
	int N = mat.size();
	vector<vector<double>> submat(N - 1, vector<double>(N-1, 0.0));

	int submat_row = 0;
	for (int i = 0; i < N; i++)
	{
		int submat_col = 0;

		// If i == row then skip that row when putting the elements in the submatrix
		if (i != row)
		{
			for (int j = 0; j < N; j++)
			{
				// If j == col then skip that column when putting the elements in the submatrix
				if (j != col)
				{
					submat[submat_row][submat_col] = mat[i][j];
					submat_col++;
				}
			}
			submat_row++;
		}
	}
	double cofactor = pow(-1.0, double(row + col)) * Determinant(submat);

	return cofactor;
}


vector<double> geometric_math::PlaneFromNormalPoint(vector<double> norm, vector<double> point)
{
	// Plane defined by equation ax + by + cz = d
	double a = norm[0];
	double b = norm[1];
	double c = norm[2];
	double d = DotProduct(norm, point);
	vector<double> plane = { a, b, c, d };
	return plane;
}

bool geometric_math::CheckPointPlaneSide(vector<double> plane, vector<double> point)
{
	vector<double> abc = { plane[0], plane[1], plane[2] };
	double d = plane[3];

	double check = DotProduct(abc, point);

	// If this dot product is greater than d, then the point is on the side of the plane in which the normal vector is pointing.
	// Otherwise it is on the opposite side of the plane
	if (check > d)
	{
		return true;
	}

	else
	{
		return false;
	}
}


double geometric_math::Rad2Deg(double angle)
{
	return angle * 180.0 / acos(-1);
}

double geometric_math::Deg2Rad(double angle)
{
	return angle * acos(-1) / 180.0;
}


void geometric_math::PrintMatrix(vector<vector<double>> mat, string mat_name)
{
	cout << endl << mat_name << ":" << endl << "[ ";
	for (int i = 0; i < mat.size(); i++)
	{
		for (int j = 0; j < mat[i].size(); j++)
		{
			cout << mat[i][j] << "  ";
		}

		if (i == mat.size() - 1)
		{
			cout << " ]";
		}

		cout << endl;
	}
}
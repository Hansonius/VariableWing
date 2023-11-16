/**
 * @brief geometric_math.h contains code to help with performing common various vector/matrix mathematical operations
 *
 * @author Josh Hansen
 * @author Nicolas Reveles
 *
 */


#ifndef GEOMETRIC_MATH_H
#define GEOMETRIC_MATH_H


#include <vector>
#include <cmath>
#include <string>


class geometric_math
{
	public:
		/**
		* \brief Resizes a matrix to be a specified set of new dimensions
		*/
		//template <typename T>
		//void ResizeMatrix(std::vector<std::vector<T>> &mat, int row_count, int col_count);
		void ResizeMatrix(std::vector<std::vector<double>> &mat, int row_count, int col_count);


		/**
		* \brief Calculates the area created from 3 points in 3D space
		* \param point_A 3x1 vector representing point in 3D space
		* \param point_B 3x1 vector representing point in 3D space
		* \param point_C 3x1 vector representing point in 3D space
		* \return Area between the 3 points
		*/
		double Triangle3DArea(std::vector<double> point_A, std::vector<double> point_B, std::vector<double> point_C);


		/**
		* \brief Calculates the cross product between two vectors in 3D space
		* \param vec1 3x1 vector in 3D space
		* \param vec2 3x1 vector in 3D space
		* \return 3x1 vector which is vec1 x vec2. Order is crucial
		*/
		std::vector<double> CrossProduct3D(std::vector<double> vec1, std::vector<double> vec2);


		/**
		* \brief Calculates the dot product between two vectors
		* \param vec1 Vector of size N
		* \param vec2 Vector of size N
		* \return Dot product of vec1 dotted with vec2
		*/
		double DotProduct(std::vector<double> vec1, std::vector<double> vec2);


		/**
		* \brief Computes the p-norm of a specified vector
		* \param vec Vector of size N
		* \param norm Which order of norm is being taken. An input of 0 will compute the infinity norm, and the 'zero norm' is not something that can be computed using this function
		* \return The specified norm of the vector
		*/
		double VectorNorm(std::vector<double> vec, int norm);


		/**
		* \brief Rotates one vector about another vector by a specified angle
		* \param vec Vector of size N to rotate
		* \param axis Vector of size N specifying the axis to rotate the vector about
		* \param theta Angle to rotate the vector by in the counter-clockwise direction
		* \return Rotated vector of size N
		*/
		std::vector<double> RotateVector(std::vector<double> vec, std::vector<double> axis, double theta);


		/**
		* \brief Finds the midpoint between two points
		* \param point_1 Point in N degree space
		* \param point_2 Point in N degree space
		* \return Point in N degree space which is midway between points 1 and 2 in all dimensions
		*/
		std::vector<double> MidPoint(std::vector<double> point_1, std::vector<double> point_2);


		/**
		* \brief Computes a scalar times a vector
		* \param scalar_value Scalar value to multiply a vector by
		* \param vec Vector of size N
		* \return Vector of scalar_value * vec
		*/
		std::vector<double> Scalar_x_Vector(double scalar_value, std::vector<double> vec);


		/**
		* \brief Computes a scalar times a matrix
		*/
		std::vector<std::vector<double>> Scalar_x_Matrix(double scalar_value, std::vector<std::vector<double>> mat);



		/**
		* \brief Computes the unit vector of a given input vector
		* \param vec Vector of size N
		* \return Unit vector in the direction of the input vector
		*/
		std::vector<double> UnitVector(std::vector<double> vec);


		/**
		* \brief Transposes an input matrix
		* \param mat Input matrix of size N x M
		* \return Transposed matrix of size M x N
		*/
		std::vector<std::vector<double> > TransposeMatrix(std::vector<std::vector<double> > mat);


		/**
		* \brief Multiplies a matrix by a vector
		* \param mat Matrix of size N x M
		* \param vec Vector of size M
		* \return Vector of size N which is mat * vec
		*/
		std::vector<double> Matrix_x_Vector(std::vector<std::vector<double> > mat, std::vector<double> vec);


		/**
		* \brief Multiplies two matrices together
		* \param mat1 Matrix of size N x M
		* \param mat2 Matrix of size M x R
		* \returns Matrix of size N x R which is mat1 * mat2
		*/
		std::vector<std::vector<double> > Matrix_x_Matrix(std::vector<std::vector<double> > mat1, std::vector<std::vector<double> > mat2);

		/**
		* \brief Multiplies two vectors together to make a matrix
		* \param vec1 Vector of size N, representing a matrix of size N x 1
		* \param vec2 Vector of size M, representing a matrix of size 1 x M
		* \return Matrix of size N x M which is vec1 * vec2
		*/
		std::vector<std::vector<double> > Vector_x_Vector(std::vector<double> vec1, std::vector<double> vec2);

		
		/**
		* \brief Adds together the components of two vectors
		* \param vec1 Vector of size N
		* \param vec2 Vector of size N
		* \return Vector of size N which is vec1 + vec2
		*/
		std::vector<double> SumVectors(std::vector<double> vec1, std::vector<double> vec2);


		/**
		* \brief Adds two matrices of the same size together
		*/
		std::vector<std::vector<double>> SumMatrices(std::vector<std::vector<double>> mat1, std::vector<std::vector<double>> mat2);




		/**
		* \brief Projects one vector onto another vector
		* \param vec1 Vector of size N
		* \param vec2 Vector of size N
		* \return Vector of size N which is the projection of vec1 onto vec2
		*/
		std::vector<double> ProjectVectorOntoVector(std::vector<double> vec1, std::vector<double> vec2);


		/**
		* \brief Projects a vector onto a plane
		* \param vec Vector of size N
		* \param plane_normal Vector of size N which specifies a plane normal to it to project onto
		* \return Returns a vector of size N which is projection of vec onto the plane specified by the plane_normal
		*/
		std::vector<double> ProjectVectorOntoPlane(std::vector<double> vec, std::vector<double> plane_normal);


		/**
		* \brief Finds the angle between two vectors
		* \param vec1 Vector of size N
		* \param vec2 Vector of size N
		* \param plane_normal Optional vector normal to the plane specified by vec1 and vec2. Done to help resolve the sign of the angle. If not specified, the returned angle will be positive. Currently, this feature only handles 3D vectors
		* \return Angle between vec1 and vec2
		*/
		double AngleBetweenVectors(std::vector<double> vec1, std::vector<double> vec2, std::vector<double> plane_normal = {});


		/**
		* \brief Finds the cross product equivalent of a vector
		*/
		std::vector<std::vector<double>> CrossProductEquivalent(std::vector<double> vec);
		
		/**
		* \brief Diagonalizes an input vector into a matrix 
		*/
		std::vector<std::vector<double>> Diagonalize(std::vector<double> vec);


		/**
		* \brief Finds the determinant of a matrix
		*/
		double Determinant(std::vector<std::vector<double>> mat);


		/**
		* \brief Finds the inverse of a matrix
		*/
		std::vector<std::vector<double>> Invert(std::vector<std::vector<double>> mat);


		/**
		* \brief Finds the cofactor of a certain index within a matrix
		*/
		double Cofactor(std::vector<std::vector<double>> mat, int i, int j);


		/**
		* \brief Finds the plane equation from the plane normal and a point on the plane
		*/
		std::vector<double> PlaneFromNormalPoint(std::vector<double> norm, std::vector<double> point);

		/**
		* \brief Checks which side of a plane a point is on. Returns 0 if it on the opposite side of the plane normal and 1 if it is on the same side as the plane normal
		*/
		bool CheckPointPlaneSide(std::vector<double> plane, std::vector<double> point);


		double Rad2Deg(double angle);
		
		double Deg2Rad(double angle);


		void PrintMatrix(std::vector<std::vector<double>> mat, std::string mat_name);
};


#endif
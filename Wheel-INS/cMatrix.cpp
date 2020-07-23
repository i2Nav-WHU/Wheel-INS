#include "NavFunc.h"

void ConvertPI(double* angle)
{
	//[0, 2pi] -> [-pi, pi]
	if (*angle > PI)
	{
		*angle -= 2 * PI;
	}
	else if (*angle < -PI)
	{
		*angle += 2 * PI;
	}
}

void CrossProduct(Vec3 a, Vec3 b, Vec3 c)
{
	c[0] = a[1] * b[2] - a[2] * b[1];
	c[1] = a[2] * b[0] - a[0] * b[2];
	c[2] = a[0] * b[1] - a[1] * b[0];
}

void euler2rotation(Mat3 rotation, Vec3 euler) 
{
	
	double sin_phi = sin(euler[0]);
	double cos_phi = cos(euler[0]);
	double sin_theta = sin(euler[1]);
	double cos_theta = cos(euler[1]);
	double sin_psi = sin(euler[2]);
	double cos_psi = cos(euler[2]);

	rotation[0] = cos_psi*cos_theta;
	rotation[3] = sin_psi*cos_theta;
	rotation[6] = -sin_theta;
	rotation[1] = (-sin_psi*cos_phi) + cos_psi*(sin_theta*sin_phi);
	rotation[4] = (cos_psi*cos_phi) + sin_psi*(sin_theta*sin_phi);
	rotation[7] = cos_theta*sin_phi;
	rotation[2] = (sin_psi*sin_phi) + cos_psi*(sin_theta*cos_phi);
	rotation[5] = (-cos_psi*sin_phi) + sin_psi*(sin_theta*cos_phi);
	rotation[8] = cos_theta*cos_phi;
}

void rotation2quat(QuatVec quat, Mat3 rotation) 
{
	
	int decide = -1;
	double max_value = -999.0;
	double UU[4] = { 1 + rotation[0] + rotation[4] + rotation[8], 
		1 + rotation[0] - rotation[4] - rotation[8],
		1 - rotation[0] + rotation[4] - rotation[8],
		1 - rotation[0] - rotation[4] + rotation[8] };
	for (int i = 0; i < 4; i++)
	{
		if (max_value<UU[i])
		{
			max_value = UU[i];
			decide = i;
		}
	}
	if (decide == 0)
	{
		quat[3] = UU[0];
		quat[0] = rotation[7] - rotation[5];
		quat[1] = rotation[2] - rotation[6];
		quat[2] = rotation[3] - rotation[1];
	}
	else if (decide == 1)
	{
		quat[3] = rotation[7] - rotation[5];
		quat[0] = UU[1];
		quat[1] = rotation[1] + rotation[3];
		quat[2] = rotation[2] + rotation[6];
	}
	else if (decide == 2)
	{
		quat[3] = rotation[2] - rotation[6];
		quat[0] = rotation[1] + rotation[3];
		quat[1] = UU[2];
		quat[2] = rotation[5] + rotation[7];
	}
	else
	{
		quat[3] = rotation[3] - rotation[1];
		quat[0] = rotation[2] + rotation[6];
		quat[1] = rotation[5] + rotation[7];
		quat[2] = UU[3];
	}
	//Normalize the quaternion
	double T = vec_norm(4, quat);

	quat[0] = quat[0] / T;
	quat[1] = quat[1] / T;
	quat[2] = quat[2] / T;
	quat[3] = quat[3] / T;
}

void quat2rotation(Mat3 rotation, QuatVec quat) 
{
	
	double tp[10];
	tp[0] = quat[0] * quat[0];
	tp[1] = quat[1] * quat[1];
	tp[2] = quat[2] * quat[2];
	tp[3] = quat[3] * quat[3];
	tp[4] = quat[0] * quat[1];
	tp[5] = quat[0] * quat[2];
	tp[6] = quat[0] * quat[3];
	tp[7] = quat[1] * quat[2];
	tp[8] = quat[1] * quat[3];
	tp[9] = quat[2] * quat[3];

	rotation[0] = tp[0] - tp[1] - tp[2] + tp[3];
	rotation[1] = 2.0*(tp[4] - tp[9]);
	rotation[2] = 2.0*(tp[5] + tp[8]);
	rotation[3] = 2.0*(tp[4] + tp[9]);
	rotation[4] = -tp[0] + tp[1] - tp[2] + tp[3];
	rotation[5] = 2.0*(tp[7] - tp[6]);
	rotation[6] = 2.0*(tp[5] - tp[8]);
	rotation[7] = 2.0*(tp[7] + tp[6]);
	rotation[8] = -tp[0] - tp[1] + tp[2] + tp[3];
}

void  rotation2euler(Vec3 euler, Mat3 rotation) {

	euler[1] = atan(-rotation[6] / sqrt(rotation[7] * rotation[7] + rotation[8] * rotation[8]));
	euler[0] = atan2(rotation[7], rotation[8]);	//atan2( R(3,2), R(3,3) );					
	euler[2] = atan2(rotation[3], rotation[0]);	//atan2( R(2,1), R(1,1));
}

void euler2quat(QuatVec quat, Vec3 euler)
{
	
	Mat3 rotation;
	euler2rotation(rotation, euler);
	rotation2quat(quat, rotation);
	//Normalize the quaternion
	double T = vec_norm(4, quat);

	quat[0] = quat[0] / T;
	quat[1] = quat[1] / T;
	quat[2] = quat[2] / T;
	quat[3] = quat[3] / T;
}

void quat2euler(Vec3 euler, QuatVec quat)
{
	
	euler[0] = atan2(2 * (quat[3] * quat[0] + quat[1] * quat[2]), 1 - 2.0*(S_2_(quat[0]) + S_2_(quat[1])));
	euler[1] = asin(2 * (quat[3] * quat[1] - quat[2] * quat[0]));
	euler[2] = atan2(2 * (quat[3] * quat[2] + quat[0] * quat[1]), 1 - 2.0*(S_2_(quat[1]) + S_2_(quat[2])));
}

void zero_array(int row, double *da)
{
	int i = 0;
	for (i = 0; i < row; i++)
	{
		da[i] = 0.0;
	}
}

double vec_norm(int len, double *da)
{
	double sum = 0.0;
	for (int i = 0; i < len; i++)
	{
		sum += da[i] * da[i];
	}
	return sqrt(sum);
}

void diag_matrix(int row, double value, double *da)
{
	int i = 0, j = 0;
	for (i = 0; i < row; i++)
	{
		for (j = 0; j < row; j++)
		{
			if (i == j)
			{
				da[i*row + j] = value;
			}
			else
			{
				da[i*row + j] = 0.0;
			}

		}
	}
}

void AddMatrix(double matrix_a[], double matrix_b[], int matrix_length, double result_matrix[])
{
	for (int i = 0; i<matrix_length; i++)
	{
		result_matrix[i] = matrix_a[i] + matrix_b[i];
	}
}

void SubtractMatrix(double matrix_a[], double matrix_b[], int matrix_length, double result_matrix[])
{
	for (int i = 0; i<matrix_length; i++)
	{
		result_matrix[i] = matrix_a[i] - matrix_b[i];
	}
}

void MultiplyMatrixWithReal(double real_number, double matrix[], int matrix_length, double result_matrix[])
{
	for (int i = 0; i<matrix_length; i++)
	{
		result_matrix[i] = real_number*matrix[i];
	}
}

void MultiplyMatrix(double matrix_a[], double matrix_b[], int matrix_a_row, int matrix_a_column, int matrix_b_column, double result_matrix[])
{
	double sum = 0;
	double median = 0;
	for (int i = 0; i<matrix_a_row; i++)
	{
		for (int k = 0; k<matrix_b_column; k++)
		{

			for (int j = 0; j<matrix_a_column; j++)
			{
				median = matrix_a[matrix_a_column*i + j] * matrix_b[j*matrix_b_column + k];
				sum = sum + median;

			}
			result_matrix[i*matrix_b_column + k] = sum;
			sum = 0;
		}
	}
}

void TransposeMatrix(double matrix[], int matrix_row, int matrix_column, double transpose_matrix[])
{
	for (int i = 0; i<matrix_column; i++)
		for (int j = 0; j<matrix_row; j++)
			transpose_matrix[i*matrix_row + j] = matrix[i + j*matrix_column];
}

int Cholesky(double matrix[], int matrix_row)
{
	int i, j, k, m, n;
	if (matrix[0] <= 0.0)
	{
		#ifdef DEBUG
			printf("fail\n");
		#endif
		return -2;
	}
	matrix[0] = sqrt(matrix[0]);
	for (i = 1; i <= matrix_row - 1; i++)
	{
		m = i*matrix_row;
		matrix[m] = matrix[m] / matrix[0];
	}
	for (j = 1; j <= matrix_row - 1; j++)
	{
		n = j*matrix_row + j;
		for (k = 0; k <= j - 1; k++)
		{
			m = j*matrix_row + k;
			matrix[n] = matrix[n] - matrix[m] * matrix[m];
		}		
		if (matrix[n] <= 0.0)
		{
			#ifdef DEBUG
				printf("fail\n");
			#endif
			return -2;
		}
		matrix[n] = sqrt(matrix[n]);
		for (i = j + 1; i <= matrix_row - 1; i++)
		{
			m = i*matrix_row + j;
			for (k = 0; k <= j - 1; k++)
				matrix[m] = matrix[m] - matrix[i*matrix_row + k] * matrix[j*matrix_row + k];
			matrix[m] = matrix[m] / matrix[n];
		}
	}
	for (i = 0; i <= matrix_row - 2; i++)
	{
		for (j = i + 1; j <= matrix_row - 1; j++)
			matrix[i*matrix_row + j] = 0.0;
	}
	return 2;
}

void UnitMatrix(int matrix_order, double unit_matrix[])
{
	for (int i = 0; i<matrix_order*matrix_order; i++)
		unit_matrix[i] = 0.0;
	for (int i = 0; i<matrix_order; i++)
	{
		unit_matrix[i*(matrix_order + 1)] = 1;
	}
}

void ZeroMatrix(int matrix_row, int matrix_colunm, double zero_matrix[])
{
	for (int i = 0; i<matrix_row*matrix_colunm; i++)
		zero_matrix[i] = 0.0;
}

void CopyMatrix(int row, int col, double matrix_result[], double matrix_b[])
{
	int i = 0, j = 0;
	for (i = 0; i<row; i++)
	{
		for (j = 0; j<col; j++)
		{
			matrix_result[i*col + j] = matrix_b[i*col + j];
		}
	}
}

int MatrixInv(int n, double a[], double b[])
{
	int i, j, k, l, u, v, is[12], js[12];   
	double d, p;

	if (n <= 0)
	{
		printf("Error dimension in MatrixInv!\n");
		exit(EXIT_FAILURE);
	}

	
	for (i = 0; i<n; i++)
	{
		for (j = 0; j<n; j++)
		{
			b[i*n + j] = a[i*n + j];
		}
	}

	for (k = 0; k<n; k++)
	{
		d = 0.0;
		for (i = k; i<n; i++)   
		{
			for (j = k; j<n; j++)
			{
				l = n*i + j;
				p = absf(b[l]);
				if (p>d)
				{
					d = p;
					is[k] = i;
					js[k] = j;
				}
			}
		}

		if (d<DBL_EPSILON)   
		{
			printf("Divided by 0 in MatrixInv!\n");
			exit(EXIT_FAILURE);
		}

		if (is[k] != k)  
		{
			for (j = 0; j<n; j++)
			{
				u = k*n + j;
				v = is[k] * n + j;
				p = b[u];
				b[u] = b[v];
				b[v] = p;
			}
		}

		if (js[k] != k)  
		{
			for (i = 0; i<n; i++)
			{
				u = i*n + k;
				v = i*n + js[k];
				p = b[u];
				b[u] = b[v];
				b[v] = p;
			}
		}

		l = k*n + k;
		b[l] = 1.0 / b[l]; 
		for (j = 0; j<n; j++)
		{
			if (j != k)
			{
				u = k*n + j;
				b[u] = b[u] * b[l];
			}
		}
		for (i = 0; i<n; i++)
		{
			if (i != k)
			{
				for (j = 0; j<n; j++)
				{
					if (j != k)
					{
						u = i*n + j;
						b[u] = b[u] - b[i*n + k] * b[k*n + j];
					}
				}
			}
		}
		for (i = 0; i<n; i++)
		{
			if (i != k)
			{
				u = i*n + k;
				b[u] = -b[u] * b[l];
			}
		}
	}

	for (k = n - 1; k >= 0; k--)  
	{
		if (js[k] != k)
		{
			for (j = 0; j<n; j++)
			{
				u = k*n + j;
				v = js[k] * n + j;
				p = b[u];
				b[u] = b[v];
				b[v] = p;
			}
		}
		if (is[k] != k)
		{
			for (i = 0; i<n; i++)
			{
				u = i*n + k;
				v = is[k] + i*n;
				p = b[u];
				b[u] = b[v];
				b[v] = p;
			}
		}
	}

	return (1);
}

bool QuatMul(double Q1[], double Q2[], double result_Q[])
{
	result_Q[3] = Q1[3] * Q2[3] - Q1[0] * Q2[0] - Q1[1] * Q2[1] - Q1[2] * Q2[2];
	result_Q[0] = Q1[0] * Q2[3] + Q1[3] * Q2[0] - Q1[2] * Q2[1] + Q1[1] * Q2[2];
	result_Q[1] = Q1[1] * Q2[3] + Q1[2] * Q2[0] + Q1[3] * Q2[1] - Q1[0] * Q2[2];
	result_Q[2] = Q1[2] * Q2[3] - Q1[1] * Q2[0] + Q1[0] * Q2[1] + Q1[3] * Q2[2];
	return true;
}

void Quatinv(double Q[], double Qinv[])
{
	double Q_mag = Q[0] * Q[0] + Q[1] * Q[1] + Q[2] * Q[2] + Q[3] * Q[3];
	Qinv[0] = -Q[0] / Q_mag;
	Qinv[1] = -Q[1] / Q_mag;
	Qinv[2] = -Q[2] / Q_mag;
	Qinv[3] = Q[3] / Q_mag;
}

bool Quat_norm(double Qbn[])
{
	double Q = 0.0;
	Q = sqrt(Qbn[0] * Qbn[0] + Qbn[1] * Qbn[1] + Qbn[2] * Qbn[2] + Qbn[3] * Qbn[3]);
	Qbn[0] = Qbn[0] / Q;
	Qbn[1] = Qbn[1] / Q;
	Qbn[2] = Qbn[2] / Q;
	Qbn[3] = Qbn[3] / Q;
	return true;
}

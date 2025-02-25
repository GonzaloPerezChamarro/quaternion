
#include "Quaternion.h"

#include <math.h>


Quaternion::Quaternion(const Quaternion & q)
{
	w = q.get_w();
	x = q.get_x();
	y = q.get_y();
	z = q.get_z();
}

Quaternion::Quaternion(float v[4])
{
	w = v[0];
	x = v[1];
	y = v[2];
	z = v[3];
}

Quaternion Quaternion::slerp(const Quaternion& q_a, const Quaternion& q_b, double time)
{
	Quaternion res;

	float theta, st, sut, sout, coeff1, coeff2;
	float dot = q_a.get_x() * q_b.get_x() +
		q_a.get_y() * q_b.get_y() +
		q_a.get_z() * q_b.get_z() +
		q_a.get_w() * q_b.get_w();

	time *= 0.5;

	theta = (float)acos(dot);
	if (theta < 0.0) theta = -theta;

	st = (float)sin(theta);
	sut = (float)sin(time*theta);
	sout = (float)sin((1 - time)*theta);
	coeff1 = sout / st;
	coeff2 = sut / st;

	res.x = coeff1 * q_a.x + coeff2 * q_b.x;
	res.y = coeff1 * q_a.y + coeff2 * q_b.y;
	res.z = coeff1 * q_a.z + coeff2 * q_b.z;
	res.w = coeff1 * q_a.w + coeff2 * q_b.w;

	res.normalize();

	return res;
}

void Quaternion::normalize()
{
	float mod = std::sqrt(x*x + y * y + z * z + w * w);
	x /= mod;
	y /= mod;
	z /= mod;
}

XMFLOAT3 Quaternion::euler() const
{
	XMFLOAT3 euler;

	double sqw, sqx, sqy, sqz;

	sqw = w * w;
	sqx = x * x;
	sqy = y * y;
	sqz = z * z;

	euler.y = asin(2.0 * (w * y - x * z));

	if (M_PI * 0.5 - fabs(euler.y) > DBL_EPSILON)
	{
		euler.z = atan2(2.0 * (x * y + z * w), sqx - sqy - sqz + sqw);
		euler.x = atan2(2.0 * (w * x + y * z), sqw - sqx - sqy + sqz);
	}
	else 
	{
		euler.z = atan2(2 * y * z - 2 * x * w,
						2 * x * z + 2 * y * w);
		euler.x = 0.0;

		if (euler.y < 0)
		{
			euler.z = M_PI - euler.y;
		}
	}

	return euler;
}

Matrix33D Quaternion::to_transform_matrix() const
{
	Matrix33D matrix;

	matrix[0][0] = 1 - 2 * y*y - 2 * z*z;
	matrix[0][1] = 2 * x*y - 2 * z*w;
	matrix[0][2] = 2 * x*z + 2 * y*w;
	matrix[1][0] = 2 * x*y + 2 * z*w;
	matrix[1][1] = 1 - 2 * x*x - 2 * z*z;
	matrix[1][2] = 2 * y*z - 2 * x*w;
	matrix[2][0] = 2 * x*z - 2 * y*w;
	matrix[2][1] = 2 * y*z + 2 * x*w;
	matrix[2][2] = 1 - 2 * x*x - 2 * y*y;

	return matrix;
}

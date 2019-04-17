#include "Quaternion.h"

Quaternion::Quaternion()
	:w(0), x(0), y(0), z(0)
{
}

Quaternion::Quaternion(float w, float x, float y, float z)
	:w(w), x(x), y(y), z(z)
{
}

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

void Quaternion::slerp(Quaternion q_a, Quaternion q_b, Quaternion &q_r, float time)
{
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

	q_r.x = coeff1 * q_a.x + coeff2 * q_b.x;
	q_r.y = coeff1 * q_a.y + coeff2 * q_b.y;
	q_r.z = coeff1 * q_a.z + coeff2 * q_b.z;
	q_r.w = coeff1 * q_a.w + coeff2 * q_b.w;

	q_r.normalize();

}

void Quaternion::normalize()
{
	float mod = std::sqrt(x*x + y * y + z * z + w * w);
	x /= mod;
	y /= mod;
	z /= mod;
}

Matrix33 Quaternion::to_transform_matrix()
{
	Matrix33 matrix;

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


#pragma once

#include <directxmath.h>

using namespace DirectX;

class Quaternion
{
	typedef std::array<std::array<double, 3>, 3> Matrix33D;

public:
	/* Components */
	double w, x, y, z;

public:

	/*----- CONSTRUCTORS -----*/

	/* Constructor */
	Quaternion(float w = 0, float x = 0, float y = 0, float z = 0) :w(w), x(x), y(y), z(z) {}

	/* Copy constructor */
	Quaternion(const Quaternion & q);

	/* Constructor from a vector3 and a real */
	Quaternion(const XMFLOAT3& c, double r) :w(r), x(c.x), y(c.y), z(c.z) {}

	/* Constructor from a float array */
	Quaternion(float v[4]);

	/*----- OPERATORS -----*/

	Quaternion operator+(const Quaternion& q) const 
	{
		return Quaternion(w + q.w, x + q.x, y + q.y, z + q.z);
	}

	Quaternion operator-(const Quaternion& q) const
	{
		return Quaternion(w - q.w, x - q.x, y - q.y, z - q.z);
	}

	Quaternion operator*(const Quaternion& q) const
	{
		return Quaternion(y * q.z - z * q.y + x * q.w + w * q.x,
						  z * q.x - x * q.z + y * q.w + w * q.y,
						  x * q.y - y * q.x + z * q.w + w * q.z,
						  w * q.w - x * q.x - y * q.y - z * q.z);
		);
	}

	Quaternion operator*(double s) const 
	{
		return Quaternion(complex() * s, real() * s);
	}

	Quaternion operator/(double s) const
	{
		return Quaternion(complex() / s, real() / s);
	}

	/*----- METHODS -----*/

	static Quaternion slerp(const Quaternion& q_a, const Quaternion& q_b, double time);

	XMFLOAT4 vector() const { return XMFLOAT4(x, y, z, w); }

	XMFLOAT3 complex() const { return XMLFLOAT3(x, y, z); }

	double real() const { return w; }

	void normalize();

	XMFLOAT3 euler() const;

	Matrix33D to_transform_matrix() const;

	Quaternion inverse() const
	{
		return conjugate() / normalize();
	}

	Quaternion conjugate() const
	{
		return Quaternion(q, -x, -y, -z);
	}
};

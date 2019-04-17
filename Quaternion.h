#pragma once

#include <vector>


class Quaternion
{

private:

	float w;
	float x;
	float y;
	float z;
	

public:

	Quaternion();

	Quaternion(float w, float x, float y, float z);

	Quaternion(const Quaternion & q);

	Quaternion(float v[4]);


public:

	float get_w()const { return w; }
	float get_x()const { return x; }
	float get_y()const { return y; }
	float get_z()const { return z; }

	static void slerp(Quaternion q_a, Quaternion q_b, Quaternion &q_r, float time);

	void normalize();

	Matrix33 & to_transform_matrix();


};

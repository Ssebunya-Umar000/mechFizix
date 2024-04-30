#ifndef TENSORCALCULATION_H
#define TENSORCALCULATION_H

#include"collision/collider.h"

namespace mech {

	Mat3x3 calculateTensor(const decimal& density, const Capsule& capsule)
	{
		Mat3x3 tensor;

		decimal height = capsule.capsuleLine.getLength();
		decimal rSq = square(capsule.radius);

		decimal cylinderMass = mathPI * height * rSq * density;
		decimal sphereMass = mathPI * decimal(2.0) * (decimal(1.0) / decimal(3.0)) * rSq * capsule.radius * density;

		tensor.rowXcol(1, 1) = rSq * cylinderMass * decimal(0.5);
		tensor.rowXcol(0, 0) = tensor.rowXcol(2, 2) = tensor.rowXcol(1, 1) * decimal(0.5) + cylinderMass * height * height * (decimal(1.0) / decimal(12.0));

		decimal temp0 = sphereMass * decimal(2.0) * rSq / decimal(5.0);
		tensor.rowXcol(1, 1) += temp0 * decimal(2.0);

		decimal temp1 = height * decimal(0.5);
		decimal temp2 = temp0 + sphereMass * (temp1 * temp1 + decimal(3.0) * (decimal(1.0) / decimal(8.0)) * height * capsule.radius);

		tensor.rowXcol(0, 0) += temp2 * decimal(2.0);
		tensor.rowXcol(2, 2) += temp2 * decimal(2.0);

		return tensor;
	}

	//solid sphere
	Mat3x3 calculateTensor(const decimal& mass, const Sphere& sphere)
	{
		Mat3x3 tensor;

		decimal v = (decimal(2.0) / decimal(5.0)) * mass * square(sphere.radius);

		tensor.rowXcol(0, 0) = v;
		tensor.rowXcol(1, 1) = v;
		tensor.rowXcol(2, 2) = v;

		return tensor;
	}

	Mat3x3 calculateTensor(const decimal& mass, const DynamicArray<Vec3, uint32>& points)
	{
		Mat3x3 tensor;
		for (uint32 x = 0, len = points.size(); x < len; ++x) {

			tensor.rowXcol(0, 0) += (points[x].y * points[x].y + points[x].z * points[x].z);
			tensor.rowXcol(1, 1) += (points[x].x * points[x].x + points[x].z * points[x].z);
			tensor.rowXcol(2, 2) += (points[x].x * points[x].x + points[x].y * points[x].y);

			tensor.rowXcol(0, 1) -= (points[x].x * points[x].y);
			tensor.rowXcol(0, 2) -= (points[x].x * points[x].z);
			tensor.rowXcol(1, 2) -= (points[x].y * points[x].z);
		}

		return tensor * mass;
	}
}

#endif

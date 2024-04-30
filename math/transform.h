/*
	MIT License

	Copyright (c) 2024 Ssebunya Umar

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in all
	copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
	SOFTWARE.
*/


//@ssebunya_umar - X(twitter)

#ifndef TRANSFORM_H
#define TRANSFORM_H

#include"quaternion.h"

namespace mech {

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	struct Transform3D {
		
		Vec3 position;
		Quaternion orientation = IDENTITY_QUATERNION;

		Transform3D(){}
		explicit Transform3D(const Vec3& p) : position(p) {}
		explicit Transform3D(const Quaternion& o) : orientation(o) {}
		explicit Transform3D(const Vec3& p, const Quaternion& o) : position(p), orientation(o) {}

		Transform3D operator*(const Transform3D& other) const { return Transform3D(this->position + this->orientation * other.position, this->orientation * other.orientation); }
		Vec3 operator*(const Vec3& vec) const { return (this->orientation * vec) + this->position; }

		Mat4x4 toMatrix() const
		{
			Mat4x4 mat;
			mat = Mat4x4(matrixFromQuarternion(this->orientation));
			mat.setColumn(3, Vec4(this->position, decimal(1.0)));
			return mat;
		}

		String toString() const { return "Transform3D(" + this->position.toString() + " " + this->orientation.toString() + ")"; }
	};

	inline static Transform3D getInverse(const Transform3D& t)
	{
		Quaternion invQuaternion = getInverse(t.orientation);
		return Transform3D(invQuaternion * (-t.position), invQuaternion);
	}

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	struct Transform3DRange {
		Transform3D inverseTransform2;
		Transform3D transform1;
		Transform3D transform2;

		Transform3DRange() {}
		Transform3DRange(const Transform3D& t1, const Transform3D& t2) : transform1(t1), transform2(t2)
		{
			this->inverseTransform2 = getInverse(this->transform2);
		}

		Transform3D interpolate(const decimal& factor) const
		{
			return this->inverseTransform2 * Transform3D(lerp(this->transform1.position, this->transform2.position, factor), slerp(this->transform1.orientation, this->transform2.orientation, factor));
		}
	};
}

#endif



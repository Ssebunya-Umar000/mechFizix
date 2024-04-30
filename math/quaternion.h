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

#ifndef QAUTERNION_H
#define QUATERNION_H

#include"matrix.h"

namespace mech {

	struct Quaternion
	{
		union {
			struct {
				decimal s, x, y, z;
			};
			struct {
				decimal s;
				Vec3 vec;
			};
			RawMatrix<decimal, 4, 1, Alignment::columnMajor> mat;
			//RawMatrix<decimal, 1, 4, Alignment::rowMajor> mat;
		};

		Quaternion() : s(decimal(0.0)), x(decimal(0.0)), y(decimal(0.0)), z(decimal(0.0)) {}
		explicit Quaternion(const decimal& s, const decimal& x, const decimal& y, const decimal& z) : s(s), x(x), y(y), z(z) {}
		explicit Quaternion(const decimal& s, const Vec3& vec) : s(s), x(vec.x), y(vec.y), z(vec.z) {}
		explicit Quaternion(const decimal* m, const byte& size) : mat(m) { ASSERT(size == 4, "array size must be equal to 4"); }
		~Quaternion() {}

		Quaternion operator+(const Quaternion& other) const { return Quaternion((this->mat + other.mat).data, 4); }
		Quaternion operator-(const Quaternion& other) const { return Quaternion((this->mat - other.mat).data, 4); }
		Quaternion operator*(const Quaternion& other) const { return Quaternion((this->s * other.s - dotProduct(this->vec, other.vec)), Vec3(other.vec * this->s + this->vec * other.s + crossProduct(this->vec, other.vec))); }
		Quaternion operator/(const Quaternion& other) const { return Quaternion(this->s / other.s, this->x / other.x, this->y / other.y, this->z / other.z); }
		Quaternion operator*(const decimal& scalar) const { return Quaternion((this->mat * scalar).data, 4); }
		Quaternion operator/(const decimal& scalar) const { return Quaternion((this->mat / scalar).data, 4); }
		Quaternion operator-() const { return Quaternion((-this->mat).data, 4); }

		Vec3 operator*(const Vec3& vector) const { return ((*this * Quaternion(decimal(0.0), vector)) * Quaternion(this->s, -this->vec)).vec; }

		void operator+=(const Quaternion& other) { *this = *this + other; }
		void operator-=(const Quaternion& other) { *this = *this - other; }
		void operator*=(const Quaternion& other) { *this = *this * other; }
		void operator*=(const decimal& scalar) { *this = *this * scalar; }
		void operator/=(const decimal& scalar) { *this = *this / scalar; }
		void operator-() { *this = Quaternion(-this->s, -this->vec); }

		bool operator==(const Quaternion& other) const { return this->mat == other.mat; }
		bool operator!=(const Quaternion& other) const { return this->mat != other.mat; }

		String toString() const { return "Quaternion(" + this->mat.toString() + ")"; }
	};

#define nanQUATERNION Quaternion(decimalNAN, nanVEC3)
#define IDENTITY_QUATERNION Quaternion(decimal(1.0), Vec3())

	inline static decimal dotProduct(const Quaternion& q1, const Quaternion& q2)
	{
		return q1.s* q2.s + dotProduct(q1.vec, q2.vec);
	}

	inline static decimal magnitudeSq(const Quaternion& q)
	{
		return q.s* q.s + magnitudeSq(q.vec);
	}

	inline static decimal magnitude(const Quaternion& q)
	{
		return mathSQRT(magnitudeSq(q));
	}

	inline static Quaternion getConjugate(const Quaternion& q)
	{
		return Quaternion(q.s, -q.vec);
	}

	inline static Quaternion getInverse(const Quaternion& q)
	{
		return getConjugate(q);
	}

	inline static Quaternion normalise(const Quaternion& q)
	{
		decimal mag = magnitude(q);

		if (mag == decimal(0.0)) {
			ASSERT(false, "attempting to normalise zero quaternion");
			return nanQUATERNION;
		}

		return Quaternion(q.s / mag, q.vec / mag);
	}

	inline static Quaternion quaternionFromAxisAngleDeg(const Vec3& axis, const decimal& angle)
	{
		decimal a = degreesToRadians(angle) * decimal(0.5);
		return Quaternion(mathCOS(a), normalise(axis) * mathSIN(a));
	}

	inline static Quaternion quaternionFromAxisAngleRad(const Vec3& axis, const decimal& angle)
	{
		decimal a = angle * decimal(0.5);
		return Quaternion(mathCOS(a), normalise(axis) * mathSIN(a));
	}

	inline static Quaternion quaternionFromMatrix(const Mat3x3& matrix)
	{
		Quaternion q;

		decimal trace = matrix.rowXcol(0, 0) + matrix.rowXcol(1, 1) + matrix.rowXcol(2, 2);

		if (trace >= 0) {

			decimal s = mathSQRT(trace + decimal(1.0));
			decimal temp = decimal(0.5) / s;

			q.s = decimal(0.5) * s;
			q.x = (matrix.rowXcol(2, 1) - matrix.rowXcol(1, 2)) / (decimal(4.0) * temp);
			q.y = (matrix.rowXcol(0, 2) - matrix.rowXcol(2, 0)) / (decimal(4.0) * temp);
			q.z = (matrix.rowXcol(1, 0) - matrix.rowXcol(0, 1)) / (decimal(4.0) * temp);

		}
		else {
			byte i = 0;
			if (matrix.rowXcol(1, 1) > matrix.rowXcol(0, 0)) i = 1;
			if (matrix.rowXcol(2, 2) > matrix.rowXcol(i, i)) i = 2;

			if (i == 0) {
				decimal s = mathSQRT(matrix.rowXcol(0, 0) - (matrix.rowXcol(1, 1) + matrix.rowXcol(2, 2)) + decimal(1.0));
				decimal temp = decimal(0.5) / s;

				q.s = matrix.rowXcol(2, 1) - matrix.rowXcol(1, 2) * temp;
				q.x = decimal(0.5) * s;
				q.y = matrix.rowXcol(0, 1) + matrix.rowXcol(1, 0) * temp;
				q.z = matrix.rowXcol(2, 0) + matrix.rowXcol(0, 2) * temp;
			}
			else if (i == 1) {
				decimal s = mathSQRT(matrix.rowXcol(1, 1) - (matrix.rowXcol(2, 2) + matrix.rowXcol(0, 0)) + decimal(1.0));
				decimal temp = decimal(0.5) / s;

				q.s = matrix.rowXcol(0, 2) - matrix.rowXcol(2, 0) * temp;
				q.x = matrix.rowXcol(0, 1) + matrix.rowXcol(1, 0) * temp;
				q.y = decimal(0.5) * s;
				q.z = matrix.rowXcol(1, 2) + matrix.rowXcol(2, 1) * temp;
			}
			else {
				decimal s = mathSQRT(matrix.rowXcol(2, 2) - (matrix.rowXcol(0, 0) + matrix.rowXcol(1, 1)) + decimal(1.0));
				decimal temp = decimal(0.5) / s;

				q.s = matrix.rowXcol(1, 0) - matrix.rowXcol(0, 1) * temp;
				q.x = matrix.rowXcol(2, 0) - matrix.rowXcol(0, 2) * temp;
				q.y = matrix.rowXcol(1, 2) - matrix.rowXcol(2, 1) * temp;
				q.z = decimal(0.5) * s;
			}
		}

		return q;
	}

	inline static Mat3x3 matrixFromQuarternion(const Quaternion& q)
	{
		Mat3x3 mat;

		decimal sx = q.s * q.x;
		decimal sy = q.s * q.y;
		decimal sz = q.s * q.z;
		decimal xx = q.x * q.x;
		decimal xy = q.x * q.y;
		decimal xz = q.x * q.z;
		decimal yy = q.y * q.y;
		decimal yz = q.y * q.z;
		decimal zz = q.z * q.z;

		mat.rowXcol(0, 0) = decimal(1.0) - decimal(2.0) * (yy + zz);
		mat.rowXcol(1, 0) = decimal(2.0) * (xy + sz);
		mat.rowXcol(2, 0) = decimal(2.0) * (xz - sy);

		mat.rowXcol(0, 1) = decimal(2.0) * (xy - sz);
		mat.rowXcol(1, 1) = decimal(1.0) - decimal(2.0) * (xx + zz);
		mat.rowXcol(2, 1) = decimal(2.0) * (yz + sx);

		mat.rowXcol(0, 2) = decimal(2.0) * (xz + sy);
		mat.rowXcol(1, 2) = decimal(2.0) * (yz - sx);
		mat.rowXcol(2, 2) = decimal(1.0) - decimal(2.0) * (xx + yy);

		return mat;
	}

	inline static Vec3 eulerAnglesFromQuaternion(const Quaternion& q)
	{
		decimal num = decimal(2.0) * (q.x * q.z - q.s * q.y);

		Vec3	angles;
		if (mathABS(num) > decimal(0.999999)) {

			angles.x = radiansToDegrees(decimal(0.0));
			angles.y = radiansToDegrees(-(mathPI / decimal(2.0)) * num / mathABS(num));
			angles.z = radiansToDegrees(mathATAN2(-(decimal(2.0) * (q.x * q.y - q.s * q.z)), -num * decimal(2.0) * (q.x * q.z + q.s * q.y)));
		}
		else {
			decimal ss = q.s * q.s;
			decimal xx = q.x * q.x;
			decimal yy = q.y * q.y;
			decimal zz = q.z * q.z;

			angles.x = radiansToDegrees(mathATAN2(decimal(2.0) * (q.y * q.z + q.s * q.x), ss - xx - yy + zz));
			angles.y = radiansToDegrees(mathASIN(-num));
			angles.z = radiansToDegrees(mathATAN2(decimal(2.0) * (q.x * q.y + q.s * q.z), ss + xx - yy - zz));
		}

		return angles;
	}

	inline static decimal getRotationAngle(const Quaternion& q, const Vec3& axis)
	{
		return q.s == decimal(0.0) ? mathPI : decimal(2.0) * mathATAN(dotProduct(axis, q.vec) / q.s);
	}

	inline static Quaternion rotationQuaternion(const Vec3& vec)
	{
		decimal mag = magnitude(vec);
		if (mag > mathEPSILON) {
			Vec3 axis = vec / mag;
			mag /= decimal(2.0);

			return Quaternion(mathCOS(mag), axis * mathSIN(mag));
		}

		return IDENTITY_QUATERNION;
	}

	inline static Quaternion slerp(const Quaternion& q1, const Quaternion& q2, const decimal& factor)
	{
		char invert = 1;

		decimal cosineTheta = dotProduct(q1, q2);
		if (cosineTheta < decimal(0.0)) {
			cosineTheta = -cosineTheta;
			invert = -invert;
		}

		if (decimal(1.0) - cosineTheta < mathEPSILON) {
			return q1 * (decimal(1.0) - factor) + q2 * (factor * invert);
		}

		decimal theta = mathACOS(cosineTheta);
		decimal sineTheta = mathSIN(theta);

		decimal coeff1 = mathSIN((decimal(1.0) - factor) * theta) / sineTheta;
		decimal coeff2 = mathSIN(factor * theta) / sineTheta * invert;

		return q1 * coeff1 + q2 * coeff2;
	}
}

#endif


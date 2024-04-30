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

#ifndef VEC_H
#define VEC_H

#include"rawMatrix.h"

namespace mech {

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//Vec2/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	struct Vec2 {

		union {
			struct {
				decimal x, y;
			};
			RawMatrix<decimal, 2, 1, Alignment::columnMajor> mat;
			//RawMatrix<decimal, 1, 2, Alignment::rowMajor> mat;
		};


		Vec2() :x(decimal(0.0)), y(decimal(0.0)) {}
		explicit Vec2(const decimal& a) : x(a), y(a) {}
		explicit Vec2(const decimal& fx, const decimal& fy) : x(fx), y(fy) {}
		explicit Vec2(const decimal* m, const byte& size) : mat(m) { ASSERT(size == 2, "array size must be equal to 2"); }

		decimal& operator[](uint32 index) { return mat.data[index]; }
		const decimal& operator[](uint32 index) const { return mat.data[index]; }

		Vec2 operator+(const Vec2& other) const { return Vec2((this->mat + other.mat).data, 2); }
		Vec2 operator-(const Vec2& other) const { return Vec2((this->mat - other.mat).data, 2); }
		Vec2 operator*(const Vec2& other) const { return Vec2(this->x * other.x, this->y * other.y); }
		Vec2 operator/(const Vec2& other) const { return Vec2(this->x / other.x, this->y / other.y); }
		Vec2 operator*(const decimal& scalar) const { return Vec2((this->mat * scalar).data, 2); }
		Vec2 operator/(const decimal& scalar) const { return Vec2((this->mat / scalar).data, 2); }
		Vec2 operator-() const { return Vec2((-this->mat).data, 2); }

		void operator+=(const Vec2& other) { this->mat += other.mat; }
		void operator-=(const Vec2& other) { this->mat -= other.mat; }
		void operator*=(const Vec2& other) { this->x *= other.x; this->y *= other.y; }
		void operator*=(const decimal& scalar) { this->mat *= scalar; }
		void operator/=(const decimal& scalar) { this->mat /= scalar; }

		bool operator==(const Vec2& other) const { return this->mat == other.mat; }
		bool operator!=(const Vec2& other) const { return this->mat != other.mat; }

		String toString() const { return "Vec2(" + this->mat.toString() + ")"; }
	};

	inline static decimal magnitudeSq(const Vec2& v) {
		return v.x * v.x + v.y + v.y;
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	struct Vec2f {

		union {
			struct {
				float x, y;
			};
			struct {
				float u, v;
			};
			RawMatrix<float, 2, 1, Alignment::columnMajor> mat;
			//RawMatrix<float, 1, 2, Alignment::rowMajor> mat;
		};

		Vec2f() :x(float(0.0)), y(float(0.0)) {}
		explicit Vec2f(const float& a) : x(a), y(a) {}
		explicit Vec2f(const float& fx, const float& fy) : x(fx), y(fy) {}
	};



	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//Vec3/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	struct Vec3 {

		union {
			struct {
				decimal x, y, z;
			};
			RawMatrix<decimal, 3, 1, Alignment::columnMajor> mat;
			//RawMatrix<decimal, 1, 3, Alignment::rowMajor> mat;
		};


		Vec3() :x(decimal(0.0)), y(decimal(0.0)), z(decimal(0.0)) {}
		explicit Vec3(const decimal& a) : x(a), y(a), z(a) {}
		explicit Vec3(const decimal& fx, const decimal& fy, const decimal& fz) : x(fx), y(fy), z(fz) {}
		explicit Vec3(const Vec2& v, const decimal& z) : x(v.x), y(v.y), z(z) {}
		explicit Vec3(const decimal* m, const byte& size) : mat(m) { ASSERT(size == 3, "array size must be equal to 3"); }

		decimal& operator[](uint32 index) { return mat.data[index]; }
		const decimal& operator[](uint32 index) const { return mat.data[index]; }
		
		Vec3 operator+(const Vec3& other) const { return Vec3((this->mat + other.mat).data, 3); }
		Vec3 operator-(const Vec3& other) const { return Vec3((this->mat - other.mat).data, 3); }
		Vec3 operator*(const Vec3& other) const { return Vec3(this->x * other.x, this->y * other.y, this->z * other.z); }
		Vec3 operator/(const Vec3& other) const { return Vec3(this->x / other.x, this->y / other.y, this->z / other.z); }
		Vec3 operator*(const decimal& scalar) const { return Vec3((this->mat * scalar).data, 3); }
		Vec3 operator/(const decimal& scalar) const { return Vec3((this->mat / scalar).data, 3); }
		Vec3 operator-() const { return Vec3((-this->mat).data, 3); }

		void operator+=(const Vec3& other) { this->mat += other.mat; }
		void operator-=(const Vec3& other) { this->mat -= other.mat; }
		void operator*=(const Vec3& other) { this->x *= other.x; this->y *= other.y; this->z *= other.z; }
		void operator*=(const decimal& scalar) { this->mat *= scalar; }
		void operator/=(const decimal& scalar) { this->mat /= scalar; }

		bool operator==(const Vec3& other) const { return this->mat == other.mat; }
		bool operator!=(const Vec3& other) const { return this->mat != other.mat; }

		String toString() const { return "Vec3(" + this->mat.toString() + ")"; }
	};

#define XAXIS Vec3(decimal(1.0), decimal(0.0), decimal(0.0))
#define YAXIS Vec3(decimal(0.0), decimal(1.0), decimal(0.0))
#define ZAXIS Vec3(decimal(0.0), decimal(0.0), decimal(1.0))

#define nanVEC3 Vec3(decimalNAN)

	inline static decimal dotProduct(const Vec3& v1, const Vec3& v2)
	{
		return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;;
	}

	inline static decimal magnitudeSq(const Vec3& v)
	{
		return dotProduct(v, v);
	}

	inline static decimal magnitude(const Vec3& v)
	{
		return mathSQRT(magnitudeSq(v));
	}

	inline static Vec3 normalise(const Vec3& v)
	{
		decimal mag = magnitude(v);

		if (mag == decimal(0.0)) {
			ASSERT(false, "attempting to normalise zero vector");
			return nanVEC3;
		}

		return v / mag;
	}

	inline static Vec3 crossProduct(const Vec3& v1, const Vec3& v2)
	{
		return Vec3(v1.y * v2.z - v1.z * v2.y, v1.z * v2.x - v1.x * v2.z, v1.x * v2.y - v1.y * v2.x);
	}

	inline static Vec3 vectorTrippleProduct(const Vec3& v1, const Vec3& v2, const Vec3& v3)
	{
		return crossProduct(v1, crossProduct(v2, v3));
	}

	inline static decimal scalarTrippleProduct(const Vec3& v1, const Vec3& v2, const Vec3& v3)
	{
		return dotProduct(v1, crossProduct(v2, v3));
	}

	inline static decimal scalarProjection(const Vec3& v1, const Vec3& v2)
	{
		return dotProduct(v1, v2) / magnitudeSq(v1);
	}

	inline static Vec3 vectorProjection(const Vec3& v1, const Vec3& v2)
	{
		return v1 * scalarProjection(v1, v2);
	}

	inline static Vec3 vectorRejection(const Vec3& v1, const Vec3& v2)
	{
		return v2 - vectorProjection(v1, v2);
	}

	inline static Vec3 getPerpendicularVector(const Vec3& v)
	{
		if (mathABS(v.x) > mathABS(v.y)) {
			return Vec3(v.z, decimal(0.0), -v.x);
		}

		return Vec3(decimal(0.0), -v.z, v.y);
	}

	inline static Vec3 getPerpendicularVectorNormalised(const Vec3& v)
	{
		if (mathABS(v.x) > mathABS(v.y)) {
			decimal s = decimal(1.0) / mathSQRT(v.z * v.z + v.x * v.x);
			return Vec3(v.z * s, decimal(0.0), -v.x * s);
		}

		decimal s = decimal(1.0) / mathSQRT(v.z * v.z + v.y * v.y);
		return Vec3(decimal(0.0), -v.z * s, v.y * s);
	}

	inline static decimal  angleBetween(const Vec3& v1, const Vec3& v2)
	{
		return mathACOS(dotProduct(v1, v2) / (magnitude(v1) * magnitude(v2)));
	}

	inline static Vec3 minVec(const Vec3& v1, const Vec3& v2)
	{
		return Vec3(mathMIN(v1.x, v2.x), mathMIN(v1.y, v2.y), mathMIN(v1.z, v2.z));
	}

	inline static Vec3 maxVec(const Vec3& v1, const Vec3& v2)
	{
		return Vec3(mathMAX(v1.x, v2.x), mathMAX(v1.y, v2.y), mathMAX(v1.z, v2.z));
	}

	inline static Vec3 lerp(const Vec3& v1, const Vec3& v2, const decimal& factor)
	{
		return v1 * (decimal(1.0) - factor) + v2 * factor;
	}

	inline static Vec3 absVec(const Vec3& v)
	{
		return Vec3(mathABS(v.x), mathABS(v.y), mathABS(v.z));
	}

	inline static bool isNanVec(const Vec3& v)
	{
		return isnan(v.x) || isnan(v.y) || isnan(v.z);
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	struct Vec3f {

		union {
			struct {
				float x, y, z;
			};
			RawMatrix<float, 3, 1, Alignment::columnMajor> mat;
			//RawMatrix<float, 1, 3, Alignment::rowMajor> mat;
		};

		Vec3f() :x(float(0.0)), y(float(0.0)), z(float(0.0)) {}
		Vec3f(const Vec3& v) : x(v.x), y(v.y), z(v.z) {}
		explicit Vec3f(const float& a) : x(a), y(a), z(a) {}
		explicit Vec3f(const float& fx, const float& fy, const float& fz) : x(fx), y(fy), z(fz) {}
	};



	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//Vec4/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	struct Vec4 {

		union {
			struct {
				decimal x, y, z, w;
			};
			RawMatrix<decimal, 4, 1, Alignment::columnMajor> mat;
			//RawMatrix<decimal, 1, 4, Alignment::rowMajor> mat;
		};

		Vec4() :x(decimal(0.0)), y(decimal(0.0)), z(decimal(0.0)), w(decimal(0.0)) {}
		explicit Vec4(const decimal& a) : x(a), y(a), z(a), w(a) {}
		explicit Vec4(const decimal& fx, const decimal& fy, const decimal& fz, const decimal& fw) : x(fx), y(fy), z(fz), w(fw) {}
		explicit Vec4(const Vec2& v, const decimal& z, const decimal& w) : x(v.x), y(v.y), z(z), w(w) {}
		explicit Vec4(const Vec3& v, const decimal& w) : x(v.x), y(v.y), z(v.z), w(w) {}
		explicit Vec4(const decimal* m, const byte& size) : mat(m) { ASSERT(size == 4, "array size must be equal to 4"); }

		decimal& operator[](uint32 index) { return mat.data[index]; }
		const decimal& operator[](uint32 index) const { return mat.data[index]; }

		Vec4 operator+(const Vec4& other) const { return Vec4((this->mat + other.mat).data, 4); }
		Vec4 operator-(const Vec4& other) const { return Vec4((this->mat - other.mat).data, 4); }
		Vec4 operator*(const Vec4& other) const { return Vec4(this->x * other.x, this->y * other.y, this->z * other.z, this->w * other.w); }
		Vec4 operator/(const Vec4& other) const { return Vec4(this->x / other.x, this->y / other.y, this->z / other.z, this->w / other.w); }
		Vec4 operator*(const decimal& scalar) const { return Vec4((this->mat * scalar).data, 4); }
		Vec4 operator/(const decimal& scalar) const { return Vec4((this->mat / scalar).data, 4); }
		Vec4 operator-() const { return Vec4((-this->mat).data, 4); }

		void operator+=(const Vec4& other) { this->mat += other.mat; }
		void operator-=(const Vec4& other) { this->mat -= other.mat; }
		void operator*=(const Vec4& other) { this->x *= other.x; this->y *= other.y; this->z *= other.z; this->w *= other.w; }
		void operator*=(const decimal& scalar) { this->mat *= scalar; }
		void operator/=(const decimal& scalar) { this->mat /= scalar; }

		bool operator==(const Vec4& other) const { return this->mat == other.mat; }
		bool operator!=(const Vec4& other) const { return this->mat != other.mat; }

		Vec3 toVec3() const { return Vec3(this->x, this->y, this->z); }

		String toString() const { return "Vec4(" + this->mat.toString() + ")"; }
	};

#define nanVEC4 Vec4(decimalNAN)

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	struct Vec4f {

		union {
			struct {
				float x, y, z, w;
			};
			RawMatrix<float, 4, 1, Alignment::columnMajor> mat;
			//RawMatrix<float, 1, 4, Alignment::rowMajor> mat;
		};

		Vec4f() :x(float(0.0)), y(float(0.0)), z(float(0.0)), w(float(0.0)) {}
		explicit Vec4f(const float& a) : x(a), y(a), z(a), w(a) {}
		explicit Vec4f(const float& fx, const float& fy, const float& fz, const float& fw) : x(fx), y(fy), z(fz), w(fw) {}
		explicit Vec4f(const Vec3f& v, const float& w) : x(v.x), y(v.y), z(v.z), w(w) {}
	};
}

#endif

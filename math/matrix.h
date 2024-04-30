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

#ifndef MATRIX_H
#define MATRIX_H

#include"vec.h"

namespace mech {

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//Mat2x2///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	struct Mat2x2 {

		RawMatrix<decimal, 2, 2, Alignment::columnMajor> mat;
		//RawMatrix<decimal, 2, 2, Alignment::rowMajor> mat;

		Mat2x2() {}
		explicit Mat2x2(const decimal& a) { this->mat.setDiagnol(a); }
		explicit Mat2x2(const Vec2& v1, const Vec2& v2) { setC(this->mat, 0, v1.mat.data); setC(this->mat, 1, v2.mat.data); }
		explicit Mat2x2(const decimal* m, const byte& size) : mat(m) { ASSERT(size == 4, "array size must be equal to 4"); }
		~Mat2x2() {}

		decimal& operator[](uint32 index) { return mat[index]; }
		const decimal& operator[](uint32 index) const { return mat[index]; }

		decimal& rowXcol(const byte& row, const byte& col) { return rXc(this->mat, row, col); }
		decimal rowXcol(const byte& row, const byte& col) const { return rXc(this->mat, row, col); }
		
		void setRow(const byte& index, const Vec2& v) { setR(this->mat, index, v.mat.data); }
		void setColumn(const byte& index, const Vec2& v) { setC(this->mat, index, v.mat.data); }

		Vec2 getRow(const byte& index) const { return Vec2(getR(this->mat, index).data, 2); }
		Vec2 getColumn(const byte& index) const { return Vec2(getC(this->mat, index).data, 2); }

		Mat2x2 operator+(const Mat2x2& other) const { return Mat2x2((this->mat + other.mat).data, 4); }
		Mat2x2 operator-(const Mat2x2& other) const { return Mat2x2((this->mat - other.mat).data, 4); }
		Mat2x2 operator*(const Mat2x2& other) const { return Mat2x2((this->mat * other.mat).data, 4); }
		Mat2x2 operator*(const decimal& scalar) const { return Mat2x2((this->mat * scalar).data, 4); }
		Mat2x2 operator/(const decimal& scalar) const { return Mat2x2((this->mat / scalar).data, 4); }
		Mat2x2 operator-() const { return Mat2x2((-this->mat).data, 4); }

		Vec2 operator*(const Vec2& vector) const { return Vec2((this->mat * vector.mat).data, 2); } //column major
		//Vec2 operator*(const Vec2& vector) const { return Vec2((vector.mat * this->mat).data, 2); } //row major

		void operator+=(const Mat2x2& other) { this->mat += other.mat; }
		void operator-=(const Mat2x2& other) { this->mat -= other.mat; }
		void operator*=(const Mat2x2& other) { this->mat *= other.mat; }
		void operator*=(const decimal& scalar) { this->mat *= scalar; }
		void operator/=(const decimal& scalar) { this->mat /= scalar; }

		bool operator==(const Mat2x2& other) const { return this->mat == other.mat; }
		bool operator!=(const Mat2x2& other) const { return this->mat != other.mat; }

		String toString() const { return "Mat2x2(" + this->mat.toString() + ")"; }
	};

#define nanMAT2X2 Mat2x2(decimalNAN)

	inline static Mat2x2 getInverse(const Mat2x2& matrix)
	{
		decimal det = matrix.rowXcol(0, 0) * matrix.rowXcol(1, 1) - matrix.rowXcol(0, 1) * matrix.rowXcol(1, 0);

		if (det == decimal(0.0)) {
			ASSERT(false, "matrix has a zero determinant");
			return nanMAT2X2;
		}

		Mat2x2 inv;
		inv.setColumn(0, Vec2(matrix.rowXcol(1, 1), -matrix.rowXcol(1, 0)));
		inv.setColumn(1, Vec2(-matrix.rowXcol(0, 1), matrix.rowXcol(0, 0)));
		return inv * decimal(1.0) / det;;
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	struct Mat2x2f {

		RawMatrix<float, 2, 2, Alignment::columnMajor> mat;
		//RawMatrix<float, 2, 2, Alignment::rowMajor> mat;

		Mat2x2f() {}
		Mat2x2f(const Mat2x2& m) { this->mat[0] = m[0]; this->mat[1] = m[1]; this->mat[2] = m[2]; this->mat[3] = m[3]; }
		explicit Mat2x2f(const float& a) { this->mat.setDiagnol(a); }
	};



	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//Mat3x3///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	struct Mat3x3 {

		RawMatrix<decimal, 3, 3, Alignment::columnMajor> mat;
		//RawMatrix<decimal, 3, 3, Alignment::rowMajor> mat;

		Mat3x3() {}
		explicit Mat3x3(const decimal& a) { this->mat.setDiagnol(a); }
		explicit Mat3x3(const Vec3& v1, const Vec3& v2, const Vec3& v3) { setC(this->mat, 0, v1.mat.data); setC(this->mat, 1, v2.mat.data); setC(this->mat, 2, v3.mat.data); }
		explicit Mat3x3(const decimal* m, const byte& size) : mat(m) { ASSERT(size == 9, "array size must be equal to 9"); }
		~Mat3x3() {}

		decimal& operator[](uint32 index) { return mat[index]; }
		const decimal& operator[](uint32 index) const { return mat[index]; }

		decimal& rowXcol(const byte& row, const byte& col) { return rXc(this->mat, row, col); }
		decimal rowXcol(const byte& row, const byte& col) const { return rXc(this->mat, row, col); }
		
		void setRow(const byte& index, const Vec3& v) { setR(this->mat, index, v.mat.data); }
		void setColumn(const byte& index, const Vec3& v) { setC(this->mat, index, v.mat.data); }

		Vec3 getRow(const byte& index) const { return Vec3(getR(this->mat, index).data, 3); }
		Vec3 getColumn(const byte& index) const { return Vec3(getC(this->mat, index).data, 3); }

		Mat3x3 operator+(const Mat3x3& other) const { return Mat3x3((this->mat + other.mat).data, 9); }
		Mat3x3 operator-(const Mat3x3& other) const { return Mat3x3((this->mat - other.mat).data, 9); }
		Mat3x3 operator*(const Mat3x3& other) const { return Mat3x3((this->mat * other.mat).data, 9); }
		Mat3x3 operator*(const decimal& scalar) const { return Mat3x3((this->mat * scalar).data, 9); }
		Mat3x3 operator/(const decimal& scalar) const { return Mat3x3((this->mat / scalar).data, 9); }
		Mat3x3 operator-() const { return Mat3x3((-this->mat).data, 9); }

		Vec3 operator*(const Vec3& vector) const { return Vec3((this->mat * vector.mat).data, 3); } //column major
		//Vec3 operator*(const Vec3& vector) const { return Vec3((vector.mat * this->mat).data, 3); } //row major

		void operator+=(const Mat3x3& other) { this->mat += other.mat; }
		void operator-=(const Mat3x3& other) { this->mat -= other.mat; }
		void operator*=(const Mat3x3& other) { this->mat *= other.mat; }
		void operator*=(const decimal& scalar) { this->mat *= scalar; }
		void operator/=(const decimal& scalar) { this->mat /= scalar; }

		bool operator==(const Mat3x3& other) const { return this->mat == other.mat; }
		bool operator!=(const Mat3x3& other) const { return this->mat != other.mat; }

		String toString() const { return "Mat3x3(" + this->mat.toString() + ")"; }
	};

#define nanMAT3X3 Mat3x3(decimalNAN)

	inline static Mat3x3 getInverse(const Mat3x3& matrix)
	{
		const Vec3 a = matrix.getColumn(0);
		const Vec3 b = matrix.getColumn(1);
		const Vec3 c = matrix.getColumn(2);

		Vec3 r = crossProduct(a, b);

		decimal det = dotProduct(r, c);

		if (det == decimal(0.0)) {
			ASSERT(false, "matrix has a zero determinant");
			return nanMAT3X3;
		}

		decimal invDet = decimal(1.0) / det;

		Mat3x3 inv;
		inv.setRow(0, crossProduct(b, c) * invDet);
		inv.setRow(1, crossProduct(c, a) * invDet);
		inv.setRow(2, r * invDet);
		return inv;
	}

	inline static Mat3x3 getTranspose(const Mat3x3& matrix)
	{
		return Mat3x3(transpose(matrix.mat).data, 9);
	}

	inline static Mat3x3 getSkewSymmetricMatrix(const Vec3& vec)
	{
		return Mat3x3(Vec3(decimal(0.0), vec.z, -vec.y), Vec3(-vec.z, decimal(0.0), vec.x), Vec3(vec.y, -vec.x, decimal(0.0)));
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	struct Mat3x3f {

		RawMatrix<float, 3, 3, Alignment::columnMajor> mat;
		//RawMatrix<float, 3, 3, Alignment::rowMajor> mat;

		Mat3x3f() {}
		Mat3x3f(const Mat3x3& m) { this->mat[0] = m[0]; this->mat[1] = m[1]; this->mat[2] = m[2]; this->mat[3] = m[3]; this->mat[4] = m[4]; this->mat[5] = m[5]; this->mat[6] = m[6]; this->mat[7] = m[7]; this->mat[8] = m[8]; }
		explicit Mat3x3f(const float& a) { this->mat.setDiagnol(a); }
	};



	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//Mat4x4///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	struct Mat4x4 {

		RawMatrix<decimal, 4, 4, Alignment::columnMajor> mat;
		//RawMatrix<decimal, 4, 4, Alignment::rowMajor> mat;

		Mat4x4() {}
		explicit Mat4x4(const decimal& a) { this->mat.setDiagnol(a); }
		explicit Mat4x4(const Vec4& v1, const Vec4& v2, const Vec4& v3, const Vec4& v4) { setC(this->mat, 0, v1.mat.data); setC(this->mat, 1, v2.mat.data); setC(this->mat, 2, v3.mat.data); setC(this->mat, 3, v4.mat.data); }
		explicit Mat4x4(const Mat3x3& other) { this->setColumn(0, Vec4(Vec3(getC(other.mat, 0).data, 3), decimal(0.0))); this->setColumn(1, Vec4(Vec3(getC(other.mat, 1).data, 3), decimal(0.0))); this->setColumn(2, Vec4(Vec3(getC(other.mat, 2).data, 3), decimal(0.0))); this->setColumn(3, Vec4(Vec3(), decimal(1.0))); }
		explicit Mat4x4(const decimal* m, const byte& size) : mat(m) { ASSERT(size == 16, "array size must be equal to 16"); }
		~Mat4x4() {}

		decimal& operator[](uint32 index) { return mat[index]; }
		const decimal& operator[](uint32 index) const { return mat[index]; }

		decimal& rowXcol(const byte& row, const byte& col) { return rXc(this->mat, row, col); }
		decimal rowXcol(const byte& row, const byte& col) const { return rXc(this->mat, row, col); }
		
		void setRow(const byte& index, const Vec4& v) { setR(this->mat, index, v.mat.data); }
		void setColumn(const byte& index, const Vec4& v) { setC(this->mat, index, v.mat.data); }
		
		Vec4 getRow(const byte& index) const { return Vec4(getR(this->mat, index).data, 4); }
		Vec4 getColumn(const byte& index) const { return Vec4(getC(this->mat, index).data, 4); }

		Mat4x4 operator+(const Mat4x4& other) const { return Mat4x4((this->mat + other.mat).data, 16); }
		Mat4x4 operator-(const Mat4x4& other) const { return Mat4x4((this->mat - other.mat).data, 16); }
		Mat4x4 operator*(const Mat4x4& other) const { return Mat4x4((this->mat * other.mat).data, 16); }
		Mat4x4 operator*(const decimal& scalar) const { return Mat4x4((this->mat * scalar).data, 16); }
		Mat4x4 operator/(const decimal& scalar) const { return Mat4x4((this->mat / scalar).data, 16); }
		Mat4x4 operator-() const { return Mat4x4((-this->mat).data, 16); }

		Vec4 operator*(const Vec4& vector) const { return Vec4((this->mat * vector.mat).data, 4); } //column major
		//Vec4 operator*(const Vec4 & vector) const { return Vec4((vector.mat * this->mat).data, 4); } //row major
		Vec3 operator*(const Vec3& vector) const { return (*this * Vec4(vector, decimal(1.0))).toVec3(); }

		void operator+=(const Mat4x4& other) { this->mat += other.mat; }
		void operator-=(const Mat4x4& other) { this->mat -= other.mat; }
		void operator*=(const Mat4x4& other) { this->mat *= other.mat; }
		void operator*=(const decimal& scalar) { this->mat *= scalar; }
		void operator/=(const decimal& scalar) { this->mat /= scalar; }

		bool operator==(const Mat4x4& other) const { return this->mat == other.mat; }
		bool operator!=(const Mat4x4& other) const { return this->mat != other.mat; }

		Mat3x3 toMat3x3() const
		{
			Mat3x3 mat;
			mat.setColumn(0, this->getColumn(0).toVec3());
			mat.setColumn(1, this->getColumn(1).toVec3());
			mat.setColumn(2, this->getColumn(2).toVec3());
			return mat;
		}

		String toString() const { return "Mat4x4(" + this->mat.toString() + ")"; }
	};

#define nanMAT4X4 Mat4x4(decimalNAN)

	inline static Mat4x4 getInverse(const Mat4x4& matrix)
	{
		Vec3 a = matrix.getColumn(0).toVec3();
		Vec3 b = matrix.getColumn(1).toVec3();
		Vec3 c = matrix.getColumn(2).toVec3();
		Vec3 d = matrix.getColumn(3).toVec3();

		Vec4 row4 = matrix.getRow(3);

		Vec3 s = crossProduct(a, b);
		Vec3 t = crossProduct(c, d);
		Vec3 u = a * row4.y - b * row4.x;
		Vec3 v = c * row4.w - d * row4.z;

		decimal det = dotProduct(s, v) + dotProduct(t, u);

		if (det == decimal(0.0)) {
			ASSERT(false, "matrix has a zero determinant");
			return nanMAT4X4;
		}

		decimal invDet = decimal(1.0) / det;

		s *= invDet;
		t *= invDet;
		u *= invDet;
		v *= invDet;

		Mat4x4 inv;
		inv.setRow(0, Vec4(crossProduct(b, v) + t * row4.y, -dotProduct(b, t)));
		inv.setRow(1, Vec4(crossProduct(v, a) - t * row4.x, dotProduct(a, t)));
		inv.setRow(2, Vec4(crossProduct(d, u) + s * row4.w, -dotProduct(d, s)));
		inv.setRow(3, Vec4(crossProduct(u, c) - s * row4.z, dotProduct(c, s)));

		return inv;
	}

	inline static Mat4x4 getTranspose(const Mat4x4& matrix)
	{
		return Mat4x4(transpose(matrix.mat).data, 16);
	}

	inline static Mat4x4 orthographicViewMatrix(const decimal& right, const decimal& left, const decimal& top, const decimal& bottom, const decimal& farPlane, const decimal& nearPlane)
	{
		Mat4x4 ort(1.0);

		ort.rowXcol(0, 0) = decimal(2.0) / (right - left);
		ort.rowXcol(1, 1) = decimal(2.0) / (top - bottom);
		ort.rowXcol(2, 2) = -decimal(2.0) / (farPlane - nearPlane);

		ort.rowXcol(0, 3) = -(right + left) / (right - left);
		ort.rowXcol(1, 3) = -(top + bottom) / (top - bottom);
		ort.rowXcol(2, 3) = -(farPlane + nearPlane) / (farPlane - nearPlane);

		return ort;
	}

	inline static Mat4x4 perspectiveViewMatrix(const decimal& nearPlane, const decimal& farPlane, const decimal& fov, const decimal& aspectRatio)
	{
		decimal a = decimal(1.0) / tanf(degreesToRadians(fov * decimal(0.5)));
		decimal nearPlaneMinusFar = nearPlane - farPlane;

		Mat4x4 pers;
		pers.rowXcol(0, 0) = a / aspectRatio;
		pers.rowXcol(1, 1) = a;
		pers.rowXcol(2, 2) = (-nearPlane - farPlane) / nearPlaneMinusFar;
		pers.rowXcol(2, 3) = (decimal(2.0) * farPlane * nearPlane) / nearPlaneMinusFar;
		pers.rowXcol(3, 2) = decimal(1.0);
		return pers;
	}

	inline static Mat4x4 lookAtMatrix(const Vec3& eye, const Vec3& target, const Vec3& up)
	{
		Vec3 zAxis = normalise(target - eye);
		Vec3 xAxis = normalise(crossProduct(zAxis, up));
		Vec3 yAxis = crossProduct(xAxis, zAxis);

		Mat4x4 result(decimal(1.0));
		result.setRow(0, Vec4(xAxis, decimal(0.0)));
		result.setRow(1, Vec4(yAxis, decimal(0.0)));
		result.setRow(2, Vec4(zAxis, decimal(0.0)));
		result.setColumn(3, Vec4(-dotProduct(xAxis, eye), -dotProduct(yAxis, eye), -dotProduct(zAxis, eye), decimal(1.0)));
		return result;
	}

	inline static Mat4x4 translationMatrix(const Vec3& pos)
	{
		Mat4x4 result(decimal(1.0));
		result.setColumn(3, Vec4(pos, decimal(1.0)));
		return result;
	}

	inline static Mat4x4 translationMatrix(const Mat4x4& mat, const Vec3& pos)
	{
		return mat * translationMatrix(pos);
	}

	inline static Mat4x4 rotationMatrix(const decimal& angle, const Vec3& axis)
	{
		decimal c = mathCOS(degreesToRadians(angle));
		decimal s = mathSIN(degreesToRadians(angle));
		decimal d = decimal(1.0) - c;

		Vec3 a = normalise(axis);

		Mat4x4 result(decimal(1.0));

		result.rowXcol(0, 0) = a.x * a.x * d + c;
		result.rowXcol(1, 0) = a.x * a.y * d + a.z * s;
		result.rowXcol(2, 0) = a.x * a.z * d - a.y * s;

		result.rowXcol(0, 1) = a.x * a.y * d - a.z * s;
		result.rowXcol(1, 1) = a.y * a.y * d + c;
		result.rowXcol(2, 1) = a.y * a.z * d + a.x * s;

		result.rowXcol(0, 2) = a.x * a.z * d + a.y * s;
		result.rowXcol(1, 2) = a.y * a.z * d - a.x * s;
		result.rowXcol(2, 2) = a.z * a.z * d + c;

		return result;
	}

	inline static Mat4x4 rotationMatrix(const Mat4x4& mat, const decimal& angle, const Vec3& axis)
	{
		return mat * rotationMatrix(angle, axis);
	}

	inline static Mat4x4 rotationMatrix(const Vec3& angle)
	{
		return rotationMatrix(angle.x, Vec3(1.0, 0.0, 0.0))* rotationMatrix(angle.y, Vec3(0.0, 1.0, 0.0))* rotationMatrix(angle.z, Vec3(0.0, 0.0, 1.0));
	}

	inline static Mat4x4 scaleMatrix(const Vec3& s)
	{
		Mat4x4 result(decimal(1.0));
		result.rowXcol(0, 0) = s.x;
		result.rowXcol(1, 1) = s.y;
		result.rowXcol(2, 2) = s.z;
		return result;
	}

	inline static Mat4x4 scaleMatrix(const Mat4x4& mat, const Vec3& s)
	{
		return mat * scaleMatrix(s);
	}
	
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	struct Mat4x4f {

		RawMatrix<float, 4, 4, Alignment::columnMajor> mat;
		//RawMatrix<float, 4, 4, Alignment::rowMajor> mat;

		Mat4x4f() {}
		Mat4x4f(const Mat4x4& m) { this->mat[0] = m[0]; this->mat[1] = m[1]; this->mat[2] = m[2]; this->mat[3] = m[3]; this->mat[4] = m[4]; this->mat[5] = m[5]; this->mat[6] = m[6]; this->mat[7] = m[7]; this->mat[8] = m[8]; this->mat[9] = m[9]; this->mat[10] = m[10]; this->mat[11] = m[11]; this->mat[12] = m[12]; this->mat[13] = m[13]; this->mat[14] = m[14]; this->mat[15] = m[15]; }
		explicit Mat4x4f(const float& a) { this->mat.setDiagnol(a); }
	};
}

#endif

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

#ifndef POINT_H
#define POINT_H

#include"../math/transform.h"

namespace mech {

	struct Point : public Vec3 {

		Point() {}
		Point(const Vec3& vec) : Vec3(vec) {}
		Point(const decimal& x, const decimal& y, const decimal& z) : Vec3(x, y, z) {}

		Vec3 getSupportPoint(const Vec3& direction) const { return *this; }
		void getSupportPoints(const Vec3& direction, Vec3& min, Vec3& max) const { min = *this; max = *this; }

		void transform(const Transform3D& t) { *this = t * Vec3(this->x, this->y, this->z); }
		Point transformed(const Transform3D& t) const { return t * Vec3(this->x, this->y, this->z); }

		String toString() const { return "Point(" + this->mat.toString() + ")"; }
	};
}

#endif

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

#ifndef MATERIAL_H
#define MATERIAL_H

#include"../core/core.h"

namespace mech {
	
	struct PhysicsMaterial {
		decimal density = decimalNAN;
		decimal restitution = decimalNAN;
		decimal frictionSqrt = decimalNAN;

		PhysicsMaterial() {}
		PhysicsMaterial(const decimal& d, const decimal& r, const decimal& f) : density(d), restitution(r), frictionSqrt(f) {}
	};

#define IRONMATERIAL PhysicsMaterial(decimal(7.8), decimal(0.01), decimal(0.22))
#define RUBBERMATERIAL PhysicsMaterial(decimal(1.3), decimal(0.6), decimal(0.24))
#define PLASTICMATERIAL PhysicsMaterial(decimal(1.4), decimal(0.1), decimal(0.15))
#define CONCRETEMATERIAL PhysicsMaterial(decimal(2.4), decimal(0.0001), decimal(0.4))
#define GROUNDMATERIAL PhysicsMaterial(decimalMAX, decimal(0.001), decimal(0.75))
}

#endif

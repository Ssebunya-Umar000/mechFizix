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

#ifndef MATHUTIL_H
#define MATHUTIL_H

#include<limits>

#include"../core/assert.h"
#include"../core/profiler.h"
#include"../core/logger.h"

namespace mech {

#if mech_ENABLE_DOUBLE_PRECISION

#define decimal double

#define decimalMAX DBL_MAX
#define decimalNAN ((double)std::numeric_limits<float>::quiet_NaN())

#define mathCOS cos
#define mathACOS acos
#define mathSIN sin
#define mathASIN asin
#define mathTAN tan
#define mathATAN atan
#define mathATAN2 atan2
#define mathSQRT sqrt
#define mathPOW pow
#define mathMIN fmin
#define mathMAX fmax
#define mathABS  fabs

#else

#define decimal float

#define decimalMAX FLT_MAX
#define decimalNAN ((float)std::numeric_limits<float>::quiet_NaN())

#define mathCOS cosf
#define mathACOS acosf
#define mathSIN sinf
#define mathASIN asinf
#define mathTAN tanf
#define mathATAN atanf
#define mathATAN2 atan2f
#define mathSQRT sqrtf
#define mathPOW powf
#define mathMIN fminf
#define mathMAX fmaxf
#define mathABS  fabsf

#endif

#define mathPI  decimal(3.14159265358979323846)
#define math2PI  decimal(2.0) * mathPI
#define mathEPSILON  decimal(0.000001)

	static bool almostEqual(const decimal& x, const decimal& y)
	{
		return (mathABS(x - y) <= mathEPSILON * mathMAX(1.0, mathMAX(mathABS(x), mathABS(y))));
	}

	static decimal degreesToRadians(const decimal& degrees)
	{
		return degrees * decimal(mathPI / decimal(180.0));
	}

	static decimal radiansToDegrees(const decimal& radians)
	{
		return (radians * decimal(180.0)) / decimal(mathPI);
	}

	static decimal clamp(const decimal& x, const decimal& min, const decimal& max)
	{
		return x < min ? min : x > max ? max : x;
	}

	static decimal square(const decimal& num)
	{
		return num * num;
	}

	static uint32 pairingFunction(const uint32& a, const uint32& b)
	{
		uint32 inA = a < b ? a : b;
		uint32 inB = inA == a ? b : a;
		return ((inA + inB) * (inA + inB + 1) / 2 + inB);
	}
}

#endif

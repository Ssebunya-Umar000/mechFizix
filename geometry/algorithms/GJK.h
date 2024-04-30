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

#ifndef GJK_H
#define GJK_H

#include"../../math/vec.h"

namespace mech {

#define MAXIMUM_GJK_ITERATIONS 50

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	struct GJK {

		struct Simplex {

			struct SimplexPoint {
				Vec3 W = nanVEC3; //point on A - B
				Vec3 P = nanVEC3; //point on A
				Vec3 Q = nanVEC3; //point on B
			};

			SimplexPoint points[4];
			byte numPoints = 0;

			void addPoint(const Vec3& support1, const Vec3& support2)
			{
				this->points[this->numPoints].W = support1 - support2;
				this->points[this->numPoints].P = support1;
				this->points[this->numPoints].Q = support2;
				this->numPoints++;
			}

			bool isAffinelyDependent()
			{
				bool result = false;

				if (this->numPoints == 1) {
					result = true;
				}
				else if (this->numPoints == 2) {
					result = magnitudeSq(this->points[0].W - this->points[1].W) > mathEPSILON;
				}
				else if (this->numPoints == 3) {
					result = magnitudeSq(crossProduct(this->points[1].W - this->points[0].W, this->points[2].W - this->points[0].W)) > mathEPSILON;
				}
				else if (this->numPoints == 4) {
					result = mathABS(dotProduct((this->points[1].W - this->points[0].W), crossProduct((this->points[2].W - this->points[0].W), this->points[3].W - this->points[0].W))) > mathEPSILON;
				}

				if (result == false) {
					--this->numPoints;
					this->points[this->numPoints] = SimplexPoint();
				}

				return result;
			}

			bool containsOrigin()
			{
				if (this->numPoints == 1) {
					return magnitudeSq(this->points[0].W) < mathEPSILON;
				}
				else if (this->numPoints == 2) {
					return dotProduct(this->points[0].W, this->points[1].W) < decimal(0.0) && almostEqual(magnitudeSq(crossProduct(this->points[0].W, this->points[1].W)), decimal(0.0));
				}
				else if (this->numPoints == 3) {

					byte i[3][2] = { {0,1}, {1,2}, {2,0} };
					decimal total = 0;
					for (byte x = 0; x < 3; ++x) {
						decimal angle = mathABS(angleBetween(this->points[i[x][0]].W, this->points[i[x][1]].W));
						total += almostEqual(angle, mathPI) ? decimal(0.0) : angle;
					}

					return almostEqual(total, math2PI);
				}
				else if (this->numPoints == 4) {

					bool in = true;
					byte i[3][3] = { {0,3,1}, {0,2,3}, {1,3,2} };
					for (byte x = 0; x < 3; ++x) {
						if (dotProduct(this->points[i[x][0]].W, crossProduct(this->points[i[x][1]].W - this->points[i[x][0]].W, this->points[i[x][2]].W - this->points[i[x][0]].W)) < decimal(0.0)) {
							in = false;
							break;
						}
					}
					return in;
				}

				return false;
			}
		};

		Simplex simplex;
		Simplex::SimplexPoint closestPoint;
		Vec4 barrycentricCoords;

		void update()
		{
			struct TaskExecutor {

				Vec2 lineBarryCentrincCoords(const Vec3& a, const Vec3& b)
				{
					Vec3 ab = b - a;
					decimal apDotab = dotProduct(-a, ab);
					decimal lenSq = magnitudeSq(ab);

					decimal v = apDotab < decimal(0.0) ? decimal(0.0) : apDotab > lenSq ? decimal(1.0) : apDotab / lenSq;

					return Vec2(decimal(1.0) - v, v);
				}

				Vec3 triangleBarrycentricCoords(const Vec3& a, const Vec3& b, const Vec3& c)
				{
					Vec3 ab = b - a;
					Vec3 ac = c - a;
					Vec3 ap = -a;
					decimal d1 = dotProduct(ab, ap);
					decimal d2 = dotProduct(ac, ap);
					if (d1 <= decimal(0.0) && d2 <= decimal(0.0)) {
						return Vec3(decimal(1.0), decimal(0.0), decimal(0.0));
					}

					Vec3 bp = -b;
					decimal d3 = dotProduct(ab, bp);
					decimal d4 = dotProduct(ac, bp);
					if (d3 >= decimal(0.0) && d4 <= d3) {
						return Vec3(decimal(0.0), decimal(1.0), decimal(0.0));
					}

					decimal vc = d1 * d4 - d3 * d2;
					if (vc <= decimal(0.0) && d1 >= decimal(0.0) && d3 <= decimal(0.0)) {

						ASSERT(mathABS(d1 - d3) > mathEPSILON, "division by zero");

						decimal v = d1 / (d1 - d3);
						return Vec3(decimal(1.0) - v, v, decimal(0.0));
					}

					Vec3 cp = -c;
					decimal d5 = dotProduct(ab, cp);
					decimal d6 = dotProduct(ac, cp);
					if (d6 >= decimal(0.0) && d5 <= d6) {
						return Vec3(decimal(0.0), decimal(0.0), decimal(1.0));
					}

					decimal vb = d5 * d2 - d1 * d6;
					if (vb <= decimal(0.0) && d2 >= decimal(0.0) && d6 <= decimal(0.0)) {

						ASSERT(mathABS(d2 - d6) > mathEPSILON, "division by zero");

						decimal w = d2 / (d2 - d6);
						return Vec3(decimal(1.0) - w, decimal(0.0), w);
					}

					decimal va = d3 * d6 - d5 * d4;
					if (va <= decimal(0.0) && (d4 - d3) >= decimal(0.0) && (d5 - d6) >= decimal(0.0)) {

						ASSERT(mathABS((d4 - d3) + (d5 - d6)) > mathEPSILON, "division by zero");

						decimal w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
						return Vec3(decimal(0.0), decimal(1.0) - w, w);
					}

					decimal denom = decimal(1.0) / (va + vb + vc);

					decimal v = vb * denom;
					decimal w = vc * denom;
					return Vec3(decimal(1.0) - v - w, v, w);
				}

				Vec4 tetrahedronBarrycentricCoords(const Vec3 & a, const Vec3 & b, const Vec3 & c, const Vec3 & d)
				{
					Vec4 bCoords;
					Vec3 points[4] = { a, b, c, d };
					byte indicies[4][3] = { {0,1,2}, {0,2,3}, {0,3,1}, {1,3,2} };
					decimal least = decimalMAX;
					for (byte x = 0; x < 4; ++x) {

						Vec3 b = this->triangleBarrycentricCoords(points[indicies[x][0]], points[indicies[x][1]], points[indicies[x][2]]);
						decimal d = magnitudeSq(points[indicies[x][0]] * b[0] + points[indicies[x][1]] * b[1] + points[indicies[x][2]] * b[2]);
						if (d < least) {
							bCoords = Vec4();
							for (byte y = 0; y < 3; ++y) {
								bCoords[indicies[x][y]] = b[y];
							}
						}
					}

					return bCoords;
				}
			};

			this->barrycentricCoords = Vec4();

			TaskExecutor ex;

			if (this->simplex.numPoints == 1) {
				this->barrycentricCoords[0] = decimal(1.0);
			}
			else if (this->simplex.numPoints == 2) {

				this->barrycentricCoords = Vec4(ex.lineBarryCentrincCoords(this->simplex.points[0].W, this->simplex.points[1].W), decimal(0.0), decimal(0.0));
			}
			else if (this->simplex.numPoints == 3) {

				if (dotProduct(this->simplex.points[0].W, crossProduct(this->simplex.points[1].W - this->simplex.points[0].W, this->simplex.points[2].W - this->simplex.points[0].W)) < decimal(0.0)){
					Simplex::SimplexPoint temp = this->simplex.points[1];
					this->simplex.points[1] = this->simplex.points[2];
					this->simplex.points[2] = temp;
				}

				this->barrycentricCoords = Vec4(ex.triangleBarrycentricCoords(this->simplex.points[0].W, this->simplex.points[1].W, this->simplex.points[2].W), decimal(0.0));
			}
			else if (this->simplex.numPoints == 4) {

				this->barrycentricCoords = ex.tetrahedronBarrycentricCoords(this->simplex.points[0].W, this->simplex.points[1].W, this->simplex.points[2].W, this->simplex.points[3].W);
			}

			this->closestPoint.P = Vec3();
			this->closestPoint.Q = Vec3();

			byte count = 0;
			for (byte x = 0; x < this->simplex.numPoints; x++) {

				this->closestPoint.P += this->simplex.points[x].P * this->barrycentricCoords[x];
				this->closestPoint.Q += this->simplex.points[x].Q * this->barrycentricCoords[x];

				if (this->barrycentricCoords[x] > decimal(0.0)) {
					this->simplex.points[count] = this->simplex.points[x];
					++count;
				}
			}
			for (byte x = count; x < this->simplex.numPoints; x++) {
				this->simplex.points[x] = Simplex::SimplexPoint();
			}

			this->closestPoint.W = this->closestPoint.P - this->closestPoint.Q;
			this->simplex.numPoints = count;
		}
	};

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	template<typename T1, typename T2>
	static bool GJKOverlap(const T1& convexShape1, const T2& convexShape2, const decimal& tolerance = mathEPSILON)
	{
		GJK gjk;

		Vec3 searchDirection = Vec3();
		decimal prevLenSq = decimalMAX;

		byte iterations = 0;
		while (iterations++ < MAXIMUM_GJK_ITERATIONS) {

			gjk.simplex.addPoint(convexShape1.getSupportPoint(searchDirection), convexShape2.getSupportPoint(-searchDirection));

			if (dotProduct(searchDirection, gjk.simplex.points[gjk.simplex.numPoints - 1].W) < decimal(0.0)) return false;
			if (gjk.simplex.numPoints == 4) return gjk.simplex.containsOrigin();
			if (gjk.simplex.isAffinelyDependent() == false) return false;

			gjk.update();

			searchDirection = -gjk.closestPoint.W;
			decimal dirLenSq = magnitudeSq(searchDirection);

			if (dirLenSq <= tolerance) return true;
			if (prevLenSq - dirLenSq <= mathEPSILON * prevLenSq) return false;
			if (dirLenSq > prevLenSq) return false;

			prevLenSq = dirLenSq;
		}

		ASSERT(iterations < MAXIMUM_GJK_ITERATIONS, "GJKOverlap failed!!");

		return false;
	}

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	struct GJKDistanceResult {
		bool overlap = false;
		Vec3 searchDirection = nanVEC3;
		Vec3 closest1 = nanVEC3;
		Vec3 closest2 = nanVEC3;
	};

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	template<typename T1, typename T2>
	static GJKDistanceResult GJKDistance(const T1& convexShape1, const T2& convexShape2, const Vec3& direction = Vec3(), const decimal& tolerance = decimal(0.01))
	{
		BEGIN_PROFILE("GJKDistance");

		GJKDistanceResult result;

		GJK gjk;

		Vec3 searchDirection = direction;
		decimal prevLenSq = decimalMAX;

		byte iterations = 0;
		while (iterations++ < MAXIMUM_GJK_ITERATIONS) {

			gjk.simplex.addPoint(convexShape1.getSupportPoint(searchDirection), convexShape2.getSupportPoint(-searchDirection));

			if (gjk.simplex.isAffinelyDependent() == false) {
				result.overlap = false;
				break;
			}

			if (gjk.simplex.numPoints == 4) {
				result.overlap = gjk.simplex.containsOrigin();
				gjk.update();
				break;
			}

			gjk.update();

			searchDirection = -gjk.closestPoint.W;
			decimal dirLenSq = magnitudeSq(searchDirection);

			if (dirLenSq <= tolerance) {
				result.overlap = true;
				break;
			}

			if (prevLenSq - dirLenSq <= mathEPSILON * prevLenSq) {
				result.overlap = false;
				break;
			}

			if (dirLenSq > prevLenSq) {
				result.overlap = false;
				break;
			}

			prevLenSq = dirLenSq;
		}

		ASSERT(iterations < MAXIMUM_GJK_ITERATIONS, "GJKDistance failed!!");

		result.searchDirection = searchDirection;
		result.closest1 = gjk.closestPoint.P;
		result.closest2 = gjk.closestPoint.Q;

		END_PROFILE;
		return result;
	}
}

#endif

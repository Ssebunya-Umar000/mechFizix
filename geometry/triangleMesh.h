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

#ifndef TRIANGLEMESH_H
#define TRIANGLEMESH_H

#include"aabb.h"
#include"triangle.h"
#include"../containers/hybridArray.h"

namespace mech {

	struct TriangleMesh {

		//bounding volume hierachy
		struct BVH {

			struct Node {

				AABB bound;
				uint16 parent = -1;
				StackArray<uint16, 8> children; //StackArray<child index, ...
				DynamicArray<uint32, uint32> indicies; //DynamicArray<triangle index, ...

				Node() {}
				Node(const AABB& aabb) : bound(aabb) {}
			};

			DynamicArray<Node, uint16> nodes;
			uint16 parentNode = -1;

			BVH() {}

			void create(decimal* triangles, const uint32& numOfTriangles, const Vec3& limit);
		};
		
		BVH bvh;
		decimal* triangles = nullptr;
		uint32 numOfTriangles = 0;

		TriangleMesh() {}
		TriangleMesh(decimal* triangleData, const uint32& numberOfTriangles, const Vec3& minimumSizeOfBVHNode = Vec3(decimal(5.0)));

		ConvexHull toConvexHull();

		bool intersects(const AABB& aabb);
		void getTrianglesOverlapped(const AABB& aabb, HybridArray<Triangle, 24, uint16>& triangles);
	};
}

#endif
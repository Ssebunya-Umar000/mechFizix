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

//https://t.me/mechFizix - Telegram (if you have a question, remark or complaint)
//@ssebunya_umar - X(twitter)

#include"triangleMesh.h"

#include"convexHull.h"
#include"polygon.h"
#include"../../containers/AVLTree.h"

namespace mech {

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void TriangleMesh::BVH::create(decimal* triangleData, const uint32& numberOfTriangles, const Vec3& minimumSizeOfBVHNode)
	{
		struct Helper {
			
			BVH* bvh;

			Helper(BVH* b) : bvh(b) {}

			AABB getMinimumAABB(decimal* triangles, const uint32& numOfTriangles)
			{
				AABB aabb(Vec3(triangles[0], triangles[1], triangles[2]), Vec3(triangles[0], triangles[1], triangles[2]));
				for (uint32 x = 3, len = numOfTriangles * 9; x < len; x += 3) {
					Vec3 v = Vec3(triangles[x + 0], triangles[x + 1], triangles[x + 2]);
					aabb.min = minVec(v, aabb.min);
					aabb.max = maxVec(v, aabb.max);
				}
				return aabb;
			}

			void process(const uint16& nodeIndex, const decimal* triangles, const uint32& numOfTriangles, const Vec3& limit)
			{
				for (byte x = 0; x < 8; ++x) {

					uint32 childIndex = this->bvh->nodes.size();
					this->bvh->nodes.pushBack(Node(this->bvh->nodes[nodeIndex].bound.partition8(x)));
					this->bvh->nodes[nodeIndex].children.pushBack(childIndex);
					BVH::Node* newNode = &this->bvh->nodes.back();
					newNode->parent = nodeIndex;

					Vec3 dimensions = newNode->bound.getDimensions();
					if (dimensions.x > limit.x && dimensions.y > limit.y && dimensions.z > limit.z) {
						process(childIndex, triangles, numOfTriangles, limit);
					}
					else {
						for (uint32 y = 0, len = numOfTriangles * 9; y < len; y += 9) {
							Vec3 a = Vec3(triangles[y + 0], triangles[y + 1], triangles[y + 2]);
							Vec3 b = Vec3(triangles[y + 3], triangles[y + 4], triangles[y + 5]);
							Vec3 c = Vec3(triangles[y + 6], triangles[y + 7], triangles[y + 8]);
							if (newNode->bound.intersects(Triangle(a, b, c))) {
								newNode->indicies.pushBack(y);
							}
						}
					}
				}
			}

			void clean()
			{
				for (uint32 x = 0, len = this->bvh->nodes.size(); x < len; ++x) {

					if (this->bvh->nodes[x].indicies.empty() && this->bvh->nodes[x].children.empty()) {
							
						this->bvh->nodes[this->bvh->nodes[x].parent].children.eraseData(x);
						this->bvh->nodes.eraseDataAtIndex(x);

						uint16 last = len - 1;
						if (x != last) {
							this->bvh->nodes[this->bvh->nodes[x].parent].children.eraseData(last);
							this->bvh->nodes[this->bvh->nodes[x].parent].children.pushBack(x);

							for (auto it = this->bvh->nodes[x].children.begin(), end = this->bvh->nodes[x].children.end(); it != end; ++it) {
								this->bvh->nodes[it.data()].parent = x;
							}
						}

						--x;
						--len;
					}
				}
			}
		};

		Helper helper(this);

		this->parentNode = 0;
		this->nodes.pushBack(Node(helper.getMinimumAABB(triangleData, numberOfTriangles)));

		helper.process(this->parentNode, triangleData, numberOfTriangles, minimumSizeOfBVHNode);
		helper.clean();
	}

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	TriangleMesh::TriangleMesh(decimal* triangleData, const uint32& numberOfTriangles, const Vec3& minimumSizeOfBVHNode) : triangles(triangleData), numOfTriangles(numberOfTriangles)
	{
		bvh.create(this->triangles, this->numOfTriangles, minimumSizeOfBVHNode);
	}

	ConvexHull TriangleMesh::toConvexHull()
	{
		HybridArray<Polygon, 24, uint16> p;
		for (uint32 x = 0, len = this->numOfTriangles * 9; x < len; x += 9) {
			Vec3 a = Vec3(this->triangles[x + 0], this->triangles[x + 1], this->triangles[x + 2]);
			Vec3 b = Vec3(this->triangles[x + 3], this->triangles[x + 4], this->triangles[x + 5]);
			Vec3 c = Vec3(this->triangles[x + 6], this->triangles[x + 7], this->triangles[x + 8]);
			p.pushBack(Triangle(a, b, c).toPolygon());
		}

		return ConvexHull(p);
	}

	bool TriangleMesh::intersects(const AABB& aabb, HybridArray<Triangle, 24, uint16>& triangles)
	{
		struct Helper {

			TriangleMesh::BVH* bvh;
			DynamicArray<uint32, uint32> indicies;

			Helper(TriangleMesh::BVH* b) : bvh(b) {}
			
			void fetchIndices(const AABB& aabb, const uint16& nodeIndex)
			{
				for (auto it1 = this->bvh->nodes[nodeIndex].children.begin(), end1 = this->bvh->nodes[nodeIndex].children.end(); it1 != end1; ++it1) {

					if (this->bvh->nodes[it1.data()].bound.intersects(aabb)) {

						if (this->bvh->nodes[it1.data()].indicies.empty()) {
							fetchIndices(aabb, it1.data());
						}
						else {
							for (auto it2 = this->bvh->nodes[it1.data()].indicies.begin(), end2 = this->bvh->nodes[it1.data()].indicies.end(); it2 != end2; ++it2) {
								this->indicies.pushBack(it2.data());
							}
						}
					}
				}
			}
		};

		bool result = false;
		if (this->bvh.nodes[this->bvh.parentNode].bound.intersects(aabb)) {

			Helper helper(&this->bvh);
			helper.fetchIndices(aabb, this->bvh.parentNode);
			AVLTree<uint16, uint16> finished;

			for (uint32 x = 0, len = helper.indicies.size(); x < len; ++x) {

				Vec3 a = Vec3(this->triangles[helper.indicies[x] + 0], this->triangles[helper.indicies[x] + 1], this->triangles[helper.indicies[x] + 2]);
				Vec3 b = Vec3(this->triangles[helper.indicies[x] + 3], this->triangles[helper.indicies[x] + 4], this->triangles[helper.indicies[x] + 5]);
				Vec3 c = Vec3(this->triangles[helper.indicies[x] + 6], this->triangles[helper.indicies[x] + 7], this->triangles[helper.indicies[x] + 8]);
				Triangle t = Triangle(a, b, c);
				if (aabb.intersects(t)) {
					if (finished.find(helper.indicies[x]) == false) {
						triangles.pushBack(t);
						finished.insert(helper.indicies[x]);
					}
					
					result = true;
				}
			}
		}

		return result;
	}
}
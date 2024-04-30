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

#ifndef QUICKHULL_H
#define QUICKHULL_H

#include"../convexHull.h"
#include"../plane.h"
#include"../triangle.h"
#include"../lineSegment.h"
#include"../../containers/rigidArray.h"
#include"../../containers/queue.h"

namespace mech {

	bool quickHull3D (ConvexHull& hull, const DynamicArray<Vec3, uint16>& pointCloud)
	{
		struct qhAlgorithm {

			struct qhHalfEdgeMesh {

				struct qhEdge {
					uint16 vertIndex = -1;
					uint16 faceIndex = -1;
					uint16 nextIndex = -1;
					uint16 twinIndex = -1;
					bool duplicate = false;

					qhEdge() {}
					qhEdge(const uint16& v) : vertIndex(v) {}
				};

				struct qhFace {
					uint16 edgeIndex = -1;
					RigidArray<byte, byte> faceVerts;

					qhFace() {}
					qhFace(const uint16& e) : edgeIndex(e) {}
				};

				RigidArray<qhEdge, uint16> edges;
				RigidArray<qhFace, uint16> faces;

				uint16 nextEdge(const uint16& index) const
				{
					return this->edges[index].nextIndex;
				}

				uint16 twinEdge(const uint16& index) const
				{
					return this->edges[index].twinIndex;
				}

				void findTwin(const uint16& index)
				{
					for (auto it = this->edges.begin(), end = this->edges.end(); it != end; ++it) {
						qhHalfEdgeMesh::qhEdge* e = &it.data();
						if (e->twinIndex == (uint16)-1 && e->nextIndex != (uint16)-1) {
							if (this->edges[this->edges[index].nextIndex].vertIndex == e->vertIndex && this->edges[e->nextIndex].vertIndex == this->edges[index].vertIndex) {
								this->edges[index].twinIndex = it.index();
								e->twinIndex = index;
								e->duplicate = true;
								break;
							}
						}
					}
				}
			};

			using Face = qhHalfEdgeMesh::qhFace;
			using Edge = qhHalfEdgeMesh::qhEdge;

			struct qhEyePoint {
				bool valid = false;
				uint16 face = -1;
				uint16 indexInPointCloud = -1;
			};

			struct qhClaimedIndex {
				uint16 indexInFinalArray = -1;
				uint16 indexInPointCloud = -1;
			};

			qhEyePoint eyePoint;
			qhHalfEdgeMesh halfEdgeMesh;
			DynamicArray<qhClaimedIndex, uint16> claimedIndicies;
			RigidArray<uint16, uint16> unClaimedIndicies;
			uint16 numFaces = 0;
			uint16 numEdges = 0;
			uint16 numIterations = 0;

			void setEyePoint(const DynamicArray<Vec3, uint16>& pointCloud)
			{
				this->eyePoint.valid = false;
				decimal maxD = -decimalMAX;
				for (auto it1 = this->unClaimedIndicies.begin(), end1 = this->unClaimedIndicies.end(); it1 != end1;) {
					uint16 index = it1.data();
					++it1;
					
					bool pass = false;
					for (auto it2 = this->halfEdgeMesh.faces.begin(), end2 = this->halfEdgeMesh.faces.end(); it2 != end2; ++it2) {
						uint16 edge = it2.data().edgeIndex;

						uint16 index1 = this->claimedIndicies[this->halfEdgeMesh.edges[edge].vertIndex].indexInPointCloud;
						uint16 index2 = this->claimedIndicies[this->halfEdgeMesh.edges[this->halfEdgeMesh.nextEdge(edge)].vertIndex].indexInPointCloud;
						uint16 index3 = this->claimedIndicies[this->halfEdgeMesh.edges[this->halfEdgeMesh.nextEdge(this->halfEdgeMesh.nextEdge(edge))].vertIndex].indexInPointCloud;
						Vec3 normal = Triangle(pointCloud[index1], pointCloud[index2], pointCloud[index3]).getNormal();
						decimal d = dotProduct(normal, pointCloud[index] - pointCloud[index1]);
						if (d > decimal(0.0)) {
							pass = true;
							
							if (d > maxD) {
								maxD = d;

								this->eyePoint.face = it2.index();
								this->eyePoint.indexInPointCloud = index;
								this->eyePoint.valid = true;
							}
						}
					}

					if (pass == false) {
						this->unClaimedIndicies.eraseData(index);
					}
				}
			}

			bool createSimplex(const DynamicArray<Vec3, uint16>& pointCloud)
			{
				if (pointCloud.size() < 4) return false;

				StackArray<uint16, 4> verts(-1);
				
				decimal max1 = -decimalMAX;
				for (uint32 x = 0, len = pointCloud.size(); x < len; ++x) {

					this->unClaimedIndicies.insert(x);

					decimal d = magnitudeSq(pointCloud[x]);
					if (d > max1) {
						max1 = d;
						verts[0] = x;
					}
				}

				decimal max2 = -decimalMAX;
				for (uint32 x = 0, len = pointCloud.size(); x < len; ++x) {
					decimal d = magnitudeSq(pointCloud[x] - pointCloud[verts[0]]);
					if (d > max2) {
						max2 = d;
						verts[1] = x;
					}
				}

				LineSegment line(pointCloud[verts[0]], pointCloud[verts[1]]);
				decimal max3 = -decimalMAX;
				for (uint32 x = 0, len = pointCloud.size(); x < len; ++x) {
					if (line.contains(pointCloud[x])) continue;
					decimal d = Triangle(pointCloud[verts[0]], pointCloud[verts[1]], pointCloud[x]).calculateArea();
					if (d > max3) {
						max3 = d;
						verts[2] = x;
					}
				}

				if (verts[2] == (uint16)-1) return false;

				Triangle triangle = Triangle(pointCloud[verts[0]], pointCloud[verts[1]], pointCloud[verts[2]]);
				Plane plane = triangle.toPlane();
				decimal max4 = -decimalMAX;
				for (uint32 x = 0, len = pointCloud.size(); x < len; ++x) {
					if (plane.contains(pointCloud[x])) continue;
					decimal d = mathABS(dotProduct(plane.normal, pointCloud[x] - triangle.closestPoint(pointCloud[x])));
					if (d > max4) {
						max4 = d;
						verts[3] = x;
					}
				}

				if (verts[3] == (uint16)-1) return false;

				if (dotProduct(plane.normal, pointCloud[verts[3]] - triangle.getCentroid()) > decimal(0.0)){
					uint16 temp = verts[2];
					verts[2] = verts[1];
					verts[1] = temp;
				}

				//face 1
				uint16 f1 = this->halfEdgeMesh.faces.insert(Face());
				this->halfEdgeMesh.faces[f1].faceVerts.insert(0);
				this->halfEdgeMesh.faces[f1].faceVerts.insert(1);
				this->halfEdgeMesh.faces[f1].faceVerts.insert(2);

				uint16 e1 = this->halfEdgeMesh.edges.insert(Edge(0));
				uint16 e2 = this->halfEdgeMesh.edges.insert(Edge(1));
				uint16 e3 = this->halfEdgeMesh.edges.insert(Edge(2));
				this->halfEdgeMesh.edges[e1].nextIndex = e2;
				this->halfEdgeMesh.edges[e2].nextIndex = e3;
				this->halfEdgeMesh.edges[e3].nextIndex = e1;
				this->halfEdgeMesh.faces[f1].edgeIndex = e1;
				this->halfEdgeMesh.edges[e1].faceIndex = this->halfEdgeMesh.edges[e2].faceIndex = this->halfEdgeMesh.edges[e3].faceIndex = f1;

				//face 2
				uint16 f2 = this->halfEdgeMesh.faces.insert(Face());
				this->halfEdgeMesh.faces[f2].faceVerts.insert(0);
				this->halfEdgeMesh.faces[f2].faceVerts.insert(3);
				this->halfEdgeMesh.faces[f2].faceVerts.insert(1);

				uint16 e4 = this->halfEdgeMesh.edges.insert(Edge(0));
				uint16 e5 = this->halfEdgeMesh.edges.insert(Edge(3));
				uint16 e6 = this->halfEdgeMesh.edges.insert(Edge(1));
				this->halfEdgeMesh.edges[e4].nextIndex = e5;
				this->halfEdgeMesh.edges[e5].nextIndex = e6;
				this->halfEdgeMesh.edges[e6].nextIndex = e4;
				this->halfEdgeMesh.faces[f2].edgeIndex = e4;
				this->halfEdgeMesh.edges[e4].faceIndex = this->halfEdgeMesh.edges[e5].faceIndex = this->halfEdgeMesh.edges[e6].faceIndex = f2;

				//face 3
				uint16 f3 = this->halfEdgeMesh.faces.insert(Face());
				this->halfEdgeMesh.faces[f3].faceVerts.insert(0);
				this->halfEdgeMesh.faces[f3].faceVerts.insert(2);
				this->halfEdgeMesh.faces[f3].faceVerts.insert(3);

				uint16 e7 = this->halfEdgeMesh.edges.insert(Edge(0));
				uint16 e8 = this->halfEdgeMesh.edges.insert(Edge(2));
				uint16 e9 = this->halfEdgeMesh.edges.insert(Edge(3));
				this->halfEdgeMesh.edges[e7].nextIndex = e8;
				this->halfEdgeMesh.edges[e8].nextIndex = e9;
				this->halfEdgeMesh.edges[e9].nextIndex = e7;
				this->halfEdgeMesh.faces[f3].edgeIndex = e7;
				this->halfEdgeMesh.edges[e7].faceIndex = this->halfEdgeMesh.edges[e8].faceIndex = this->halfEdgeMesh.edges[e9].faceIndex = f3;

				//face 4
				uint16 f4 = this->halfEdgeMesh.faces.insert(Face());
				this->halfEdgeMesh.faces[f4].faceVerts.insert(1);
				this->halfEdgeMesh.faces[f4].faceVerts.insert(3);
				this->halfEdgeMesh.faces[f4].faceVerts.insert(2);

				uint16 e10 = this->halfEdgeMesh.edges.insert(Edge(1));
				uint16 e11 = this->halfEdgeMesh.edges.insert(Edge(3));
				uint16 e12 = this->halfEdgeMesh.edges.insert(Edge(2));
				this->halfEdgeMesh.edges[e10].nextIndex = e11;
				this->halfEdgeMesh.edges[e11].nextIndex = e12;
				this->halfEdgeMesh.edges[e12].nextIndex = e10;
				this->halfEdgeMesh.faces[f4].edgeIndex = e10;
				this->halfEdgeMesh.edges[e10].faceIndex = this->halfEdgeMesh.edges[e11].faceIndex = this->halfEdgeMesh.edges[e12].faceIndex = f4;

				//twining
				this->halfEdgeMesh.edges[e1].twinIndex = e6;
				this->halfEdgeMesh.edges[e6].twinIndex = e1;

				this->halfEdgeMesh.edges[e2].twinIndex = e12;
				this->halfEdgeMesh.edges[e12].twinIndex = e2;

				this->halfEdgeMesh.edges[e3].twinIndex = e7;
				this->halfEdgeMesh.edges[e7].twinIndex = e3;

				this->halfEdgeMesh.edges[e4].twinIndex = e9;
				this->halfEdgeMesh.edges[e9].twinIndex = e4;

				this->halfEdgeMesh.edges[e5].twinIndex = e10;
				this->halfEdgeMesh.edges[e10].twinIndex = e5;

				this->halfEdgeMesh.edges[e8].twinIndex = e11;
				this->halfEdgeMesh.edges[e11].twinIndex = e8;

				this->numFaces = 4;
				this->numEdges = 12;

				qhClaimedIndex ind0, ind1, ind2, ind3;

				ind0.indexInFinalArray = 0;
				ind0.indexInPointCloud = verts[0];
				ind1.indexInFinalArray = 1;
				ind1.indexInPointCloud = verts[1];
				ind2.indexInFinalArray = 2;
				ind2.indexInPointCloud = verts[2];
				ind3.indexInFinalArray = 3;
				ind3.indexInPointCloud = verts[3];

				this->claimedIndicies.pushBack(ind0);
				this->claimedIndicies.pushBack(ind1);
				this->claimedIndicies.pushBack(ind2);
				this->claimedIndicies.pushBack(ind3);

				for (byte x = 0; x < 4; ++x) {
					this->unClaimedIndicies.eraseData(verts[x]);
				}

				this->setEyePoint(pointCloud);

				return true;
			}

			uint16 prevEdge(uint16 edge)
			{
				uint16 prev = -1;
				uint16 e = edge;
				do {
					prev = e;
					e = this->halfEdgeMesh.nextEdge(e);
				} while (e != edge);

				return prev;
			}

			void iterate(const DynamicArray<Vec3, uint16>& pointCloud)
			{
				RigidArray<uint16, uint16> facesToRemove;
				facesToRemove.insert(this->eyePoint.face);

				//find faces that are visible from the eye point
				uint16 edge = this->halfEdgeMesh.edges[this->halfEdgeMesh.faces[this->eyePoint.face].edgeIndex].twinIndex;
				Queue<uint16, uint16> edgeList;
				while (true) {

					uint16 index1 = this->claimedIndicies[this->halfEdgeMesh.edges[edge].vertIndex].indexInPointCloud;
					uint16 index2 = this->claimedIndicies[this->halfEdgeMesh.edges[this->halfEdgeMesh.nextEdge(edge)].vertIndex].indexInPointCloud;
					uint16 index3 = this->claimedIndicies[this->halfEdgeMesh.edges[this->halfEdgeMesh.nextEdge(this->halfEdgeMesh.nextEdge(edge))].vertIndex].indexInPointCloud;
					Plane plane = Triangle(pointCloud[index1], pointCloud[index2], pointCloud[index3]).toPlane();

					if (plane.getDistanceFromPlane(pointCloud[this->eyePoint.indexInPointCloud]) > decimal(0.0)) {
						facesToRemove.insert(this->halfEdgeMesh.edges[edge].faceIndex);

						uint16 e = edge;
						do {
							if (facesToRemove.find(this->halfEdgeMesh.edges[this->halfEdgeMesh.edges[e].twinIndex].faceIndex) == nullptr) {
								edgeList.push(this->halfEdgeMesh.edges[e].twinIndex);
							}
							e = this->halfEdgeMesh.nextEdge(e);
						} while (this->halfEdgeMesh.nextEdge(e) != edge);
					}

					if (edgeList.empty() == true) {
						if (this->halfEdgeMesh.edges[this->halfEdgeMesh.edges[edge].twinIndex].faceIndex == this->eyePoint.face) {
							if (this->halfEdgeMesh.edges[this->halfEdgeMesh.edges[edge].twinIndex].nextIndex == this->halfEdgeMesh.faces[this->eyePoint.face].edgeIndex) {
								break;
							}
							else {
								edge = this->halfEdgeMesh.edges[this->halfEdgeMesh.edges[this->halfEdgeMesh.edges[edge].twinIndex].nextIndex].twinIndex;
							}
						}
						else {
							break;
						}
					}
					else {
						edge = edgeList.front();
						edgeList.pop();
					}
				}

				//find horizon edges and edges that should be removed
				RigidArray<uint16, uint16> edgesToLink;
				RigidArray<uint16, uint16> edgesToRemove;
				for (auto it1 = facesToRemove.begin(), end1 = facesToRemove.end(); it1 != end1; ++it1) {
					uint16 face = it1.data();

					uint16 edge = this->halfEdgeMesh.faces[face].edgeIndex;
					do {
						if (facesToRemove.find(this->halfEdgeMesh.edges[this->halfEdgeMesh.edges[edge].twinIndex].faceIndex)) {
							edgesToRemove.insert(edge);
						}
						else {
							edgesToLink.insert(edge);
						}

						edge = this->halfEdgeMesh.nextEdge(edge);
					} while (edge != this->halfEdgeMesh.faces[face].edgeIndex);

				}
				for (auto it = edgesToRemove.begin(), end = edgesToRemove.end(); it != end; ++it) {
					this->halfEdgeMesh.edges.eraseDataAtIndex(it.data());
					this->numEdges--;
				}
				for (auto it = facesToRemove.begin(), end = facesToRemove.end(); it != end; ++it) {
					this->halfEdgeMesh.faces.eraseDataAtIndex(it.data());
					this->numFaces--;
				}

				//create new faces with the edges of the horizon
				RigidArray<uint16, uint16> newFaces;

				qhClaimedIndex claimedIndex;
				claimedIndex.indexInFinalArray = this->claimedIndicies.size();
				claimedIndex.indexInPointCloud = this->eyePoint.indexInPointCloud;
				for (auto it1 = edgesToLink.begin(), end1 = edgesToLink.end(); it1 != end1; ++it1) {
					uint16 edge = it1.data();

					uint16 newFace = this->halfEdgeMesh.faces.insert(Face());

					uint16 newEdge1 = this->halfEdgeMesh.edges.insert(Edge());
					this->halfEdgeMesh.edges[newEdge1].vertIndex = claimedIndex.indexInFinalArray;
					this->halfEdgeMesh.edges[newEdge1].nextIndex = edge;
					this->halfEdgeMesh.edges[newEdge1].faceIndex = newFace;

					uint16 newEdge2 = this->halfEdgeMesh.edges.insert(Edge());
					this->halfEdgeMesh.edges[newEdge2].vertIndex = this->halfEdgeMesh.edges[this->halfEdgeMesh.edges[edge].twinIndex].vertIndex;
					this->halfEdgeMesh.edges[newEdge2].nextIndex = newEdge1;
					this->halfEdgeMesh.edges[newEdge2].faceIndex = newFace;

					this->halfEdgeMesh.edges[edge].faceIndex = newFace;
					this->halfEdgeMesh.edges[edge].nextIndex = newEdge2;

					this->halfEdgeMesh.faces[newFace].edgeIndex = newEdge1;
					uint16 e1 = this->halfEdgeMesh.faces[newFace].edgeIndex;
					do {

						for (auto it2 = this->halfEdgeMesh.edges.begin(), end2 = this->halfEdgeMesh.edges.end(); it2 != end2; ++it2) {
							uint16 e2 = it2.index();
							if (this->halfEdgeMesh.edges[e2].twinIndex == (uint16)-1) {
								if (this->halfEdgeMesh.edges[this->halfEdgeMesh.nextEdge(e1)].vertIndex == this->halfEdgeMesh.edges[e2].vertIndex && this->halfEdgeMesh.edges[this->halfEdgeMesh.nextEdge(e2)].vertIndex == this->halfEdgeMesh.edges[e1].vertIndex) {
									this->halfEdgeMesh.edges[e1].twinIndex = e2;
									this->halfEdgeMesh.edges[e2].twinIndex = e1;
									break;
								}
							}
						}

						this->halfEdgeMesh.faces[newFace].faceVerts.insert(this->halfEdgeMesh.edges[e1].vertIndex);

						e1 = this->halfEdgeMesh.nextEdge(e1);
					} while (e1 != this->halfEdgeMesh.faces[newFace].edgeIndex);

					newFaces.insert(newFace);

					this->numEdges += 2;
					this->numFaces += 1;
				}
				this->claimedIndicies.pushBack(claimedIndex);

				//look for co-planarity and merge faces that are co-planar
				for (auto it = newFaces.begin(), end = newFaces.end(); it != end;) {

					uint16 face = it.data();
					uint16 edge = this->halfEdgeMesh.faces[face].edgeIndex;

					bool repeat = false;

					uint16 index1 = this->claimedIndicies[this->halfEdgeMesh.edges[edge].vertIndex].indexInPointCloud;
					uint16 index2 = this->claimedIndicies[this->halfEdgeMesh.edges[this->halfEdgeMesh.nextEdge(edge)].vertIndex].indexInPointCloud;
					uint16 index3 = this->claimedIndicies[this->halfEdgeMesh.edges[this->halfEdgeMesh.nextEdge(this->halfEdgeMesh.nextEdge(edge))].vertIndex].indexInPointCloud;
					Plane plane = Triangle(pointCloud[index1], pointCloud[index2], pointCloud[index3]).toPlane();
					do {
						uint16 face2Edge = this->halfEdgeMesh.edges[edge].twinIndex;
						uint16 ind1 = this->claimedIndicies[this->halfEdgeMesh.edges[face2Edge].vertIndex].indexInPointCloud;
						uint16 ind2 = this->claimedIndicies[this->halfEdgeMesh.edges[this->halfEdgeMesh.nextEdge(face2Edge)].vertIndex].indexInPointCloud;
						uint16 ind3 = this->claimedIndicies[this->halfEdgeMesh.edges[this->halfEdgeMesh.nextEdge(this->halfEdgeMesh.nextEdge(face2Edge))].vertIndex].indexInPointCloud;
						if (plane.contains(Triangle(pointCloud[ind1], pointCloud[ind2], pointCloud[ind3]))) {
						
							//merge faces
							uint16 face1Edge = edge;

							uint16 p1 = this->prevEdge(face1Edge);
							uint16 p2 = this->prevEdge(face2Edge);

							this->halfEdgeMesh.edges[p1].nextIndex = this->halfEdgeMesh.edges[face2Edge].nextIndex;
							this->halfEdgeMesh.edges[p2].nextIndex = this->halfEdgeMesh.edges[face1Edge].nextIndex;

							this->halfEdgeMesh.faces[face].faceVerts.shallowClear(false);
							this->halfEdgeMesh.faces[face].edgeIndex = p1;
							uint16 e = this->halfEdgeMesh.faces[face].edgeIndex;
							do {
								this->halfEdgeMesh.edges[e].faceIndex = face;
								this->halfEdgeMesh.faces[face].faceVerts.insert(this->halfEdgeMesh.edges[e].vertIndex);

								e = this->halfEdgeMesh.edges[e].nextIndex;
							} while (e != this->halfEdgeMesh.faces[face].edgeIndex);

							newFaces.eraseData(this->halfEdgeMesh.edges[face2Edge].faceIndex);
							this->halfEdgeMesh.faces.eraseDataAtIndex(this->halfEdgeMesh.edges[face2Edge].faceIndex);

							this->halfEdgeMesh.edges.eraseDataAtIndex(face1Edge);
							this->halfEdgeMesh.edges.eraseDataAtIndex(face2Edge);

							this->numEdges -= 2;
							--this->numFaces;

							repeat = true;
							break;
						}

						edge = this->halfEdgeMesh.edges[edge].nextIndex;
					} while (edge != this->halfEdgeMesh.faces[face].edgeIndex);

					if (repeat == false) ++it;
				}

				//look for concave edges and resolve them
				for (auto it = newFaces.begin(), end = newFaces.end(); it != end;) {

					uint16 face = it.data();
					uint16 edge = this->halfEdgeMesh.faces[face].edgeIndex;

					bool repeat = false;

					uint16 index1 = this->claimedIndicies[this->halfEdgeMesh.edges[edge].vertIndex].indexInPointCloud;
					uint16 index2 = this->claimedIndicies[this->halfEdgeMesh.edges[this->halfEdgeMesh.nextEdge(edge)].vertIndex].indexInPointCloud;
					uint16 index3 = this->claimedIndicies[this->halfEdgeMesh.edges[this->halfEdgeMesh.nextEdge(this->halfEdgeMesh.nextEdge(edge))].vertIndex].indexInPointCloud;
					Plane plane = Triangle(pointCloud[index1], pointCloud[index2], pointCloud[index3]).toPlane();
					do {

						uint16  twin = this->halfEdgeMesh.edges[edge].twinIndex;
						if (plane.getDistanceFromPlane(pointCloud[this->claimedIndicies[this->halfEdgeMesh.edges[this->halfEdgeMesh.nextEdge(this->halfEdgeMesh.nextEdge(twin))].vertIndex].indexInPointCloud]) > decimal(0.0)) {

							uint16 p1 = this->prevEdge(edge);
							uint16 p2 = this->prevEdge(twin);

							uint16 nE1 = this->halfEdgeMesh.edges.insert(Edge());
							uint16 nE2 = this->halfEdgeMesh.edges.insert(Edge());
							this->halfEdgeMesh.edges[nE1].twinIndex = nE2;
							this->halfEdgeMesh.edges[nE2].twinIndex = nE1;

							this->halfEdgeMesh.edges[nE1].vertIndex = this->halfEdgeMesh.edges[p2].vertIndex;
							this->halfEdgeMesh.edges[nE1].nextIndex = p1;
							this->halfEdgeMesh.edges[p1].nextIndex = this->halfEdgeMesh.edges[twin].nextIndex;
							this->halfEdgeMesh.edges[prevEdge(p2)].nextIndex = nE1;

							this->halfEdgeMesh.faces[face].faceVerts.shallowClear(false);
							this->halfEdgeMesh.faces[face].edgeIndex = p1;
							uint16 e1 = this->halfEdgeMesh.faces[face].edgeIndex;
							do {
								this->halfEdgeMesh.edges[e1].faceIndex = face;
								this->halfEdgeMesh.faces[face].faceVerts.insert(this->halfEdgeMesh.edges[e1].vertIndex);
								e1 = this->halfEdgeMesh.edges[e1].nextIndex;
							} while (e1 != this->halfEdgeMesh.faces[face].edgeIndex);

							this->halfEdgeMesh.edges[nE2].vertIndex = this->halfEdgeMesh.edges[p1].vertIndex;
							this->halfEdgeMesh.edges[nE2].nextIndex = p2;
							this->halfEdgeMesh.edges[p2].nextIndex = this->halfEdgeMesh.edges[edge].nextIndex;
							this->halfEdgeMesh.edges[this->halfEdgeMesh.nextEdge(this->halfEdgeMesh.nextEdge(edge))].nextIndex = nE2;

							uint16 f = this->halfEdgeMesh.edges[twin].faceIndex;
							this->halfEdgeMesh.faces[f].faceVerts.shallowClear(false);
							this->halfEdgeMesh.faces[f].edgeIndex = p2;
							uint16 e2 = this->halfEdgeMesh.faces[f].edgeIndex;
							do {
								this->halfEdgeMesh.edges[e2].faceIndex = f;
								this->halfEdgeMesh.faces[f].faceVerts.insert(this->halfEdgeMesh.edges[e2].vertIndex);
								e2 = this->halfEdgeMesh.edges[e2].nextIndex;
							} while (e2 != this->halfEdgeMesh.faces[f].edgeIndex);

							this->halfEdgeMesh.edges.eraseDataAtIndex(edge);
							this->halfEdgeMesh.edges.eraseDataAtIndex(twin);

							repeat = true;
							break;
						}

						edge = this->halfEdgeMesh.edges[edge].nextIndex;

					} while (edge != this->halfEdgeMesh.faces[face].edgeIndex);

					if (repeat == false) ++it;
				}

				//erase newly added vertex from unclaimed list
				this->unClaimedIndicies.eraseData(this->eyePoint.indexInPointCloud);

				this->setEyePoint(pointCloud);
				numIterations++;
			}

			bool isCredible()
			{
				return this->claimedIndicies.size() - this->numEdges / 2 + this->numFaces == 2;
			}

			void fillHull(ConvexHull& hull, const DynamicArray<Vec3, uint16>& pointCloud)
			{
				for (uint32 x = 0, len = this->claimedIndicies.size(); x < len; ++x) {
					hull.vertices.pushBack(pointCloud[this->claimedIndicies[x].indexInPointCloud]);
				}

				for (auto it1 = this->halfEdgeMesh.faces.begin(), end1 = this->halfEdgeMesh.faces.end(); it1 != end1; ++it1) {

					uint16 fHull = hull.halfEdgeMesh.faces.size();
					hull.halfEdgeMesh.faces.pushBack(HalfEdgeMesh::hFace());

					for (auto it2 = it1.data().faceVerts.begin(), end2 = it1.data().faceVerts.end(); it2 != end2; ++it2) {
						hull.halfEdgeMesh.faces[fHull].faceVerts.pushBack(it2.data());

					}

					uint16 pEHull = -1;
					uint16 e = it1.data().edgeIndex;
					do {

						uint16 eHull = hull.halfEdgeMesh.edges.size();
						hull.halfEdgeMesh.edges.pushBack(HalfEdgeMesh::hEdge());
						hull.halfEdgeMesh.edges[eHull].vertIndex = this->halfEdgeMesh.edges[e].vertIndex;
						hull.halfEdgeMesh.edges[eHull].faceIndex = fHull;

						if (pEHull == (uint16)-1) {
							hull.halfEdgeMesh.faces[fHull].edgeIndex = eHull;
						}
						else {
							hull.halfEdgeMesh.edges[pEHull].nextIndex = eHull;
						}
						pEHull = eHull;

						e = this->halfEdgeMesh.nextEdge(e);

					} while (e != it1.data().edgeIndex);

					hull.halfEdgeMesh.edges[pEHull].nextIndex = hull.halfEdgeMesh.faces[fHull].edgeIndex;
				}

				for (auto it = hull.halfEdgeMesh.edges.begin(), end = hull.halfEdgeMesh.edges.end(); it != end; ++it) {
					hull.halfEdgeMesh.findTwin(it.index());
				}

				hull.vertices.wrap();
				hull.halfEdgeMesh.faces.wrap();
				hull.halfEdgeMesh.edges.wrap();
			}
		};

		qhAlgorithm algorithm;

		if (algorithm.createSimplex(pointCloud) == false) return false;

		while (algorithm.eyePoint.valid == true) {
			algorithm.iterate(pointCloud);
		}

		//test credibility
		if (algorithm.isCredible() == false) return false;

		//copy data to the convex hull
		algorithm.fillHull(hull, pointCloud);

		return true;
	}
}

#endif

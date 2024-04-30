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

#ifndef DEBUGRENDERER_H
#define DEBUGRENDERER_H

//#include"../graphics/renderer.h"
//#include"../graphics/primitives.h"

/*
	NOTE:
	For the debug renderer to work, provide your own rendering code
*/

namespace mech {

#if mech_ENABLE_DEBUG_RENDERER

	class DebugRenderer {

	private:

		struct RenderableData {
			Renderable renderable;
			Vec3f colour;
			uint32 id = -1;

			RenderableData() {}
			RenderableData(const Renderable& r, const Vec3f& c) : renderable(r), colour(c) {}
		};

		RigidArray<RenderableData, uint32> mData;

		DebugRenderer() {}

	public:

		~DebugRenderer() {}

		static DebugRenderer* get() { static DebugRenderer staticInstance; return &staticInstance; }

		void send(Renderer& renderer, const uint32& sceneID)
		{
			for (auto it = this->mData.begin(), end = this->mData.end(); it != end; ++it) {
				it.data().id = renderer.addRenderable(sceneID, it.data().renderable);
				renderer.attach(sceneID, it.data().id, it.data().colour, "uColour0");
				renderer.attach(sceneID, it.data().id, Mat4x4f(1.0), "uModel");
			}
		}

		void clean(Renderer& renderer, const uint32& sceneID)
		{
			for (auto it = this->mData.begin(), end = this->mData.end(); it != end; ++it) {
				renderer.eraseRenderable(sceneID, it.data().id);
			}
			this->mData.shallowClear(false);
		}

		void add(const Vec3& point, const Vec3f& colour)
		{
			this->mData.insert(RenderableData(getRenderable(point), colour));
		}

		void add(const LineSegment& lineSegment, const Vec3f& colour)
		{
			this->mData.insert(RenderableData(getRenderable(lineSegment), colour));
		}

		void add(const Triangle& triangle, const Vec3f& colour)
		{
			this->mData.insert(RenderableData(getRenderable(triangle, true), colour));
		}

		void add(const AABB& aabb, const Vec3f& colour)
		{
			this->mData.insert(RenderableData(getRenderable(aabb, true), colour));
		}
	};

#define DEBUG_RENDERER_ADD(primitive, colour) DebugRenderer::get()->add(primitive, colour)
#define DEBUG_RENDERER_SEND(renderer, sceneID) DebugRenderer::get()->send(renderer, sceneID)
#define DEBUG_RENDERER_CLEAN(renderer, sceneID) DebugRenderer::get()->clean(renderer, sceneID)

#else

#define DEBUG_RENDERER_ADD(primitive, colour) ((void)0)
#define DEBUG_RENDERER_SEND(renderer, sceneID) ((void)0)
#define DEBUG_RENDERER_CLEAN(renderer, sceneID) ((void)0)

#endif

}

#endif

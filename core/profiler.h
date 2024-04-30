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

#ifndef PROFILER_H
#define PROFILER_H

#include"timer.h"
#include"../containers/stack.h"
#include"../containers/string.h"

#include<fstream>

namespace mech {

#if mech_ENABLE_PROFILER

	class Profiler {

	private:

		struct ProfileInfo {
			Timer timer;
			String name;
			uint32 depth = -1;

			ProfileInfo() {}
			ProfileInfo(const char* n, const uint32& d) : name(n), depth(d) {}
		};

		std::ofstream mFile;
		Stack<ProfileInfo, uint32> mProfileStack;

		Profiler()
		{
			this->mFile.open("profile.mech");
		}

	public:

		~Profiler()
		{
			this->mFile.close();
		}

		static Profiler* get() { static Profiler staticInstance; return &staticInstance; }

		void begin(const char* name)
		{
			uint32 d = this->mProfileStack.empty() ? 0 : this->mProfileStack.top().depth + 1;
			this->mProfileStack.push(ProfileInfo(name, d));

			for (uint32 x = 0; x < d; ++x) this->mFile << "\t";
			for (uint32 x = 0; x < d; ++x) this->mFile << "*";
			this->mFile << "PROFILE->" << name << "\n";
		}

		void end()
		{
			for (uint32 x = 0, len = this->mProfileStack.top().depth; x < len; ++x) this->mFile << "\t";
			for (uint32 x = 0, len = this->mProfileStack.top().depth; x < len; ++x) this->mFile << "*";
			this->mFile << "END->" << this->mProfileStack.top().name.cString() << " TIME->" << toString2(this->mProfileStack.top().timer.elapsedNanoSeconds()).cString() << "nanosec\n";
			this->mProfileStack.pop();

			if (this->mProfileStack.empty()) {
				this->mFile << "\n";
			}
		}
	};

#define BEGIN_PROFILE(name) Profiler::get()->begin(name)
#define END_PROFILE Profiler::get()->end()

#else

#define BEGIN_PROFILE(name) ((void)0)
#define END_PROFILE ((void)0)

#endif

}

#endif

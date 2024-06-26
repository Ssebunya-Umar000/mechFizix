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

#ifndef LOGGER_H
#define LOGGER_H

#include<fstream>

namespace mech {

#if mech_ENABLE_LOGGER

	class Logger {

	private:

		std::ofstream mFile;

		Logger()
		{
			this->mFile.open("logs.mech");
		}

	public:

		~Logger()
		{
			this->mFile.close();
		}

		static Logger* get() { static Logger staticInstance; return &staticInstance; }

		void log(const char* str)
		{
			this->mFile << str << '\n';
		}
	};

#define LOG(str) Logger::get()->log(str)

#else

#define LOG(str) ((void)0)

#endif

}

#endif

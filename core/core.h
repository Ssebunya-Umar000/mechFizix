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

#ifndef CORE_H
#define CORE_H

#include<assert.h>

namespace mech {

#define int32 int
#define uint32 unsigned int
#define uint16 unsigned short
#define uint64 unsigned long long
#define byte unsigned char

#define mech_ENABLE_DOUBLE_PRECISION 1
#define mech_ENABLE_PROFILER 1
#define mech_ENABLE_LOGGER 1

	template<typename T>
	bool isAValidIndex(const T& number)
	{
		return number != (T)(-1);
	}
}

#endif

/*
TODO:
-time of impact - unwanted behaviour noticed.
*/

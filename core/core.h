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

#ifndef CORE_H
#define CORE_H

namespace mech {

	//Configuration defines
	/////////////////////////////////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////////////////////////
#define mech_ENABLE_DOUBLE_PRECISION 1
#define mech_ENABLE_DEBUG_RENDERER 0
#define mech_ENABLE_PROFILER 0
#define mech_ENABLE_LOGGER 0
	/////////////////////////////////////////////////////////////////////////////////////////////////////

	// Determine platform
	/////////////////////////////////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////////////////////////
#if defined(_WIN32) || defined(_WIN64)
#define mechPLATFORM_WINDOWS

#elif defined(__ANDROID__)
#define mechPLATFORM_ANDROID

#elif defined(__linux__)
#define mechPLATFORM_LINUX

#elif defined(__APPLE__)

#include <TargetConditionals.h>
#if defined(TARGET_OS_IPHONE) && !TARGET_OS_IPHONE
#define mechPLATFORM_MACOS
#else
#define mechPLATFORM_IOS
#endif

#endif
	/////////////////////////////////////////////////////////////////////////////////////////////////////

	//Detect CPU architecture
	/////////////////////////////////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////////////////////////
#if defined(__x86_64__) || defined(_M_X64) || defined(__i386__) || defined(_M_IX86)
#define mechCPU_X86

#elif defined(__aarch64__) || defined(_M_ARM64) || defined(__arm__) || defined(_M_ARM)
#define mechCPU_ARM

#else
#error Unsupported CPU architecture
#endif
	/////////////////////////////////////////////////////////////////////////////////////////////////////

	//Define mechBREAKPOINT
	/////////////////////////////////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////////////////////////
#if defined(mechPLATFORM_WINDOWS)
#define mechBREAKPOINT		__debugbreak()

#elif defined(mechPLATFORM_LINUX) || defined(mechPLATFORM_ANDROID) || defined(mechPLATFORM_MACOS) || defined(mechPLATFORM_IOS)

#if defined(mechCPU_X86)
#define mechBREAKPOINT		__asm volatile ("int $0x3")
#elif defined(mechCPU_ARM)
#define mechBREAKPOINT		__builtin_trap()
#endif

#else
#error Unknown platform
#endif
	/////////////////////////////////////////////////////////////////////////////////////////////////////

	//Arithmetic types
	/////////////////////////////////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////////////////////////
#define int32 int
#define uint32 unsigned int
#define uint16 unsigned short
#define uint64 unsigned long long
#define byte unsigned char
	/////////////////////////////////////////////////////////////////////////////////////////////////////

	/////////////////////////////////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////////////////////////
	template<typename T>
	bool isAValidIndex(const T& number)
	{
		return number != (T)(-1);
	}
	/////////////////////////////////////////////////////////////////////////////////////////////////////
}

#endif

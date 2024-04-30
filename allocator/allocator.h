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

#ifndef ALLOCATOR_H
#define ALLOCATOR_H

#include"../core/core.h"

namespace mech {

	//call this function before doing anything.
	//@param - allocationFcn -> use this to provide a custom allocation function.
	//@param - deallocationFcn -> use this to provide a custom deallocation function.
	//if both functions are not provided or only one is provided, a default pair of functions are used instead.
	void initialiseAllocator(void* (*allocationFcn) (const uint64&), void (*deallocationFcn) (void*, const uint64&));
	
	void* allocate(const uint64& sizeInBytes);
	void deallocate(void* data, const uint64& sizeInBytes);

	char* stringAllocate(const uint64& sizeInBytes);
	void stringDeallocate(char* data, const uint64& sizeInBytes);
}

#endif
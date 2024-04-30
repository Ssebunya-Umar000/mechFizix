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

#ifndef PAIR_H
#define PAIR_H

namespace mech {

	template<typename T1, typename T2>
	struct Pair {
		T1 first;
		T2 second;

		Pair() : second(T2()), first(T1()) {}
		Pair(const T1& f) : second(T2()), first(f) {}
		Pair(const T1& f, const T2& s) : second(s), first(f) {}

		bool operator==(const Pair<T1, T2>& other) { return this->first == other.first; }
		bool operator!=(const Pair<T1, T2>& other) { return this->first != other.first; }
		bool operator>(const Pair<T1, T2>& other) { return this->first > other.first; }
		bool operator<(const Pair<T1, T2>& other) { return this->first < other.first; }
	};
}

#endif
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

#ifndef TIMER_H
#define TIMER_H

#include<chrono>

namespace mech {

	struct Timer {
		
		std::chrono::time_point<std::chrono::high_resolution_clock> startPoint;

		Timer()
		{
			this->startPoint = std::chrono::high_resolution_clock::now();
		}

		void reset()
		{
			this->startPoint = std::chrono::high_resolution_clock::now();
		}

		double elapsedSeconds()
		{
			std::chrono::time_point<std::chrono::high_resolution_clock> end = std::chrono::high_resolution_clock::now();
			std::chrono::duration<double> duration = end - this->startPoint;
			this->startPoint = end;
			return duration.count();
		}

		double elapsedMilliSeconds()
		{
			std::chrono::time_point<std::chrono::high_resolution_clock> end = std::chrono::high_resolution_clock::now();
			std::chrono::duration<double> duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - this->startPoint);
			this->startPoint = end;
			return duration.count();
		}

		double elapsedMicroSeconds()
		{
			std::chrono::time_point<std::chrono::high_resolution_clock> end = std::chrono::high_resolution_clock::now();
			std::chrono::duration<double> duration = std::chrono::duration_cast<std::chrono::microseconds>(end - this->startPoint);
			this->startPoint = end;
			return duration.count();
		}

		double elapsedNanoSeconds()
		{
			std::chrono::time_point<std::chrono::high_resolution_clock> end = std::chrono::high_resolution_clock::now();
			std::chrono::duration<double> duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - this->startPoint);
			this->startPoint = end;
			return duration.count();
		}

		double currentSeconds()
		{
			std::chrono::time_point<std::chrono::high_resolution_clock> end = std::chrono::high_resolution_clock::now();
			std::chrono::duration<double> duration = end - this->startPoint;
			return duration.count();
		}

		double currentMilliSeconds()
		{
			std::chrono::time_point<std::chrono::high_resolution_clock> end = std::chrono::high_resolution_clock::now();
			std::chrono::duration<double> duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - this->startPoint);
			return duration.count();
		}

		double currentMicroSeconds()
		{
			std::chrono::time_point<std::chrono::high_resolution_clock> end = std::chrono::high_resolution_clock::now();
			std::chrono::duration<double> duration = std::chrono::duration_cast<std::chrono::microseconds>(end - this->startPoint);
			return duration.count();
		}

		double currentNanoSeconds()
		{
			std::chrono::time_point<std::chrono::high_resolution_clock> end = std::chrono::high_resolution_clock::now();
			std::chrono::duration<double> duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - this->startPoint);
			return duration.count();
		}
	};
}

#endif
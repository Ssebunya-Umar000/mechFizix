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

#ifndef STACK_H
#define STACK_H

#include"dynamicArray.h"

namespace mech {

	/*
		this class represents a stack.
		the data is stored in a dynamic array.
		NOTE: the address of the data can change, do not store pointers!
		NOTE: the index of the data can change, do not store indicies!
	*/
	template<typename T, typename sizeType>
	class Stack {

	private:

		DynamicArray<T, sizeType> mData;

	public:
		
		void push(const T& data)
		{
			this->mData.pushBack(data);
		}

		void pop()
		{
			this->mData.popBack();
		}

		T* find(const T& data)
		{
			return this->mData.find(data);
		}

		T& top()
		{
			return this->mData.back();
		}

		sizeType size()
		{
			return this->mData.size();
		}

		bool empty() const
		{
			return this->mData.empty();
		}

		void wrap()
		{
			this->mData.wrap();
		}

		void shallowClear(bool callDestructors)
		{
			this->mData.shallowClear(callDestructors);
		}

		void clear()
		{
			this->mData.clear();
		}

		////////////////////////////////////////////////////////////////////////////////////////////////////////
		class Iterator {
		private:
			const Stack<T, sizeType>* mStack;
			sizeType mCurrentIndex = -1;

		public:

			Iterator(const Stack<T, sizeType>* stack) : mStack(stack) {}

			Iterator(const Stack<T, sizeType>* stack, const sizeType& index) : mStack(stack)
			{
				if (this->mStack->empty() == false) {
					this->mCurrentIndex = index;
				}
			}

			Iterator& operator++()
			{
				--this->mCurrentIndex;

				if (this->mCurrentIndex == (sizeType)(-1) || this->mCurrentIndex < 0) {
					this->mCurrentIndex = -1;
				}

				return *this;
			}

			Iterator& operator++(int)
			{
				++*this;

				return *this;
			}

			T& data()
			{
				return this->mStack->mData[this->mCurrentIndex];
			}

			T& operator*()
			{
				return this->mStack->mData[this->mCurrentIndex];
			}

			sizeType index()
			{
				return this->mCurrentIndex;
			}

			bool operator==(const Iterator& other)
			{
				return this->mCurrentIndex == other.mCurrentIndex && this->mStack == other.mStack;
			}

			bool operator!=(const Iterator& other)
			{
				return this->mCurrentIndex != other.mCurrentIndex || this->mStack != other.mStack;
			}

			bool isEnd() const
			{
				return this->mCurrentIndex == (sizeType)(-1);
			}
		};

		Iterator begin() const
		{
			return Iterator(this, this->mData.size() - 1);
		}

		Iterator end() const
		{
			return Iterator(this);
		}
	};
}

#endif

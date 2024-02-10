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

		////////////////////////////////////////////////////////////////////////////////////////////////////////
		class Iterator {
		private:
			T* mCurrent = nullptr;
			const Stack<T, sizeType>* mStack = nullptr;
			sizeType mIndex = -1;

		public:

			Iterator() {}

			Iterator(const Stack<T, sizeType>* stack) : mStack(stack)
			{
				if (this->mStack->empty() == false) {
					this->mIndex = this->mStack->mData.size() - 1;
					this->mCurrent = &this->mStack->mData[this->mIndex];
				}
			}

			T& data()
			{
				return *this->mCurrent;
			}

			T& operator*()
			{
				return *this->mCurrent;
			}

			Iterator& operator++()
			{
				--this->mIndex;

				if (this->mIndex == (sizeType)(-1) || this->mIndex < 0) {
					this->mCurrent = nullptr;
				}
				else {
					this->mCurrent = &this->mStack->mData[this->mIndex];
				}

				return *this;
			}

			Iterator& operator++(int)
			{
				++*this;

				return *this;
			}

			bool operator==(const Iterator& other)
			{
				return this->mCurrent == other.mCurrent;
			}

			bool operator!=(const Iterator& other)
			{
				return this->mCurrent != other.mCurrent;
			}

			bool isVoid() const
			{
				return this->mCurrent == nullptr;
			}
		};

		Iterator begin() const
		{
			return Iterator(this);
		}

		Iterator end() const
		{
			return Iterator();
		}
	};
}

#endif

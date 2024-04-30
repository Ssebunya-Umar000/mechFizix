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

#ifndef STACKARRAY_H
#define STACKARRAY_H

#include"../core/core.h"
#include"../core/assert.h"

namespace mech {

	/*
		this class represents an array of data on the stack
	*/
	template<typename T, uint32 capacity>
	class StackArray {
	private:

		T mData[capacity] = {};
		uint32 mCount = 0;

	public:
		
		StackArray() {}

		explicit StackArray(const T& data)
		{
			for (uint32 x = 0; x < capacity; ++x) {
				mData[x] = data;
			}
		}

		explicit StackArray(const T* data, const uint32& count)
		{
			ASSERT(count <= capacity, "count should be less than or equal to the capacity!!");
			for (uint32 x = 0; x < count; ++x) {
				this->pushBack(data[x]);
			}
		}

		void pushBack(const T& data)
		{
			ASSERT(this->mCount < capacity, "array is already full");
			this->mData[this->mCount] = data;
			++this->mCount;
		}

		//DO NOT use for data holding pointers or else you will create dangling pointers!!
		void pushBack(const T* data, const uint32& size)
		{
			ASSERT(this->mCount + size <= capacity, "data can not fit into the array!!");
			std::memcpy(&this->mData[this->mCount], data, size * sizeof(T));
			this->mCount += size;
		}

		void eraseData(const T& data)
		{
			for (uint32 x = 0; x < this->mCount; ++x) {
				if (this->mData[x] == data) {
					this->eraseDataAtIndex(x);
				}
			}
		}

		void eraseDataAtIndex(const uint32& index)
		{
			ASSERT(index >= 0 && index < this->mCount, "index is out of range");

			--this->mCount;
			if (index < this->mCount) {
				this->mData[index] = this->mData[this->mCount];
			}
			this->mData[this->mCount] = T();
		}

		void popBack()
		{
			ASSERT(this->mCount >= 1, "array is empty");
			--this->mCount;
			this->mData[this->mCount] = T();
		}

		T* find(const T& data)
		{
			for (uint32 x = 0; x < this->mCount; ++x) {
				if (this->mData[x] == data) {
					return &this->mData[x];
				}
			}

			return nullptr;
		}

		uint32 findIndex(const T& data)
		{
			for (uint32 x = 0; x < this->mCount; ++x) {
				if (this->mData[x] == data) {
					return x;
				}
			}

			return -1;
		}

		T& operator[](const uint32& index)
		{
			ASSERT(index < capacity, "index is out of range");
			return this->mData[index];
		}

		const T& operator[](const uint32& index) const
		{
			ASSERT(index < capacity, "index is out of range");
			return this->mData[index];
		}

		T& front()
		{
			ASSERT(this->mCount >= 1, "array is empty");
			return this->mData[0];
		}

		const T& front() const
		{
			ASSERT(this->mCount >= 1, "array is empty");
			return this->mData[0];
		}

		T& back()
		{
			ASSERT(this->mCount >= 1, "array is empty");
			return this->mData[this->mCount - 1];
		}

		const T& back() const
		{
			ASSERT(this->mCount >= 1, "array is empty");
			return this->mData[this->mCount - 1];
		}

		uint32 size() const
		{
			return this->mCount;
		}

		void setSize(const uint32& size)
		{
			ASSERT(size >= 0 && size <= capacity, "size is not acceptable");
			this->mCount = size;
		}

		T* data()
		{
			return this->mData;
		}

		const T* data() const
		{
			return this->mData;
		}

		bool empty() const
		{
			return this->mCount == 0;
		}

		void clear()
		{
			this->mCount = 0;
		}

		///////////////////////////////////////////////////
		class Iterator {
		private:
			const StackArray<T, capacity>* mArray;
			uint32 mCurrentIndex = -1;

		public:

			Iterator(const StackArray<T, capacity>* array) : mArray(array) {}

			Iterator(const StackArray<T, capacity>* array, const uint32& index) : mArray(array)
			{
				if (this->mArray->empty() == false) {
					this->mCurrentIndex = index;
				}
			}

			Iterator& operator++()
			{
				++this->mCurrentIndex;
				if (this->mArray->mCount == this->mCurrentIndex) {
					this->mCurrentIndex = -1;
				}

				return *this;
			}

			Iterator& operator--()
			{
				if (this->mCurrentIndex == 0) {
					this->mCurrentIndex = -1;
				}
				else {
					this->mCurrentIndex--;
				}

				return *this;
			}

			Iterator& operator++(int)
			{
				++*this;
				return *this;
			}

			Iterator& operator--(int)
			{
				--* this;
				return *this;
			}

			const T& data() const
			{
				return this->mArray->mData[this->mCurrentIndex];
			}

			T& operator*()
			{
				return this->mArray->mData[this->mCurrentIndex];
			}

			uint32 index()
			{
				return this->mCurrentIndex;
			}

			bool operator==(const Iterator& other)
			{
				return this->mCurrentIndex == other.mCurrentIndex && this->mArray == other.mArray;
			}

			bool operator!=(const Iterator& other)
			{
				return this->mCurrentIndex != other.mCurrentIndex || this->mArray != other.mArray;
			}

			bool isEnd() const
			{
				return this->mCurrentIndex == (uint32)(-1);
			}
		};

		Iterator begin() const
		{
			return Iterator(this, 0);
		}

		Iterator last() const
		{
			return Iterator(this, this->mCount - 1);
		}

		Iterator end() const
		{
			return Iterator(this);
		}
	};
}

#endif

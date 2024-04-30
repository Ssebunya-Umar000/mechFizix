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

#ifndef DYNAMICARRAY_H
#define DYNAMICARRAY_H

#include"../allocator/allocator.h"
#include"../core/assert.h"

namespace mech {

#define MINIMUM_ARRAY_CAPACITY 3
#define ARRAY_GROWTH_FACTOR 1.5

	/*
		this class represents an array allocated on the heap
		NOTE: the address of the data can change, do not store pointers!
		NOTE: the index of the data can change, do not store indicies!
	*/
	template<typename T, typename sizeType>
	class DynamicArray {

	private:

		void requestMemory(const sizeType& size)
		{
			this->mCapacity = size;
			this->mData = (T*)allocate(this->mCapacity * sizeof(T));
		}

		void destructData(T* data, const sizeType& capacity)
		{
			for (sizeType x = 0; x < this->mCount; ++x) {
				data[x].~T();
			}
			deallocate(data, capacity * sizeof(T));
		}

		void releaseMemory()
		{
			this->destructData(this->mData, this->mCapacity);
		}

		void shiftData(T* prevData, const sizeType& prevCapacity)
		{
			for (sizeType x = 0; x < this->mCount; ++x) {
				this->mData[x] = (T&&)(prevData[x]);
			}

			this->destructData(prevData, prevCapacity);
		}

		void hijack(const sizeType& otherCapacity, const sizeType& otherCount, T* otherData)
		{
			this->releaseMemory();
			
			this->mData = otherData;
			this->mCapacity = otherCapacity;
			this->mCount = otherCount;
		}


		void clone(const sizeType& otherCount, const T* otherData)
		{
			this->clear();
			
			if (otherCount > 0) {
				
				this->mCount = otherCount;
				this->requestMemory(otherCount);

				for (sizeType x = 0; x < otherCount; ++x) {
					this->mData[x] = otherData[x];
				}
			}
		}

		void checkMemory()
		{
			if (this->mCapacity == 0) {
				this->requestMemory(MINIMUM_ARRAY_CAPACITY);
			}
			else if (this->mCount == this->mCapacity) {
				
				sizeType prevCapacity = this->mCapacity;
				sizeType newCapacity = (sizeType)(double(prevCapacity) * ARRAY_GROWTH_FACTOR);

				T* temp = this->mData;
				this->requestMemory(newCapacity == 1 ? 2 : newCapacity);
				this->shiftData(temp, prevCapacity);
			}
		}

		T* mData = nullptr;
		sizeType mCount = 0;
		sizeType mCapacity = 0;

	public:

		DynamicArray() {}

		~DynamicArray()
		{
			this->clear();
		}

		explicit DynamicArray(const sizeType& capacity)
		{
			this->requestMemory(capacity);
		}

		explicit DynamicArray(const T* data, const sizeType& size)
		{
			this->requestMemory(size);
			for (sizeType x = 0; x < size; ++x) {
				this->pushBack(data[x]);
			}
		}

		DynamicArray(const DynamicArray<T, sizeType>& other)
		{
			clone(other.mCount, other.mData);
		}

		DynamicArray(DynamicArray<T, sizeType>&& other) noexcept
		{
			hijack(other.mCapacity, other.mCount, other.mData);
			other.reset();
		}

		template<typename otherSizeType>
		DynamicArray(const DynamicArray<T, otherSizeType>& other)
		{
			clone(other.size(), other.data());
		}

		template<typename otherSizeType>
		DynamicArray(DynamicArray<T, otherSizeType>&& other) noexcept
		{
			hijack(other.capacity(), other.size(), other.data());
			other.reset();
		}

		DynamicArray<T, sizeType>& operator=(const DynamicArray<T, sizeType>& other)
		{
			if (this != &other) {
				clone(other.mCount, other.mData);
			}
			return *this;
		}

		DynamicArray<T, sizeType>& operator=(DynamicArray<T, sizeType>&& other) noexcept
		{
			hijack(other.mCapacity, other.mCount, other.mData);
			other.reset();
			return *this;
		}

		template<typename otherSizeType>
		DynamicArray<T, sizeType>& operator=(const DynamicArray<T, otherSizeType>& other)
		{
			clone(other.size(), other.data());
			return *this;
		}

		template<typename otherSizeType>
		DynamicArray<T, sizeType>& operator=(DynamicArray<T, otherSizeType>&& other) noexcept
		{
			hijack(other.capacity(), other.size(), other.data());
			other.reset();
			return *this;
		}

		void swap(DynamicArray<T, sizeType>& other)
		{
			T* temp1 = this->mData;
			sizeType temp2 = this->mCount;
			sizeType temp3 = this->mCapacity;

			this->mData = other.mData;
			this->mCount = other.mCount;
			this->mCapacity = other.mCapacity;

			other.mData = temp1;
			other.mCount = temp2;
			other.mCapacity = temp3;
		}

		void pushBack(const T& data)
		{
			this->checkMemory();

			this->mData[this->mCount] = data;
			++this->mCount;
		}

		void pushBack(T&& data)
		{
			this->checkMemory();

			this->mData[this->mCount] = data;
			++this->mCount;
		}

		//DO NOT use for data holding pointers or else you will create dangling pointers!!
		void pushBack(const T* data, const sizeType& size)
		{
			if (this->mCapacity <= this->mCount + size) {
				this->resize(this->mCount + size);
			}
			std::memcpy(&this->mData[this->mCount], data, size * sizeof(T));
			this->mCount += size;
		}

		//DO NOT use for data holding pointers or else you will create dangling pointers!!
		void pushBack(const DynamicArray<T, sizeType>& data)
		{
			this->pushBack(data.data(), data.size());
		}

		void eraseData(const T& data)
		{
			for (uint32 x = 0; x < this->mCount; ++x) {
				if (this->mData[x] == data) {
					this->eraseDataAtIndex(x);
				}
			}
		}

		void eraseDataAtIndex(const sizeType index)
		{
			ASSERT(this->mData != nullptr && index >= 0 && index < this->mCount, "array is empty or index is out of range");

			--this->mCount;
			if (index < this->mCount) {
				this->mData[index] = (T&&)(this->mData[this->mCount]);
			}
			else {
				this->mData[index].~T();
			}
		}

		void popBack()
		{
			ASSERT(this->mCount > 0, "array is empty");

			--this->mCount;
			this->mData[this->mCount].~T();
		}

		T* find(const T& data)
		{
			for (sizeType x = 0; x < this->mCount; ++x) {
				if (this->mData[x] == data) {
					return &this->mData[x];
				}
			}

			return nullptr;
		}

		void resize(const sizeType& capacity)
		{
			if (capacity <= 0) return;

			sizeType prevCapacity = this->mCapacity;

			T* temp = this->mData;
			this->requestMemory(capacity);

			if (temp != nullptr) {

				if (this->mCount > this->mCapacity) {
					this->mCount = this->mCapacity;
				}

				this->shiftData(temp, prevCapacity);
			}
		}

		void reserve(const sizeType& capacity)
		{
			this->resize(capacity);
			this->mCount = this->mCapacity;
		}

		void setSize(const sizeType& s)
		{
			ASSERT(s < this->mCapacity, "size should be less than the capacity!!");
			this->mCount = s;
		}

		T* data() const
		{
			return this->mData;
		}

		T& operator[](const sizeType& index) const
		{
			ASSERT((this->mCount > 0) && (index >= 0) && (index < this->mCount), "array is empty or index is out of range");
			return this->mData[index];
		}

		T& front() const
		{
			ASSERT(this->mCount >= 1, "array is empty");
			return this->mData[0];
		}

		T& back() const
		{
			ASSERT(this->mCount >= 1, "array is empty");
			return this->mData[this->mCount - 1];
		}

		sizeType size() const
		{
			return this->mCount;
		}

		sizeType capacity() const
		{
			return this->mCapacity;
		}

		bool empty() const
		{
			return this->mCount == 0;
		}

		void wrap()
		{
			if (this->mCount == 0) {
				this->clear();
			}
			else {
				this->resize(this->mCount);
			}
		}

		void shallowClear(bool callDestructors)
		{
			if (callDestructors == true) {
				for (sizeType x = 0; x < this->mCount; ++x) {
					this->mData[x].~T();
				}
			}

			this->mCount = 0;
		}

		void clear()
		{
			this->releaseMemory();
			this->reset();
		}

		void reset()
		{
			this->mData = nullptr;
			this->mCapacity = 0;
			this->mCount = 0;
		}

		///////////////////////////////////////////////////
		class Iterator {
		private:
			const DynamicArray<T, sizeType>* mArray;
			sizeType mCurrentIndex = -1;

		public:
			
			Iterator(const DynamicArray<T, sizeType>* array) : mArray(array) {}

			Iterator(const DynamicArray<T, sizeType>* array, const sizeType& index) : mArray(array)
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
					--this->mCurrentIndex;
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
				--*this;
				return *this;
			}

			T& data() const
			{
				return this->mArray->mData[this->mCurrentIndex];
			}

			T& operator*()
			{
				return this->mArray->mData[this->mCurrentIndex];
			}

			bool operator==(const Iterator& other) const
			{
				return this->mCurrentIndex == other.mCurrentIndex && this->mArray == other.mArray;
			}

			bool operator!=(const Iterator& other) const
			{
				return this->mCurrentIndex != other.mCurrentIndex || this->mArray != other.mArray;
			}

			bool isEnd() const
			{
				return this->mCurrentIndex == (sizeType)(-1);
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

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

#ifndef HYBRIDARRAY_H
#define HYBRIDARRAY_H

#include"stackArray.h"
#include"dynamicArray.h"

namespace mech {

	/*
		this class represents two arrays, one on the stack and the other on the heap.
		the heap array is only used if the stack array is full.
		NOTE: the address of the data can change, do not store pointers!
		NOTE: the index of the data can change, do not store indicies!
	*/
	template<typename T, uint32 stackCapacity, typename sizeType>
	class HybridArray {

	private:

		StackArray<T, stackCapacity> mStackData;
		DynamicArray<T, sizeType> mHeapData;

	public:

		HybridArray() {}

		template<uint32 otherStackCapacity, typename otherSizeType>
		HybridArray(const HybridArray<T, otherStackCapacity, otherSizeType>& other)
		{
			for (sizeType x = 0, len = other.size(); x < len; ++x) {
				this->pushBack(other[x]);
			}
		}

		template<uint32 otherStackCapacity, typename otherSizeType>
		HybridArray<T, stackCapacity, sizeType>& operator=(const HybridArray<T, otherStackCapacity, otherSizeType>& other)
		{
			for (sizeType x = 0, len = other.size(); x < len; ++x) {
				this->pushBack(other[x]);
			}

			return *this;
		}

		template<uint32 otherStackCapacity>
		explicit HybridArray(const StackArray<T, otherStackCapacity>& stackArray)
		{
			for (sizeType x = 0, len = stackArray.size(); x < len; ++x) {
				this->pushBack(stackArray[x]);
			}
		}

		explicit HybridArray(const DynamicArray<T, sizeType>& dynamicArray)
		{
			for (sizeType x = 0, len = dynamicArray.size(); x < len; ++x) {
				this->pushBack(dynamicArray[x]);
			}
		}

		DynamicArray<T, sizeType> toDynamicArray()
		{
			DynamicArray<T, sizeType> dynamicArray;
			dynamicArray.reserve(this->size());
			for (sizeType x = 0, len = this->size(); x < len; ++x) {
				dynamicArray[x] = this->operator[](x);
			}

			return dynamicArray;
		}

		void pushBack(const T& data)
		{
			if (this->mStackData.size() < stackCapacity) {
				this->mStackData.pushBack(data);
			}
			else {
				this->mHeapData.pushBack(data);
			}
		}

		void eraseData(const T& data)
		{
			sizeType index = this->mStackData.findIndex(data);
			if (index == (sizeType)(-1)) {
				this->mHeapData.eraseData(data);
			}
			else {
				this->mStackData.eraseDataAtIndex(index);

				if (this->mHeapData.empty() == false) {
					this->mStackData.pushBack(this->mHeapData.back());
					this->mHeapData.popBack();
				}
			}
		}

		void eraseDataAtIndex(const sizeType index)
		{
			if (index < stackCapacity) {
				this->mStackData.eraseDataAtIndex(index);

				if (this->mHeapData.empty() == false) {
					this->mStackData.pushBack(this->mHeapData.back());
					this->mHeapData.popBack();
				}
			}
			else {
				this->mHeapData.eraseDataAtIndex(index - stackCapacity);
			}
		}

		T* find(const T& data)
		{
			T* pointer = this->mStackData.find(data);
			if (pointer == nullptr) {
				return this->mHeapData.find(data);
			}

			return pointer;
		}

		T& operator[](const sizeType& index)
		{
			if (index < stackCapacity) {
				return this->mStackData[index];
			}
			else {
				return this->mHeapData[index - stackCapacity];
			}
		}

		const T& operator[](const sizeType& index) const
		{
			if (index < stackCapacity) {
				return this->mStackData[index];
			}
			else {
				return this->mHeapData[index - stackCapacity];
			}
		}

		T& front()
		{
			return this->mStackData.front();
		}

		const T& front() const
		{
			return this->mStackData.front();
		}

		T& back()
		{
			if (this->mHeapData.size() > 0) {
				return this->mHeapData.back();
			}
			return this->mStackData.back();
		}

		const T& back() const
		{
			if (this->mHeapData.size() > 0) {
				return this->mHeapData.back();
			}
			return this->mStackData.back();
		}

		sizeType size() const
		{
			return this->mStackData.size() + this->mHeapData.size();
		}

		void setSize(const sizeType& size)
		{
			if (size <= stackCapacity) {
				this->mStackData.setSize(size);
			}
			else {
				this->mStackData.setSize(stackCapacity);
				this->mHeapData.setSize(size - stackCapacity);
			}
		}

		bool empty() const
		{
			return this->mStackData.empty();
		}

		void wrap()
		{
			this->mHeapData.wrap();
		}

		void shallowClear(bool callDestructors)
		{
			this->mStackData.clear();
			this->mHeapData.shallowClear(callDestructors);
		}

		void clear()
		{
			this->mStackData.clear();
			this->mHeapData.clear();
		}

		///////////////////////////////////////////////////
		class Iterator {
		private:
			const HybridArray<T, stackCapacity, sizeType>* mArray;
			sizeType mCurrentIndex = -1;

		public:

			Iterator(const HybridArray<T, stackCapacity, sizeType>* array) : mArray(array) {}

			Iterator(const HybridArray<T, stackCapacity, sizeType>* array, const sizeType& index) : mArray(array)
			{
				if (this->mArray->empty() == false) {
					this->mCurrentIndex = index;
				}
			}

			Iterator& operator++()
			{
				++this->mCurrentIndex;
				if (this->mArray->size() == this->mCurrentIndex) {
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
				--* this;
				return *this;
			}

			sizeType index() const
			{
				return this->mCurrentIndex;
			}

			const T& data() const
			{
				return this->mArray->operator[](this->mCurrentIndex);
			}

			T& operator*()
			{
				return this->mArray->operator[](this->mCurrentIndex);
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
			return Iterator(this, ((this->mHeapData.size() - 1) + stackCapacity));
		}

		Iterator end() const
		{
			return Iterator(this);
		}
	};
}

#endif

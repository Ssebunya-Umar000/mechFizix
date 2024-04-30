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

#ifndef RIGIDARRAY_H
#define RIGIDARRAY_H

#include"bitSet.h"

namespace mech {

	/*
		this class represents a rigid array, data is guaranteed to remain at its original index of insertion even when an element is erased, unlike DynamicArray class.
		the class keeps track of free slots -> slots where data has been erased and inserts new data to these slots.
		NOTE: the address of the data can change, do not store pointers!
	*/
	template<typename T, typename sizeType>
	class RigidArray {

	private:

		DynamicArray<T, sizeType> mData;
		DynamicArray<sizeType, sizeType> mFreeslots;
		BitSet<sizeType> mBitSet;

	public:

		void swap(RigidArray<T, sizeType>& other)
		{
			this->mData.swap(other.mData);
			this->mFreeslots.swap(other.mFreeslots);
			this->mBitSet.swap(other.mBitSet);
		}

		sizeType insert(const T& data)
		{
			sizeType index;

			if (this->mFreeslots.empty() == true) {
				index = this->mData.size();
				this->mData.pushBack(data);
			}
			else {
				index = this->mFreeslots.back();
				this->mFreeslots.popBack();
				this->mData[index] = data;
			}

			this->mBitSet.toggleOn(index);

			return index;
		}

		void eraseData(const T& data)
		{
			for (sizeType x = 0, len = this->mData.size(); x < len; ++x) {
				if (this->mBitSet[x] == true) {
					if (this->mData[x] == data) {
						this->eraseDataAtIndex(x);
					}
				}
			}
		}

		void eraseDataAtIndex(const sizeType index)
		{
			ASSERT(this->mBitSet[index] == true, "Data at index is already erased");

			if (index == this->mData.size() - 1) {
				this->mData.popBack();
			}
			else {
				this->mData[index] = T();
				this->mFreeslots.pushBack(index);
			}

			this->mBitSet.toggleOff(index);
		}

		void popBack()
		{
			this->mData.popBack();
			this->mBitSet.toggleOff(this->mData.size() - 1);
		}

		T* find(const T& data)
		{
			for (sizeType x = 0, len = this->mData.size(); x < len; ++x) {
				if (this->mBitSet[x] == true) {
					if (this->mData[x] == data) {
						return &this->mData[x];
					}
				}
			}

			return nullptr;
		}

		void resize(const sizeType& newSize)
		{
			this->mData.resize(newSize);
		}

		T& operator[](const sizeType& index) const
		{
			ASSERT(this->mBitSet[index] == true, "index has no data");
			return this->mData[index];
		}

		T& back() const
		{
			return this->mData.back();
		}

		bool empty() const
		{
			return this->size() == 0;
		}

		void wrap()
		{
			this->mData.wrap();
			this->mFreeslots.wrap();
		}

		sizeType size() const
		{
			return this->mData.size() - this->mFreeslots.size();
		}

		sizeType internalSize() const
		{
			return this->mData.size();
		}

		bool isIndexOccupied(const sizeType& index) const
		{
			if (index >= 0 && index < this->mData.size()) {
				return this->mBitSet[index];
			}

			return false;
		}

		void shallowClear(bool callDestructors)
		{
			this->mBitSet.reset();
			this->mData.shallowClear(callDestructors);
			this->mFreeslots.shallowClear(callDestructors);
		}

		void clear()
		{
			this->mData.clear();
			this->mFreeslots.clear();
			this->mBitSet.clear();
		}

		///////////////////////////////////////////////////
		class Iterator {
		private:

			void moveToNext()
			{
				sizeType size = this->mArray->mData.size();

				while (this->mCurrentIndex < size) {
					if (this->mArray->mBitSet[this->mCurrentIndex] == true) return;
					++this->mCurrentIndex;
				}

				this->mCurrentIndex = -1;
			}

			const RigidArray<T, sizeType>* mArray;
			sizeType mCurrentIndex = -1;

		public:

			Iterator(const RigidArray<T, sizeType> * array) : mArray(array) {}

			Iterator(const RigidArray<T, sizeType> * array, const sizeType & index) : mArray(array)
			{
				if (this->mArray->empty() == false) {
					this->mCurrentIndex = index;
					this->moveToNext();
				}
			}

			Iterator& operator++()
			{
				++this->mCurrentIndex;
				this->moveToNext();

				return *this;
			}

			Iterator& operator++(int)
			{
				++* this;
				return *this;
			}

			T& data()
			{
				return this->mArray->mData[this->mCurrentIndex];
			}

			T& operator*()
			{
				return this->mArray->mData[this->mCurrentIndex];
			}

			sizeType index()
			{
				return this->mCurrentIndex;
			}

			bool operator==(const Iterator & other)
			{
				return this->mCurrentIndex == other.mCurrentIndex && this->mArray == other.mArray;
			}

			bool operator!=(const Iterator & other)
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

		Iterator end() const
		{
			return Iterator(this);
		}
	};
}

#endif

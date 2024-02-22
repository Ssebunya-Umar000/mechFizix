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

#ifndef RIGIDARRAY_H
#define RIGIDARRAY_H

#include"dynamicArray.h"

namespace mech {

	/*
		this class represents a rigid array, data is guaranteed to remain at its original index of insertion even when an element is erased, unlike DynamicArray class.
		the class keeps track of free slots -> slots where data has been erased and inserts new data to these slots.
		NOTE: the address of the data can change, do not store pointers!
	*/
	template<typename T, typename sizeType>
	class RigidArray {

	private:

		struct Node {
			T data = T();
			sizeType nextFreeSlot = -1;
			bool occupied = false;

			Node() {}
			Node(const T& d) : data(d), occupied(true) {}
		};

		DynamicArray<Node, sizeType> mData;
		sizeType mFreeSlot = -1;
		sizeType mFreeSlotCount = 0;

	public:
		
		void swap(RigidArray<T, sizeType>& other)
		{
			this->mData.swap(other.mData);

			sizeType temp1 = this->mFreeSlot;
			sizeType temp2 = this->mFreeSlotCount;

			this->mFreeSlot = other.mFreeSlot;
			this->mFreeSlotCount = other.mFreeSlotCount;

			other.mFreeSlot = temp1;
			other.mFreeSlotCount = temp2;
		}

		sizeType insert(const T& data)
		{
			sizeType index;

			if (this->mFreeSlot == (sizeType)(-1)) {
				index = this->mData.size();
				this->mData.pushBack(Node());
			}
			else {
				index = this->mFreeSlot;
				this->mFreeSlot = this->mData[this->mFreeSlot].nextFreeSlot;
				--this->mFreeSlotCount;
			}

			this->mData[index] = Node(data);
			return index;
		}

		void eraseData(const T& data)
		{
			for (sizeType x = 0, len = this->mData.size(); x < len; ++x) {
				if (this->mData[x].occupied == true) {
					if (this->mData[x].data == data) {
						this->eraseDataAtIndex(x);
					}
				}
			}
		}

		void eraseDataAtIndex(const sizeType index)
		{
			assert(this->mData[index].occupied == true);

			if (index == this->mData.size() - 1) {
				this->mData.popBack();
			}
			else {
				this->mData[index] = Node();
				this->mData[index].nextFreeSlot = this->mFreeSlot;
				this->mFreeSlot = index;
				++this->mFreeSlotCount;
			}
		}

		void popBack()
		{
			this->mData[this->mData.size() - 1] = Node();
			this->mData.popBack();
		}

		T* find(const T& data)
		{
			for (sizeType x = 0, len = this->mData.size(); x < len; ++x) {
				if (this->mData[x].occupied == true) {
					if (this->mData[x].data == data) {
						return &this->mData[x].data;
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
			assert(this->mData[index].occupied == true);
			return this->mData[index].data;
		}

		T& back() const
		{
			return this->mData.back().data;
		}

		bool empty() const
		{
			return this->actualSize() == 0;
		}

		void wrap()
		{
			this->mData.wrap();
		}

		sizeType internalSize() const
		{
			return this->mData.size();
		}

		sizeType actualSize() const
		{
			return this->mData.size() - this->mFreeSlotCount;
		}

		bool isIndexOccupied(const sizeType& index) const
		{
			if (index >= 0 && index < this->mData.size()) {
				return this->mData[index].occupied;
			}

			return false;
		}

		void shallowClear()
		{
			this->mData.shallowClear();
			this->mFreeSlot = -1;
			this->mFreeSlotCount = 0;
		}

		void deepClear()
		{
			this->mData.deepClear();
			this->mFreeSlot = -1;
			this->mFreeSlotCount = 0;
		}

		///////////////////////////////////////////////////
		class Iterator {
		private:

			void moveToNext()
			{
				sizeType size = this->mArray->mData.size();

				while (this->mCurrentIndex < size) {
					if (this->mArray->mData[this->mCurrentIndex].occupied == true) return;
					++this->mCurrentIndex;
				}

				this->mCurrentIndex = -1;
			}

			const RigidArray<T, sizeType>* mArray;
			sizeType mCurrentIndex = -1;

		public:

			Iterator(const RigidArray<T, sizeType>* array) : mArray(array) {}

			Iterator(const RigidArray<T, sizeType>* array, const sizeType& index) : mArray(array)
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
				++*this;
				return *this;
			}

			T& data()
			{
				return this->mArray->mData[this->mCurrentIndex].data;
			}

			T& operator*()
			{
				return this->mArray->mData[this->mCurrentIndex].data;
			}

			sizeType index()
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

			bool isVoid() const
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

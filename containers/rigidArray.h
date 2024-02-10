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
				if (this->mData[x].occupied) {
					if (this->mData[x].data == data) {
						this->eraseDataAtIndex(x);
					}
				}
			}
		}

		void eraseDataAtIndex(const sizeType& index)
		{
			assert(this->mData[index].occupied);

			this->mData[index] = Node();
			if (index == this->mData.size() - 1) {
				this->mData.popBack();
			}
			else {
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
				if (this->mData[x].occupied) {
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
			assert(this->mData[index].occupied);
			return this->mData[index].data;
		}

		T& back() const
		{
			return this->mData.back().data;
		}

		bool empty() const
		{
			return this->actualCount() == 0;
		}

		void wrap()
		{
			this->mData.wrap();
		}

		sizeType internalCount() const
		{
			return this->mData.size();
		}

		sizeType actualCount() const
		{
			return this->mData.size() - this->mFreeSlotCount;
		}

		bool isIndexOccupied(const sizeType& index) const
		{
			return (index >= 0 && index < this->mData.size()) && this->mData[index].occupied;
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

				while (this->mIndex < size) {
					if (this->mArray->mData[this->mIndex].occupied) {
						this->mCurrent = &this->mArray->mData[this->mIndex];
						return;
					}
					++this->mIndex;
				}

				this->mCurrent = nullptr;
			}

			Node* mCurrent = nullptr;
			const RigidArray<T, sizeType>* mArray = nullptr;
			sizeType mIndex = 0;

		public:

			Iterator() {}

			Iterator(const RigidArray<T, sizeType>* array) : mArray(array)
			{
				this->moveToNext();
			}

			Iterator& operator++()
			{
				++this->mIndex;
				this->moveToNext();

				return *this;
			}

			Iterator& operator++(int)
			{
				++* this;
				return *this;
			}

			T& data() const
			{
				return this->mCurrent->data;
			}

			T& operator*()
			{
				return this->mCurrent->data;
			}

			sizeType index()
			{
				return this->mIndex;
			}

			bool operator==(const Iterator& other) const
			{
				return this->mCurrent == other.mCurrent;
			}

			bool operator!=(const Iterator& other) const
			{
				return !(*this == other);
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

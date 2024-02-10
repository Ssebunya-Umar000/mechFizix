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

#ifndef SORTEDARRAY_H
#define SORTEDARRAY_H

#include"rigidArray.h"

namespace mech {

	/*
		the data is stored in a rigid array, which ensures that data will always remain at the index it has been inserted - see RigidArray<T, sizeType>.
		NOTE: the address of the data might however change, do not store pointers!!
	*/
	template<typename T, typename sizeType>
	class SortedArray {

	private:
		
		struct Node {
			T data = T();
			sizeType min = -1;
			sizeType max = -1;

			Node() {}
			Node(const T& d) : data(d) {}
		};

		RigidArray<Node, sizeType> mData;
		sizeType mMinimum = -1;
		sizeType mMaximum = -1;

	public:
		
		sizeType insert(const T& data)
		{
			if (this->mMinimum == (sizeType)(-1)) {
				
				this->mMinimum = this->mMaximum = this->mData.insert(Node(data));

				return this->mMinimum;
			}
			else {

				sizeType currentMin = this->mMinimum;
				sizeType currentMax = this->mMaximum;

				while (true) {

					if (this->mData[currentMin].data > data) {

						sizeType index = this->mData.insert(Node(data));

						if (this->mData[currentMin].min == (sizeType)(-1)) {
							this->mMinimum = index;
						}
						else {
							this->mData[index].min = this->mData[currentMin].min;
							this->mData[this->mData[currentMin].min].max = index;
						}

						this->mData[currentMin].min = index;
						this->mData[index].max = currentMin;

						return index;
					}
					else if (this->mData[currentMax].data < data) {

						sizeType index = this->mData.insert(Node(data));

						if (this->mData[currentMax].max == (sizeType)(-1)) {
							this->mMaximum = index;
						}
						else {
							this->mData[index].max = this->mData[currentMax].max;
							this->mData[this->mData[currentMax].max].min = index;
						}

						this->mData[currentMax].max = index;
						this->mData[index].min = currentMax;

						return index;
					}
					else if (this->mData[currentMin].data == data) {
						return currentMin;
					}
					else if (this->mData[currentMax].data == data) {
						return currentMax;
					}

					currentMin = this->mData[currentMin].max;
					currentMax = this->mData[currentMax].min;
				}
			}

			return -1;
		}

		void eraseData(const T& data)
		{
			if (this->mMinimum == (sizeType)(-1)) return;

			sizeType currentMin = this->mMinimum;
			sizeType currentMax = this->mMaximum;

			while (true) {

				if (this->mData[currentMin].data == data) {
					this->eraseDataAtIndex(currentMin);
					break;
				}

				if (currentMin == currentMax) break;

				if (this->mData[currentMax].data == data) {
					this->eraseDataAtIndex(currentMax);
					break;
				}

				if (this->mData[currentMin].max == currentMax) break;

				currentMin = this->mData[currentMin].max;
				currentMax = this->mData[currentMax].min;
			}
		}

		void eraseDataAtIndex(const sizeType& index)
		{
			if (index == this->mMinimum) {
				this->mMinimum = this->mData[index].max;
				if (this->mMinimum == (sizeType)(-1)) {
					this->mMaximum = -1;
				}
				else {
					this->mData[this->mMinimum].min = -1;
				}
			}
			else if (index == this->mMaximum) {
				this->mMaximum = this->mData[index].min;
				if (this->mMaximum == (sizeType)(-1)) {
					this->mMinimum = -1;
				}
				else {
					this->mData[this->mMaximum].max = -1;
				}
			}
			else {
				this->mData[this->mData[index].max].min = this->mData[index].min;
				this->mData[this->mData[index].min].max = this->mData[index].max;
			}

			this->mData.eraseDataAtIndex(index);
		}

		T* find(const T& data)
		{
			if (this->mMinimum == (sizeType)(-1)) return nullptr;

			sizeType currentMin = this->mMinimum;
			sizeType currentMax = this->mMaximum;

			while (true) {

				if (this->mData[currentMin].data == data) return &this->mData[currentMin].data;

				if (currentMin == currentMax) break;

				if (this->mData[currentMax].data == data) return &this->mData[currentMax].data;

				if (this->mData[currentMin].max == currentMax) break;

				currentMin = this->mData[currentMin].max;
				currentMax = this->mData[currentMax].min;
			}

			return nullptr;
		}

		T& minimum() const
		{
			return this->mData[this->mMinimum].data;
		}

		T& maximum() const
		{
			return this->mData[this->mMaximum].data;
		}

		T& operator[](const sizeType& index) const
		{
			return this->mData[index].data;
		}

		sizeType size()
		{
			return this->mData.actualCount();
		}

		bool empty()
		{
			return this->mMinimum == (sizeType)(-1);
		}

		void wrap()
		{
			this->mData.wrap();
		}

		void shallowClear()
		{
			this->mData.shallowClear();
			this->mMinimum = -1;
			this->mMaximum = -1;
		}

		void deepClear()
		{
			this->mData.deepClear();
			this->mMinimum = -1;
			this->mMaximum = -1;
		}

		//////////////////////////////////////////////////
		class Iterator {
		private:
			Node* mCurrent = nullptr;
			const SortedArray<T, sizeType>* mList = nullptr;
			sizeType mIndex = -1;

		public:

			Iterator() {}

			Iterator(const SortedArray<T, sizeType>* list, const sizeType& index) : mList(list), mIndex(index)
			{
				if (this->mIndex != (sizeType)(-1)) {
					this->mCurrent = &this->mList->mData[this->mIndex];
				}
			}

			Iterator& operator++()
			{
				this->mIndex = this->mCurrent->max;
				if (this->mIndex == (sizeType)(-1)) {
					this->mCurrent = nullptr;
				}
				else {
					this->mCurrent = &this->mList->mData[this->mIndex];
				}

				return *this;
			}

			Iterator& operator--()
			{
				this->mIndex = this->mCurrent->min;
				if (this->mIndex == (sizeType)(-1)) {
					this->mCurrent = nullptr;
				}
				else {
					this->mCurrent = &this->mList->mData[this->mIndex];
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

			T& data()
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
			return Iterator(this, this->mMinimum);
		}

		Iterator last() const
		{
			return Iterator(this, this->mMaximum);
		}

		Iterator end() const
		{
			return Iterator();
		}
	};
}

#endif
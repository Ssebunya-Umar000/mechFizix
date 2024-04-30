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
			sizeType prev = -1;
			sizeType next = -1;

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

						if (this->mData[currentMin].prev == (sizeType)(-1)) {
							this->mMinimum = index;
						}
						else {
							this->mData[index].prev = this->mData[currentMin].prev;
							this->mData[this->mData[currentMin].prev].next = index;
						}

						this->mData[currentMin].prev = index;
						this->mData[index].next = currentMin;

						return index;
					}
					else if (this->mData[currentMax].data < data) {

						sizeType index = this->mData.insert(Node(data));

						if (this->mData[currentMax].next == (sizeType)(-1)) {
							this->mMaximum = index;
						}
						else {
							this->mData[index].next = this->mData[currentMax].next;
							this->mData[this->mData[currentMax].next].prev = index;
						}

						this->mData[currentMax].next = index;
						this->mData[index].prev = currentMax;

						return index;
					}
					else if (this->mData[currentMin].data == data) {
						return currentMin;
					}
					else if (this->mData[currentMax].data == data) {
						return currentMax;
					}

					currentMin = this->mData[currentMin].next;
					currentMax = this->mData[currentMax].prev;
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

				if (this->mData[currentMin].next == currentMax) break;

				currentMin = this->mData[currentMin].next;
				currentMax = this->mData[currentMax].prev;
			}
		}

		void eraseDataAtIndex(const sizeType index)
		{
			if (index == this->mMinimum) {
				this->mMinimum = this->mData[index].next;
				if (this->mMinimum == (sizeType)(-1)) {
					this->mMaximum = -1;
				}
				else {
					this->mData[this->mMinimum].prev = -1;
				}
			}
			else if (index == this->mMaximum) {
				this->mMaximum = this->mData[index].prev;
				if (this->mMaximum == (sizeType)(-1)) {
					this->mMinimum = -1;
				}
				else {
					this->mData[this->mMaximum].next = -1;
				}
			}
			else {
				this->mData[this->mData[index].next].prev = this->mData[index].prev;
				this->mData[this->mData[index].prev].next = this->mData[index].next;
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

				if (this->mData[currentMin].next == currentMax) break;

				currentMin = this->mData[currentMin].next;
				currentMax = this->mData[currentMax].prev;
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

		sizeType size() const
		{
			return this->mData.size();
		}

		bool empty() const
		{
			return this->mMinimum == (sizeType)(-1);
		}

		void wrap()
		{
			this->mData.wrap();
		}

		void shallowClear(bool callDestructors)
		{
			this->mData.shallowClear(callDestructors);
			this->mMinimum = -1;
			this->mMaximum = -1;
		}

		void clear()
		{
			this->mData.clear();
			this->mMinimum = -1;
			this->mMaximum = -1;
		}

		//////////////////////////////////////////////////
		class Iterator {
		private:
			const SortedArray<T, sizeType>* mArray;
			sizeType mCurrentIndex = -1;

		public:

			Iterator(const SortedArray<T, sizeType>* arr) : mArray(arr) {}

			Iterator(const SortedArray<T, sizeType>* arr, const sizeType& index) : mArray(arr)
			{
				if (this->mArray->empty() == false) {
					this->mCurrentIndex = index;
				}
			}

			Iterator& operator++()
			{
				this->mCurrentIndex = this->mArray->mData[this->mCurrentIndex].next;

				return *this;
			}

			Iterator& operator--()
			{
				this->mCurrentIndex = this->mArray->mData[this->mCurrentIndex].prev;

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

			bool isEnd() const
			{
				return this->mCurrentIndex == (sizeType)(-1);
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
			return Iterator(this);
		}
	};
}

#endif
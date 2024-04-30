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

#ifndef QUEUE_H
#define QUEUE_H

#include"rigidArray.h"

namespace mech {

	/*
		this class represents a queue - first in first out
		the data is stored in a rigid array, which ensures that data will always remain at the index it has been inserted - see RigidArray<T, sizeType>.
		NOTE: the address of the data might however change, do not store pointers!!
	*/
	template<typename T, typename sizeType>
	class Queue {

	private:

		struct Node {
			T data = T();
			sizeType next = -1;

			Node() {}
			Node(const T& d) : data(d) {}
		};

		RigidArray<Node, sizeType> mData;
		sizeType mFront = -1;
		sizeType mBack = -1;

	public:
		
		sizeType push(const T& data)
		{
			sizeType index = this->mData.insert(Node(data));

			if (this->mFront == (sizeType)(-1)) {
				this->mFront = this->mBack = index;
			}
			else {
				this->mData[this->mBack].next = index;
				this->mBack = index;
			}

			return index;
		}

		void pop()
		{
			if (this->mFront == (sizeType)(-1)) return;

			sizeType index = this->mFront;
			if (this->mFront == this->mBack) {
				this->mFront = this->mBack = -1;
			}
			else {
				this->mFront = this->mData[this->mFront].next;
			}

			this->mData.eraseDataAtIndex(index);
		}

		void eraseData(const T& data)
		{
			sizeType current = this->mFront;
			sizeType parent = -1;
			while (current != (sizeType)(-1)) {

				if (this->mData[current].data == data) {

					if (parent == (sizeType)(-1)) {
						if (this->mFront == this->mBack) {
							this->mFront = this->mBack = -1;
						}
						else {
							this->mFront = this->mData[this->mFront].next;
						}
					}
					else {
						if (current == this->mBack) {
							this->mBack = parent;
						}
						this->mData[parent].next = this->mData[current].next;
					}

					this->mData.eraseDataAtIndex(current);
					break;
				}

				parent = current;
				current = this->mData[current].next;
			}
		}

		void eraseDataAtIndex(const sizeType index)
		{
			if (index == this->mFront) {
				this->pop();
			}
			else {
				
				sizeType current = this->mFront;
				while (current != (sizeType)(-1)) {

					if (this->mData[current].next == index) break;

					current = this->mData[current].next;
				}

				if (this->mData[current].next == this->mBack) {
					this->mBack = current;
				}
				this->mData[current].next = this->mData[index].next;

				this->mData.eraseDataAtIndex(index);
			}
		}

		T* find(const T& data)
		{
			sizeType current = this->mFront;
			while (current != (sizeType)(-1)) {

				if (this->mData[current].data == data) {
					return &this->mData[current].data;
				}

				current = this->mData[current].next;
			}

			return nullptr;
		}

		T& operator[](const sizeType& index) const
		{
			return this->mData[index].data;
		}

		T& front() const
		{
			return this->mData[this->mFront].data;
		}

		sizeType size() const
		{
			return this->mData.size();
		}

		bool empty() const
		{
			return this->mFront == (sizeType)(-1);
		}

		void wrap()
		{
			this->mData.wrap();
		}

		void shallowClear(bool callDestructors)
		{
			this->mData.shallowClear(callDestructors);
			this->mFront = -1;
			this->mBack = -1;
		}

		void clear()
		{
			this->mData.clear();
			this->mFront = -1;
			this->mBack = -1;
		}

		//////////////////////////////////////////////////
		class Iterator {
		private:
			const Queue<T, sizeType>* mQueue;
			sizeType mCurrentIndex = -1;

		public:

			Iterator(const Queue<T, sizeType>* queue) : mQueue(queue) {}

			Iterator(const Queue<T, sizeType>* queue, const sizeType& index) : mQueue(queue)
			{
				if (this->mQueue->empty() == false) {
					this->mCurrentIndex = index;
				}
			}

			Iterator& operator++()
			{
				this->mCurrentIndex = this->mQueue->mData[this->mCurrentIndex].next;

				return *this;
			}

			Iterator& operator++(int)
			{
				++*this;

				return *this;
			}

			T& data()
			{
				return this->mQueue->mData[this->mCurrentIndex].data;
			}

			T& operator*()
			{
				return this->mQueue->mData[this->mCurrentIndex].data;
			}

			sizeType index()
			{
				return this->mCurrentIndex;
			}

			bool operator==(const Iterator& other)
			{
				return this->mCurrentIndex == other.mCurrentIndex && this->mQueue == other.mQueue;
			}

			bool operator!=(const Iterator& other)
			{
				return this->mCurrentIndex != other.mCurrentIndex || this->mQueue != other.mQueue;
			}

			bool isEnd() const
			{
				return this->mCurrentIndex == (sizeType)(-1);
			}
		};

		Iterator begin() const
		{
			return Iterator(this, this->mFront);
		}

		Iterator end() const
		{
			return Iterator(this);
		}
	};
}

#endif

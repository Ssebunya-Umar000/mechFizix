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

#ifndef HASHTABLE_H
#define HASHTABLE_H

#include"rigidArray.h"
#include"pair.h"

namespace mech {

	///////////////////////////////////////////////////////////////////////////////////////////////////////////
	static uint64 hash(const int& number)
	{
		return number < 0 ? -number : number;
	}

	static uint64 hash(const uint32& number)
	{
		return number;
	}

	static uint64 hash(const void* ptr)
	{
		uint64 number = uint64(ptr);
		return number;
	}

	static uint64 hash(const char* str)
	{
		uint64 number = 0;
		uint64 index = 0;
		while (str[index] != '\0') {
			number += uint64(str[index]);
			index++;
		}
		return number;
	}

	template<typename T1, typename T2>
	static uint64 hash(const Pair<T1, T2>& pair)
	{
		return hash(pair.first);
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////
	/*
		a simple implementation of a hash table that uses linear probing to resolve collisions
		the data is stored in a rigid array, which ensures that data will always remain at the index it has been inserted - see RigidArray<T, sizeType>.
		NOTE: the address of the data might however change, do not store pointers!!
	*/
	template<typename T, typename sizeType>
	class HashTable {

	private:

		struct Node {
			T data = T();
			sizeType next = -1;
			Node() {}
			Node(const T& d) : data(d) {}
		};

		void prep(const sizeType& tableSize)
		{
			this->mData.resize(tableSize * this->mTableWeight);

			this->mHeadIndicies.reserve(tableSize);
			for (sizeType x = 0; x < tableSize; ++x) {
				this->mHeadIndicies[x] = -1;
			}
		}

		RigidArray<Node, sizeType> mData;
		DynamicArray<sizeType, sizeType> mHeadIndicies;
		sizeType mTableWeight = 10;

	public:

		HashTable() {}

		HashTable(const sizeType& tableSize, const sizeType& tableWeight)
		{
			this->mTableWeight = tableWeight;
			this->prep(tableSize);
		}

		sizeType insert(const T& data)
		{
			if (this->mHeadIndicies.size() == 0) {
				this->prep(5);
			}
			else if (this->size() == this->mData.size() - this->mHeadIndicies.size()) {

				RigidArray<Node, sizeType> temp;
				temp.swap(this->mData);

				this->prep(this->mHeadIndicies.size() * 2);

				for (sizeType x = 0, len = temp.size(); x < len; ++x) {
					this->insert(temp[x].data);
				}
			}

			sizeType hashValue = (hash(data) % this->mHeadIndicies.size());

			if (this->mHeadIndicies[hashValue] == (sizeType)(-1)) {
				this->mHeadIndicies[hashValue] = this->mData.insert(Node(data));
				return this->mHeadIndicies[hashValue];
			}
			else {

				sizeType index = this->mHeadIndicies[hashValue];
				sizeType last = -1;
				while (index != (sizeType)(-1)) {
					
					if (this->mData[index].data == data) {
						return index;
					}

					last = index;
					index = this->mData[index].next;
				}
				
				this->mData[last].next = this->mData.insert(Node(data));
				return this->mData[last].next;
			}

			return -1;
		}

		void eraseData(const T& data)
		{
			if (this->mHeadIndicies.size() == 0) return;

			sizeType hashValue = (hash(data) % this->mHeadIndicies.size());

			if (this->mHeadIndicies[hashValue] == (sizeType)(-1)) return;

			sizeType index = this->mHeadIndicies[hashValue];
			sizeType prev = -1;
			while (index != (sizeType)(-1)) {

				if (this->mData[index].data == data) {

					if (index == this->mHeadIndicies[hashValue]) {
						this->mHeadIndicies[hashValue] = this->mData[this->mHeadIndicies[hashValue]].next;
					}
					else {
						this->mData[prev].next = this->mData[index].next;
					}

					this->mData.eraseDataAtIndex(index);
					return;
				}

				prev = index;
				index = this->mData[index].next;
			}
		}

		void eraseDataAtIndex(const sizeType index)
		{
			this->eraseData(this->mData[index].data);
		}

		T* find(const T& data)
		{
			if (this->mHeadIndicies.size() == 0) return nullptr;

			sizeType index = this->mHeadIndicies[(hash(data) % this->mHeadIndicies.size())];

			while (index != (sizeType)(-1)) {
				if (this->mData[index].data == data) {
					return &this->mData[index].data;
				}

				index = this->mData[index].next;
			}

			return nullptr;
		}

		T& operator[](const sizeType& index) const
		{
			return this->mData[index].data;
		}

		bool empty() const
		{
			return this->size() == 0;
		}

		sizeType size() const
		{
			return this->mData.size();
		}

		void shallowClear(bool callDestructors)
		{
			this->mData.shallowClear(callDestructors);
			this->prep(this->mHeadIndicies.size());
		}

		void clear()
		{
			this->mData.clear();
			this->mBitSet.clear();
		}

		//////////////////////////////////////////////////////////
		class Iterator {
		private:

			void moveToNext()
			{
				sizeType size = this->mTable->mData.internalSize();
				while (this->mCurrentIndex < size) {
					if (this->mTable->mData.isIndexOccupied(this->mCurrentIndex)) return;
					++this->mCurrentIndex;
				}

				this->mCurrentIndex = -1;
			}

			const HashTable<T, sizeType>* mTable;
			sizeType mCurrentIndex = -1;

		public:

			Iterator(const HashTable<T, sizeType>* table) : mTable(table) {}

			Iterator(const HashTable<T, sizeType>* table, const sizeType& index) : mTable(table)
			{
				if (this->mTable->empty() == false) {
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
				return this->mTable->mData[this->mCurrentIndex].data;
			}

			T& operator*()
			{
				return this->mTable->mData[this->mCurrentIndex].data;
			}

			bool operator==(const Iterator& other)
			{
				return this->mCurrentIndex == other.mCurrentIndex && this->mTable == other.mTable;
			}

			bool operator!=(const Iterator& other)
			{
				return this->mCurrentIndex != other.mCurrentIndex || this->mTable != other.mTable;
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
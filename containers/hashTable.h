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

#ifndef HASHTABLE_H
#define HASHTABLE_H

#include"dynamicArray.h"
#include"pair.h"

namespace mech {

#define TABLE_WEIGHT 10
#define DEFAULT_TABLE_SIZE 5

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
	*/
	template<typename T, typename sizeType>
	class HashTable {

	private:

		/*
			------------flags------------
			is occupied      - 0b00000001
			is a hash head   - 0b00000010
		*/

		struct Node {
			T data = T();
			sizeType next = -1;
			byte flags = 0;
		};

		void prep(const sizeType& tableSize)
		{
			this->mTableSize = tableSize;
			this->mData.reserve(this->mTableSize * TABLE_WEIGHT);
			for (sizeType x = 0, len = this->mTableSize * TABLE_WEIGHT; x < len; x += TABLE_WEIGHT) {
				this->mData[x].flags |= 0b00000010;
			}
		}

		DynamicArray<Node, sizeType> mData;
		sizeType mTableSize = 0;
		sizeType mCount = 0;

	public:

		HashTable() {}

		HashTable(const sizeType& tableSize)
		{
			this->prep(tableSize);
		}

		T* insert(const T& data)
		{
			if (this->mTableSize == 0) {
				this->prep(DEFAULT_TABLE_SIZE);
			}
			else if (this->mCount == this->mData.size() - this->mTableSize) {
				
				DynamicArray<Node, sizeType> temp;
				temp.swap(this->mData);

				this->prep(this->mTableSize * 2);

				this->mCount = 0;
				for (sizeType x = 0, len = temp.size(); x < len; ++x) {
					this->insert(temp[x].data);
				}
			}

			sizeType index = (hash(data) % this->mTableSize) * TABLE_WEIGHT;
			sizeType prevIndex = -1;
			bool prevSet = false;

			while (this->mData[index].flags & 0b00000001) {

				if (this->mData[index].data == data) {
					return &this->mData[index].data;
				}

				if (prevSet == false) {

					if (this->mData[index].next == (sizeType)(-1)) {
						prevIndex = index;
						prevSet = true;
						++index;
					}
					else {
						index = this->mData[index].next;
					}
				}
				else {
					++index;
				}
				
				if (index == (this->mTableSize * TABLE_WEIGHT) - 1) {
					index = 0;
				}

				if ((this->mData[index].flags & 0b00000010)) {
					++index;
				}
			}
		
			this->mData[index].data = data;
			this->mData[index].next = -1;
			this->mData[index].flags |= 0b00000001;
			if (prevSet == true) {
				this->mData[prevIndex].next = index;
			}
			++this->mCount;

			return &this->mData[index].data;
		}

		void eraseData(const T& data)
		{
			if (this->mTableSize == 0) return;

			sizeType index = (hash(data) % this->mTableSize) * TABLE_WEIGHT;
			sizeType prevIndex = -1;

			if (this->mData[index].flags & 0b00000001) {

				while (index != (sizeType)(-1)) {

					if (this->mData[index].data == data) {

						if ((this->mData[index].flags & 0b00000010) && this->mData[index].next != (sizeType)(-1)) {

							sizeType temp = this->mData[index].next;

							this->mData[index].data = this->mData[temp].data;
							this->mData[index].next = this->mData[temp].next;

							this->mData[temp].flags &= 0b11111110;
							this->mData[temp].next = -1;
						}
						else {
							if (prevIndex != (sizeType)(-1)) {
								this->mData[prevIndex].next = this->mData[index].next;
							}

							this->mData[index].flags &= 0b11111110;
							this->mData[index].next = -1;
						}

						--this->mCount;
						break;
					}

					prevIndex = index;
					index = this->mData[index].next;
				}
			}
		}

		T* find(const T& data)
		{
			if (this->mTableSize == 0) return nullptr;

			sizeType index = (hash(data) % this->mTableSize) * TABLE_WEIGHT;

			if (this->mData[index].flags & 0b00000001) {

				while (index != (sizeType)(-1)) {
					if (this->mData[index].data == data) {
						return &this->mData[index].data;
					}

					index = this->mData[index].next;
				}
			}

			return nullptr;
		}

		bool empty() const
		{
			return this->mCount == 0;
		}

		sizeType size() const
		{
			return this->mCount;
		}

		void shallowClear()
		{
			this->mCount = 0;

			for (sizeType x = 0, len = this->mTableSize * TABLE_WEIGHT; x < len; x += TABLE_WEIGHT) {

				sizeType index = x;
				if (this->mData[index].flags & 0b00000001) {

					while (index != (sizeType)(-1)) {

						sizeType temp = this->mData[index].next;
						this->mData[index].flags &= 0b11111110;
						this->mData[index].next = -1;
						index = temp;
					}
				}
			}
		}

		void deepClear()
		{
			this->mData.deepClear();
			this->mTableSize = 0;
			this->mCount = 0;
		}

		//////////////////////////////////////////////////////////
		class Iterator {
		private:

			void moveToNext()
			{
				while (this->mTableIndex < this->mTable->mTableSize) {

					if (this->mTable->mData[this->mTableIndex * TABLE_WEIGHT].flags & 0b00000001) {
						this->mCurrent = &this->mTable->mData[this->mTableIndex * TABLE_WEIGHT];
						break;
					}

					++this->mTableIndex;
				}
			}

			Node* mCurrent = nullptr;
			const HashTable<T, sizeType>* mTable = nullptr;
			sizeType mTableIndex = 0;

		public:
			
			Iterator() {}

			Iterator(const HashTable<T, sizeType>* table) : mTable(table)
			{
				if (this->mTable->empty() == false) {
					this->moveToNext();
				}
			}

			Iterator& operator++()
			{
				sizeType next = this->mCurrent->next;
				if (next == (sizeType)(-1)) {
					this->mCurrent = nullptr;
					++this->mTableIndex;
					this->moveToNext();
				}
				else {
					this->mCurrent = &this->mTable->mData[next];
				}

				return *this;
			}

			Iterator& operator++(int)
			{
				++* this;

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

			bool isValid()
			{
				return this->mCurrent->flags & 0b00000001;
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
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

#include"allocator.h"

#include"../core/assert.h"

#include<cstdlib>
#include<cstring>

namespace mech {

#define STRIDE 64
#define NUM_OF_BLOCKS 64
#define MAX_UNIT_SIZE 4096
#define BLOCK_SIZE 8192

#define SMALL_STRING_SIZE 24
#define NUM_OF_STRING_BLOCKS 400

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	class PoolAllocator {

	private:

		struct MemoryUnit {
			MemoryUnit* next = nullptr;
		};

		struct MemoryBlock {
			MemoryUnit* memoryUnits = nullptr;
		};

		MemoryBlock* mMemoryBlocks = nullptr;
		MemoryUnit* mFreeMemoryUnits[NUM_OF_BLOCKS] = {};

		uint16 mAllocatedMemoryBlocks = 0;
		uint16 mCurrentMemoryBlocks = 0;

		uint16 mUnitSizes[NUM_OF_BLOCKS] = {};
		uint16 mSizeMap[MAX_UNIT_SIZE + 1] = {};

		void* (*mAllocationFcn) (const uint64&) = nullptr;
		void (*mDeallocationFcn) (void*, const uint64&) = nullptr;

		uint32 mTotalAllocations = 0;
		uint32 mTotalDeallocations = 0;

	public:
		PoolAllocator() {}
		PoolAllocator(PoolAllocator&) = delete;
		PoolAllocator& operator=(PoolAllocator&) = delete;

		~PoolAllocator()
		{
			for (uint16 i = 0; i < this->mCurrentMemoryBlocks; i++) {
				this->mDeallocationFcn(this->mMemoryBlocks[i].memoryUnits, BLOCK_SIZE);
			}

			this->mDeallocationFcn(this->mMemoryBlocks, this->mAllocatedMemoryBlocks * sizeof(MemoryBlock));

			ASSERT(this->mTotalAllocations == this->mTotalDeallocations, "memory leak detected");
		}

		void initialise(void* (*allocationFcn) (const uint64&), void (*deallocationFcn) (void*, const uint64&))
		{
			for (uint16 i = 0; i < NUM_OF_BLOCKS; i++) {
				this->mUnitSizes[i] = (i + (uint16)1) * (uint16)STRIDE;
			}

			uint16 j = 0;
			this->mSizeMap[0] = -1;
			for (uint16 i = 1; i <= MAX_UNIT_SIZE; i++) {
				if (i <= this->mUnitSizes[j]) {
					this->mSizeMap[i] = j;
				}
				else {
					j++;
					this->mSizeMap[i] = j;
				}
			}

			this->mAllocationFcn = allocationFcn;
			this->mDeallocationFcn = deallocationFcn;

			this->mAllocatedMemoryBlocks = 64;

			uint64 size = this->mAllocatedMemoryBlocks * sizeof(MemoryBlock);
			this->mMemoryBlocks = (MemoryBlock*)(this->mAllocationFcn(size));
		}

		void* allocate(const uint64& sizeInBytes)
		{
			if (sizeInBytes == 0) return nullptr;

			void* pointer = nullptr;

			if (sizeInBytes > MAX_UNIT_SIZE) {
				pointer = this->mAllocationFcn(sizeInBytes);
			}
			else {
				uint16 indexHeap = this->mSizeMap[sizeInBytes];

				if (this->mFreeMemoryUnits[indexHeap] != nullptr) {
					MemoryUnit* unit = this->mFreeMemoryUnits[indexHeap];
					this->mFreeMemoryUnits[indexHeap] = unit->next;
					pointer = unit;
				}
				else {

					if (this->mCurrentMemoryBlocks == this->mAllocatedMemoryBlocks) {

						this->mAllocatedMemoryBlocks += 64;

						MemoryBlock* temp = this->mMemoryBlocks;
						this->mMemoryBlocks = (MemoryBlock*)(this->mAllocationFcn(this->mAllocatedMemoryBlocks * sizeof(MemoryBlock)));

						for (uint64 x = 0; x < this->mCurrentMemoryBlocks; ++x) {
							this->mMemoryBlocks[x] = temp[x];
						}
						memset(this->mMemoryBlocks + this->mCurrentMemoryBlocks, 0, 64 * sizeof(MemoryBlock));

						this->mDeallocationFcn(temp, this->mCurrentMemoryBlocks * sizeof(MemoryBlock));
					}

					MemoryBlock* newBlock = this->mMemoryBlocks + this->mCurrentMemoryBlocks;
					newBlock->memoryUnits = (MemoryUnit*)(this->mAllocationFcn(BLOCK_SIZE));

					uint64 unitSize = this->mUnitSizes[indexHeap];
					uint64 nbUnits = BLOCK_SIZE / unitSize;

					char* memoryUnitsStart = (char*)(newBlock->memoryUnits);
					for (uint64 i = 0; i < nbUnits - 1; i++) {
						MemoryUnit* unit = (MemoryUnit*)(memoryUnitsStart + unitSize * i);
						MemoryUnit * next = (MemoryUnit*)(memoryUnitsStart + unitSize * (i + 1));
						unit->next = next;
					}

					MemoryUnit * lastUnit = (MemoryUnit*)((memoryUnitsStart + unitSize * (nbUnits - 1)));
					lastUnit->next = nullptr;

					this->mFreeMemoryUnits[indexHeap] = newBlock->memoryUnits->next;
					++this->mCurrentMemoryBlocks;

					pointer = newBlock->memoryUnits;
				}
			}

			memset(pointer, 0, sizeInBytes);

			++this->mTotalAllocations;

			return pointer;
		}

		void deallocate(void* pointer, const uint64 & sizeInBytes)
		{
			if (pointer == nullptr || sizeInBytes == 0) return;

			if (sizeInBytes > MAX_UNIT_SIZE) {
				this->mDeallocationFcn(pointer, sizeInBytes);
			}
			else {
				uint16 indexHeap = this->mSizeMap[sizeInBytes];

				MemoryUnit* releasedUnit = (MemoryUnit*)(pointer);

				releasedUnit->next = this->mFreeMemoryUnits[indexHeap];
				this->mFreeMemoryUnits[indexHeap] = releasedUnit;
			}

			++this->mTotalDeallocations;
		}
	};
	PoolAllocator poolAllocator;

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	class StringAlloctor {

	private:

		struct Block {
			char data[SMALL_STRING_SIZE] = {};
			uint16 next = -1;
		};

		Block mBlocks[NUM_OF_STRING_BLOCKS] = {};
		Block* mFreeBlock = nullptr;

		uint32 mTotalAllocations = 0;
		uint32 mTotalDeallocations = 0;

	public:

		StringAlloctor()
		{
			for (uint16 x = 0, len = NUM_OF_STRING_BLOCKS - 1; x < len; ++x) {
				this->mBlocks[x].next = x + 1;
			}
			this->mFreeBlock = &this->mBlocks[0];
		}

		~StringAlloctor()
		{
			ASSERT(this->mTotalAllocations == this->mTotalDeallocations, "memory leak detected");
		}

		char* allocate(const uint64& sizeInBytes)
		{
			if (sizeInBytes == 0) return nullptr;

			if (sizeInBytes < SMALL_STRING_SIZE) {

				ASSERT(this->mFreeBlock->next < NUM_OF_STRING_BLOCKS, "no free block!!");

				Block* block = this->mFreeBlock;
				this->mFreeBlock = &this->mBlocks[this->mFreeBlock->next];
				block->next = -1;
				++this->mTotalAllocations;
				return block->data;
			}
			else {
				return (char*)poolAllocator.allocate(sizeInBytes);
			}
		}

		void deallocate(char* data, const uint64& sizeInBytes)
		{
			if (data == nullptr) return;

			if (sizeInBytes < SMALL_STRING_SIZE) {
				Block* block = ((Block*)data);
				block->next = this->mFreeBlock - this->mBlocks;
				this->mFreeBlock = block;
				++this->mTotalDeallocations;
			}
			else {
				poolAllocator.deallocate(data, sizeInBytes);
			}
		}
	};
	StringAlloctor stringAllocator;

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void* defaultAllocateFunction(const uint64& size)
	{
		return malloc(size);
	}

	void defaultDeallocateFunction(void* pointer, const uint64& size)
	{
		free(pointer);
	}

	void initialiseAllocator(void* (*allocationFcn) (const uint64&), void (*deallocationFcn) (void*, const uint64&))
	{
		if (allocationFcn == nullptr || deallocationFcn == nullptr) {
			poolAllocator.initialise(defaultAllocateFunction, defaultDeallocateFunction);
		}
		else {
			poolAllocator.initialise(allocationFcn, deallocationFcn);
		}
	}

	void* allocate(const uint64& sizeInBytes)
	{
		return poolAllocator.allocate(sizeInBytes);
	}

	void deallocate(void* data, const uint64& sizeInBytes)
	{
		poolAllocator.deallocate(data, sizeInBytes);
	}
	
	char* stringAllocate(const uint64& sizeInBytes)
	{
		return stringAllocator.allocate(sizeInBytes);
	}

	void stringDeallocate(char* data, const uint64& sizeInBytes)
	{
		stringAllocator.deallocate(data, sizeInBytes);
	}
}
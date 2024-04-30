#ifndef BSTREE_H
#define BSTREE_H

#include"rigidArray.h"
#include"stack.h"

namespace mech {

	template<typename T, typename sizeType>
	class BinarySearchTree {

	private:

		struct Node {
			T data;
			sizeType right = -1;
			sizeType left = -1;

			Node() {}
			Node(const T& d) : data(d) {}
		};

		RigidArray<Node, sizeType> mData;
		sizeType mRootIndex = -1;

	public:

		sizeType insert(const T& data)
		{
			sizeType currentIndex = this->mRootIndex;
			sizeType parentIndex = -1;

			while (currentIndex != (sizeType)(-1)) {

				parentIndex = currentIndex;

				if (this->mData[currentIndex].data > data) {
					currentIndex = this->mData[currentIndex].left;
				}
				else if (this->mData[currentIndex].data < data) {
					currentIndex = this->mData[currentIndex].right;
				}
				else {
					return currentIndex;
				}
			}

			sizeType index = this->mData.insert(Node(data));

			if (parentIndex == (sizeType)(-1)) {
				this->mRootIndex = index;
			}
			else if (this->mData[parentIndex].data < data) {
				this->mData[parentIndex].right = index;
			}
			else {
				this->mData[parentIndex].left = index;
			}

			return index;
		}

		T* find(const T& data)
		{
			sizeType currentIndex = this->mRootIndex;

			while (currentIndex != (sizeType)(-1)) {

				if (this->mData[currentIndex].data > data) {
					currentIndex = this->mData[currentIndex].left;
				}
				else if (this->mData[currentIndex].data < data) {
					currentIndex = this->mData[currentIndex].right;
				}
				else {
					return &this->mData[currentIndex].data;
				}
			}

			return nullptr;
		}

		void eraseData(const T& data)
		{
			sizeType currentIndex = this->mRootIndex;
			sizeType parentIndex = -1;

			while (currentIndex != (sizeType)(-1)) {

				if (this->mData[currentIndex].data < data) {
					parentIndex = currentIndex;
					currentIndex = this->mData[currentIndex].right;
				}
				else if (this->mData[currentIndex].data > data) {
					parentIndex = currentIndex;
					currentIndex = this->mData[currentIndex].left;
				}
				else {
					
					sizeType successor = -1;

					if (this->mData[currentIndex].left == (sizeType)(-1) || this->mData[currentIndex].right == (sizeType)(-1)) {
						successor = this->mData[currentIndex].left != (sizeType)(-1) ? this->mData[currentIndex].left : this->mData[currentIndex].right;
					}
					else {
						sizeType parent = currentIndex;
						successor = this->mData[currentIndex].right;

						while (this->mData[successor].left != (sizeType)(-1)) {
							parent = successor;
							successor = this->mData[successor].left;
						}

						if (parent != currentIndex) {
							
							if (this->mData[successor].right != (sizeType)(-1)) {
								this->mData[parent].left = this->mData[successor].right;
							}
							else {
								this->mData[parent].left = -1;
							}

							this->mData[successor].right = this->mData[currentIndex].right;
						}

						this->mData[successor].left = this->mData[currentIndex].left;
					}

					if (parentIndex == (sizeType)(-1)) {
						this->mRootIndex = successor;
					}
					else {
						if (this->mData[parentIndex].right == currentIndex) {
							this->mData[parentIndex].right = successor;
						}
						else {
							this->mData[parentIndex].left = successor;
						}
					}

					this->mData.eraseDataAtIndex(currentIndex);
					return;
				}
			}
		}

		void eraseDataAtIndex(const sizeType index)
		{
			this->eraseData(this->mData[index].data);
		}

		T& operator[](const sizeType& index) const
		{
			return this->mData[index].data;
		}

		T& rootData()
		{
			return this->mData[this->mRootIndex].data;
		}

		sizeType size() const
		{
			return this->mData.size();
		}

		bool empty() const
		{
			return this->mData.empty();
		}

		void wrap()
		{
			this->mData.wrap();
		}

		void shallowClear(bool callDestructors)
		{
			this->mData.shallowClear(callDestructors);
			this->mRootIndex = -1;
		}

		void clear()
		{
			this->mData.clear();
			this->mRootIndex = -1;
		}

		/////////////////////////////////////////////////////////////////////////////////////////////
		class Iterator {
		private:

			void moveToNext()
			{
				sizeType size = this->mTree->mData.internalSize();
				while (this->mCurrentIndex < size) {

					if (this->mTree->mData.isIndexOccupied(this->mCurrentIndex)) {
						return;
					}

					++this->mCurrentIndex;
				}

				this->mCurrentIndex = -1;
			}

			const BinarySearchTree<T, sizeType>* mTree;
			sizeType mCurrentIndex = -1;

		public:

			Iterator(const BinarySearchTree<T, sizeType>* tree) : mTree(tree) {}

			Iterator(const BinarySearchTree<T, sizeType>* tree, const sizeType& index) : mTree(tree)
			{
				if (this->mTree->empty() == false) {
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
				return this->mTree->mData[this->mCurrentIndex].data;
			}

			T& operator*()
			{
				return this->mTree->mData[this->mCurrentIndex].data;
			}

			sizeType index()
			{
				return this->mCurrentIndex;
			}

			bool operator==(const Iterator& other)
			{
				return this->mCurrentIndex == other.mCurrentIndex && this->mTree == other.mTree;
			}

			bool operator!=(const Iterator& other)
			{
				return this->mCurrentIndex != other.mCurrentIndex || this->mTree != other.mTree;
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

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

#ifndef AVLTREE_H
#define AVLTREE_H

#include"rigidArray.h"
#include"stack.h"

namespace mech {

	/*
		this class represents a self balancing binary search tree, the AVL tree.
		the data is stored in a rigid array, which ensures that data will always remain at the index it has been inserted - see RigidArray<T, sizeType>.
		NOTE: the address of the data might however change, do not store pointers!!
	*/
	template<typename T, typename sizeType>
	class AVLTree {

	private:

		struct Node {
			T data;
			sizeType right = -1;
			sizeType left = -1;
			byte height = 0;

			Node() : data(T()) {}
			Node(const T& d) : data(d), height(1) {}
		};

		byte getHeight(const sizeType& nodeIndex)
		{
			if (nodeIndex == (sizeType)(-1)) return 0;
			return this->mData[nodeIndex].height;
		}

		char getBalanceFactor(const sizeType& nodeIndex)
		{
			if (nodeIndex == (sizeType)(-1)) return 0;
			return getHeight(this->mData[nodeIndex].left) - getHeight(this->mData[nodeIndex].right);
		}

		void updateHeight(const sizeType& nodeIndex)
		{
			byte heightR = getHeight(this->mData[nodeIndex].right);
			byte heightL = getHeight(this->mData[nodeIndex].left);
			this->mData[nodeIndex].height = (heightR > heightL ? heightR : heightL) + 1;
		}

		sizeType rotateLeft(const sizeType& nodeIndex)
		{
			sizeType newRoot = this->mData[nodeIndex].right;

			this->mData[nodeIndex].right = this->mData[newRoot].left;
			this->mData[newRoot].left = nodeIndex;

			updateHeight(nodeIndex);
			updateHeight(newRoot);

			return newRoot;
		}

		sizeType rotateRight(const sizeType& nodeIndex)
		{
			sizeType newRoot = this->mData[nodeIndex].left;

			this->mData[nodeIndex].left = this->mData[newRoot].right;
			this->mData[newRoot].right = nodeIndex;

			updateHeight(nodeIndex);
			updateHeight(newRoot);

			return newRoot;
		}

		void balance(const Stack<sizeType, sizeType>& indexStack)
		{
			for (auto it = indexStack.begin(), end = indexStack.end(); it != end;) {

				sizeType nodeIndex = it.data();
				++it;

				updateHeight(nodeIndex);
				sizeType index = nodeIndex;

				char balanceFactor = getBalanceFactor(index);
				if (balanceFactor > 1) {

					if (getBalanceFactor(this->mData[index].left) >= 0) {
						index = rotateRight(index);
					}
					else if (getBalanceFactor(this->mData[index].left) < 0) {
						this->mData[index].left = rotateLeft(this->mData[index].left);
						index = rotateRight(index);
					}
				}
				else if (balanceFactor < -1) {

					if (getBalanceFactor(this->mData[index].right) <= 0) {
						index = rotateLeft(index);
					}
					else if (getBalanceFactor(this->mData[index].right) > 0) {
						this->mData[index].right = rotateRight(this->mData[index].right);
						index = rotateLeft(index);
					}
				}

				if (it.isVoid()) {
					this->mRootIndex = index;
				}
				else {
					sizeType parent = it.data();

					if (this->mData[parent].right == nodeIndex) {
						this->mData[parent].right = index;
					}
					else {
						this->mData[parent].left = index;
					}
				}
			}
		}

		RigidArray<Node, sizeType> mData;
		sizeType mRootIndex = -1;

	public:

		sizeType insert(const T& data)
		{
			sizeType currentIndex = this->mRootIndex;
			Stack<sizeType, sizeType> indexStack;

			while (currentIndex != (sizeType)(-1)) {

				indexStack.push(currentIndex);

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

			if (indexStack.empty()) {
				this->mRootIndex = index;
			}
			else {
				sizeType parent = indexStack.top();

				if (this->mData[parent].data < data) {
					this->mData[parent].right = index;
				}
				else {
					this->mData[parent].left = index;
				}

				balance(indexStack);
			}

			return index;
		}

		void eraseData(const T& data)
		{
			sizeType currentIndex = this->mRootIndex;
			Stack<sizeType, sizeType> indexStack;

			while (currentIndex != (sizeType)(-1)) {

				if (this->mData[currentIndex].data > data) {
					indexStack.push(currentIndex);
					currentIndex = this->mData[currentIndex].left;
				}
				else if (this->mData[currentIndex].data < data) {
					indexStack.push(currentIndex);
					currentIndex = this->mData[currentIndex].right;
				}
				else {

					sizeType successor = -1;

					if (this->mData[currentIndex].left == (sizeType)(-1) || this->mData[currentIndex].right == (sizeType)(-1)) {
						successor = this->mData[currentIndex].left != (sizeType)(-1) ? this->mData[currentIndex].left : this->mData[currentIndex].right;
					}
					else {
						sizeType successorParent = currentIndex;
						successor = this->mData[currentIndex].right;

						while (this->mData[successor].left != (sizeType)(-1)) {
							successorParent = successor;
							successor = this->mData[successor].left;
						}

						if (successorParent != currentIndex) {

							if (this->mData[successor].right != (sizeType)(-1)) {
								this->mData[successorParent].left = this->mData[successor].right;
							}
							else {
								this->mData[successorParent].left = -1;
							}

							this->mData[successor].right = this->mData[currentIndex].right;
						}

						this->mData[successor].left = this->mData[currentIndex].left;
					}

					if (indexStack.empty() == true) {
						this->mRootIndex = successor;
					}
					else {
						sizeType parent = indexStack.top();
						if (this->mData[parent].right == currentIndex) {
							this->mData[parent].right = successor;
						}
						else {
							this->mData[parent].left = successor;
						}
					}

					if (successor != (sizeType)(-1)) {
						indexStack.push(successor);
					}

					balance(indexStack);

					this->mData.eraseDataAtIndex(currentIndex);
					return;
				}
			}
		}

		void eraseDataAtIndex(const sizeType& index)
		{
			this->eraseData(this->mData[index].data);
		}

		T* find(const T& data)
		{
			sizeType currentIndex = this->mRootIndex;

			while (currentIndex != (sizeType)(-1)) {

				if (this->mData[currentIndex].data < data) {
					currentIndex = this->mData[currentIndex].right;
				}
				else if (this->mData[currentIndex].data > data) {
					currentIndex = this->mData[currentIndex].left;
				}
				else {
					return &this->mData[currentIndex].data;
				}
			}

			return nullptr;
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
			return this->mData.actualCount();
		}

		bool empty() const
		{
			return this->mData.empty();
		}

		void wrap()
		{
			this->mData.wrap();
		}

		void shallowClear()
		{
			this->mData.shallowClear();
			this->mRootIndex = -1;
		}

		void deepClear()
		{
			this->mData.deepClear();
			this->mRootIndex = -1;
		}

		/////////////////////////////////////////////////////////////////////////////
		class Iterator {
		private:

			void moveToNext()
			{
				sizeType size = this->mTree->mData.internalCount();
				while (this->mIndex < size) {

					if (this->mTree->mData.isIndexOccupied(this->mIndex)) {
						this->mCurrent = &this->mTree->mData[this->mIndex];
						return;
					}

					++this->mIndex;
				}

				this->mCurrent = nullptr;
			}

			Node* mCurrent = nullptr;
			const AVLTree<T, sizeType>* mTree = nullptr;
			sizeType mIndex = 0;

		public:

			Iterator() {}

			Iterator(const AVLTree<T, sizeType>* tree) : mTree(tree)
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
				++*this;

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
			return Iterator(this);
		}

		Iterator end() const
		{
			return Iterator();
		}
	};
}

#endif

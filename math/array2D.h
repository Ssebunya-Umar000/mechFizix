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

#ifndef ARRAY2D_H
#define ARRAY2D_H

#include"math.h"

namespace mech {

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////
	enum class Alignment : byte { columnMajor = 1<<0, rowMajor = 1<<1 };

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/*
		this struct represents a 2 dimensional array of numbers and operations perfomed on the following matrix operation rules
	*/
	template<typename T, uint16 rows, uint16 columns, Alignment alignment>
	struct Array2D {

		union {
			struct {
				T dataCxR[columns][rows];
			};
			struct {
				T dataRxC[rows][columns];
			};
			T data[rows * columns] = {};
		};

		Array2D()
		{
			uint16 len = rows * columns;
			for (uint16 x = 0; x < len; ++x) {
				this->data[x] = T();
			}
		}

		explicit Array2D(const T* d)
		{
			uint16 len = rows * columns;
			for (uint16 x = 0; x < len; ++x) {
				this->data[x] = d[x];
			}
		}

		template<uint16 otherColumns>
		Array2D<T, rows, otherColumns, Alignment::columnMajor> operator*(const Array2D<T, columns, otherColumns, Alignment::columnMajor>& other) const
		{
			Array2D<T, rows, otherColumns, Alignment::columnMajor> result;

			for (uint16 x = 0; x < otherColumns; ++x) {
				for (uint16 y = 0; y < rows; ++y) {
					result.data[x * otherColumns + y] = T(0);
					for (uint16 z = 0; z < columns; ++z) {
						result.data[x * otherColumns + y] += this->dataCxR[z][y] * other.dataCxR[x][z];
					}
				}
			}

			return result;
		}

		template<uint16 otherColumns>
		Array2D<T, rows, otherColumns, Alignment::rowMajor> operator*(const Array2D<T, columns, otherColumns, Alignment::rowMajor>& other) const
		{
			Array2D<T, rows, otherColumns, Alignment::rowMajor> result;

			for (uint16 x = 0; x < rows; ++x) {
				for (uint16 y = 0; y < otherColumns; ++y) {
					result.data[x * rows + y] = T(0);
					for (uint16 z = 0; z < columns; ++z) {
						result.data[x * rows + y] += this->dataRxC[x][z] * other.dataRxC[z][y];
					}
				}
			}

			return result;
		}

		Array2D<T, rows, columns, alignment> operator+(const Array2D<T, rows, columns, alignment>& other) const
		{
			Array2D<T, rows, columns, alignment> result;

			uint16 len = rows * columns;
			for (uint16 x = 0; x < len; ++x) {
				result.data[x] = this->data[x] + other.data[x];
			}

			return result;
		}

		Array2D<T, rows, columns, alignment> operator-(const Array2D<T, rows, columns, alignment>& other) const
		{
			Array2D<T, rows, columns, alignment> result;

			uint16 len = rows * columns;
			for (uint16 x = 0; x < len; ++x) {
				result.data[x] = this->data[x] - other.data[x];
			}

			return result;
		}

		Array2D<T, rows, columns, alignment> operator-() const
		{
			Array2D<T, rows, columns, alignment> result;

			uint16 len = rows * columns;
			for (uint16 x = 0; x < len; ++x) {
				result.data[x] = -this->data[x];
			}

			return result;
		}

		Array2D<T, rows, columns, alignment> operator*(const T& scalar) const
		{
			Array2D<T, rows, columns, alignment> result;

			uint16 len = rows * columns;
			for (uint16 x = 0; x < len; ++x) {
				result.data[x] = this->data[x] * scalar;
			}

			return result;
		}

		Array2D<T, rows, columns, alignment> operator/(const T& scalar) const
		{
			Array2D<T, rows, columns, alignment> result;

			uint16 len = rows * columns;
			for (uint16 x = 0; x < len; ++x) {
				result.data[x] = this->data[x] / scalar;
			}

			return result;
		}

		void operator*=(const Array2D<T, rows, columns, alignment>& other)
		{
			*this = *this * other;
		}

		void operator+=(const Array2D<T, rows, columns, alignment>& other)
		{
			*this = *this + other;
		}

		void operator-=(const Array2D<T, rows, columns, alignment>& other)
		{
			*this = *this - other;
		}

		void operator*=(const T& scalar)
		{
			*this = *this * scalar;
		}

		void operator/=(const T& scalar)
		{
			*this = *this / scalar;
		}

		void operator-()
		{
			*this = -*this;
		}

		bool operator==(const Array2D<T, rows, columns, alignment>& other) const
		{
			uint16 len = rows * columns;
			for (uint16 x = 0; x < len; ++x) {
				if (almostEqual(this->data[x], other.data[x]) == false) return false;
			}

			return true;
		}

		bool operator!=(const Array2D<T, rows, columns, alignment>& other) const
		{
			uint16 len = rows * columns;
			for (uint16 x = 0; x < len; ++x) {
				if (almostEqual(this->data[x], other.data[x]) == false) return true;
			}

			return false;
		}

		void setDiagnol(const T& d)
		{
			uint16 len = rows * columns;
			uint16 stride = rows + 1;
			for (uint16 x = 0; x < len; x += stride) {
				this->data[x] = d;
			}
		}
	};

	template<typename T, uint16 rows, uint16 columns>
	void setR(Array2D<T, rows, columns, Alignment::columnMajor>& mat, const uint16& index, const T* in)
	{
		for (uint16 x = 0; x < columns; ++x) {
			mat.dataCxR[x][index] = in[x];
		}
	}

	template<typename T, uint16 rows, uint16 columns>
	void setR(Array2D<T, rows, columns, Alignment::rowMajor>& mat, const uint16& index, const T* in)
	{
		for (uint16 x = 0; x < columns; ++x) {
			mat.dataRxC[index][x] = in[x];
		}
	}

	template<typename T, uint16 rows, uint16 columns>
	void setC(Array2D<T, rows, columns, Alignment::columnMajor>& mat, const uint16& index, const T* in)
	{
		for (uint16 x = 0; x < rows; ++x) {
			mat.dataCxR[index][x] = in[x];
		}
	}

	template<typename T, uint16 rows, uint16 columns>
	void setC(Array2D<T, rows, columns, Alignment::rowMajor>& mat, const uint16& index, const T* in)
	{
		for (uint16 x = 0; x < rows; ++x) {
			mat.dataRxC[x][index] = in[x];
		}
	}

	template<typename T, uint16 rows, uint16 columns>
	Array2D<T, 1, columns, Alignment::rowMajor> getR(const Array2D<T, rows, columns, Alignment::columnMajor>& mat, const uint16& index)
	{
		Array2D<T, 1, columns, Alignment::rowMajor> out;
		for (uint16 x = 0; x < columns; ++x) {
			out.dataRxC[0][x] = mat.dataCxR[x][index];
		}
		return out;
	}

	template<typename T, uint16 rows, uint16 columns>
	Array2D<T, 1, columns, Alignment::rowMajor> getR(const Array2D<T, rows, columns, Alignment::rowMajor>& mat, const uint16& index)
	{
		Array2D<T, 1, columns, Alignment::rowMajor> out;
		for (uint16 x = 0; x < columns; ++x) {
			out.dataRxC[0][x] = mat.dataRxC[index][x];
		}
		return out;
	}

	template<typename T, uint16 rows, uint16 columns>
	Array2D<T, rows, 1, Alignment::columnMajor> getC(const Array2D<T, rows, columns, Alignment::columnMajor>& mat, const uint16& index)
	{
		Array2D<T, rows, 1, Alignment::columnMajor> out;
		for (uint16 x = 0; x < rows; ++x) {
			out.dataCxR[0][x] = mat.dataCxR[index][x];
		}
		return out;
	}

	template<typename T, uint16 rows, uint16 columns>
	Array2D<T, rows, 1, Alignment::columnMajor> getC(const Array2D<T, rows, columns, Alignment::rowMajor>& mat, const uint16& index)
	{
		Array2D<T, rows, 1, Alignment::columnMajor> out;
		for (uint16 x = 0; x < rows; ++x) {
			out.dataCxR[0][x] = mat.dataRxC[x][index];
		}
		return out;
	}

	template<typename T, uint16 rows, uint16 columns>
	T& rXc(Array2D<T, rows, columns, Alignment::columnMajor>& mat, const uint16& row, const uint16& col)
	{
		return mat.dataCxR[col][row];
	}

	template<typename T, uint16 rows, uint16 columns>
	T rXc(const Array2D<T, rows, columns, Alignment::columnMajor>& mat, const uint16& row, const uint16& col)
	{
		return mat.dataCxR[col][row];
	}

	template<typename T, uint16 rows, uint16 columns>
	T& rXc(Array2D<T, rows, columns, Alignment::rowMajor>& mat, const uint16& row, const uint16& col)
	{
		return mat.dataRxC[row][col];
	}

	template<typename T, uint16 rows, uint16 columns>
	T rXc(const Array2D<T, rows, columns, Alignment::rowMajor>& mat, const uint16& row, const uint16& col)
	{
		return mat.dataRxC[row][col];
	}

	template<typename T, uint16 rows, uint16 columns>
	Array2D<T, columns, rows, Alignment::columnMajor> transpose(const Array2D<T, rows, columns, Alignment::columnMajor>& mat)
	{
		Array2D<T, columns, rows, Alignment::columnMajor> t;

		for (uint16 x = 0; x < columns; ++x) {
			for (uint16 y = 0; y < rows; ++y) {
				t.dataCxR[y][x] = mat.dataCxR[x][y];
			}
		}

		return t;
	}

	template<typename T, uint16 rows, uint16 columns>
	Array2D<T, columns, rows, Alignment::rowMajor> transpose(const Array2D<T, rows, columns, Alignment::rowMajor>& mat)
	{
		Array2D<T, columns, rows, Alignment::rowMajor> t;

		for (uint16 x = 0; x < rows; ++x) {
			for (uint16 y = 0; y < columns; ++y) {
				t.dataRxC[y][x] = mat.dataRxC[x][y];
			}
		}

		return t;
	}
}

#endif

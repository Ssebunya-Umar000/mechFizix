#ifndef STRING_H
#define STRING_H

#include"../allocator/allocator.h"

namespace mech {

	class String {

	private:

		void clone(const String& other)
		{
			this->clear();
			
			if (other.mLength > 0) {

				this->mLength = other.mLength;
				this->mData = stringAllocate(this->mLength + 1);
				for (uint64 x = 0; x < this->mLength; ++x) {
					this->mData[x] = other.mData[x];
				}
				this->mData[this->mLength] = '\0';
			}
		}

		void hijack(String& other)
		{
			this->clear();

			this->mData = other.mData;
			this->mLength = other.mLength;
			other.mData = nullptr;
			other.mLength = 0;
		}

		char* mData = nullptr;
		uint64 mLength = 0;

	public:

		String() {}

		explicit String(const char* str)
		{
			while (str[this->mLength] != '\0') {
				this->mLength++;
			}

			if (this->mLength > 0) {

				this->mData = stringAllocate(this->mLength + 1);
				for (uint64 x = 0; x < this->mLength; ++x) {
					this->mData[x] = str[x];
				}
				this->mData[this->mLength] = '\0';
			}
		}

		~String()
		{
			this->clear();
		}

		String(const String& other)
		{
			clone(other);
		}

		String(String&& other) noexcept
		{
			hijack(other);
		}

		String& operator=(const String& other)
		{
			if (this != &other) {
				clone(other);
			}
			return *this;
		}

		String& operator=(String&& other) noexcept
		{
			hijack(other);
			return *this;
		}

		void operator+=(const String& other)
		{
			if (other.mLength > 0) {

				uint64 oldLength = this->mLength;
				char* temp = this->mData;

				this->mLength += other.mLength;
				this->mData = stringAllocate(this->mLength + 1);
				for (uint64 x = 0; x < oldLength; ++x) {
					this->mData[x] = temp[x];
				}
				for (uint64 x = oldLength; x < this->mLength; ++x) {
					this->mData[x] = other.mData[x - oldLength];
				}
				this->mData[this->mLength] = '\0';

				stringDeallocate(temp, oldLength + 1);
			}
		}

		void operator+=(const char* str)
		{
			*this += String(str);
		}

		String operator+(const String& other) const
		{
			String str = *this;
			str += other;
			return str;
		}

		String operator+(const char* str) const
		{
			String s = *this;
			s += str;
			return s;
		}

		const char& operator[](const uint64& index) const
		{
			ASSERT((this->mLength > 0) && (index >= 0) && (index < this->mLength), "index out of range or data is null");
			return this->mData[index];
		}

		uint64 length() const
		{
			return this->mLength;
		}

		const char* cString() const
		{
			return this->mData;
		}

		bool empty() const
		{
			return this->mLength == 0;
		}

		void reduce(const uint32& num)
		{
			this->mLength -= num;
			this->mData[this->mLength] = '\0';
		}

		void clear()
		{
			stringDeallocate(this->mData, this->mLength + 1);
			this->mData = nullptr;
			this->mLength = 0;
		}
	};

	//////////////////////////////////////////////////////////////////////////////////////////////////////////
	static String operator+(const char* charStr, const String& str)
	{
		return String(charStr) + str;
	}

	static uint32 length(const char* str)
	{
		uint32 len = 0;
		while (str[len] != '\0') {
			len++;
		}

		return len;
	}

	static String toString1(const long long& number)
	{
		if (number == 0) return String("0");

		unsigned long long numDigits = 0;
		long long temp1 = number;
		while (temp1 != 0) {
			temp1 /= 10;
			numDigits++;
		}

		char str[50];
		unsigned long long index = numDigits;
		long long temp2 = number;
		if (temp2 < 0) {
			str[0] = '-';
			str[numDigits + 1] = '\0';
			index++;
			temp2 = -temp2;
		}
		else {
			str[numDigits] = '\0';
		}

		while (temp2 != 0) {
			str[--index] = '0' + (temp2 % 10);
			temp2 /= 10;
		}

		return String(str);
	}

	static String toString2(const double& number)
	{
		double n = number;

		if ((n < 0)) n = -n;

		long long intPart = n;
		String str;
		if (intPart == 0) {
			str = String("0");
		}
		else {
			while (intPart > 0) {
				char digit = (intPart % 10);
				str = toString1(digit) + str;
				intPart /= 10;
			}
		}

		str += String(".");

		double fractionPart = n - (long long)(n);
		for (int i = 0; i < 16; ++i) {
			fractionPart *= 10;
			int digit = (int)(fractionPart);
			str += toString1(digit);
			fractionPart -= digit;
		}

		if (number < 0) str = String("-") + str;

		return str;
	}

	static String toString3(const void* address)
	{
		static const char hexDigits[] = "0123456789abcdef";

		unsigned long long addrValue = (unsigned long long)address;

		char str[30];
		uint16 index = 0;
		for (int32 x = sizeof(addrValue) * 2 - 1; x >= 0; --x) {
			str[index] = hexDigits[(addrValue >> (x * 4)) & 0xF];
			index++;
		}
		str[index] = '\0';

		return String(str);
	}
}

#endif
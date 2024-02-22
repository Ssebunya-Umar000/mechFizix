#ifndef STRING_H
#define STRING_H

#include"stackArray.h"
#include"dynamicArray.h"

namespace mech{

#define SMALL_STRING_SIZE 24

	class String {

	private:

		StackArray<char, SMALL_STRING_SIZE> mStackData;
		DynamicArray<char, uint32> mHeapData;

	public:

		String() {}

		explicit String(const char* str)
		{
			uint32 len = 0;
			while (str[len] != '\0') {
				len++;
			}

			if (len > 0) {

				if (len + 1 < SMALL_STRING_SIZE) {
					this->mStackData.size() = len + 1;
					for (uint32 x = 0; x < len; ++x) {
						this->mStackData[x] = str[x];
					}
					this->mStackData[len] = '\0';
				}
				else {
					this->mHeapData.reserve(len + 1);
					for (uint32 x = 0; x < len; ++x) {
						this->mHeapData[x] = str[x];
					}
					this->mHeapData[len] = '\0';
				}
			}
		}

		void operator+=(const String& other)
		{
			uint32 otherLen = other.length();
			if (otherLen > 0) {

				uint32 thisLen = this->length();
				uint32 newLength = thisLen + otherLen;

				if (newLength + 1 < SMALL_STRING_SIZE) {

					this->mStackData.size() = newLength + 1;
					for (uint32 x = thisLen; x < newLength; ++x) {
						this->mStackData[x] = other.mStackData[x - thisLen];
					}
					this->mStackData[newLength] = '\0';
				}
				else {

					this->mHeapData.reserve(newLength + 1);

					if (thisLen + 1 < SMALL_STRING_SIZE) {
						for (uint32 x = 0; x < thisLen; ++x) {
							this->mHeapData[x] = this->mStackData[x];
						}
						this->mStackData.clear();
					}

					if (otherLen + 1 < SMALL_STRING_SIZE) {
						for (uint32 x = thisLen; x < newLength; ++x) {
							this->mHeapData[x] = other.mStackData[x - thisLen];
						}
					}
					else {
						for (uint32 x = thisLen; x < newLength; ++x) {
							this->mHeapData[x] = other.mHeapData[x - thisLen];
						}
					}

					this->mHeapData[newLength] = '\0';
				}
			}
		}

		String operator+(const String& other) const
		{
			String str = *this;
			str += other;
			return str;
		}

		uint32 length() const
		{
			if (this->mStackData.empty()) {
				return this->mHeapData.empty() ? 0 : this->mHeapData.size() - 1;
			}

			return this->mStackData.size() - 1;
		}

		const char* cString() const
		{
			if (this->mStackData.empty()) {
				return this->mHeapData.data();
			}

			return this->mStackData.data();
		}

		bool empty() const
		{
			return this->mStackData.empty() && this->mHeapData.empty();
		}

		void reduce(const uint32& num)
		{
			if (this->mStackData.empty()) {
				for (uint32 x = 0; x < num; ++x) {
					this->mHeapData.popBack();
				}
				this->mHeapData[this->mHeapData.size() - 1] = '\0';
			}
			else {
				for (uint32 x = 0; x < num; ++x) {
					this->mStackData.popBack();
				}
				this->mStackData[this->mStackData.size() - 1] = '\0';
			}
		}

		void clear()
		{
			if (this->mStackData.empty()) {
				this->mHeapData.deepClear();
			}
		}
	};

	//////////////////////////////////////////////////////////////////////////////////////////////////////////
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
#pragma once
#ifndef __FUNCTION_H__
#define __FUNCTION_H__
#include <io.h>
#include <string>
#include <vector>
#include <Windows.h>

using namespace std;

int getFiles(string path, vector<string>& files, string extend = "", bool is_whole = false);

void setTextRed();

void setTextGreen();

void setTextBlue();

void setTextYellow();

void setTextWhite();

void setTextColor(int16_t color);

template<typename T>
T maxNum(vector<T> vec) {
	T result;
	result = vec[0];
	for (auto& num : vec) {
		if (result < num) result = num;
	}
	return result;
}

template<typename T>
T minNum(vector<T> vec) {
	T result;
	result = vec[0];
	for (auto& num : vec) {
		if (result > num) result = num;
	}
	return result;
}
#endif

#include "function.h"

int getFiles(string path, vector<string>& files, string extend, bool is_whole) {
	_finddata_t fileInfo;
	intptr_t handle;
	string _path(path);

	if (extend == "")_path.append("\\*");
	else _path.append("\\*." + extend);

	handle = _findfirst(_path.c_str(), &fileInfo);
	if (handle != -1) {
		do {
			if (strcmp(fileInfo.name, ".") != 0 && strcmp(fileInfo.name, "..") != 0) {
				if (is_whole)files.push_back(string(path).append("\\").append(fileInfo.name));
				else files.push_back(string(fileInfo.name));
			}
		} while (_findnext(handle, &fileInfo) == 0);
		_findclose(handle);
	}
	else
	{
		return -1;
	}
	return 0;
}

void setTextRed() {
	SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), FOREGROUND_RED);
}

void setTextGreen() {
	SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), FOREGROUND_GREEN);
}

void setTextBlue() {
	SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), FOREGROUND_BLUE);
}

void setTextYellow() {
	SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), FOREGROUND_RED | FOREGROUND_GREEN);
}

void setTextWhite() {
	SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), FOREGROUND_BLUE | FOREGROUND_RED | FOREGROUND_GREEN);
}

void setTextColor(int16_t color) {
	SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), color);
}
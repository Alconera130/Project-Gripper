#pragma once

namespace gripper {

bool equalsIgnoreCase(const char *left, const char *right);
bool startsWithIgnoreCase(const char *text, const char *prefix);
char *trim(char *text);
bool parseLong(const char *text, long *value);
int parseFinger(const char *token);

}  // namespace gripper

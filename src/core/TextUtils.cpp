#include "core/TextUtils.h"

#include <ctype.h>
#include <stdlib.h>
#include <string.h>

#include "GripperConfig.h"

namespace gripper {

bool equalsIgnoreCase(const char *left, const char *right) {
    if (left == nullptr || right == nullptr) {
        return false;
    }
    while (*left != '\0' && *right != '\0') {
        if (tolower(static_cast<unsigned char>(*left)) !=
                tolower(static_cast<unsigned char>(*right))) {
            return false;
        }
        ++left;
        ++right;
    }
    return *left == '\0' && *right == '\0';
}

bool startsWithIgnoreCase(const char *text, const char *prefix) {
    if (text == nullptr || prefix == nullptr) {
        return false;
    }
    while (*prefix != '\0') {
        if (*text == '\0' ||
                tolower(static_cast<unsigned char>(*text)) !=
                        tolower(static_cast<unsigned char>(*prefix))) {
            return false;
        }
        ++text;
        ++prefix;
    }
    return true;
}

char *trim(char *text) {
    if (text == nullptr) {
        return nullptr;
    }
    while (*text != '\0' && isspace(static_cast<unsigned char>(*text))) {
        ++text;
    }
    char *end = text + strlen(text);
    while (end > text && isspace(static_cast<unsigned char>(*(end - 1)))) {
        --end;
    }
    *end = '\0';
    return text;
}

bool parseLong(const char *text, long *value) {
    if (text == nullptr || value == nullptr || *text == '\0') {
        return false;
    }
    char *end = nullptr;
    const long parsed = strtol(text, &end, 10);
    if (end == text || *end != '\0') {
        return false;
    }
    *value = parsed;
    return true;
}

int parseFinger(const char *token) {
    if (token == nullptr) {
        return -1;
    }
    long numeric = 0;
    if (parseLong(token, &numeric)) {
        return numeric >= 1 && numeric <= kFingerCount
                              ? static_cast<int>(numeric - 1)
                              : -1;
    }
    for (uint8_t finger = 0; finger < kFingerCount; ++finger) {
        if (equalsIgnoreCase(token, kFingerNames[finger])) {
            return finger;
        }
    }
    return equalsIgnoreCase(token, "little") ? 4 : -1;
}

}  // namespace gripper

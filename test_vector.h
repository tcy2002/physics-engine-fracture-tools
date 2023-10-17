#pragma once

#include <iostream>
#include <vector>

template <typename T>
ATTRIBUTE_ALIGNED16(class) test_vector {
private:
    std::vector<T> list;

public:
    T& operator[](uint32_t idx) {
        return list[idx];
    }

    auto begin() const {
        return list.cbegin();
    }

    auto end() const {
        return list.cend();
    }

    uint32_t size() const {
        return list.size();
    }

    bool empty() const {
        return list.empty();
    }

    bool contains(const T& item) {
        bool found = false;
        for (auto& i : list) {
            if (i == item) {
                found = true;
                break;
            }
        }
        return found;
    }

    uint32_t index_of(const T& item) {
        uint32_t size = list.size();
        for (uint32_t i = 0; i < size; i++) {
            if (list[i] == item) {
                return i;
            }
        }
        return NOT_FOUND;
    }

    bool push_back(const T& item) {
        if (contains(item)) {
            return false;
        }
        list.push_back(item);
        return true;
    }

    void pop_back() {
        if (list.empty()) {
            return;
        }
        list.pop_back();
    }

    bool insert(uint32_t idx, const T& item) {
        if (idx > size()) {
            return false;
        }
        if (contains(item)) {
            return false;
        }
        list.insert(list.begin() + idx, item);
        return true;
    }

    bool append(const std::vector<T>& items) {
        bool success = true;
        for (auto& item : items) {
            success &= push_back(item);
        }
        return success;
    }

    void erase(uint32_t idx) {
        if (idx >= size()) {
            return;
        }
        if (idx == size() - 1) {
            pop_back();
            return;
        }
        list.erase(list.begin() + idx);
    }

    bool remove(const T& item) {
        uint32_t idx = index_of(item);
        if (idx == NOT_FOUND) {
            return false;
        }
        erase(idx);
        return true;
    }

    bool replace(uint32_t idx, const T& item) {
        if (idx >= size() || contains(item)) {
            return false;
        }
        list[idx] = item;
        return true;
    }

    bool replace(const T& from, const T& to) {
        uint32_t idx = index_of(from);
        if (idx == NOT_FOUND || contains(to)) {
            return false;
        }
        list[idx] = to;
        return true;
    }

    void clear() {
        list.clear();
    }
};
#pragma once

#include "general.h"
#include <iostream>
#include <vector>
#include <cstdint>

#define NOT_FOUND UINT32_MAX

// example: hash function for std::string
uint32_t hash_func(const std::string& str) {
    uint32_t hash = 0;
    for (auto c : str) {
        hash = hash * 131 + c;
    }
    return hash;
}

// example: equal function for std::string
bool equal(const std::string& a, const std::string& b) {
    return a == b;
}

/**
 * @brief A hash vector, similar usage to std::vector, but
 * can provide high efficiency on sequential write and random
 * search.
 *
 * Capacity and hash function is user-given.
 *
 * Must register the hash function of custom type like:
 * uint32_t hash_func(const T& item) {}
 * and the equal function like:
 * bool equal(const T& a, const T& b) {}
 * reason: some '==' operator has already been overloaded, and
 * custom type may use other strategy to compare equality, e.g.
 * Vector3
 */
template <typename T>
ATTRIBUTE_ALIGNED16(class) hash_vector {
private:
    struct link_node {
        uint32_t val;
        link_node* next;
        link_node(): val(NOT_FOUND), next(nullptr) {}
        explicit link_node(uint32_t val, link_node* next=nullptr): val(val), next(next) {}
    };

    std::vector<T> list;
    std::vector<link_node*> index_table;
    uint32_t cap;

    uint32_t hash(const T& item) {
        return hash_func(item) % cap;
    }

public:
    explicit hash_vector(uint32_t capacity): cap(capacity) {
        index_table.resize(capacity);
        for (auto& p : index_table) {
            p = new link_node();
        }
    }

    hash_vector(const hash_vector<T>& other): cap(other.cap) {
        *this = other;
    }

    hash_vector& operator=(const hash_vector<T>& other) {
        list = other.list;
        cap = other.cap;
        for (auto p : index_table) {
            while (p != nullptr) {
                auto tmp = p;
                p = p->next;
                delete tmp;
            }
        }
        index_table.resize(cap);
        for (auto& p : index_table) {
            p = new link_node();
        }
        uint32_t size = list.size();
        for (uint32_t i = 0; i < size; i++) {
            uint32_t idx = hash(list[i]);
            index_table[idx]->next = new link_node(i, index_table[idx]->next);
        }
        return *this;
    }

    ~hash_vector() {
        for (auto p : index_table) {
            while (p != nullptr) {
                auto tmp = p;
                p = p->next;
                delete tmp;
            }
        }
    }

    /**
     * @brief not safe: you should not modify the essential data
     * that affect hash value, otherwise consider using replace.
     */
    T& operator[](uint32_t idx) {
        return list[idx];
    }

    /**
     * @brief not safe: you should not modify the essential data
     * that affect hash value, otherwise consider using replace.
     */
    T& back() {
        return list.back();
    }

    auto begin() {
        return list.cbegin();
    }

    auto end() {
        return list.cend();
    }

    uint32_t size() const {
        return list.size();
    }

    uint32_t capacity() const {
        return capacity;
    }

    bool empty() const {
        return list.empty();
    }

    std::vector<T> to_vector() const {
        return list;
    }

    bool contains(const T& item) {
        auto p = index_table[hash(item)];
        while (p->next != nullptr) {
            p = p->next;
            if (equal(list[p->val], item)) {
                return true;
            }
        }
        return false;
    }

    uint32_t index_of(const T& item) {
        auto p = index_table[hash(item)];
        while (p->next != nullptr) {
            p = p->next;
            if (equal(list[p->val], item)) {
                return p->val;
            }
        }
        return NOT_FOUND;
    }

    bool push_back(const T& item) {
        auto p = index_table[hash(item)];
        while (p->next != nullptr) {
            p = p->next;
            if (equal(list[p->val], item)) {
                return false;
            }
        }
        p->next = new link_node(size());
        list.push_back(item);
        return true;
    }

    void pop_back() {
        if (list.empty()) {
            return;
        }
        size_t idx = size() - 1;
        auto p = index_table[hash(list[idx])];
        while (p->next != nullptr && p->next->val != idx) {
            p = p->next;
        }
        auto tmp = p->next;
        p->next = p->next->next;
        delete tmp;
        list.pop_back();
    }

    bool insert(uint32_t idx, const T& item) {
        if (idx > size()) {
            return false;
        }
        auto p = index_table[hash(item)];
        while (p->next != nullptr) {
            p = p->next;
            if (equal(list[p->val], item)) {
                return false;
            }
        }
        if (idx == size()) {
            p->next = new link_node(idx);
            list.push_back(item);
            return true;
        }
        for (auto q : index_table) {
            while (q->next != nullptr) {
                q = q->next;
                if (q->val >= idx) {
                    q->val++;
                }
            }
        }
        p->next = new link_node(idx);
        list.insert(list.begin() + idx, item);
        return true;
    }

    bool append(const std::vector<T>& other) {
        bool flag = true;
        for (auto& item : other) {
            flag &= push_back(item);
        }
        return flag;
    }

    bool append(const hash_vector<T>& items) {
        return append(items.to_vector());
    }

    void erase(uint32_t idx) {
        if (idx >= size()) {
            return;
        }
        if (idx == size() - 1) {
            pop_back();
            return;
        }
        for (auto p : index_table) {
            while (p->next != nullptr) {
                if (p->next->val == idx) {
                    auto tmp = p->next;
                    p->next = p->next->next;
                    delete tmp;
                    if (p->next == nullptr) {
                        break;
                    }
                }
                p = p->next;
                if (p->val > idx) {
                    p->val--;
                }
            }
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
        if (idx >= size()) {
            return false;
        }
        auto p = index_table[hash(item)];
        while (p->next != nullptr) {
            p = p->next;
            if (equal(list[p->val], item)) {
                return false;
            }
        }
        p->next = new link_node(idx);
        p = index_table[hash(list[idx])];
        while (p->next != nullptr) {
            if (p->next->val == idx) {
                auto tmp = p->next;
                p->next = p->next->next;
                delete tmp;
                break;
            }
            p = p->next;
        }
        list[idx] = item;
        return true;
    }

    bool replace(const T& from, const T& to) {
        uint32_t idx = index_of(from);
        if (idx == NOT_FOUND) {
            return false;
        }
        return replace(idx, to);
    }

    void clear() {
        list.clear();
        for (auto& p : index_table) {
            auto q = p->next;
            while (q != nullptr) {
                auto tmp = q;
                q = q->next;
                delete tmp;
            }
            p->next = nullptr;
        }
    }
};

//
// Created by liyutong on 2024/6/11.
//

#ifndef OPTITRACK_BRIDGE_RINGBUF_H
#define OPTITRACK_BRIDGE_RINGBUF_H

#include <iostream>
#include <mutex>

typedef enum {
    RingBufferErr_OK,
    RingBufferErr_FAIL,
} RingBufferErr;

template<typename T>
class RingBuffer {
public:
    explicit RingBuffer(size_t size = 1024) : size(size), head(1), counter(1), mutex(), buf(size) {}

    ~RingBuffer() = default;

    RingBufferErr Reset() {
        std::lock_guard<std::mutex> lock(mutex);
        counter = 1;
        head = 1;
        return RingBufferErr_OK;
    }

    RingBufferErr Push(T &item) {
        std::lock_guard<std::mutex> lock(mutex);
        buf[head] = item;
        head = (head + 1) % size;
        counter++;
        return RingBufferErr_OK;
    }

    std::tuple<std::unique_ptr<T>, int64_t, RingBufferErr> Peek(int64_t index) {
        if (index >= counter) return std::make_tuple(nullptr, -1, RingBufferErr_FAIL);
        if (index < 0 || index <= (counter - size)) index = counter - 1;
        int64_t place = index % size;
        return std::make_tuple(std::make_unique<T>(buf[place]), index, RingBufferErr_OK);
    }

    size_t GetSize() { return size; }

    size_t GetCounter() { return counter; }

private:
    size_t size;
    std::vector<T> buf;
    size_t head;
    size_t counter;
    std::mutex mutex;
};


#endif //OPTITRACK_BRIDGE_RINGBUF_H

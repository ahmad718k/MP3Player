#include "MP3BitReader.h"

MP3BitReader::MP3BitReader(const uint8_t *data, size_t size)
    : data_(data), size_(size), bitpos_(0) {}

bool MP3BitReader::canRead(unsigned n) const {
    return bitpos_ + n <= size_ * 8u;
}

uint32_t MP3BitReader::read(unsigned n) {
    if (n == 0) return 0;
    if (n > 32 || !canRead(n)) return 0;

    uint32_t v = 0;
    for (unsigned i = 0; i < n; ++i) {
        size_t p = bitpos_++;
        v = (v << 1) | ((data_[p >> 3] >> (7 - (p & 7))) & 1u);
    }
    return v;
}

uint32_t MP3BitReader::peek(unsigned n) {
    size_t old = bitpos_;
    uint32_t v = read(n);
    bitpos_ = old;
    return v;
}

void MP3BitReader::skip(unsigned n) {
    if (canRead(n)) bitpos_ += n;
    else bitpos_ = size_ * 8u;
}

size_t MP3BitReader::bitsRead() const { return bitpos_; }
size_t MP3BitReader::bitsLeft() const { return size_ * 8u - bitpos_; }

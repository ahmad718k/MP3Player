#include "MP3BitReader.h"

MP3BitReader::MP3BitReader(const uint8_t *data, size_t size)
    : data_(data), size_(size), bitpos_(0), limitBits_(size * 8u) {}

MP3BitReader::MP3BitReader(const uint8_t *data, size_t size, size_t startBit, size_t bitLimit)
    : data_(data), size_(size), bitpos_(startBit), limitBits_(startBit + bitLimit)
{
    if (limitBits_ > size_ * 8u) limitBits_ = size_ * 8u;
    if (bitpos_ > limitBits_) bitpos_ = limitBits_;
}

void MP3BitReader::setBitLimit(size_t bitLimit) {
    size_t lim = bitpos_ + bitLimit;
    if (lim > size_ * 8u) lim = size_ * 8u;
    limitBits_ = lim;
    if (bitpos_ > limitBits_) bitpos_ = limitBits_;
}

size_t MP3BitReader::bitLimit() const { return limitBits_; }

bool MP3BitReader::canRead(unsigned n) const {
    return bitpos_ + n <= limitBits_;
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
    else bitpos_ = limitBits_;
}

size_t MP3BitReader::bitsRead() const { return bitpos_; }
size_t MP3BitReader::bitsLeft() const { return limitBits_ - bitpos_; }

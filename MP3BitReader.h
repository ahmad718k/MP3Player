#ifndef MP3_BIT_READER_H
#define MP3_BIT_READER_H

#include <stdint.h>
#include <stddef.h>

class MP3BitReader {
public:
    MP3BitReader(const uint8_t *data, size_t size);
    MP3BitReader(const uint8_t *data, size_t size, size_t startBit, size_t bitLimit);
    void setBitLimit(size_t bitLimit);
    size_t bitLimit() const;

    uint32_t read(unsigned n);
    uint32_t peek(unsigned n);
    void skip(unsigned n);

    size_t bitsRead() const;
    size_t bitsLeft() const;
    bool canRead(unsigned n) const;

private:
    const uint8_t *data_;
    size_t size_;
    size_t bitpos_;
    size_t limitBits_;
};

#endif

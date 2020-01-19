#ifndef TYPES_H
#define TYPES_H

#include <cstdint>
#include <cstddef>

typedef unsigned int uint;
typedef unsigned long ulong;

typedef unsigned long long ullong;
typedef long long llong;

// typedef uint32_t size_t;
typedef int32_t ssize_t;

struct iovec {
	void *iov_base;
	size_t iov_len;
};

struct iolist {
	struct iovec *iov;
	size_t iov_len;
	
	struct iolist *next;
};

typedef struct iovec iovec_t;
typedef struct iolist iolist_t;

#endif

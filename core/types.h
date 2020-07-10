#ifndef TYPES_H
#define TYPES_H

#include <cstdint>
#include <cstddef>
#include <sys/types.h>

typedef unsigned long long ullong;
typedef long long llong;

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

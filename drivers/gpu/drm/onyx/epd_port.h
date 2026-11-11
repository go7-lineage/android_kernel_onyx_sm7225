/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef EPD_PORT_H
#define EPD_PORT_H

#ifdef __KERNEL__
#include <linux/errno.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/types.h>

static inline void *epd_zalloc(size_t n)
{
	return kvzalloc(n, GFP_KERNEL);
}

static inline void epd_free(void *p)
{
	kvfree(p);
}

#ifdef EPD_NEON
#include <asm/neon.h>
#include <asm/simd.h>
#ifdef scoped_ksimd
#define EPD_SIMD_STATE		struct user_fpsimd_state epd_simd_st __maybe_unused __uninitialized
#define epd_simd_begin()	kernel_neon_begin(&epd_simd_st)
#define epd_simd_end()		kernel_neon_end(&epd_simd_st)
#else
#define epd_simd_begin()	kernel_neon_begin()
#define epd_simd_end()		kernel_neon_end()
#endif
#endif
#else
#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;
typedef int16_t s16;

static inline void *epd_zalloc(size_t n)
{
	return calloc(1, n ? n : 1);
}

static inline void epd_free(void *p)
{
	free(p);
}
#endif

#ifndef epd_simd_begin
#define epd_simd_begin()	do { } while (0)
#define epd_simd_end()		do { } while (0)
#endif
#ifndef EPD_SIMD_STATE
#define EPD_SIMD_STATE		int epd_simd_st __attribute__((unused))
#endif

#endif

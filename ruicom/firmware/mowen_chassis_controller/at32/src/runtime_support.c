#include <stddef.h>
#include <stdint.h>

void *memcpy(void *dst, const void *src, size_t n) {
  uint8_t *d = (uint8_t *)dst;
  const uint8_t *s = (const uint8_t *)src;
  while (n-- != 0U) {
    *d++ = *s++;
  }
  return dst;
}

void *memmove(void *dst, const void *src, size_t n) {
  uint8_t *d = (uint8_t *)dst;
  const uint8_t *s = (const uint8_t *)src;
  if (d == s || n == 0U) return dst;
  if (d < s) {
    while (n-- != 0U) {
      *d++ = *s++;
    }
  } else {
    d += n;
    s += n;
    while (n-- != 0U) {
      *--d = *--s;
    }
  }
  return dst;
}

void *memset(void *dst, int value, size_t n) {
  uint8_t *d = (uint8_t *)dst;
  while (n-- != 0U) {
    *d++ = (uint8_t)value;
  }
  return dst;
}

int memcmp(const void *lhs, const void *rhs, size_t n) {
  const uint8_t *a = (const uint8_t *)lhs;
  const uint8_t *b = (const uint8_t *)rhs;
  while (n-- != 0U) {
    if (*a != *b) return (int)*a - (int)*b;
    ++a;
    ++b;
  }
  return 0;
}

void __aeabi_memcpy(void *dst, const void *src, size_t n) { (void)memcpy(dst, src, n); }
void __aeabi_memmove(void *dst, const void *src, size_t n) { (void)memmove(dst, src, n); }
void __aeabi_memset(void *dst, size_t n, int value) { (void)memset(dst, value, n); }

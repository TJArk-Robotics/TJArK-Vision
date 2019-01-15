/***************************************************
 *
 * This is the official code release of Team TJArk
 * in RoboCup SPL.
 *
 * Licensed under the GNU General Public License v3.0
 * @copyright TJArk, Tongji University, China
 *
 * @file  SIMD.h
 * @brief Additional functions for SIMD.
 * @date  2019-01-14
 *
 * @auther Mr. Tree <tongjilishu@163.com>
 * <a href="mailto:tongjilishu@163.com">Mr. Tree</a>
 *
 ***************************************************/
#ifndef SIMD_H
#define SIMD_H

#include <tmmintrin.h>

inline __m128i _mm_srli_epi8(__m128i a, int bits)
{
  __m128i u = _mm_unpacklo_epi8(a, _mm_setzero_si128());
  __m128i v = _mm_unpackhi_epi8(a, _mm_setzero_si128());
  u = _mm_srli_epi16(u, bits);
  v = _mm_srli_epi16(v, bits);
  return _mm_packus_epi16(u, v);
}

inline __m128i _mm_slli_epi8(__m128i a, int bits)
{
  __m128i u = _mm_unpacklo_epi8(a, _mm_setzero_si128());
  __m128i v = _mm_unpackhi_epi8(a, _mm_setzero_si128());
  u = _mm_slli_epi16(u, bits);
  v = _mm_slli_epi16(v, bits);
  return _mm_packus_epi16(u, v);
}

#endif

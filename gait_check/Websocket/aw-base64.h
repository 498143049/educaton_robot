
/*
 Copyright (c) 2014 Malte Hildingsson, malte (at) afterwi.se
 
 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:
 
 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.
 
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE SOFTWARE.
 */

#ifndef AW_BASE64_H
#define AW_BASE64_H

#include <stddef.h>
#ifdef __AVR__
    #include <avr/pgmspace.h>
#else
    #define PROGMEM
#endif

#ifdef __cplusplus
extern "C" {
#endif
    
    static inline size_t base64len(size_t n) {
        return (n + 2) / 3 * 4;
    }
    
    static size_t base64(char *dst, size_t dst_len, const unsigned char *src, size_t src_len)
{
static const char *b64 = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
int i, j, a, b, c;

if (base64len(src_len) > dst_len) return 0;
for (i = j = 0; i < src_len; i += 3)
{
a = src[i];
b = i + 1 >= src_len ? 0 : src[i + 1];
c = i + 2 >= src_len ? 0 : src[i + 2];

dst[j++] = b64[a >> 2];
dst[j++] = b64[((a & 3) << 4) | (b >> 4)];
if (i + 1 < src_len) { dst[j++] = b64[(b & 15) << 2 | (c >> 6)]; }
if (i + 2 < src_len) { dst[j++] = b64[c & 63]; }
}
while (j % 4 != 0) { dst[j++] = '='; }
dst[j++] = '\0';
return j;
}
    
#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* AW_BASE64_H */


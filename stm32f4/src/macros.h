/*
   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify it under the
   terms of the GNU General Public License as published by the Free Software
   Foundation, either version 3 of the License, or (at your option) any later
   version.

   Hackflight is distributed in the hope that it will be useful, but WITHOUT ANY
   WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
   PARTICULAR PURPOSE. See the GNU General Public License for more details.

   You should have received a copy of the GNU General Public License along with
   Hackflight. If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

#define NOOP do {} while (0)

#define ARRAYLEN(x) (sizeof(x) / sizeof((x)[0]))

#define CONST_CAST(type, value) ((type)(value))

#define CONCAT_HELPER(x,y) x ## y
#define CONCAT(x,y) CONCAT_HELPER(x, y)
#define CONCAT2(_1,_2) CONCAT(_1, _2)
#define CONCAT3(_1,_2,_3)  CONCAT(CONCAT(_1, _2), _3)
#define CONCAT4(_1,_2,_3,_4)  CONCAT(CONCAT3(_1, _2, _3), _4)

#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

#define EXPAND_I(x) x
#define EXPAND(x) EXPAND_I(x)

// expand to t if bit is 1, f when bit is 0. Other bit values are not supported
#define PP_IIF(bit, t, f) PP_IIF_I(bit, t, f)
#define PP_IIF_I(bit, t, f) PP_IIF_ ## bit(t, f)
#define PP_IIF_0(t, f) f
#define PP_IIF_1(t, f) t

// Expand all arguments and call macro with them. When expansion of some argument contains ',', it will be passed as multiple arguments
// #define TAKE3(_1,_2,_3) CONCAT3(_1,_2,_3)
// #define MULTI2 A,B
// PP_CALL(TAKE3, MULTI2, C) expands to ABC
#define PP_CALL(macro, ...) macro(__VA_ARGS__)

#define UNUSED(x) (void)(x) // Variables and parameters that are not used

#define DISCARD(x) (void)(x) // To explicitly ignore result of x (usually an I/O register access).

#define STATIC_ASSERT(condition, name) _Static_assert((condition), #name)


#define BIT(x) (1 << (x))

/*
http://resnet.uoregon.edu/~gurney_j/jmpc/bitwise.html
 */
#define BITCOUNT(x) (((BX_(x)+(BX_(x)>>4)) & 0x0F0F0F0F) % 255)
#define BX_(x) ((x) - (((x)>>1)&0x77777777) - (((x)>>2)&0x33333333) - (((x)>>3)&0x11111111))


// non ISO variant from linux kernel; checks ptr type, but triggers 'ISO C forbids braced-groups within expressions [-Wpedantic]'
//  __extension__ is here to disable this warning
#define CONTAINER_OF(ptr, type, member)  ( __extension__ ({     \
            const typeof( ((type *)0)->member ) *__mptr = (ptr);    \
            (type *)( (char *)__mptr - offsetof(type,member) );}))

static inline int16_t cmp16(uint16_t a, uint16_t b) { return (int16_t)(a-b); }

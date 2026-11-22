/* Go's pure-Go math.Atan, Atan2, Sin and Cos (Cephes-derived), ported so the
 * C bot rounds exactly like the Go bot: glibc differs in the last bit, and in
 * a deterministic engine one bit of aim eventually changes a whole match.
 * Source: Go 1.26 src/math/{atan,atan2,sin}.go.
 * Copyright 2009 The Go Authors. All rights reserved. BSD-style license:
 * https://go.dev/LICENSE. Build with -ffp-contract=off (Go never fuses on amd64).
 */
#include <math.h>
#include <stdint.h>

#include "bot.h"

#define GO_PI 3.14159265358979323846264338327950288419716939937510582097494459

static double xatan(double x) {
    const double P0 = -8.750608600031904122785e-01, P1 = -1.615753718733365076637e+01,
                 P2 = -7.500855792314704667340e+01, P3 = -1.228866684490136173410e+02,
                 P4 = -6.485021904942025371773e+01, Q0 = +2.485846490142306297962e+01,
                 Q1 = +1.650270098316988542046e+02, Q2 = +4.328810604912902668951e+02,
                 Q3 = +4.853903996359136964868e+02, Q4 = +1.945506571482613964425e+02;
    double z = x * x;
    z = z * ((((P0 * z + P1) * z + P2) * z + P3) * z + P4) / (((((z + Q0) * z + Q1) * z + Q2) * z + Q3) * z + Q4);
    z = x * z + x;
    return z;
}

static double satan(double x) {
    const double Morebits = 6.123233995736765886130e-17, Tan3pio8 = 2.41421356237309504880;
    if (x <= 0.66) return xatan(x);
    if (x > Tan3pio8) return GO_PI / 2 - xatan(1 / x) + Morebits;
    return GO_PI / 4 + xatan((x - 1) / (x + 1)) + 0.5 * Morebits;
}

double go_atan(double x) {
    if (x == 0) return x;
    if (x > 0) return satan(x);
    return -satan(-x);
}

double go_atan2(double y, double x) {
    if (isnan(y) || isnan(x)) return NAN;
    if (y == 0) {
        if (x >= 0 && !signbit(x)) return copysign(0, y);
        return copysign(GO_PI, y);
    }
    if (x == 0) return copysign(GO_PI / 2, y);
    if (isinf(x)) {
        if (x > 0) return isinf(y) ? copysign(GO_PI / 4, y) : copysign(0, y);
        return isinf(y) ? copysign(3 * GO_PI / 4, y) : copysign(GO_PI, y);
    }
    if (isinf(y)) return copysign(GO_PI / 2, y);
    double q = go_atan(y / x);
    if (x < 0) {
        if (q <= 0) return q + GO_PI;
        return q - GO_PI;
    }
    return q;
}

static const double SIN_C[6] = {1.58962301576546568060e-10, -2.50507477628578072866e-8, 2.75573136213857245213e-6,
                                -1.98412698295895385996e-4, 8.33333333332211858878e-3, -1.66666666666666307295e-1};
static const double COS_C[6] = {-1.13585365213876817300e-11, 2.08757008419747316778e-9, -2.75573141792967388112e-7,
                                2.48015872888517045348e-5, -1.38888888888730564116e-3, 4.16666666666665929218e-2};
#define PI4A 7.85398125648498535156e-1
#define PI4B 3.77489470793079817668e-8
#define PI4C 2.69515142907905952645e-15
#define REDUCE_THRESHOLD (double)(1 << 29)

double go_cos(double x) {
    if (isnan(x) || isinf(x)) return NAN;
    if (fabs(x) >= REDUCE_THRESHOLD) return cos(x);   /* Go uses Payne-Hanek here; never reached by the bot */
    int sign = 0;
    x = fabs(x);
    uint64_t j = (uint64_t)(x * (4 / GO_PI));
    double y = (double)j;
    if (j & 1) { j++; y++; }
    j &= 7;
    double z = ((x - y * PI4A) - y * PI4B) - y * PI4C;
    if (j > 3) { j -= 4; sign = !sign; }
    if (j > 1) sign = !sign;
    double zz = z * z;
    if (j == 1 || j == 2)
        y = z + z * zz * ((((((SIN_C[0] * zz) + SIN_C[1]) * zz + SIN_C[2]) * zz + SIN_C[3]) * zz + SIN_C[4]) * zz + SIN_C[5]);
    else
        y = 1.0 - 0.5 * zz + zz * zz * ((((((COS_C[0] * zz) + COS_C[1]) * zz + COS_C[2]) * zz + COS_C[3]) * zz + COS_C[4]) * zz + COS_C[5]);
    if (sign) y = -y;
    return y;
}

double go_sin(double x) {
    if (x == 0 || isnan(x)) return x;
    if (isinf(x)) return NAN;
    if (fabs(x) >= REDUCE_THRESHOLD) return sin(x);
    int sign = 0;
    if (x < 0) { x = -x; sign = 1; }
    uint64_t j = (uint64_t)(x * (4 / GO_PI));
    double y = (double)j;
    if (j & 1) { j++; y++; }
    j &= 7;
    double z = ((x - y * PI4A) - y * PI4B) - y * PI4C;
    if (j > 3) { sign = !sign; j -= 4; }
    double zz = z * z;
    if (j == 1 || j == 2)
        y = 1.0 - 0.5 * zz + zz * zz * ((((((COS_C[0] * zz) + COS_C[1]) * zz + COS_C[2]) * zz + COS_C[3]) * zz + COS_C[4]) * zz + COS_C[5]);
    else
        y = z + z * zz * ((((((SIN_C[0] * zz) + SIN_C[1]) * zz + SIN_C[2]) * zz + SIN_C[3]) * zz + SIN_C[4]) * zz + SIN_C[5]);
    if (sign) y = -y;
    return y;
}

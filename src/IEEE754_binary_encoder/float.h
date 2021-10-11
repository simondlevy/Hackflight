#ifndef IEE754_FLOAT_H
#define IEE754_FLOAT_H

void IEE754_binary64_encode( double x, char out[8] );
double IEE754_binary64_decode( char out[8] );
void IEE754_binary32_encode( float x, char out[4] );
float IEE754_binary32_decode( char out[4] );

#endif
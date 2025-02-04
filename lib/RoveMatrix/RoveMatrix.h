#ifndef ROVE_MATRIX_H
#define ROVE_MATRIX_H

#include <math.h>

typedef struct Vector {
    float x;
    float y;
    float z;
} Vector;

typedef struct TransfMatrix {
    float m0, m4, m8, m12;  // Matrix first row (4 components)
    float m1, m5, m9, m13;  // Matrix second row (4 components)
    float m2, m6, m10, m14; // Matrix third row (4 components)
    float m3, m7, m11, m15; // Matrix fourth row (4 components)
} TransfMatrix;

TransfMatrix Rotate(float x, float y, float z);
TransfMatrix Translate(float x, float y, float z);
TransfMatrix operator * (const TransfMatrix& left, const TransfMatrix& right);
Vector operator * (Vector v, TransfMatrix mat);
void operator *= (Vector &v, float n);

#endif /*ROVE_MATRIX_H*/
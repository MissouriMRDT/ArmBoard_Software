#include "RoveMatrix.h"

TransfMatrix TransfMatrix::getRotation() const {
    return {
        m00, m01, m02, 0,
        m10, m11, m12, 0,
        m20, m21, m22, 0,
        //0, 0, 0, 1
    };
}

Vector TransfMatrix::getTranslation() const {
    return { m03, m13, m23 };
}

TransfMatrix Transpose(const TransfMatrix& mat) {
    return {
        mat.m00, mat.m10, mat.m20, 0,
        mat.m01, mat.m11, mat.m21, 0,
        mat.m02, mat.m12, mat.m22, 0,
        // 0, 0, 0, 1
    };
}

TransfMatrix Identity(float scale) {
    return {
        scale, 0, 0, 0,
        0, scale, 0, 0,
        0, 0, scale, 0,
        // 0, 0, 0, scale
    };
}

TransfMatrix Rotation(float x, float y, float z) 
{
    TransfMatrix result = Identity();

    float cosz = cosf(-z);
    float sinz = sinf(-z);
    float cosy = cosf(-y);
    float siny = sinf(-y);
    float cosx = cosf(-x);
    float sinx = sinf(-x);

    result.m00 = cosz*cosy;
    result.m10 = (cosz*siny*sinx) - (sinz*cosx);
    result.m20 = (cosz*siny*cosx) + (sinz*sinx);
    result.m01 = sinz*cosy;
    result.m11 = (sinz*siny*sinx) + (cosz*cosx);
    result.m21 = (sinz*siny*cosx) - (cosz*sinx);
    result.m02 = -siny;
    result.m12 = cosy*sinx;
    result.m22 = cosy*cosx;

    return result;
}

TransfMatrix Translation(float x, float y, float z) 
{
    TransfMatrix result = Identity();
    result.m03 = x;
    result.m13 = y;
    result.m23 = z;
    return result;
}

TransfMatrix operator * (const TransfMatrix& left, const TransfMatrix& right) 
{
    TransfMatrix result = { 0 };

    // well, if it ain't broke don't fix it
    result.m00 = left.m00*right.m00 + left.m01*right.m10 + left.m02*right.m20 + left.m03*0;
    result.m01 = left.m00*right.m01 + left.m01*right.m11 + left.m02*right.m21 + left.m03*0;
    result.m02 = left.m00*right.m02 + left.m01*right.m12 + left.m02*right.m22 + left.m03*0;
    result.m03 = left.m00*right.m03 + left.m01*right.m13 + left.m02*right.m23 + left.m03*1;
    result.m10 = left.m10*right.m00 + left.m11*right.m10 + left.m12*right.m20 + left.m13*0;
    result.m11 = left.m10*right.m01 + left.m11*right.m11 + left.m12*right.m21 + left.m13*0;
    result.m12 = left.m10*right.m02 + left.m11*right.m12 + left.m12*right.m22 + left.m13*0;
    result.m13 = left.m10*right.m03 + left.m11*right.m13 + left.m12*right.m23 + left.m13*1;
    result.m20 = left.m20*right.m00 + left.m21*right.m10 + left.m22*right.m20 + left.m23*0;
    result.m21 = left.m20*right.m01 + left.m21*right.m11 + left.m22*right.m21 + left.m23*0;
    result.m22 = left.m20*right.m02 + left.m21*right.m12 + left.m22*right.m22 + left.m23*0;
    result.m23 = left.m20*right.m03 + left.m21*right.m13 + left.m22*right.m23 + left.m23*1;

    return result;
}

Vector operator * (const TransfMatrix& mat, const Vector& v)
{
    Vector result = { 0, 0, 0 };

    float x = v.x;
    float y = v.y;
    float z = v.z;

    result.x = mat.m00*x + mat.m01*y + mat.m02*z + mat.m03;
    result.y = mat.m10*x + mat.m11*y + mat.m12*z + mat.m13;
    result.z = mat.m20*x + mat.m21*y + mat.m22*z + mat.m23;

    return result;
}

Vector operator * (float n, const Vector& v) {
    return {
        n * v.x,
        n * v.y,
        n * v.z
    };
}
Vector operator * (const Vector& v, float n) {
    return {
        v.x * n,
        v.y * n,
        v.z * n
    };
}

void operator *= (Vector &v, float n) {
    v.x *= n;
    v.y *= n;
    v.z *= n;
}

Vector operator + (const Vector &v1, const Vector &v2) {
    return {v1.x + v2.x, v1.y + v2.y, v1.z + v2.z};
}

Vector operator - (const Vector &v1, const Vector &v2) {
    return {v1.x - v2.x, v1.y - v2.y, v1.z - v2.z};
}

Vector operator-(const Vector& v) {
    return {-v.x, -v.y, -v.z};
}

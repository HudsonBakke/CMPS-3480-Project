#ifndef MATH_H
#define MATH_H

struct Vec3 {
    float x;
    float y;
    float z;

    Vec3();
    Vec3(float x, float y, float z);

    Vec3 Cross(const Vec3& other) const;
    float Dot(const Vec3& other) const;
    float LengthSquared() const;
    float Length() const;
    void Normalize();
    Vec3 Normalized() const;

    Vec3 operator+(const Vec3& other) const;
    Vec3 operator-(const Vec3& other) const;
    Vec3 operator*(float scalar) const;
};

Vec3 operator*(float scalar, const Vec3& vector);

// Analytic two-link IK. Unreachable targets are clamped without stretching.
void SolveTwoBoneIK(const Vec3& root, const Vec3& target,
                    const Vec3& bendDirection, float upperLength, float lowerLength,
                    Vec3& joint, Vec3& end);

#endif


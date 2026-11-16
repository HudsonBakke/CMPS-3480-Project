#include <cmath>
#include <algorithm>

#include "Math.h"

//
// Vec3 implementation
//

Vec3::Vec3()
{
    x = y = z = 0;
}

Vec3::Vec3(float x, float y, float z)
    : x(x), y(y), z(z)
{ }

Vec3 Vec3::Cross(const Vec3& other) const
{
    Vec3 result;
    result.x = (this->y * other.z) - (this->z * other.y);
    result.y = (this->z * other.x) - (this->x * other.z);
    result.z = (this->x * other.y) - (this->y * other.x);
    return result;
}

float Vec3::Dot(const Vec3& other) const
{
    return
        (this->x * other.x) +
        (this->y * other.y) +
        (this->z * other.z);
}

float Vec3::LengthSquared() const
{
    return
        (this->x * this->x) +
        (this->y * this->y) +
        (this->z * this->z);
}

float Vec3::Length() const
{
    return std::sqrt(this->LengthSquared());
}

Vec3 Vec3::Normalized() const
{
    Vec3 result = *this;
    result.Normalize();
    return result;
}

void Vec3::Normalize()
{
    float length = Length();
    if (length > 0.0f) {
        x /= length;
        y /= length;
        z /= length;
    }
}

Vec3 Vec3::operator+(const Vec3& other) const
{
    Vec3 result;
    result.x = this->x + other.x;
    result.y = this->y + other.y;
    result.z = this->z + other.z;
    return result;
}

Vec3 Vec3::operator-(const Vec3& other) const
{
    Vec3 result;
    result.x = this->x - other.x;
    result.y = this->y - other.y;
    result.z = this->z - other.z;
    return result;
}

Vec3 Vec3::operator*(float scalar) const
{
    Vec3 result;
    result.x = this->x * scalar;
    result.y = this->y * scalar;
    result.z = this->z * scalar;
    return result;
}

Vec3 operator*(float scalar, const Vec3& vector)
{
    Vec3 result;
    result.x = scalar * vector.x;
    result.y = scalar * vector.y;
    result.z = scalar * vector.z;
    return result;
}


void SolveTwoBoneIK(const Vec3& root, const Vec3& target,
                    const Vec3& bendDirection, float upperLength, float lowerLength,
                    Vec3& joint, Vec3& end)
{
    Vec3 delta = target - root;
    float distance = delta.Length();
    Vec3 axis = distance > 0.00001f ? delta * (1.0f / distance) : Vec3(0, -1, 0);
    distance = std::clamp(distance,
        std::abs(upperLength - lowerLength) + 0.0001f,
        upperLength + lowerLength - 0.0001f);
    Vec3 bend = bendDirection - axis * bendDirection.Dot(axis);
    if (bend.LengthSquared() < 0.00001f) {
        Vec3 fallback = std::abs(axis.z) < 0.9f ? Vec3(0, 0, -1) : Vec3(1, 0, 0);
        bend = fallback - axis * fallback.Dot(axis);
    }
    bend.Normalize();
    float along = (upperLength * upperLength - lowerLength * lowerLength +
                   distance * distance) / (2.0f * distance);
    float height = std::sqrt(std::max(0.0f, upperLength * upperLength - along * along));
    joint = root + axis * along + bend * height;
    end = root + axis * distance;
}

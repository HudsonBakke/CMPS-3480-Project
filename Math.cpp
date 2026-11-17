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


// A two-link leg is a triangle: hip H, knee K, ankle A. Its two fixed sides
// have lengths upperLength and lowerLength. The target determines the third side.
// There are infinitely many knees on a circle around H->A; bendDirection picks
// the forward-facing point on that circle, removing the backwards-knee ambiguity.
void SolveTwoBoneIK(const Vec3& root, const Vec3& target,
                    const Vec3& bendDirection, float upperLength, float lowerLength,
                    Vec3& joint, Vec3& end)
{
    // 1. Build the triangle's baseline: axis is a unit vector from hip to ankle.
    // A coincident target has no direction, so use a deterministic downward axis.
    Vec3 delta = target - root;
    float distance = delta.Length();
    Vec3 axis = distance > 0.00001f ? delta * (1.0f / distance) : Vec3(0, -1, 0);

    // 2. Triangle inequality: |upper-lower| <= distance <= upper+lower.
    // Tiny margins avoid fully folded/straight singularities and division by 0.
    // An unreachable target is moved onto this interval; bones never stretch.
    distance = std::clamp(distance,
        std::abs(upperLength - lowerLength) + 0.0001f,
        upperLength + lowerLength - 0.0001f);

    // 3. Remove bendDirection's component along the baseline. What remains is
    // perpendicular to axis and points toward the desired side of the knee.
    // These two unit vectors span the plane in which the knee bends like a hinge.
    Vec3 bend = bendDirection - axis * bendDirection.Dot(axis);
    if (bend.LengthSquared() < 0.00001f) {
        // If the hint is parallel to the leg it cannot select a plane. Choose a
        // fallback that is not parallel, then project it in exactly the same way.
        Vec3 fallback = std::abs(axis.z) < 0.9f ? Vec3(0, 0, -1) : Vec3(1, 0, 0);
        bend = fallback - axis * fallback.Dot(axis);
    }
    bend.Normalize();

    // 4. Project the knee onto the baseline. From the law of cosines:
    // along = upper*cos(hipAngle) = (upper^2-lower^2+distance^2)/(2*distance).
    // The knee's perpendicular distance then follows from Pythagoras.
    float along = (upperLength * upperLength - lowerLength * lowerLength +
                   distance * distance) / (2.0f * distance);
    float height = std::sqrt(std::max(0.0f, upperLength * upperLength - along * along));

    // 5. Convert the two coordinates in the bending plane back to world space.
    // Returning the reached ankle as well as the knee preserves both lengths
    // even for an unreachable request, rather than hiding the error in a shin.
    joint = root + axis * along + bend * height;
    end = root + axis * distance;
}

#ifndef CHARACTER_H
#define CHARACTER_H

#include "Math.h"

struct GLUquadric;

struct CharacterPose {
    Vec3 pelvis;
    Vec3 head;
    Vec3 hips[2];
    Vec3 knees[2];
    Vec3 ankles[2];
    Vec3 shoulders[2];
    Vec3 elbows[2];
    Vec3 hands[2];
};

class Character {
public:
    Vec3 position;
    float yaw = 0.0f;

    Character() = default;
    Character(const Character&) = delete;
    Character& operator=(const Character&) = delete;

    void InitGraphics();
    void CleanupGraphics();
    void Reset();
    void Update(float dt, float distanceMoved);
    void Draw() const;

    Vec3 Forward() const;
    Vec3 Right() const;
    const CharacterPose& Pose() const;

private:
    CharacterPose pose;
    float walkPhase = 0.0f;
    float walkBlend = 0.0f;
    GLUquadric *quadric = nullptr;

    void UpdatePose();
    void DrawSphere(const Vec3& center, float radius) const;
    void DrawBone(const Vec3& start, const Vec3& end, float radius) const;
    void DrawBoot(const Vec3& ankle) const;
};

#endif

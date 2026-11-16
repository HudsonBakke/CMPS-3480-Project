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
    Vec3 footRight[2];
    Vec3 footUp[2];
    Vec3 footForward[2];
    bool footPlanted[2] = { true, true };
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
    using GroundHeight = float (*)(float x, float z);
    void Update(float dt, float distanceMoved, GroundHeight ground);
    void Draw() const;

    Vec3 Forward() const;
    Vec3 Right() const;
    const CharacterPose& Pose() const;

private:
    struct FootContact {
        Vec3 sole;
        Vec3 up = Vec3(0, 1, 0);
        float yaw = 0.0f;
    };
    struct FootState {
        FootContact contact;
        FootContact start;
        FootContact landing;
        float progress = 0.0f;
        float duration = 0.24f;
        float clearance = 0.22f;
        bool swinging = false;
    };

    CharacterPose pose;
    FootState feet[2];
    Vec3 previousPosition;
    bool feetInitialized = false;
    bool wasMoving = false;
    int activeFoot = -1;
    int nextFoot = 0;
    float pelvisY = 0.0f;
    float walkPhase = 0.0f;
    float walkBlend = 0.0f;
    GLUquadric *quadric = nullptr;

    void UpdatePose(float dt);
    void InitializeFeet(GroundHeight ground);
    void ConstrainRoot();
    void BeginStep(int index, const Vec3& velocity, GroundHeight ground);
    void AdvanceStep(float dt, GroundHeight ground);
    FootContact SampleFoot(const Vec3& location, float heading, GroundHeight ground) const;
    static void FootBasis(const FootContact& contact, Vec3& right, Vec3& forward);
    static void FitSole(FootContact& contact, GroundHeight ground);
    void DrawSphere(const Vec3& center, float radius) const;
    void DrawBone(const Vec3& start, const Vec3& end, float radius) const;
    void DrawBoot(int index) const;
};

#endif


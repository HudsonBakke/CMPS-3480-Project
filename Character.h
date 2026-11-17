#ifndef CHARACTER_H
#define CHARACTER_H

#include "Math.h"

struct GLUquadric;

// World-space joint positions and boot axes consumed by the renderer.
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
    Vec3 position; // Proposed root position, corrected for terrain/reach in Update.
    float yaw = 0.0f; // Current body heading in radians; turn through Update.

    Character() = default;
    Character(const Character&) = delete;
    Character& operator=(const Character&) = delete;

    void InitGraphics();
    void CleanupGraphics();
    void Reset();
    using GroundHeight = float (*)(float x, float z);
    // Scene supplies translation; turnRate is the requested body rotation in
    // radians/second. The feet may limit the actual rotation until a step lands.
    void Update(float dt, float distanceMoved, GroundHeight ground, float turnRate = 0.0f);
    void Draw() const;

    Vec3 Forward() const;
    Vec3 Right() const;
    const CharacterPose& Pose() const;

private:
    // A contact is a rigid sole pose. Once planted, all three fields stay fixed
    // in WORLD space, even as the pelvis moves above the support foot.
    struct FootContact {
        Vec3 sole;
        Vec3 up = Vec3(0, 1, 0);
        float yaw = 0.0f;
    };
    // At most one FootState may be swinging. The other is the world-space
    // support contact, including during pivots and movement-mode transitions.
    struct FootState {
        FootContact contact;
        FootContact start;
        FootContact landing;
        Vec3 pivotCenter;
        bool pivot = false;
        float progress = 0.0f; // Normalized swing time, from liftoff (0) to landing (1).
        float duration = 0.24f; // Seconds, independent of frame rate.
        float clearance = 0.22f; // Height of the swing arch in world units.
        bool swinging = false;
    };

    CharacterPose pose;
    FootState feet[2];
    Vec3 previousPosition;
    bool feetInitialized = false;
    bool wasMoving = false;
    bool wasPivoting = false;
    float previousTurnRate = 0.0f;
    float requestedTurnRate = 0.0f;
    int nextFoot = 0;
    float pelvisY = 0.0f;
    float walkPhase = 0.0f;
    float walkBlend = 0.0f;
    GLUquadric *quadric = nullptr;

    void UpdatePose(float dt);
    void InitializeFeet(GroundHeight ground);
    void ConstrainRoot();
    bool FeetSwinging() const;
    void TurnHips(float dt);
    float LimitLandingHeading(int index, float heading) const;
    void BeginStep(int index, const Vec3& velocity, GroundHeight ground);
    void BeginPivot(int index, GroundHeight ground);
    void AdvanceSteps(float dt, GroundHeight ground);
    FootContact SampleFoot(const Vec3& location, float heading, GroundHeight ground) const;
    static void FootBasis(const FootContact& contact, Vec3& right, Vec3& forward);
    static void FitSole(FootContact& contact, GroundHeight ground);
    void DrawSphere(const Vec3& center, float radius) const;
    void DrawBone(const Vec3& start, const Vec3& end, float radius) const;
    void DrawBoot(int index) const;
};

#endif




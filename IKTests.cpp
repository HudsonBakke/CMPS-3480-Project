#include "Character.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

float noise2(float vec[2]);

namespace {
constexpr float PI = 3.14159265358979323846f;
const char *scenario = "solver";
int frame = 0;
int checkedFrames = 0;

void Require(bool condition, const char *message)
{
    if (!condition) {
        std::fprintf(stderr, "%s, frame %d: %s\n", scenario, frame, message);
        std::exit(EXIT_FAILURE);
    }
}

bool Near(float a, float b, float tolerance = 0.0003f)
{
    return std::abs(a - b) < tolerance;
}

float Flat(float, float) { return 0.0f; }
float Ramp(float x, float z) { return 0.45f * x - 0.30f * z; }
float Hills(float x, float z)
{
    return 0.60f * std::sin(1.1f * x) + 0.45f * std::cos(0.9f * z);
}
float Step(float x, float) { return x >= 1.0f ? 0.25f : 0.0f; }
float Noise(float x, float z)
{
    float p[2] = {x * 0.045f + 37.25f, z * 0.045f + 61.75f};
    float value = noise2(p);
    p[0] *= 2; p[1] *= 2;
    value += .35f * noise2(p);
    p[0] *= 2; p[1] *= 2;
    return 10.0f * (value + .12f * noise2(p));
}
// Same triangle interpolation as the displayed mesh, including its diagonal.
float Mesh(float x, float z)
{
    float ix = std::floor(x), iz = std::floor(z);
    float fx = x - ix, fz = z - iz;
    float a = Noise(ix, iz), b = Noise(ix + 1, iz);
    float c = Noise(ix, iz + 1), d = Noise(ix + 1, iz + 1);
    return fx + fz <= 1 ? a + fx * (b - a) + fz * (c - a) :
        d + (1 - fx) * (c - d) + (1 - fz) * (b - d);
}

void CheckPose(const Character& man, const CharacterPose& previous,
               Character::GroundHeight ground)
{
    const CharacterPose& p = man.Pose();
    Require(std::isfinite(p.head.y), "nonfinite pose");
    Require(Near(man.position.y, ground(man.position.x, man.position.z)), "root height differs from terrain");
    for (int i = 0; i < 2; ++i) {
        Require(Near((p.knees[i] - p.hips[i]).Length(), .82f), "thigh stretched");
        Require(Near((p.ankles[i] - p.knees[i]).Length(), .82f), "shin stretched");
        Require(Near((p.elbows[i] - p.shoulders[i]).Length(), .46f), "upper arm stretched");
        Require(Near((p.hands[i] - p.elbows[i]).Length(), .43f), "forearm stretched");
        Require(p.footUp[i].y > 0 && Near(p.footUp[i].Length(), 1), "invalid sole normal");
        Require(Near(p.footRight[i].Dot(p.footUp[i]), 0), "invalid foot basis");
        if (previous.footPlanted[i] && p.footPlanted[i]) {
            float drift = (p.ankles[i] - previous.ankles[i]).Length();
            if (drift > .0005f)
                std::fprintf(stderr, "foot %d drift %.6f\n", i, drift);
            Require(drift < .0005f, "support foot skated");
            Require((p.footForward[i] - previous.footForward[i]).Length() < .0001f,
                    "planted foot rotated");
            Require((p.footUp[i] - previous.footUp[i]).Length() < .0001f,
                    "planted foot tilted");
        }
        // Check an offset footprint grid in addition to the runtime fitter samples.
        Vec3 sole = p.ankles[i] - p.footUp[i] * .18f;
        for (int z = 0; z <= 6; ++z) {
            for (int x = 0; x <= 6; ++x) {
                Vec3 point = sole + p.footRight[i] * (-.17f + .34f * x / 6) +
                             p.footForward[i] * (-.15f + .51f * z / 6);
                float clearance = point.y - ground(point.x, point.z);
                if (clearance < -.003f)
                    std::fprintf(stderr, "foot %d planted %d clearance %.6f at %.3f %.3f\n",
                                 i, p.footPlanted[i], clearance, point.x, point.z);
                Require(clearance >= -.003f, "boot penetrated terrain");
            }
        }
    }
    ++checkedFrames;
}

void Walk(const char *name, Character::GroundHeight ground, float dt, float speed,
          bool turn, bool stop)
{
    scenario = name;
    Character man;
    man.Reset();
    man.position.y = ground(0, 0);
    man.yaw = turn ? 0.0f : PI / 2;
    man.Update(0, 0, ground);
    int touchdowns = 0;
    float distance = 0;
    for (frame = 0; frame < int(8.0f / dt); ++frame) {
        CharacterPose before = man.Pose();
        float heading = turn ? (frame < int(2 / dt) ? 0.0f :
                               frame < int(4 / dt) ? PI :
                               frame < int(6 / dt) ? -PI / 2 : 0.8f * frame * dt) : PI / 2;
        bool moving = !stop || frame < int(4.0f / dt);
        float turnRate = moving && turn ? std::clamp(
            std::remainder(heading - man.yaw, 2 * PI) / dt, -1.6f, 1.6f) : 0.0f;
        Vec3 old = man.position;
        if (moving)
            man.position = man.position + man.Forward() * (speed * dt);
        man.position.y = ground(man.position.x, man.position.z);
        man.Update(dt, moving ? std::abs(speed) * dt : 0, ground, turnRate);
        distance += (man.position - old).Length();
        CheckPose(man, before, ground);
        for (int i=0;i<2;++i)
            if (!before.footPlanted[i] && man.Pose().footPlanted[i]) ++touchdowns;
    }
    Require(touchdowns > 4, "steps did not complete");
    Require(distance > 5.0f, "locomotion stalled");
    if (stop) {
        Require(man.Pose().footPlanted[0] && man.Pose().footPlanted[1], "did not settle after stopping");
        CharacterPose before = man.Pose();
        for(int i=0;i<100;++i) man.Update(dt, 0, ground);
        Require((man.Pose().ankles[0]-before.ankles[0]).Length()<.0001f, "idle foot drift");
    }
}

void CheckTurningJoints(const Character& man)
{
    const auto& pose = man.Pose();
    for (int i = 0; i < 2; ++i) {
        // On level ground, the boot basis directly exposes its yaw. The hips
        // should never get more than about 20 degrees ahead of either foot.
        float footYaw = std::atan2(pose.footForward[i].x, -pose.footForward[i].z);
        float twist = std::abs(std::remainder(man.yaw - footYaw, 2 * PI));
        if (twist > 0.38f) std::fprintf(stderr,"foot %d hip twist %.3f rad\n",i,twist);
        Require(twist < 0.38f, "hip/foot twist limit exceeded");
        Vec3 thigh = pose.knees[i] - pose.hips[i];
        Vec3 shin = pose.ankles[i] - pose.knees[i];
        Vec3 hinge = thigh.Cross(shin).Normalized();
        // The hinge axis stays close to boot right: knee flexion happens in
        // the foot's sagittal plane, not in a plane spun by the torso heading.
        float alignment = std::abs(hinge.Dot(pose.footRight[i]));
        if (alignment < .96f) std::fprintf(stderr,"hinge alignment %.4f\n",alignment);
        Require(alignment > .96f, "knee hinge twisted away from foot");
    }
}

void Pivot(const char *name, float direction, float dt, float initialYaw,
           Character::GroundHeight ground = Flat)
{
    scenario = name;
    Character man;
    man.Reset();
    man.yaw = initialYaw;
    man.Update(0, 0, ground);
    CharacterPose start = man.Pose();
    Vec3 forward = man.Forward();
    float accumulatedTurn = 0;
    bool oppositeArcs = false;
    int liftOffs[2] = {};
    for (frame = 0; frame < int(5 / dt); ++frame) {
        auto previous = man.Pose();
        float previousYaw = man.yaw;
        man.Update(dt, 0, ground, direction * 1.6f);
        CheckPose(man, previous, ground);
        if (ground == Flat)
            CheckTurningJoints(man);
        float change = std::remainder(man.yaw - previousYaw, 2 * PI);
        Require(std::abs(change) <= 1.6f * dt + .0001f, "body yaw snapped");
        accumulatedTurn += change;
        for (int i = 0; i < 2; ++i)
            if (previous.footPlanted[i] && !man.Pose().footPlanted[i]) ++liftOffs[i];
        float left = (man.Pose().ankles[0] - start.ankles[0]).Dot(forward);
        float right = (man.Pose().ankles[1] - start.ankles[1]).Dot(forward);
        if (!man.Pose().footPlanted[0] && !man.Pose().footPlanted[1] &&
            direction * left > .01f && direction * right < -.01f)
            oppositeArcs = true;
        Require(std::hypot(man.position.x, man.position.z) < .0001f, "in-place pivot translated the root");
    }
    Require(direction * accumulatedTurn > 3.5f, "pivot stopped making angular progress");
    Require(oppositeArcs, "pivot feet did not take opposing arcs");
    Require(liftOffs[0] > 3 && liftOffs[1] > 3, "pivot did not use both feet");
    // Reversing a turn while both feet are airborne must replan continuously.
    for (frame = 0; frame < int(2 / dt); ++frame) {
        auto previous = man.Pose();
        man.Update(dt, 0, ground, -direction * 1.6f);
        CheckPose(man, previous, ground);
        if (ground == Flat)
            CheckTurningJoints(man);
    }
    for (frame = 0; frame < int(3 / dt); ++frame) {
        auto previous = man.Pose();
        man.Update(dt, 0, ground);
        CheckPose(man, previous, ground);
    }
    Require(man.Pose().footPlanted[0] && man.Pose().footPlanted[1], "pivot did not settle");
}

void CurvedWalk(float direction, float dt)
{
    scenario = "inside/outside strides";
    Character man;
    man.Reset();
    man.Update(0, 0, Flat);
    float footTravel[2] = {};
    for (frame = 0; frame < int(8 / dt); ++frame) {
        auto previous = man.Pose();
        man.position = man.position + man.Forward() * (3.0f * dt);
        float previousYaw = man.yaw;
        man.Update(dt, 3.0f * dt, Flat, direction * 1.1f);
        Require(std::abs(std::remainder(man.yaw - previousYaw, 2 * PI)) <= 1.6f * dt + .0001f,
                "walking turn snapped");
        CheckPose(man, previous, Flat);
        CheckTurningJoints(man);
        for (int i = 0; i < 2; ++i) {
            Vec3 travel = man.Pose().ankles[i] - previous.ankles[i];
            travel.y = 0;
            footTravel[i] += travel.Length();
        }
    }
    int outside = direction > 0 ? 0 : 1;
    if (footTravel[outside] <= footTravel[1-outside] * 1.05f)
        std::fprintf(stderr,"foot paths left %.3f right %.3f\n",footTravel[0],footTravel[1]);
    Require(footTravel[outside] > footTravel[1-outside] * 1.05f,
            "outside foot did not take a longer path");
}


void MixedModes(float dt, Character::GroundHeight ground)
{
    scenario = "walking/pivot/backward transitions";
    Character man;
    man.Reset();
    man.Update(0, 0, ground);
    for (frame = 0; frame < int(10 / dt); ++frame) {
        int mode = int(frame * dt) % 5;
        float speed = mode == 0 ? 3.0f : mode == 2 ? 6.0f : mode == 3 ? -3.0f : 0.0f;
        float turn = mode == 1 || mode == 2 ? 1.6f : mode == 3 ? -1.6f : 0.0f;
        auto previous = man.Pose();
        man.position = man.position + man.Forward() * (speed * dt);
        man.Update(dt, std::abs(speed) * dt, ground, turn);
        CheckPose(man, previous, ground);
        if (ground == Flat)
            CheckTurningJoints(man);
    }
}

}

int main()
{
    for (float upper : { .82f, 1.1f }) {
        for (float lower : { .82f, .65f }) {
            for (Vec3 target : {Vec3(0,-1,0), Vec3(), Vec3(10,4,-3), Vec3(.1f,-.2f,.3f), Vec3(0,0,-1), Vec3(0,0,1)}) {
                Vec3 knee, ankle;
                SolveTwoBoneIK(Vec3(),target,Vec3(0,0,-1),upper,lower,knee,ankle);
                Require(Near(knee.Length(), upper), "IK first segment length");
                Require(Near((ankle-knee).Length(), lower), "IK second segment length");
                Require(std::isfinite(ankle.y), "IK singularity");
            }
        }
    }
    for (float dt : {1.0f/30,1.0f/60,1.0f/144}) {
        Walk("flat walking", Flat, dt, 3, false, true);
        Walk("flat fast turns", Flat, dt, 6, true, true);
        Walk("ramp up/down", Ramp, dt, 3, true, true);
        Walk("hills", Hills, dt, 3, false, true);
        Walk("hills fast turns", Hills, dt, 6, true, true);
        Walk("low step", Step, dt, 3, false, true);
        Walk("mesh", Mesh, dt, 3, true, true);
        Walk("mesh fast", Mesh, dt, 6, true, false);
        Walk("backward mesh", Mesh, dt, -3, false, true);
        Pivot("right in-place turn", 1, dt, 0);
        Pivot("left turn across angle wrap", -1, dt, -3.0f);
        Pivot("right pivot on hills", 1, dt, 0, Hills);
        Pivot("left pivot on mesh", -1, dt, 0, Mesh);
        CurvedWalk(1, dt);
        CurvedWalk(-1, dt);
        MixedModes(dt, Flat);
        MixedModes(dt, Mesh);
    }
    std::printf("Passed solver edge cases and %d frames of terrain contact, swing clearance, planted-foot locking, turns, stops, hinge alignment, opposing pivot arcs, unequal inside/outside strides and fixed limb lengths.\n", checkedFrames);
}


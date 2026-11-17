#include "Character.h"

// The Makefile defines CHARACTER_HEADLESS only for `make test`, using -D.
// That build exercises the same pose/IK code without linking a graphics system.
// Normal `make` leaves the macro undefined, so the drawing code is compiled.
#ifndef CHARACTER_HEADLESS
#include <GL/gl.h>
#include <GL/glu.h>
#endif
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace {
constexpr float PI = 3.14159265358979323846f;
constexpr float LEG_LENGTH = 0.82f;
constexpr float ANKLE_HEIGHT = 0.18f;
constexpr float HEAD_RADIUS = 0.30f;
constexpr float SOLE_CLEARANCE = 0.025f;
constexpr float MAX_LEG_REACH = 2.0f * LEG_LENGTH - 0.015f;
// Bound the torso/hip heading relative to BOTH foot headings. This prevents the
// body from rotating far over fixed feet and forcing the knees/ankles to twist.
constexpr float MAX_HIP_TWIST = 20.0f * PI / 180.0f;
constexpr float MAX_TURN_RATE = 1.6f;

float AngleDifference(float to, float from)
{
    // Shortest signed angle, including turns across the -PI/PI boundary.
    return std::remainder(to - from, 2.0f * PI);
}
}

Vec3 Character::Forward() const
{
    return Vec3(std::sin(yaw), 0.0f, -std::cos(yaw));
}

Vec3 Character::Right() const
{
    return Vec3(std::cos(yaw), 0.0f, std::sin(yaw));
}

const CharacterPose& Character::Pose() const
{
    return pose;
}

void Character::InitGraphics()
{
#ifndef CHARACTER_HEADLESS
    quadric = gluNewQuadric();
    if (!quadric) {
        std::fprintf(stderr, "Could not allocate model quadric.\n");
        std::exit(EXIT_FAILURE);
    }
    gluQuadricNormals(quadric, GLU_SMOOTH);
#endif
}

void Character::CleanupGraphics()
{
#ifndef CHARACTER_HEADLESS
    if (quadric)
        gluDeleteQuadric(quadric);
#endif
    quadric = nullptr;
}

void Character::Reset()
{
    position = Vec3();
    yaw = 0.0f;
    walkPhase = walkBlend = 0.0f;
    feetInitialized = wasMoving = wasPivoting = false;
    requestedTurnRate = previousTurnRate = 0.0f;
    nextFoot = 0;
    pose = CharacterPose();
}

void Character::FootBasis(const FootContact& contact, Vec3& right, Vec3& forward)
{
    // Remove the part of the facing vector that points along the surface normal.
    // The remaining vector lies in the terrain's tangent plane. Cross products
    // then give an orthonormal right/up/forward frame for the rigid boot.
    Vec3 facing(std::sin(contact.yaw), 0, -std::cos(contact.yaw));
    forward = (facing - contact.up * facing.Dot(contact.up)).Normalized();
    right = forward.Cross(contact.up).Normalized();
    forward = contact.up.Cross(right).Normalized();
}

void Character::FitSole(FootContact& contact, GroundHeight ground)
{
    Vec3 right, forward;
    FootBasis(contact, right, forward);
    // A tilted plane touching the terrain at its center may cut into a hill at
    // the toe or heel. Find the highest required sole Y over a sampled footprint.
    // For each offset: soleY + offset.y >= terrainY + clearance.
    float requiredY = -1.0e10f;
    // Fit a rigid sole above its entire footprint, including heel and toe.
    // Sampling the interior also catches a triangle edge beneath the boot.
    for (int z = 0; z <= 8; ++z) {
        for (int x = 0; x <= 8; ++x) {
            Vec3 offset = right * (-0.17f + 0.34f * x / 8.0f) +
                          forward * (-0.15f + 0.51f * z / 8.0f);
            requiredY = std::max(requiredY,
                ground(contact.sole.x + offset.x, contact.sole.z + offset.z) +
                SOLE_CLEARANCE - offset.y);
        }
    }
    contact.sole.y = requiredY;
}

Character::FootContact Character::SampleFoot(const Vec3& location, float heading,
                                            GroundHeight ground) const
{
    FootContact contact;
    contact.sole = location;
    contact.yaw = heading;
    // Central differences approximate partial derivatives of height h(x,z).
    // An upward normal to y=h(x,z) is proportional to (-dh/dx, 1, -dh/dz).
    // Multiplying all components by 2*h avoids two divisions before normalizing.
    const float h = 0.18f;
    contact.up = Vec3(ground(location.x - h, location.z) - ground(location.x + h, location.z),
                      2.0f * h,
                      ground(location.x, location.z - h) - ground(location.x, location.z + h)
                     ).Normalized();
    FitSole(contact, ground);
    return contact;
}

void Character::InitializeFeet(GroundHeight ground)
{
    // Initial/reset contacts are the only placements made without a swing.
    // Afterwards the controller must lift a foot before changing its heading.
    for (int i = 0; i < 2; ++i) {
        float side = i == 0 ? -1.0f : 1.0f;
        feet[i] = FootState();
        feet[i].contact = SampleFoot(position + Right() * (side * 0.25f), yaw, ground);
    }
    position.y = ground(position.x, position.z);
    previousPosition = position;
    pelvisY = position.y + ANKLE_HEIGHT + 1.52f;
    feetInitialized = true;
}

void Character::ConstrainRoot()
{
    // Preserve planted targets during abrupt turns. The root must remain within
    // horizontal reach of each support foot; the pelvis handles vertical reach.
    for (int pass = 0; pass < 4; ++pass) {
        for (int i = 0; i < 2; ++i) {
            if (feet[i].swinging && feet[i].delay <= 0.0f)
                continue;
            float side = i == 0 ? -1.0f : 1.0f;
            // hip = root + lateralOffset, so subtracting that offset from the
            // foot gives the center of the root's permitted horizontal disk.
            Vec3 center = feet[i].contact.sole - Right() * (side * 0.22f);
            Vec3 delta(position.x - center.x, 0, position.z - center.z);
            float distance = delta.Length();
            if (distance > 1.0f) {
                delta = delta * (1.0f / distance);
                position.x = center.x + delta.x;
                position.z = center.z + delta.z;
            }
        }
    }
}

bool Character::FeetSwinging() const
{
    // A delayed foot is part of an already scheduled pivot, even though it is
    // still supporting the body until its delay expires.
    return feet[0].swinging || feet[1].swinging;
}

void Character::TurnHips(float dt)
{
    // The torso does not snap to a movement heading. Rotate it at a bounded rate,
    // and intersect the allowed heading intervals of the two feet. A planted
    // foot keeps its heading; an airborne one gradually gains a new heading.
    float desired = yaw + requestedTurnRate * dt;
    float lower = yaw - MAX_TURN_RATE * dt;
    float upper = yaw + MAX_TURN_RATE * dt;
    for (int i = 0; i < 2; ++i) {
        // Express each heading near yaw so wrapping at +/-PI cannot cause a snap.
        float footYaw = yaw + AngleDifference(feet[i].contact.yaw, yaw);
        lower = std::max(lower, footYaw - MAX_HIP_TWIST);
        upper = std::min(upper, footYaw + MAX_HIP_TWIST);
    }
    if (lower <= upper)
        yaw = std::remainder(std::clamp(desired, lower, upper), 2.0f * PI);
}

float Character::LimitLandingHeading(int index, float heading) const
{
    // Keep the two feet's heading intervals overlapping. The next contact must
    // leave at least one possible pelvis heading that respects both twist limits.
    float other = feet[1 - index].contact.yaw;
    float limit = 2.0f * MAX_HIP_TWIST - 0.05f;
    return other + std::clamp(AngleDifference(heading, other), -limit, limit);
}

void Character::BeginStep(int index, const Vec3& velocity, GroundHeight ground)
{
    FootState& foot = feet[index];
    float speed = velocity.Length();
    float signedSpeed = velocity.Dot(Forward()); // Negative when walking backward.
    float rise = ground(position.x + Forward().x * 0.7f,
                        position.z + Forward().z * 0.7f) - position.y;
    float travel = 0.70f / (1.0f + 0.8f * std::abs(rise));
    foot.duration = speed > 0.01f ? std::clamp(travel / speed, 0.12f, 0.45f) : 0.22f;
    foot.delay = 0.0f;
    foot.pivot = false;

    // Predict the body's curved path during this swing, then place the foot at
    // its lateral offset from that FUTURE body frame. Rotation of the offset is
    // essential: on a right turn the left/outside foot follows a longer arc.
    // A small-angle linear approximation would be v_foot = v - side*r*omega.
    float turn = std::clamp(requestedTurnRate * foot.duration, -0.38f, 0.38f);
    float landingYaw = LimitLandingHeading(index, yaw + turn);
    Vec3 futureRight(std::cos(landingYaw), 0, std::sin(landingYaw));
    Vec3 futureForward(std::sin(landingYaw), 0, -std::cos(landingYaw));
    Vec3 futureRoot = position;
    if (speed > 0.01f) {
        if (std::abs(requestedTurnRate) > 0.001f) {
            // Integral of forward(yaw + omega*t) over the swing duration.
            float endYaw = yaw + requestedTurnRate * foot.duration;
            float radius = signedSpeed / requestedTurnRate;
            futureRoot = futureRoot + Vec3(
                radius * (std::cos(yaw) - std::cos(endYaw)), 0,
                radius * (std::sin(yaw) - std::sin(endYaw)));
        } else {
            futureRoot = futureRoot + velocity * foot.duration;
        }
    }
    float side = index == 0 ? -1.0f : 1.0f;
    Vec3 destination = futureRoot + futureRight * (side * 0.25f);
    if (speed > 0.01f)
        destination = destination + futureForward * (signedSpeed < 0 ? -0.10f : 0.10f);

    // Save a complete start/landing pose. Neither is dragged along with the root
    // later; the swing interpolates between these two committed WORLD contacts.
    foot.start = foot.contact;
    foot.landing = SampleFoot(destination, landingYaw, ground);
    float stepUp = foot.landing.sole.y - foot.start.sole.y;
    foot.clearance = std::clamp(0.22f + std::max(0.0f, stepUp) * 0.45f, 0.22f, 0.6f);
    foot.progress = 0.0f;
    foot.swinging = true;
}

void Character::BeginPivot(GroundHeight ground)
{
    // A pivot rotates the TWO contact positions around a stationary root. For
    // right turns, the left foot travels forward and the right foot backward.
    // Their swings overlap, with a short delay for the trailing foot, rather
    // than spinning either sole against the floor under a rotating torso.
    float heading = yaw + std::clamp(requestedTurnRate * 0.30f, -0.44f, 0.44f);
    for (int i = 0; i < 2; ++i)
        heading = LimitLandingHeading(i, heading);
    Vec3 futureRight(std::cos(heading), 0, std::sin(heading));
    for (int i = 0; i < 2; ++i) {
        FootState& foot = feet[i];
        float side = i == 0 ? -1.0f : 1.0f;
        foot.start = foot.contact;
        foot.landing = SampleFoot(position + futureRight * (side * 0.25f), heading, ground);
        foot.pivotCenter = position;
        foot.pivot = true;
        foot.duration = 0.30f;
        bool lead = requestedTurnRate > 0 ? i == 0 : i == 1;
        foot.delay = lead ? 0.0f : 0.05f;
        foot.clearance = 0.09f + 0.3f * std::max(0.0f, foot.landing.sole.y - foot.start.sole.y);
        foot.progress = 0.0f;
        foot.swinging = true;
    }
}

void Character::AdvanceSteps(float dt, GroundHeight ground)
{
    for (int i = 0; i < 2; ++i) {
        FootState& foot = feet[i];
        if (!foot.swinging)
            continue;
        float stepDt = dt;
        if (foot.delay > 0.0f) {
            float wait = std::min(foot.delay, stepDt);
            foot.delay -= wait;
            stepDt -= wait;
            if (stepDt <= 0.0f)
                continue;
        }
        foot.progress = std::min(1.0f, foot.progress + stepDt / foot.duration);
        float u = foot.progress;
        // Smoothstep has zero horizontal velocity at liftoff and touchdown.
        // The quartic arch is zero at both ends, peaks at one halfway through,
        // and also has zero derivative at both ends, avoiding a vertical snap.
        float t = u * u * (3.0f - 2.0f * u);
        float arc = 16.0f * u * u * (1.0f - u) * (1.0f - u);
        foot.contact.sole = foot.start.sole * (1.0f - t) + foot.landing.sole * t;
        if (foot.pivot) {
            // Interpolate the angle/radius around the root, not a chord cutting
            // between the legs. The saved start radius also handles entering a
            // pivot from a spread-out walking stance without teleporting a foot.
            Vec3 from = foot.start.sole - foot.pivotCenter;
            Vec3 to = foot.landing.sole - foot.pivotCenter;
            from.y = to.y = 0.0f;
            float angle = std::atan2(from.x, from.z) +
                AngleDifference(std::atan2(to.x, to.z), std::atan2(from.x, from.z)) * t;
            float radius = from.Length() * (1.0f - t) + to.Length() * t;
            foot.contact.sole.x = foot.pivotCenter.x + radius * std::sin(angle);
            foot.contact.sole.z = foot.pivotCenter.z + radius * std::cos(angle);
        }
        foot.contact.sole.y += foot.clearance * arc;
        foot.contact.up = (foot.start.up * (1.0f - t) + foot.landing.up * t).Normalized();
        foot.contact.yaw = foot.start.yaw + AngleDifference(foot.landing.yaw, foot.start.yaw) * t;
        // The endpoint alone cannot detect a hill between the two contacts.
        // Fit the CURRENT tilted boot footprint too, and lift it if needed.
        FootContact floor = foot.contact;
        FitSole(floor, ground);
        foot.contact.sole.y = std::max(foot.contact.sole.y, floor.sole.y + 0.08f * arc);
        if (u >= 1.0f) {
            foot.contact = foot.landing;
            foot.swinging = false;
            nextFoot = 1 - i;
        }
    }
}

void Character::Update(float dt, float distanceMoved, GroundHeight ground, float turnRate)
{
    if (!feetInitialized)
        InitializeFeet(ground);
    dt = std::max(0.0f, dt);
    requestedTurnRate = std::clamp(turnRate, -MAX_TURN_RATE, MAX_TURN_RATE);
    // Separate translation from rotation: a pivot has zero linear distance but
    // still needs scheduled foot motion. Camera orbit never enters this method.
    bool translating = distanceMoved > 0.000001f;
    bool turning = std::abs(requestedTurnRate) > 0.001f;
    bool pivoting = turning && !translating;
    bool moving = translating || turning;
    float oldYaw = yaw;
    TurnHips(dt);

    // Scene has proposed a root translation. Derive velocity before constraining
    // reach, so a temporarily blocked root can still request a recovery step.
    Vec3 requested(position.x - previousPosition.x, 0, position.z - previousPosition.z);
    Vec3 velocity = dt > 0.0f ? requested * (1.0f / dt) : Vec3();
    ConstrainRoot();
    position.y = ground(position.x, position.z);

    bool reversedTurn = requestedTurnRate * previousTurnRate < -0.001f;
    if (pivoting && (!wasPivoting || reversedTurn)) {
        BeginPivot(ground);
    } else {
        for (int i = 0; i < 2; ++i) {
            if (!feet[i].swinging)
                continue;
            Vec3 targetOffset = feet[i].landing.sole - position;
            targetOffset.y = 0;
            // Replan from the current airborne pose when stopping, leaving a
            // pivot, or reaching too far. Do not restart for every tiny yaw
            // change: constantly resetting progress can prevent touchdown.
            if ((wasMoving && !moving) || (feet[i].pivot && translating) ||
                (!feet[i].pivot && targetOffset.Length() > 1.25f))
                BeginStep(i, translating ? velocity : Vec3(), ground);
        }
        if (!FeetSwinging()) {
            if (pivoting) {
                BeginPivot(ground);
            } else if (translating) {
                BeginStep(nextFoot, velocity, ground);
            } else {
                // With no input, finish by stepping each foot near its hip.
                // Contact errors include heading, so feet never rotate on the
                // ground just to match a newly stopped torso.
                int settle = -1;
                float largestError = 0.06f;
                for (int i = 0; i < 2; ++i) {
                    float side = i == 0 ? -1.0f : 1.0f;
                    Vec3 home = position + Right() * (side * 0.25f);
                    Vec3 offset = home - feet[i].contact.sole;
                    offset.y = 0;
                    float error = offset.Length() + 0.15f * std::abs(AngleDifference(yaw, feet[i].contact.yaw));
                    if (error > largestError) {
                        largestError = error;
                        settle = i;
                    }
                }
                if (settle >= 0)
                    BeginStep(settle, Vec3(), ground);
            }
        }
    }
    AdvanceSteps(dt, ground);
    ConstrainRoot();
    position.y = ground(position.x, position.z);
    Vec3 traveled(position.x - previousPosition.x, 0, position.z - previousPosition.z);
    // Include angular travel so in-place steps have a small, gentle body bob.
    float gaitDistance = traveled.Length() + 0.25f * std::abs(AngleDifference(yaw, oldYaw));
    walkPhase = std::fmod(walkPhase + gaitDistance * 2.0f * PI / 2.2f, 2.0f * PI);
    // Pivoting needs less arm motion than walking. Exponential easing depends
    // on elapsed seconds rather than a fixed blend amount on every frame.
    float targetBlend = translating ? 1.0f : turning ? 0.45f : 0.0f;
    walkBlend += (targetBlend - walkBlend) * (1.0f - std::exp(-12.0f * dt));
    if (!moving && walkBlend < 0.0001f)
        walkBlend = 0;
    UpdatePose(dt);
    previousPosition = position;
    wasMoving = moving;
    wasPivoting = pivoting;
    previousTurnRate = requestedTurnRate;
}

void Character::UpdatePose(float dt)
{
    Vec3 right = Right();
    Vec3 forward = Forward();
    // Both legs share ONE pelvis height. Each ankle supplies a permitted Y
    // interval for that pelvis; intersect them before solving either leg.
    float lower = -1.0e10f, upper = 1.0e10f;
    Vec3 targets[2];
    for (int i = 0; i < 2; ++i) {
        const FootContact& foot = feet[i].contact;
        FootBasis(foot, pose.footRight[i], pose.footForward[i]);
        pose.footUp[i] = foot.up;
        pose.footPlanted[i] = !feet[i].swinging || feet[i].delay > 0.0f;
        // IK ends at the ankle joint, which sits above the sole along boot up.
        targets[i] = foot.sole + foot.up * ANKLE_HEIGHT;
        float side = i == 0 ? -1.0f : 1.0f;
        Vec3 hip = position + right * (side * 0.22f);
        float dx = targets[i].x - hip.x, dz = targets[i].z - hip.z;
        // Pythagoras: reach^2 = horizontal^2 + vertical^2. A foot farther out
        // horizontally leaves less vertical reach, so the body must crouch.
        float vertical = std::sqrt(std::max(0.0f, MAX_LEG_REACH * MAX_LEG_REACH - dx * dx - dz * dz));
        // Keep the hips above the ankles, while the upper bound enforces reach.
        lower = std::max(lower, targets[i].y + 0.30f);
        upper = std::min(upper, targets[i].y + vertical);
    }
    float bob = 0.02f * walkBlend * (1.0f - std::cos(2.0f * walkPhase));
    float desired = std::min(position.y + ANKLE_HEIGHT + 1.52f + bob, upper);
    // Ease toward a comfortable height, then clamp to the reachable interval.
    // The final clamp is mandatory: smoothing alone could briefly stretch a leg.
    pelvisY += (desired - pelvisY) * (1.0f - std::exp(-12.0f * dt));
    if (lower <= upper)
        pelvisY = std::clamp(pelvisY, lower, upper);
    else
        // Extreme/discontinuous terrain may have no common interval. Prefer the
        // non-stretching upper bound; SolveTwoBoneIK still clamps impossible
        // individual requests instead of producing invalid triangle geometry.
        pelvisY = upper;
    pose.pelvis = Vec3(position.x, pelvisY, position.z);
    pose.head = pose.pelvis + Vec3(0, 1.36f, 0);

    Vec3 down(0, -1, 0);
    for (int i = 0; i < 2; ++i) {
        float side = i == 0 ? -1.0f : 1.0f;
        pose.hips[i] = pose.pelvis + right * (side * 0.22f);
        // The KNEE bending plane follows this FOOT's heading, not the torso.
        // A planted toe keeps its heading, so its shin/knee do not corkscrew when
        // the torso turns. Relative yaw is absorbed at the hip ball joint and is
        // bounded by TurnHips until a new stepping contact relieves the twist.
        const FootContact& foot = feet[i].contact;
        Vec3 kneeForward(std::sin(foot.yaw), 0, -std::cos(foot.yaw));
        SolveTwoBoneIK(pose.hips[i], targets[i], kneeForward, LEG_LENGTH, LEG_LENGTH,
                       pose.knees[i], pose.ankles[i]);
        // Follow the solved stride so arm opposition stays in sync even when
        // a slope changes step length or a stop triggers a settling step.
        float swing = std::clamp((pose.ankles[i] - pose.hips[i]).Dot(forward) / 0.65f,
                                 -1.0f, 1.0f);
        float armAngle = -0.40f * walkBlend * swing;
        float forearmAngle = armAngle + 0.20f + 0.15f * walkBlend * (1.0f - swing);
        pose.shoulders[i] = pose.pelvis + Vec3(0, 0.86f, 0) + right * (side * 0.43f);
        pose.elbows[i] = pose.shoulders[i] +
            (forward * std::sin(armAngle) + down * std::cos(armAngle)) * 0.46f;
        pose.hands[i] = pose.elbows[i] +
            (forward * std::sin(forearmAngle) + down * std::cos(forearmAngle)) * 0.43f;
    }
}

#ifndef CHARACTER_HEADLESS
void Character::DrawSphere(const Vec3& center, float radius) const
{
    glPushMatrix();
    glTranslatef(center.x, center.y, center.z);
    gluSphere(quadric, radius, 16, 12);
    glPopMatrix();
}

void Character::DrawBone(const Vec3& start, const Vec3& end, float radius) const
{
    Vec3 direction = end - start;
    float length = direction.Length();
    if (length <= 0.00001f)
        return;
    direction.Normalize();
    glPushMatrix();
    glTranslatef(start.x, start.y, start.z);
    // GLU cylinders point along local +Z; rotate that axis toward the bone.
    float angle = acos(std::clamp(direction.z, -1.0f, 1.0f)) * 180.0f / PI;
    Vec3 axis(-direction.y, direction.x, 0.0f);
    if (axis.LengthSquared() > 0.000001f)
        glRotatef(angle, axis.x, axis.y, axis.z);
    else if (direction.z < 0.0f)
        glRotatef(180.0f, 1.0f, 0.0f, 0.0f);
    gluCylinder(quadric, radius, radius, length, 16, 1);
    glPopMatrix();
}

void Character::DrawBoot(int index) const
{
    const Vec3& ankle = pose.ankles[index];
    const Vec3& right = pose.footRight[index];
    const Vec3& forward = pose.footForward[index];
    const Vec3& up = pose.footUp[index];
    const float sides[4] = { -0.17f, 0.17f, 0.17f, -0.17f };
    const float depths[4] = { -0.15f, -0.15f, 0.36f, 0.36f };
    Vec3 bottom[4], top[4];
    for (int i = 0; i < 4; ++i) {
        bottom[i] = ankle + right * sides[i] + forward * depths[i];
        bottom[i] = bottom[i] - up * ANKLE_HEIGHT;
        top[i] = bottom[i] + up * (ANKLE_HEIGHT + 0.08f);
    }

    glColor3f(0.12f, 0.10f, 0.09f);
    glBegin(GL_QUADS);
    glNormal3f(up.x, up.y, up.z);
    for (int i = 0; i < 4; ++i)
        glVertex3f(top[i].x, top[i].y, top[i].z);
    glNormal3f(-up.x, -up.y, -up.z);
    for (int i = 3; i >= 0; --i)
        glVertex3f(bottom[i].x, bottom[i].y, bottom[i].z);
    for (int i = 0; i < 4; ++i) {
        int j = (i + 1) % 4;
        Vec3 normal = (bottom[j] - bottom[i]).Cross(top[i] - bottom[i]).Normalized();
        glNormal3f(normal.x, normal.y, normal.z);
        glVertex3f(bottom[i].x, bottom[i].y, bottom[i].z);
        glVertex3f(bottom[j].x, bottom[j].y, bottom[j].z);
        glVertex3f(top[j].x, top[j].y, top[j].z);
        glVertex3f(top[i].x, top[i].y, top[i].z);
    }
    glEnd();
}

void Character::Draw() const
{
    // Cylinders form the limbs, with spheres covering the animated joints.
    glColor3f(0.14f, 0.20f, 0.38f);
    DrawBone(pose.hips[0], pose.hips[1], 0.22f);
    for (int i = 0; i < 2; ++i) {
        DrawSphere(pose.hips[i], 0.16f);
        DrawBone(pose.hips[i], pose.knees[i], 0.14f);
        DrawSphere(pose.knees[i], 0.145f);
        DrawBone(pose.knees[i], pose.ankles[i], 0.12f);
        DrawSphere(pose.ankles[i], 0.125f);
    }

    glColor3f(0.78f, 0.22f, 0.14f);
    Vec3 chest = pose.pelvis + Vec3(0.0f, 0.75f, 0.0f);
    DrawBone(pose.pelvis, chest, 0.31f);
    DrawSphere(pose.pelvis, 0.31f);
    DrawSphere(chest, 0.33f);
    DrawBone(pose.shoulders[0], pose.shoulders[1], 0.16f);
    for (int i = 0; i < 2; ++i) {
        DrawSphere(pose.shoulders[i], 0.16f);
        DrawBone(pose.shoulders[i], pose.elbows[i], 0.115f);
    }

    glColor3f(0.88f, 0.66f, 0.44f);
    for (int i = 0; i < 2; ++i) {
        DrawSphere(pose.elbows[i], 0.115f);
        DrawBone(pose.elbows[i], pose.hands[i], 0.09f);
        DrawSphere(pose.hands[i], 0.13f);
    }
    DrawBone(chest, pose.head, 0.12f);
    DrawSphere(pose.head, HEAD_RADIUS);

    // Eyes and a small nose make the model's facing direction visible.
    Vec3 forward = Forward();
    Vec3 right = Right();
    glColor3f(0.08f, 0.07f, 0.06f);
    DrawSphere(pose.head + forward * 0.269f + right * 0.10f +
                Vec3(0.0f, 0.06f, 0.0f), 0.035f);
    DrawSphere(pose.head + forward * 0.269f - right * 0.10f +
                Vec3(0.0f, 0.06f, 0.0f), 0.035f);
    glColor3f(0.82f, 0.56f, 0.37f);
    DrawSphere(pose.head + forward * 0.30f, 0.06f);

    DrawBoot(0);
    DrawBoot(1);
}



#endif // CHARACTER_HEADLESS


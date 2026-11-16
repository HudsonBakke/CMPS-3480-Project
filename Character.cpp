#include "Character.h"

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
    feetInitialized = wasMoving = false;
    activeFoot = -1;
    nextFoot = 0;
    pose = CharacterPose();
}

void Character::FootBasis(const FootContact& contact, Vec3& right, Vec3& forward)
{
    Vec3 facing(std::sin(contact.yaw), 0, -std::cos(contact.yaw));
    forward = (facing - contact.up * facing.Dot(contact.up)).Normalized();
    right = forward.Cross(contact.up).Normalized();
    forward = contact.up.Cross(right).Normalized();
}

void Character::FitSole(FootContact& contact, GroundHeight ground)
{
    Vec3 right, forward;
    FootBasis(contact, right, forward);
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
            if (feet[i].swinging)
                continue;
            float side = i == 0 ? -1.0f : 1.0f;
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

void Character::BeginStep(int index, const Vec3& velocity, GroundHeight ground)
{
    FootState& foot = feet[index];
    float speed = velocity.Length();
    float rise = ground(position.x + Forward().x * 0.7f,
                        position.z + Forward().z * 0.7f) - position.y;
    // Shorter advances on slopes, with duration following the actual speed.
    float travel = 0.70f / (1.0f + 0.8f * std::abs(rise));
    foot.duration = speed > 0.01f ? std::clamp(travel / speed, 0.12f, 0.45f) : 0.22f;
    float side = index == 0 ? -1.0f : 1.0f;
    Vec3 destination = position + Right() * (side * 0.25f);
    if (speed > 0.01f)
        destination = destination + velocity * foot.duration + Forward() * 0.10f;
    foot.start = foot.contact;
    foot.landing = SampleFoot(destination, yaw, ground);
    float stepUp = foot.landing.sole.y - foot.start.sole.y;
    foot.clearance = std::clamp(0.22f + std::max(0.0f, stepUp) * 0.45f, 0.22f, 0.6f);
    foot.progress = 0.0f;
    foot.swinging = true;
    activeFoot = index;
}

void Character::AdvanceStep(float dt, GroundHeight ground)
{
    if (activeFoot < 0)
        return;
    FootState& foot = feet[activeFoot];
    foot.progress = std::min(1.0f, foot.progress + dt / foot.duration);
    float u = foot.progress;
    float t = u * u * (3.0f - 2.0f * u);
    float arc = 16.0f * u * u * (1.0f - u) * (1.0f - u);
    foot.contact.sole = foot.start.sole * (1.0f - t) + foot.landing.sole * t;
    foot.contact.sole.y += foot.clearance * arc;
    foot.contact.up = (foot.start.up * (1.0f - t) + foot.landing.up * t).Normalized();
    foot.contact.yaw = foot.start.yaw +
        std::remainder(foot.landing.yaw - foot.start.yaw, 2.0f * PI) * t;
    // Ground clearance is checked throughout the swing, not just at touchdown.
    FootContact floor = foot.contact;
    FitSole(floor, ground);
    foot.contact.sole.y = std::max(foot.contact.sole.y, floor.sole.y + 0.08f * arc);
    if (u >= 1.0f) {
        foot.contact = foot.landing;
        foot.swinging = false;
        nextFoot = 1 - activeFoot;
        activeFoot = -1;
    }
}

void Character::Update(float dt, float distanceMoved, GroundHeight ground)
{
    if (!feetInitialized)
        InitializeFeet(ground);
    dt = std::max(0.0f, dt);
    Vec3 requested(position.x - previousPosition.x, 0, position.z - previousPosition.z);
    bool moving = distanceMoved > 0.000001f;
    Vec3 velocity = dt > 0.0f ? requested * (1.0f / dt) : Vec3();
    ConstrainRoot();
    position.y = ground(position.x, position.z);

    if (activeFoot >= 0) {
        const FootState& foot = feet[activeFoot];
        Vec3 targetOffset = foot.landing.sole - position;
        targetOffset.y = 0.0f;
        bool turning = moving && std::abs(std::remainder(yaw - foot.landing.yaw, 2.0f * PI)) > 0.7f;
        if ((wasMoving && !moving) || turning || targetOffset.Length() > 1.25f)
            BeginStep(activeFoot, moving ? velocity : Vec3(), ground);
    }
    if (activeFoot < 0) {
        if (moving) {
            BeginStep(nextFoot, velocity, ground);
        } else {
            // Finish with two comfortable support feet after stopping or turning.
            int settle = -1;
            float largestError = 0.06f;
            for (int i = 0; i < 2; ++i) {
                float side = i == 0 ? -1.0f : 1.0f;
                Vec3 home = position + Right() * (side * 0.25f);
                Vec3 offset = home - feet[i].contact.sole;
                offset.y = 0.0f;
                float error = offset.Length() +
                    0.15f * std::abs(std::remainder(yaw - feet[i].contact.yaw, 2.0f * PI));
                if (error > largestError) {
                    largestError = error;
                    settle = i;
                }
            }
            if (settle >= 0)
                BeginStep(settle, Vec3(), ground);
        }
    }
    AdvanceStep(dt, ground);
    ConstrainRoot();
    position.y = ground(position.x, position.z);
    Vec3 traveled(position.x - previousPosition.x, 0, position.z - previousPosition.z);
    walkPhase = std::fmod(walkPhase + traveled.Length() * 2.0f * PI / 2.2f, 2.0f * PI);
    float targetBlend = moving ? 1.0f : 0.0f;
    walkBlend += (targetBlend - walkBlend) * (1.0f - std::exp(-12.0f * dt));
    if (!moving && walkBlend < 0.0001f)
        walkBlend = 0.0f;
    UpdatePose(dt);
    previousPosition = position;
    wasMoving = moving;
}

void Character::UpdatePose(float dt)
{
    Vec3 right = Right();
    Vec3 forward = Forward();
    float lower = -1.0e10f, upper = 1.0e10f;
    Vec3 targets[2];
    for (int i = 0; i < 2; ++i) {
        const FootContact& foot = feet[i].contact;
        FootBasis(foot, pose.footRight[i], pose.footForward[i]);
        pose.footUp[i] = foot.up;
        pose.footPlanted[i] = !feet[i].swinging;
        targets[i] = foot.sole + foot.up * ANKLE_HEIGHT;
        float side = i == 0 ? -1.0f : 1.0f;
        Vec3 hip = position + right * (side * 0.22f);
        float dx = targets[i].x - hip.x, dz = targets[i].z - hip.z;
        float vertical = std::sqrt(std::max(0.0f, MAX_LEG_REACH * MAX_LEG_REACH - dx * dx - dz * dz));
        lower = std::max(lower, targets[i].y + 0.30f);
        upper = std::min(upper, targets[i].y + vertical);
    }
    float bob = 0.02f * walkBlend * (1.0f - std::cos(2.0f * walkPhase));
    float desired = std::min(position.y + ANKLE_HEIGHT + 1.52f + bob, upper);
    pelvisY += (desired - pelvisY) * (1.0f - std::exp(-12.0f * dt));
    if (lower <= upper)
        pelvisY = std::clamp(pelvisY, lower, upper);
    else
        pelvisY = upper;
    pose.pelvis = Vec3(position.x, pelvisY, position.z);
    pose.head = pose.pelvis + Vec3(0, 1.36f, 0);

    Vec3 down(0, -1, 0);
    for (int i = 0; i < 2; ++i) {
        float side = i == 0 ? -1.0f : 1.0f;
        pose.hips[i] = pose.pelvis + right * (side * 0.22f);
        SolveTwoBoneIK(pose.hips[i], targets[i], forward, LEG_LENGTH, LEG_LENGTH,
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

#include "Character.h"

#include <GL/gl.h>
#include <GL/glu.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace {
constexpr float PI = 3.14159265358979323846f;
constexpr float LEG_LENGTH = 0.82f;
constexpr float ANKLE_HEIGHT = 0.18f;
constexpr float HEAD_RADIUS = 0.30f;
constexpr float STRIDE_LENGTH = 2.2f;
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
    quadric = gluNewQuadric();
    if (!quadric) {
        std::fprintf(stderr, "Could not allocate model quadric.\n");
        std::exit(EXIT_FAILURE);
    }
    gluQuadricNormals(quadric, GLU_SMOOTH);
}

void Character::CleanupGraphics()
{
    if (quadric)
        gluDeleteQuadric(quadric);
    quadric = nullptr;
}

void Character::Reset()
{
    position = Vec3();
    yaw = 0.0f;
    walkPhase = 0.0f;
    walkBlend = 0.0f;
    UpdatePose();
}

void Character::Update(float dt, float distanceMoved)
{
    // Travel controls cadence: faster movement produces a faster cycle, and
    // pressing against the map boundary does not make the character walk in place.
    distanceMoved = std::max(0.0f, distanceMoved);
    walkPhase = std::fmod(walkPhase + distanceMoved * 2.0f * PI / STRIDE_LENGTH,
                         2.0f * PI);
    float targetBlend = (distanceMoved > 0.000001f) ? 1.0f : 0.0f;
    float smoothing = 1.0f - std::exp(-12.0f * std::max(0.0f, dt));
    walkBlend += (targetBlend - walkBlend) * smoothing;
    if (targetBlend == 0.0f && walkBlend < 0.0001f)
        walkBlend = 0.0f;
    UpdatePose();
}

void Character::UpdatePose()
{
    Vec3 right = Right();
    Vec3 forward = Forward();
    Vec3 down(0.0f, -1.0f, 0.0f);

    // The scene supplies one terrain height for the root. Everything below is
    // forward kinematics in the model's frame, with no per-foot terrain queries.
    float bob = 0.025f * walkBlend * (1.0f - std::cos(2.0f * walkPhase));
    pose.pelvis = position + Vec3(0.0f, ANKLE_HEIGHT + 2.0f * LEG_LENGTH + bob, 0.0f);
    pose.head = pose.pelvis + Vec3(0.0f, 1.36f, 0.0f);

    for (int i = 0; i < 2; ++i) {
        float side = (i == 0) ? -1.0f : 1.0f;
        float phase = walkPhase + ((i == 0) ? 0.0f : PI);
        float swing = std::cos(phase);
        // The knee bends more while this leg swings forward.
        float kneeBend = 0.12f + 0.70f * walkBlend * std::max(0.0f, -std::sin(phase));
        float hipAngle = 0.06f + 0.45f * walkBlend * swing;
        float shinAngle = hipAngle - kneeBend;
        pose.hips[i] = pose.pelvis + right * (side * 0.22f);
        pose.knees[i] = pose.hips[i] +
            (forward * std::sin(hipAngle) + down * std::cos(hipAngle)) * LEG_LENGTH;
        pose.ankles[i] = pose.knees[i] +
            (forward * std::sin(shinAngle) + down * std::cos(shinAngle)) * LEG_LENGTH;

        // Each arm swings opposite the leg on the same side.
        float armAngle = -0.40f * walkBlend * swing;
        float elbowBend = 0.20f + 0.15f * walkBlend * (1.0f - swing);
        float forearmAngle = armAngle + elbowBend;
        pose.shoulders[i] = pose.pelvis + Vec3(0.0f, 0.86f, 0.0f) +
                            right * (side * 0.43f);
        pose.elbows[i] = pose.shoulders[i] +
            (forward * std::sin(armAngle) + down * std::cos(armAngle)) * 0.46f;
        pose.hands[i] = pose.elbows[i] +
            (forward * std::sin(forearmAngle) + down * std::cos(forearmAngle)) * 0.43f;
    }
}

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

void Character::DrawBoot(const Vec3& ankle) const
{
    // A rigid boot follows its model-space ankle. Ground fitting comes later.
    Vec3 right = Right();
    Vec3 forward = Forward();
    const float sides[4] = { -0.17f, 0.17f, 0.17f, -0.17f };
    const float depths[4] = { -0.15f, -0.15f, 0.36f, 0.36f };
    Vec3 bottom[4], top[4];
    for (int i = 0; i < 4; ++i) {
        bottom[i] = ankle + right * sides[i] + forward * depths[i];
        bottom[i].y = ankle.y - ANKLE_HEIGHT;
        top[i] = Vec3(bottom[i].x, ankle.y + 0.08f, bottom[i].z);
    }

    glColor3f(0.12f, 0.10f, 0.09f);
    glBegin(GL_QUADS);
    glNormal3f(0.0f, 1.0f, 0.0f);
    for (int i = 0; i < 4; ++i)
        glVertex3f(top[i].x, top[i].y, top[i].z);
    glNormal3f(0.0f, -1.0f, 0.0f);
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

    DrawBoot(pose.ankles[0]);
    DrawBoot(pose.ankles[1]);
}


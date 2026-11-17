#include "Scene.h"
#include "Character.h"
#include "Math.h"
#include "fonts.h"

#include <GL/gl.h>
#include <GL/glu.h>
#include <algorithm>
#include <cmath>

float noise2(float vec[2]);

namespace {
constexpr float PI = 3.14159265358979323846f;

// The terrain covers 128 x 128 world units, centered on the origin.
constexpr int TERRAIN_CELLS = 128;
constexpr float TERRAIN_SPACING = 1.0f;
constexpr float TERRAIN_FREQUENCY = 0.045f;
constexpr float TERRAIN_HEIGHT = 10.0f;

struct TerrainVertex {
    Vec3 position;
    Vec3 normal;
};

TerrainVertex terrain[TERRAIN_CELLS + 1][TERRAIN_CELLS + 1];

float terrain_height(float x, float z)
{
    // Noninteger offsets keep the grid away from noise lattice coincidences.
    float sample[2] = {
        x * TERRAIN_FREQUENCY + 37.25f,
        z * TERRAIN_FREQUENCY + 61.75f
    };
    float height = noise2(sample);
    sample[0] *= 2.0f;
    sample[1] *= 2.0f;
    height += 0.35f * noise2(sample);
    sample[0] *= 2.0f;
    sample[1] *= 2.0f;
    height += 0.12f * noise2(sample);
    return height * TERRAIN_HEIGHT;
}

void init_terrain(void)
{
    for (int z = 0; z <= TERRAIN_CELLS; ++z) {
        for (int x = 0; x <= TERRAIN_CELLS; ++x) {
            float wx = (x - TERRAIN_CELLS / 2) * TERRAIN_SPACING;
            float wz = (z - TERRAIN_CELLS / 2) * TERRAIN_SPACING;
            TerrainVertex& vertex = terrain[z][x];
            vertex.position = Vec3(wx, terrain_height(wx, wz), wz);

            // Central differences approximate the surface's upward normal.
            float h = TERRAIN_SPACING;
            vertex.normal = Vec3(
                terrain_height(wx - h, wz) - terrain_height(wx + h, wz),
                2.0f * h,
                terrain_height(wx, wz - h) - terrain_height(wx, wz + h)
            ).Normalized();
        }
    }
}

// Interpolate the actual mesh triangles, rather than resampling the noise.
// fx/fz are the coordinates inside one grid cell. The strip's diagonal joins
// (1,0) to (0,1), so fx+fz selects the triangle. Each return is a barycentric
// weighted height: the character touches the same planes that OpenGL draws.
float ground_height(float x, float z)
{
    float gx = x / TERRAIN_SPACING + TERRAIN_CELLS / 2;
    float gz = z / TERRAIN_SPACING + TERRAIN_CELLS / 2;
    gx = std::clamp(gx, 0.0f, float(TERRAIN_CELLS));
    gz = std::clamp(gz, 0.0f, float(TERRAIN_CELLS));
    int ix = std::min(int(gx), TERRAIN_CELLS - 1);
    int iz = std::min(int(gz), TERRAIN_CELLS - 1);
    float fx = gx - ix;
    float fz = gz - iz;
    float h00 = terrain[iz][ix].position.y;
    float h10 = terrain[iz][ix + 1].position.y;
    float h01 = terrain[iz + 1][ix].position.y;
    float h11 = terrain[iz + 1][ix + 1].position.y;
    if (fx + fz <= 1.0f)
        return h00 + fx * (h10 - h00) + fz * (h01 - h00);
    return h11 + (1.0f - fx) * (h01 - h11) +
                 (1.0f - fz) * (h10 - h11);
}

struct OrbitCamera {
    float yaw = 0.0f;
    float elevation = 0.30f;
    float distance = 7.5f;

    Vec3 Position(const Vec3& target) const
    {
        float horizontal = distance * cos(elevation);
        Vec3 eye = target + Vec3(horizontal * sin(yaw),
                                distance * sin(elevation),
                                horizontal * cos(yaw));
        // Keep the view above the mesh when orbiting near rising ground.
        eye.y = std::max(eye.y, ground_height(eye.x, eye.z) + 0.5f);
        return eye;
    }
} camera;

Character man;

void reset_scene()
{
    man.Reset();
    camera = OrbitCamera();
    man.position.y = ground_height(man.position.x, man.position.z);
    man.Update(0.0f, 0.0f, ground_height);
}

void draw_terrain_vertex(const TerrainVertex& vertex)
{
    float tint = std::max(0.0f, std::min(1.0f,
        0.5f + vertex.position.y / (2.0f * TERRAIN_HEIGHT)));
    glColor3f(0.18f + 0.22f * tint, 0.35f + 0.30f * tint,
              0.10f + 0.12f * tint);
    glNormal3f(vertex.normal.x, vertex.normal.y, vertex.normal.z);
    glVertex3f(vertex.position.x, vertex.position.y, vertex.position.z);
}

} // namespace

void init_scene(void)
{
    init_terrain();
    man.InitGraphics();
    reset_scene();
}

void cleanup_scene(void)
{
    man.CleanupGraphics();
}

void physics(float dt, const Controls& controls)
{
    if (controls.reset) {
        reset_scene();
        return;
    }

    camera.yaw += controls.orbitHorizontal * 1.2f * dt;
    camera.elevation += controls.orbitVertical * 1.2f * dt;
    camera.elevation = std::clamp(camera.elevation, 0.10f, 1.35f);
    camera.yaw = std::remainder(camera.yaw, 2.0f * PI);

    // Walking direction belongs to the character, independently of the camera.
    // Character::Update integrates A/D turning while its foot planner schedules
    // the required contacts; changing view direction never snaps the body yaw.
    Vec3 movement = man.Forward() * controls.moveForward;
    movement.Normalize();

    Vec3 previous = man.position;
    float speed = controls.fast ? 6.0f : 3.0f;
    // Move more cautiously on slopes; the foot planner also shortens its steps.
    float sampleDistance = 0.5f;
    Vec3 ahead = man.position + movement * sampleDistance;
    float slope = (ground_height(ahead.x, ahead.z) - ground_height(man.position.x, man.position.z)) / sampleDistance;
    speed /= 1.0f + 0.65f * std::max(0.0f, slope) + 0.25f * std::abs(slope);
    man.position = man.position + movement * (speed * dt);
    // Leave enough margin for an anticipated step plus the boot's toe/heel.
    float edge = TERRAIN_CELLS * TERRAIN_SPACING * 0.5f - 1.5f;
    man.position.x = std::clamp(man.position.x, -edge, edge);
    man.position.z = std::clamp(man.position.z, -edge, edge);
    Vec3 step(man.position.x - previous.x, 0.0f, man.position.z - previous.z);

    // The root follows the mesh; Character plans terrain contacts and solves leg IK.
    man.position.y = ground_height(man.position.x, man.position.z);
    man.Update(dt, step.Length(), ground_height, controls.turn * 1.6f);
}

void render(int width, int height, const float lightPosition[4])
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_TEXTURE_2D);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0, (double)width / std::max(1, height), 0.1, 500.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    Vec3 target = man.Pose().head;
    Vec3 eye = camera.Position(target);
    gluLookAt(eye.x, eye.y, eye.z,
              target.x, target.y, target.z, 0.0, 1.0, 0.0);
    glEnable(GL_COLOR_MATERIAL);
    glEnable(GL_LIGHTING);
    glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);

    for (int z = 0; z < TERRAIN_CELLS; ++z) {
        glBegin(GL_TRIANGLE_STRIP);
        for (int x = 0; x <= TERRAIN_CELLS; ++x) {
            draw_terrain_vertex(terrain[z][x]);
            draw_terrain_vertex(terrain[z + 1][x]);
        }
        glEnd();
    }

    man.Draw();

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0, width, 0, height, -1, 1);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glDisable(GL_LIGHTING);
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_TEXTURE_2D);

    Rect r = {};
    r.left = 10;
    r.bot = height - 20;
    ggprint(&r, 12, 18, 0x00ffffff, "W/S: walk | A/D: turn | Arrows: orbit camera");
    ggprint(&r, 12, 18, 0x00ffffff, "Shift: faster | R: reset | Esc: quit");
    ggprint(&r, 12, 18, 0x00ffffff, "Man: %.1f, %.1f, %.1f",
            man.position.x, man.position.y, man.position.z);
}



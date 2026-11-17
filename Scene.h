#ifndef SCENE_H
#define SCENE_H

struct Controls {
    float moveForward = 0.0f;
    float turn = 0.0f; // A/D turn the character; arrows control only the camera.
    float orbitHorizontal = 0.0f;
    float orbitVertical = 0.0f;
    bool fast = false;
    bool reset = false;
};

void init_scene(void);
void cleanup_scene(void);
void physics(float dt, const Controls& controls);
void render(int width, int height, const float lightPosition[4]);

#endif


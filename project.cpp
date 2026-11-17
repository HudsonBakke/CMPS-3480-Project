// Framework author: Gordon Griesel

#include "Framework.h"
#include "Scene.h"
#include "fonts.h"

#include <algorithm>
#include <chrono>
#include <unistd.h>

int main(void)
{
    init_opengl();
    glEnable(GL_TEXTURE_2D);
    initialize_fonts();
    init_scene();

    auto previous = std::chrono::steady_clock::now();
    bool done = false;
    while (!done) {
        while (x11.getXPending()) {
            XEvent e = x11.getXNextEvent();
            x11.check_resize(&e);
            check_mouse(&e);
            if (x11.close_requested(e) || check_keys(&e))
                done = true;
        }
        if (done)
            break;

        char keys[32];
        x11.poll_keys(keys);
        auto held = [&](KeySym key) { return x11.key_down(keys, key); };
        Controls controls;
        controls.moveForward = held(XK_w) - held(XK_s);
        controls.turn = held(XK_d) - held(XK_a);
        controls.orbitHorizontal = held(XK_Right) - held(XK_Left);
        controls.orbitVertical = held(XK_Up) - held(XK_Down);
        controls.fast = held(XK_Shift_L) || held(XK_Shift_R);
        controls.reset = held(XK_r);

        auto now = std::chrono::steady_clock::now();
        float dt = std::chrono::duration<float>(now - previous).count();
        previous = now;
        physics(std::min(dt, 0.05f), controls);
        render(g.xres, g.yres, g.lightPosition);
        x11.swapBuffers();
        usleep(1000);
    }

    cleanup_scene();
    cleanup_fonts();
    return 0;
}


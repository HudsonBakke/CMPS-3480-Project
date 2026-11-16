#ifndef FRAMEWORK_H
#define FRAMEWORK_H

#include <X11/Xlib.h>
#include <X11/keysym.h>
#include <GL/gl.h>
#include <GL/glx.h>
#include <GL/glu.h>

// Framework author: Gordon Griesel

class Image {
public:
    int width = 0, height = 0;
    unsigned char *data = nullptr;
    ~Image();
    Image(const char *fname);
};

class Global {
public:
    int xres, yres;
    GLfloat lightAmbient[4];
    GLfloat lightDiffuse[4];
    GLfloat lightSpecular[4];
    GLfloat lightPosition[4];
    Global();
};

class X11_wrapper {
private:
    Display *dpy;
    Window win;
    GLXContext glc;
    Atom wmDelete;
public:
    X11_wrapper(void);
    ~X11_wrapper(void);
    void set_title(void);
    void setup_screen_res(const int w, const int h);
    void reshape_window(int width, int height);
    void check_resize(XEvent *e);
    bool getXPending();
    XEvent getXNextEvent();
    bool close_requested(const XEvent& e) const;
    void poll_keys(char keys[32]) const;
    bool key_down(const char keys[32], KeySym key) const;
    void swapBuffers();
};

extern Global g;
extern X11_wrapper x11;

void init_opengl(void);
void check_mouse(XEvent *e);
int check_keys(XEvent *e);

#endif

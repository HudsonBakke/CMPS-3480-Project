// Framework author: Gordon Griesel

#include "Framework.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <unistd.h>

Image::~Image()
{
    delete[] data;
}

Image::Image(const char *fname)
{
    if (fname[0] == '\0')
        return;
    char name[40];
    strcpy(name, fname);
    int slen = strlen(name);
    name[slen - 4] = '\0';
    char ppmname[80];
    sprintf(ppmname, "%s.ppm", name);
    char ts[100];
    sprintf(ts, "convert %s %s", fname, ppmname);
    system(ts);
    FILE *fpi = fopen(ppmname, "r");
    if (fpi) {
        char line[200];
        fgets(line, 200, fpi);
        fgets(line, 200, fpi);

        while (line[0] == '#' || strlen(line) < 2)
            fgets(line, 200, fpi);
        sscanf(line, "%i %i", &width, &height);
        fgets(line, 200, fpi);

        int n = width * height * 3;
        data = new unsigned char[n];
        for (int i = 0; i < n; i++)
            data[i] = fgetc(fpi);
        fclose(fpi);
    } else {
        printf("ERROR opening image: %s\n", ppmname);
        exit(0);
    }
    unlink(ppmname);
}

Global::Global()
{
    xres = 640;
    yres = 480;
    GLfloat la[] = { 0.3f, 0.3f, 0.3f, 1.0f };
    GLfloat ld[] = { 1.0f, 1.0f, 1.0f, 1.0f };
    GLfloat ls[] = { 0.5f, 0.5f, 0.5f, 1.0f };
    GLfloat lp[] = { -0.5f, 1.0f, 0.4f, 0.0f };
    memcpy(lightAmbient, la, sizeof(GLfloat) * 4);
    memcpy(lightDiffuse, ld, sizeof(GLfloat) * 4);
    memcpy(lightSpecular, ls, sizeof(GLfloat) * 4);
    memcpy(lightPosition, lp, sizeof(GLfloat) * 4);
}

X11_wrapper::X11_wrapper(void)
{
    GLint att[] = { GLX_RGBA, GLX_DEPTH_SIZE, 24, GLX_DOUBLEBUFFER, None };
    XSetWindowAttributes swa;
    setup_screen_res(640, 480);
    dpy = XOpenDisplay(NULL);
    if (dpy == NULL) {
        printf("\n\tcannot connect to X server\n\n");
        exit(EXIT_FAILURE);
    }
    Window root = DefaultRootWindow(dpy);
    XVisualInfo *vi = glXChooseVisual(dpy, 0, att);
    if (vi == NULL) {
        printf("\n\tno appropriate visual found\n\n");
        exit(EXIT_FAILURE);
    }
    Colormap cmap = XCreateColormap(dpy, root, vi->visual, AllocNone);
    swa.colormap = cmap;
    swa.event_mask = ExposureMask | KeyPressMask | KeyReleaseMask |
        StructureNotifyMask | SubstructureNotifyMask;
    win = XCreateWindow(dpy, root, 0, 0, g.xres, g.yres, 0,
        vi->depth, InputOutput, vi->visual,
        CWColormap | CWEventMask, &swa);
    wmDelete = XInternAtom(dpy, "WM_DELETE_WINDOW", False);
    XSetWMProtocols(dpy, win, &wmDelete, 1);
    set_title();
    glc = glXCreateContext(dpy, vi, NULL, GL_TRUE);
    glXMakeCurrent(dpy, win, glc);
}

X11_wrapper::~X11_wrapper(void)
{
    XDestroyWindow(dpy, win);
    XCloseDisplay(dpy);
}

void X11_wrapper::set_title(void)
{
    XMapWindow(dpy, win);
    XStoreName(dpy, win, "CMPS 3480 Project");
}

void X11_wrapper::setup_screen_res(const int w, const int h)
{
    g.xres = w;
    g.yres = h;
}

void X11_wrapper::reshape_window(int width, int height)
{
    setup_screen_res(width, height);

    glViewport(0, 0, (GLint)width, (GLint)height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glOrtho(0, g.xres, 0, g.yres, -1, 1);
    set_title();
}

void X11_wrapper::check_resize(XEvent *e)
{
    if (e->type != ConfigureNotify)
        return;
    XConfigureEvent xce = e->xconfigure;
    if (xce.width != g.xres || xce.height != g.yres) {
        reshape_window(xce.width, xce.height);
    }
}

bool X11_wrapper::getXPending()
{
    return XPending(dpy);
}

XEvent X11_wrapper::getXNextEvent()
{
    XEvent e;
    XNextEvent(dpy, &e);
    return e;
}

bool X11_wrapper::close_requested(const XEvent& e) const
{
    return e.type == ClientMessage &&
        static_cast<Atom>(e.xclient.data.l[0]) == wmDelete;
}

void X11_wrapper::poll_keys(char keys[32]) const
{
    Window focused;
    int revert;
    XGetInputFocus(dpy, &focused, &revert);
    memset(keys, 0, 32);
    if (focused == win)
        XQueryKeymap(dpy, keys);
}

bool X11_wrapper::key_down(const char keys[32], KeySym key) const
{
    KeyCode code = XKeysymToKeycode(dpy, key);
    return code != 0 && (keys[code / 8] & (1 << (code % 8)));
}

void X11_wrapper::swapBuffers()
{
    glXSwapBuffers(dpy, win);
}

Global g;
X11_wrapper x11;

void init_opengl(void)
{
    glClearColor(0.52f, 0.72f, 0.90f, 1.0f);
    glClearDepth(1.0);
    glDepthFunc(GL_LESS);
    glEnable(GL_DEPTH_TEST);
    glShadeModel(GL_SMOOTH);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0f, (GLfloat)g.xres / (GLfloat)g.yres, 0.1f, 100.0f);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(0, 5, 10, 0, 0, 0, 0, 1, 0);

    glEnable(GL_COLOR_MATERIAL);
    glEnable(GL_LIGHTING);
    glLightfv(GL_LIGHT0, GL_AMBIENT, g.lightAmbient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, g.lightDiffuse);
    glLightfv(GL_LIGHT0, GL_SPECULAR, g.lightSpecular);
    glLightfv(GL_LIGHT0, GL_POSITION, g.lightPosition);
    glEnable(GL_LIGHT0);
}

void check_mouse(XEvent *e)
{
    static int savex = 0;
    static int savey = 0;
    if (e->type == ButtonRelease)
    {
        return;
    }
    if (e->type == ButtonPress)
    {
        if (e->xbutton.button == 1) {
        }
        if (e->xbutton.button == 3) {
        }
    }
    if (savex != e->xbutton.x || savey != e->xbutton.y)
    {
        savex = e->xbutton.x;
        savey = e->xbutton.y;
    }
}

int check_keys(XEvent *e)
{
    if (e->type != KeyPress)
        return 0;
    int key = XLookupKeysym(&e->xkey, 0);
    switch (key)
    {
        case XK_Escape:
            return 1;
    }
    return 0;
}


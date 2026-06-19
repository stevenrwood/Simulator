/*
  sim_view.c - optional 3D machine view for the grblHAL simulator (-view).

  Win32 + OpenGL (immediate mode), no external dependencies beyond the system
  opengl32 / glu32 / gdi32 libraries. Runs on its own thread that owns the window
  and a snapshot of the machine geometry + live tool position; the grbl and socket
  threads are untouched and only push data in via the setters below.
*/

#include "sim_view.h"

#ifdef _WIN32

#include <windows.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <math.h>
#include <string.h>
#include <stdio.h>

#define DEG (3.14159265f / 180.0f)

static CRITICAL_SECTION lock;
static sim_view_geometry_t geom;
static float tool[3];
static volatile int running = 0;

static HWND  hwnd;
static HDC   hdc;
static HGLRC hglrc;
static GLUquadric *quad;            // reused for the puck cylinder + tool cone

// Orbit camera (mutated only on the view thread via WndProc / render). Default view: looking at the front
// of the machine (home = back-left-top), 10 deg to the right of dead-front and 30 deg above the spoilboard.
//   eye = target + dist * (cos(pitch)cos(yaw), cos(pitch)sin(yaw), sin(pitch)),  Z up.
//   yaw -90 = dead front (-Y); -80 swings 10 deg toward +X (operator's right). pitch = elevation.
static float cam_yaw = -80.0f, cam_pitch = 30.0f, cam_dist = 500.0f;
static float cam_target[3];
static int   dragging = 0, last_mx, last_my, framed = 0;

// ---- data feed (called from the grbl/realtime threads) -------------------------------------------

bool sim_view_active (void)
{
    return running != 0;
}

void sim_view_set_geometry (const sim_view_geometry_t *g)
{
    if(!running)
        return;
    EnterCriticalSection(&lock);
    geom = *g;
    LeaveCriticalSection(&lock);
}

void sim_view_set_tool (float x, float y, float z)
{
    if(!running)
        return;
    EnterCriticalSection(&lock);
    tool[0] = x; tool[1] = y; tool[2] = z;
    LeaveCriticalSection(&lock);
}

// ---- drawing helpers -----------------------------------------------------------------------------

static void box_solid (const float *mn, const float *mx)
{
    glBegin(GL_QUADS);
    glNormal3f(0, 0, -1); glVertex3f(mn[0],mn[1],mn[2]); glVertex3f(mn[0],mx[1],mn[2]); glVertex3f(mx[0],mx[1],mn[2]); glVertex3f(mx[0],mn[1],mn[2]);
    glNormal3f(0, 0,  1); glVertex3f(mn[0],mn[1],mx[2]); glVertex3f(mx[0],mn[1],mx[2]); glVertex3f(mx[0],mx[1],mx[2]); glVertex3f(mn[0],mx[1],mx[2]);
    glNormal3f(0,-1, 0);  glVertex3f(mn[0],mn[1],mn[2]); glVertex3f(mx[0],mn[1],mn[2]); glVertex3f(mx[0],mn[1],mx[2]); glVertex3f(mn[0],mn[1],mx[2]);
    glNormal3f(0, 1, 0);  glVertex3f(mn[0],mx[1],mn[2]); glVertex3f(mn[0],mx[1],mx[2]); glVertex3f(mx[0],mx[1],mx[2]); glVertex3f(mx[0],mx[1],mn[2]);
    glNormal3f(-1,0, 0);  glVertex3f(mn[0],mn[1],mn[2]); glVertex3f(mn[0],mn[1],mx[2]); glVertex3f(mn[0],mx[1],mx[2]); glVertex3f(mn[0],mx[1],mn[2]);
    glNormal3f(1, 0, 0);  glVertex3f(mx[0],mn[1],mn[2]); glVertex3f(mx[0],mx[1],mn[2]); glVertex3f(mx[0],mx[1],mx[2]); glVertex3f(mx[0],mn[1],mx[2]);
    glEnd();
}

static void box_wire (const float *mn, const float *mx)
{
    glBegin(GL_LINES);
    // bottom rectangle, top rectangle, vertical edges
    float zs[2]; zs[0] = mn[2]; zs[1] = mx[2];
    for(int k = 0; k < 2; k++) {
        float z = zs[k];
        glVertex3f(mn[0],mn[1],z); glVertex3f(mx[0],mn[1],z);
        glVertex3f(mx[0],mn[1],z); glVertex3f(mx[0],mx[1],z);
        glVertex3f(mx[0],mx[1],z); glVertex3f(mn[0],mx[1],z);
        glVertex3f(mn[0],mx[1],z); glVertex3f(mn[0],mn[1],z);
    }
    glVertex3f(mn[0],mn[1],mn[2]); glVertex3f(mn[0],mn[1],mx[2]);
    glVertex3f(mx[0],mn[1],mn[2]); glVertex3f(mx[0],mn[1],mx[2]);
    glVertex3f(mx[0],mx[1],mn[2]); glVertex3f(mx[0],mx[1],mx[2]);
    glVertex3f(mn[0],mx[1],mn[2]); glVertex3f(mn[0],mx[1],mx[2]);
    glEnd();
}

// Filled grid on the spoilboard plane, spanning the envelope XY, every <step> mm.
static void grid (float x0, float y0, float x1, float y1, float z, float step)
{
    glBegin(GL_LINES);
    for(float x = ceilf(x0/step)*step; x <= x1; x += step) { glVertex3f(x,y0,z); glVertex3f(x,y1,z); }
    for(float y = ceilf(y0/step)*step; y <= y1; y += step) { glVertex3f(x0,y,z); glVertex3f(x1,y,z); }
    glEnd();
}

static void cylinder (float cx, float cy, float z0, float z1, float r)
{
    glPushMatrix();
    glTranslatef(cx, cy, z0);
    gluCylinder(quad, r, r, z1 - z0, 24, 1);   // side
    gluDisk(quad, 0, r, 24, 1);                 // bottom cap (faces -Z, fine enough)
    glTranslatef(0, 0, z1 - z0);
    gluDisk(quad, 0, r, 24, 1);                 // top cap
    glPopMatrix();
}

// Tool: a cone with its tip at the controlled point, widening upward (spindle-ish).
static void tool_cone (float x, float y, float z)
{
    glPushMatrix();
    glTranslatef(x, y, z);
    gluCylinder(quad, 0.0, 6.0, 30.0, 20, 1);   // tip (r=0) at z, base (r=6) 30mm up
    glPopMatrix();
}

// ---- render --------------------------------------------------------------------------------------

static void frame_camera (const sim_view_geometry_t *g)
{
    // Frame once, the first time REAL geometry arrives (the initial pushed struct is all zeros). Until
    // then the default camera just shows the empty stage; after, the user can orbit/zoom freely.
    if(framed)
        return;
    if(!(g->have_fixtures || g->env_max[0] != g->env_min[0] || g->env_max[1] != g->env_min[1]))
        return;

    float lo[3], hi[3];
    for(int i = 0; i < 3; i++) {
        if(g->have_fixtures) {                          // frame on the stock + toolsetter (the area of interest)
            lo[i] = fminf(fminf(g->stock_min[i], g->stock_max[i]), fminf(g->puck_min[i], g->puck_max[i]));
            hi[i] = fmaxf(fmaxf(g->stock_min[i], g->stock_max[i]), fmaxf(g->puck_min[i], g->puck_max[i]));
        } else {                                        // no fixtures: frame the whole envelope
            lo[i] = fminf(g->env_min[i], g->env_max[i]);
            hi[i] = fmaxf(g->env_min[i], g->env_max[i]);
        }
        cam_target[i] = (lo[i] + hi[i]) * 0.5f;
    }
    float dx = hi[0]-lo[0], dy = hi[1]-lo[1], dz = hi[2]-lo[2];
    float diag = sqrtf(dx*dx + dy*dy + dz*dz);
    cam_dist = (diag > 1.0f ? diag : 200.0f) * 1.6f;
    framed = 1;
}

static void render (void)
{
    sim_view_geometry_t g;
    float t[3];
    EnterCriticalSection(&lock);
    g = geom; t[0] = tool[0]; t[1] = tool[1]; t[2] = tool[2];
    LeaveCriticalSection(&lock);

    frame_camera(&g);

    RECT rc; GetClientRect(hwnd, &rc);
    int w = rc.right, h = rc.bottom; if(h < 1) h = 1;
    glViewport(0, 0, w, h);
    glClearColor(0.10f, 0.11f, 0.13f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);

    glMatrixMode(GL_PROJECTION); glLoadIdentity();
    gluPerspective(45.0, (double)w / (double)h, 1.0, 20000.0);

    glMatrixMode(GL_MODELVIEW); glLoadIdentity();
    float cy = cosf(cam_yaw*DEG), sy = sinf(cam_yaw*DEG);
    float cp = cosf(cam_pitch*DEG), sp = sinf(cam_pitch*DEG);
    float ex = cam_target[0] + cam_dist*cp*cy;
    float ey = cam_target[1] + cam_dist*cp*sy;
    float ez = cam_target[2] + cam_dist*sp;
    gluLookAt(ex, ey, ez, cam_target[0], cam_target[1], cam_target[2], 0, 0, 1);

    // directional light fixed in world space (set after the camera transform)
    GLfloat lpos[4] = { 0.4f, -0.3f, 1.0f, 0.0f };
    glLightfv(GL_LIGHT0, GL_POSITION, lpos);

    // --- unlit reference geometry (grid + envelope) ---
    glDisable(GL_LIGHTING);
    glColor3f(0.25f, 0.27f, 0.30f);
    grid(g.env_min[0], g.env_min[1], g.env_max[0], g.env_max[1], g.spoil_z, 50.0f);
    glColor3f(0.40f, 0.42f, 0.46f);
    box_wire(g.env_min, g.env_max);

    // --- lit solids ---
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    GLfloat amb[4] = { 0.35f, 0.35f, 0.38f, 1.0f };
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, amb);

    if(g.have_fixtures) {
        // spoilboard: thin slab just under the plane so the stock visibly sits on it
        float sp_mn[3] = { g.env_min[0], g.env_min[1], g.spoil_z - 6.0f };
        float sp_mx[3] = { g.env_max[0], g.env_max[1], g.spoil_z };
        glColor3f(0.30f, 0.22f, 0.14f);                 // brown spoilboard
        box_solid(sp_mn, sp_mx);
        glColor3f(0.80f, 0.66f, 0.40f);                 // tan stock
        box_solid(g.stock_min, g.stock_max);
        glColor3f(0.55f, 0.58f, 0.62f);                 // grey toolsetter puck
        cylinder((g.puck_min[0]+g.puck_max[0])*0.5f, (g.puck_min[1]+g.puck_max[1])*0.5f,
                 g.puck_min[2], g.puck_max[2], (g.puck_max[0]-g.puck_min[0])*0.5f);
    }

    glColor3f(0.95f, 0.35f, 0.20f);                     // tool cone
    tool_cone(t[0], t[1], t[2]);

    SwapBuffers(hdc);
}

// ---- window / GL setup ---------------------------------------------------------------------------

static LRESULT CALLBACK wndproc (HWND h, UINT msg, WPARAM wp, LPARAM lp)
{
    switch(msg) {
        case WM_CLOSE:
            running = 0;                                // close view only; the simulator keeps running
            return 0;
        case WM_LBUTTONDOWN:
            dragging = 1; last_mx = (short)LOWORD(lp); last_my = (short)HIWORD(lp); SetCapture(h);
            return 0;
        case WM_LBUTTONUP:
            dragging = 0; ReleaseCapture();
            return 0;
        case WM_MOUSEMOVE:
            if(dragging) {
                int mx = (short)LOWORD(lp), my = (short)HIWORD(lp);
                cam_yaw   += (mx - last_mx) * 0.4f;
                cam_pitch += (my - last_my) * 0.4f;
                if(cam_pitch < 5.0f)  cam_pitch = 5.0f;
                if(cam_pitch > 85.0f) cam_pitch = 85.0f;
                last_mx = mx; last_my = my;
            }
            return 0;
        case WM_MOUSEWHEEL:
            cam_dist *= (GET_WHEEL_DELTA_WPARAM(wp) > 0) ? 0.9f : 1.1f;
            if(cam_dist < 10.0f) cam_dist = 10.0f;
            return 0;
    }
    return DefWindowProc(h, msg, wp, lp);
}

static int gl_create (void)
{
    WNDCLASSA wc;
    memset(&wc, 0, sizeof(wc));
    wc.lpfnWndProc = wndproc;
    wc.hInstance = GetModuleHandle(NULL);
    wc.lpszClassName = "grblHALSimView";
    wc.hCursor = LoadCursor(NULL, IDC_ARROW);
    RegisterClassA(&wc);

    hwnd = CreateWindowA("grblHALSimView", "grblHAL_sim - 3D machine view",
                         WS_OVERLAPPEDWINDOW | WS_VISIBLE, CW_USEDEFAULT, CW_USEDEFAULT,
                         960, 720, NULL, NULL, wc.hInstance, NULL);
    if(!hwnd)
        return 0;

    hdc = GetDC(hwnd);

    PIXELFORMATDESCRIPTOR pfd;
    memset(&pfd, 0, sizeof(pfd));
    pfd.nSize = sizeof(pfd);
    pfd.nVersion = 1;
    pfd.dwFlags = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER;
    pfd.iPixelType = PFD_TYPE_RGBA;
    pfd.cColorBits = 32;
    pfd.cDepthBits = 24;
    int pf = ChoosePixelFormat(hdc, &pfd);
    SetPixelFormat(hdc, pf, &pfd);

    hglrc = wglCreateContext(hdc);
    wglMakeCurrent(hdc, hglrc);
    return 1;
}

static DWORD WINAPI view_thread (LPVOID arg)
{
    (void)arg;
    if(!gl_create()) {
        fprintf(stderr, "view: could not create the 3D window\n");
        running = 0;
        return 0;
    }
    fprintf(stderr, "view: 3D machine view opened\n");
    quad = gluNewQuadric();
    gluQuadricNormals(quad, GLU_SMOOTH);

    while(running) {
        MSG m;
        while(PeekMessage(&m, NULL, 0, 0, PM_REMOVE)) {
            TranslateMessage(&m);
            DispatchMessage(&m);
        }
        render();
        Sleep(16);                                      // ~60 fps
    }

    wglMakeCurrent(NULL, NULL);
    if(hglrc) wglDeleteContext(hglrc);
    if(hwnd)  DestroyWindow(hwnd);
    hwnd = NULL;
    return 0;
}

void sim_view_start (void)
{
    if(running)
        return;
    InitializeCriticalSection(&lock);
    memset(&geom, 0, sizeof(geom));
    running = 1;
    CreateThread(NULL, 0, view_thread, NULL, 0, NULL);
}

#else  // ---- non-Windows: no-op stubs --------------------------------------------------------------

bool sim_view_active (void) { return false; }
void sim_view_start (void) {}
void sim_view_set_geometry (const sim_view_geometry_t *g) { (void)g; }
void sim_view_set_tool (float x, float y, float z) { (void)x; (void)y; (void)z; }

#endif

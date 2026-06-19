/*
  sim_view.c - optional 3D machine view for the grblHAL simulator (-view).

  Win32 + OpenGL (immediate mode), no external dependencies beyond the system
  opengl32 / glu32 / gdi32 libraries. Runs on its own thread that owns the window
  and a snapshot of the machine geometry + live tool position; the grbl and socket
  threads are untouched and only push data in via the setters below.
*/

#include "sim_view.h"
#include "sim_setup.h"

#ifdef _WIN32

#include <windows.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>

#define DEG (3.14159265f / 180.0f)

#define ID_SETTINGS 1001
#define ID_FORMAT   1002
#define ID_SHOWLOG  1003

extern void sim_request_format_reboot (void);   // main.c

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

// 2D overlay: a bitmap font, the latest [MSG:...] line. Buttons live in the window menu bar.
static GLuint font_base = 0;
static int    char_w = 8, char_h = 16;
static char   message[160] = "";
static HMENU  hmenu;

// Show Log opens a dedicated window holding the action log (reliable, unlike toggling the console).
static HWND   log_hwnd = NULL, log_edit = NULL;
static char   logbuf[20000];
static int    loglen = 0;
static volatile int logdirty = 0;

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

void sim_view_set_message (const char *s)
{
    if(!running)
        return;
    const char *p = s;
    if(strncmp(p, "[MSG:", 5) == 0)             // strip the [MSG:...] wrapper for display
        p += 5;
    EnterCriticalSection(&lock);
    strncpy(message, p, sizeof(message) - 1);
    message[sizeof(message) - 1] = '\0';
    size_t n = strlen(message);
    if(n && message[n - 1] == ']')
        message[n - 1] = '\0';
    LeaveCriticalSection(&lock);
}

void sim_view_log_append (const char *s)
{
    if(!running)
        return;
    int sl = (int)strlen(s);
    if(sl > (int)sizeof(logbuf) - 3)
        sl = (int)sizeof(logbuf) - 3;
    EnterCriticalSection(&lock);
    if(loglen + sl + 3 >= (int)sizeof(logbuf)) {        // make room by dropping from the front
        int drop = loglen + sl + 3 - (int)sizeof(logbuf);
        if(drop > loglen) drop = loglen;
        memmove(logbuf, logbuf + drop, loglen - drop);
        loglen -= drop;
    }
    memcpy(logbuf + loglen, s, sl); loglen += sl;
    logbuf[loglen++] = '\r'; logbuf[loglen++] = '\n'; logbuf[loglen] = '\0';
    logdirty = 1;
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

// Tool: a funnel standing tip-down on the controlled point - a thin spout at the tip widening to a wide
// bowl, so it reads clearly and the contact point is obvious.
static void tool_funnel (float x, float y, float z)
{
    glPushMatrix();
    glTranslatef(x, y, z);
    gluCylinder(quad, 2.0, 2.0, 14.0, 16, 1);    // spout: thin tube standing on the tip
    gluDisk(quad, 0.0, 2.0, 16, 1);              // close the bottom of the spout
    glTranslatef(0, 0, 14.0);
    gluCylinder(quad, 2.0, 18.0, 34.0, 28, 1);    // bowl: widens from the spout to a wide rim
    glPopMatrix();
}

// Draw a string with the bitmap font at screen (ortho) position (x,y) = lower-left of the text.
static void text2d (int x, int y, const char *s)
{
    glRasterPos2i(x, y);
    glListBase(font_base);
    glCallLists((GLsizei)strlen(s), GL_UNSIGNED_BYTE, (const GLubyte *)s);
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
    char msg[160];
    EnterCriticalSection(&lock);
    g = geom; t[0] = tool[0]; t[1] = tool[1]; t[2] = tool[2];
    strncpy(msg, message, sizeof msg); msg[sizeof msg - 1] = '\0';
    LeaveCriticalSection(&lock);

    frame_camera(&g);

    RECT rc; GetClientRect(hwnd, &rc);
    int w = rc.right, h = rc.bottom; if(h < 1) h = 1;
    glViewport(0, 0, w, h);
    glClearColor(0.90f, 0.90f, 0.92f, 1.0f);            // light background
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
    glColor3f(0.62f, 0.63f, 0.66f);
    grid(g.env_min[0], g.env_min[1], g.env_max[0], g.env_max[1], g.spoil_z, 50.0f);
    glColor3f(0.40f, 0.40f, 0.45f);
    box_wire(g.env_min, g.env_max);

    // --- lit solids ---
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    GLfloat amb[4] = { 0.45f, 0.45f, 0.48f, 1.0f };
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, amb);

    if(g.have_fixtures) {
        // spoilboard: thin slab just under the plane so the stock visibly sits on it
        float sp_mn[3] = { g.env_min[0], g.env_min[1], g.spoil_z - 6.0f };
        float sp_mx[3] = { g.env_max[0], g.env_max[1], g.spoil_z };
        glColor3f(0.34f, 0.25f, 0.16f);                 // brown spoilboard
        box_solid(sp_mn, sp_mx);
        glColor3f(0.82f, 0.68f, 0.42f);                 // tan stock
        box_solid(g.stock_min, g.stock_max);
        glColor3f(0.50f, 0.53f, 0.58f);                 // grey toolsetter puck
        cylinder((g.puck_min[0]+g.puck_max[0])*0.5f, (g.puck_min[1]+g.puck_max[1])*0.5f,
                 g.puck_min[2], g.puck_max[2], (g.puck_max[0]-g.puck_min[0])*0.5f);
    }

    glColor3f(0.90f, 0.12f, 0.12f);                     // tool (bright red funnel)
    tool_funnel(t[0], t[1], t[2]);

    // --- 2D overlay: machine position (top-right), MSG status (bottom), Show Log button (top-left) ---
    glDisable(GL_LIGHTING);
    glDisable(GL_DEPTH_TEST);
    glMatrixMode(GL_PROJECTION); glPushMatrix(); glLoadIdentity(); gluOrtho2D(0, w, 0, h);
    glMatrixMode(GL_MODELVIEW);  glPushMatrix(); glLoadIdentity();

    char line[64];
    int rx = w - 15 * char_w;
    glColor3f(0.08f, 0.08f, 0.10f);
    snprintf(line, sizeof line, "X %10.3f", t[0]); text2d(rx, h - char_h - 6,      line);
    snprintf(line, sizeof line, "Y %10.3f", t[1]); text2d(rx, h - 2*char_h - 8,    line);
    snprintf(line, sizeof line, "Z %10.3f", t[2]); text2d(rx, h - 3*char_h - 10,   line);

    if(msg[0]) {
        glColor3f(0.05f, 0.22f, 0.55f);
        text2d(10, 8, msg);
    }

    glMatrixMode(GL_PROJECTION); glPopMatrix();
    glMatrixMode(GL_MODELVIEW);  glPopMatrix();

    SwapBuffers(hdc);
}

// ---- settings dialog -----------------------------------------------------------------------------
// A code-built (no .rc resource) modal dialog that edits the live -setup fixture values. The field table
// maps each editable value to its offset in sim_setup_values_t so the rows are generated in a loop.

static const struct { const char *label; size_t off; } setup_fields[] = {
    { "Spoilboard Z",      offsetof(sim_setup_values_t, spoilboard_z)      },
    { "Stock corner X",    offsetof(sim_setup_values_t, stock_corner_x)    },
    { "Stock corner Y",    offsetof(sim_setup_values_t, stock_corner_y)    },
    { "Stock size X",      offsetof(sim_setup_values_t, stock_size_x)      },
    { "Stock size Y",      offsetof(sim_setup_values_t, stock_size_y)      },
    { "Stock size Z",      offsetof(sim_setup_values_t, stock_size_z)      },
    { "Toolsetter X",      offsetof(sim_setup_values_t, toolsetter_x)      },
    { "Toolsetter Y",      offsetof(sim_setup_values_t, toolsetter_y)      },
    { "Toolsetter height", offsetof(sim_setup_values_t, toolsetter_height) },
    { "Toolchange X",      offsetof(sim_setup_values_t, toolchange_x)      },
    { "Toolchange Y",      offsetof(sim_setup_values_t, toolchange_y)      },
};
#define N_SETUP_FIELDS ((int)(sizeof(setup_fields) / sizeof(setup_fields[0])))
#define IDC_SAVE   200
#define IDC_CANCEL 201

static HWND cfg_hwnd = NULL, cfg_edit[N_SETUP_FIELDS];
static int  cfg_done = 0;            // 0 = open, 1 = save, 2 = cancel

static LRESULT CALLBACK cfgproc (HWND h, UINT msg, WPARAM wp, LPARAM lp)
{
    if(msg == WM_COMMAND) {
        int id = LOWORD(wp);
        if(id == IDC_SAVE)        cfg_done = 1;
        else if(id == IDC_CANCEL) cfg_done = 2;
        return 0;
    }
    if(msg == WM_CLOSE) { cfg_done = 2; return 0; }
    return DefWindowProc(h, msg, wp, lp);
}

static void settings_dialog_show (void)
{
    sim_setup_values_t v;
    if(!sim_setup_get_values(&v)) {
        MessageBoxA(hwnd, "No fixture setup is active.\nLaunch the simulator with -setup <file> to define one.",
                    "Settings", MB_OK | MB_ICONINFORMATION);
        return;
    }

    static int registered = 0;
    HINSTANCE hi = GetModuleHandle(NULL);
    if(!registered) {
        WNDCLASSA wc; memset(&wc, 0, sizeof wc);
        wc.lpfnWndProc = cfgproc;
        wc.hInstance = hi;
        wc.lpszClassName = "grblHALSimSettings";
        wc.hCursor = LoadCursor(NULL, IDC_ARROW);
        wc.hbrBackground = (HBRUSH)(COLOR_BTNFACE + 1);
        RegisterClassA(&wc);
        registered = 1;
    }

    const int rowh = 26, top = 14, lblw = 130, editw = 90, pad = 14;
    int clientw = pad + lblw + 8 + editw + pad;
    int clienth = top + N_SETUP_FIELDS * rowh + 12 + 30 + pad;
    RECT r = { 0, 0, clientw, clienth };
    DWORD style = WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU;
    AdjustWindowRect(&r, style, FALSE);
    cfg_hwnd = CreateWindowExA(WS_EX_CONTROLPARENT | WS_EX_DLGMODALFRAME, "grblHALSimSettings",
                               "Fixture settings", style, CW_USEDEFAULT, CW_USEDEFAULT,
                               r.right - r.left, r.bottom - r.top, hwnd, NULL, hi, NULL);

    HFONT f = (HFONT)GetStockObject(DEFAULT_GUI_FONT);
    for(int i = 0; i < N_SETUP_FIELDS; i++) {
        int y = top + i * rowh;
        HWND lbl = CreateWindowA("STATIC", setup_fields[i].label, WS_CHILD | WS_VISIBLE | SS_RIGHT,
                                 pad, y + 3, lblw, 18, cfg_hwnd, NULL, hi, NULL);
        char txt[32];
        snprintf(txt, sizeof txt, "%.3f", *(float *)((char *)&v + setup_fields[i].off));
        cfg_edit[i] = CreateWindowA("EDIT", txt, WS_CHILD | WS_VISIBLE | WS_BORDER | WS_TABSTOP | ES_AUTOHSCROLL,
                                    pad + lblw + 8, y, editw, 20, cfg_hwnd, NULL, hi, NULL);
        SendMessageA(lbl, WM_SETFONT, (WPARAM)f, TRUE);
        SendMessageA(cfg_edit[i], WM_SETFONT, (WPARAM)f, TRUE);
    }
    int by = top + N_SETUP_FIELDS * rowh + 8;
    HWND bs = CreateWindowA("BUTTON", "Save", WS_CHILD | WS_VISIBLE | WS_TABSTOP | BS_DEFPUSHBUTTON,
                            clientw - pad - 2 * 80 - 8, by, 80, 26, cfg_hwnd, (HMENU)IDC_SAVE, hi, NULL);
    HWND bc = CreateWindowA("BUTTON", "Cancel", WS_CHILD | WS_VISIBLE | WS_TABSTOP,
                            clientw - pad - 80, by, 80, 26, cfg_hwnd, (HMENU)IDC_CANCEL, hi, NULL);
    SendMessageA(bs, WM_SETFONT, (WPARAM)f, TRUE);
    SendMessageA(bc, WM_SETFONT, (WPARAM)f, TRUE);

    cfg_done = 0;
    EnableWindow(hwnd, FALSE);                          // modal with respect to the 3D window
    ShowWindow(cfg_hwnd, SW_SHOW);
    SetForegroundWindow(cfg_hwnd);

    MSG m;                                              // nested modal pump (the 3D view pauses meanwhile)
    while(cfg_done == 0 && GetMessage(&m, NULL, 0, 0)) {
        if(!IsDialogMessage(cfg_hwnd, &m)) {
            TranslateMessage(&m);
            DispatchMessage(&m);
        }
    }

    if(cfg_done == 1) {                                 // Save: read the fields back and apply
        for(int i = 0; i < N_SETUP_FIELDS; i++) {
            char txt[32];
            GetWindowTextA(cfg_edit[i], txt, sizeof txt);
            *(float *)((char *)&v + setup_fields[i].off) = (float)atof(txt);
        }
        sim_setup_set_values(&v);
    }

    EnableWindow(hwnd, TRUE);
    DestroyWindow(cfg_hwnd);
    cfg_hwnd = NULL;
    SetForegroundWindow(hwnd);
}

// ---- log window ----------------------------------------------------------------------------------

static void log_refresh (void)
{
    static char buf[sizeof(logbuf)];
    if(!log_edit)
        return;
    EnterCriticalSection(&lock);
    memcpy(buf, logbuf, loglen + 1);
    logdirty = 0;
    LeaveCriticalSection(&lock);
    SetWindowTextA(log_edit, buf);
    int n = GetWindowTextLengthA(log_edit);             // keep the newest lines in view
    SendMessageA(log_edit, EM_SETSEL, n, n);
    SendMessageA(log_edit, EM_SCROLLCARET, 0, 0);
}

static LRESULT CALLBACK logproc (HWND h, UINT msg, WPARAM wp, LPARAM lp)
{
    if(msg == WM_SIZE) { if(log_edit) MoveWindow(log_edit, 0, 0, LOWORD(lp), HIWORD(lp), TRUE); return 0; }
    if(msg == WM_CLOSE) { ShowWindow(h, SW_HIDE); return 0; }   // hide (keep it for next time)
    return DefWindowProc(h, msg, wp, lp);
}

static void log_window_show (void)
{
    if(!log_hwnd) {
        WNDCLASSA wc;
        memset(&wc, 0, sizeof wc);
        wc.lpfnWndProc = logproc;
        wc.hInstance = GetModuleHandle(NULL);
        wc.lpszClassName = "grblHALSimLog";
        wc.hCursor = LoadCursor(NULL, IDC_ARROW);
        wc.hbrBackground = (HBRUSH)(COLOR_WINDOW + 1);
        RegisterClassA(&wc);
        log_hwnd = CreateWindowA("grblHALSimLog", "grblHAL_sim - action log", WS_OVERLAPPEDWINDOW,
                                 CW_USEDEFAULT, CW_USEDEFAULT, 760, 480, NULL, NULL, wc.hInstance, NULL);
        log_edit = CreateWindowA("EDIT", "",
                                 WS_CHILD | WS_VISIBLE | WS_VSCROLL | ES_MULTILINE | ES_READONLY | ES_AUTOVSCROLL,
                                 0, 0, 760, 480, log_hwnd, NULL, GetModuleHandle(NULL), NULL);
        HFONT f = CreateFontA(-13, 0, 0, 0, FW_NORMAL, FALSE, FALSE, FALSE, ANSI_CHARSET, OUT_TT_PRECIS,
                              CLIP_DEFAULT_PRECIS, CLEARTYPE_QUALITY, FIXED_PITCH | FF_MODERN, "Consolas");
        SendMessageA(log_edit, WM_SETFONT, (WPARAM)f, TRUE);
    }
    log_refresh();
    ShowWindow(log_hwnd, SW_SHOW);
    SetForegroundWindow(log_hwnd);
}

// ---- window / GL setup ---------------------------------------------------------------------------

static LRESULT CALLBACK wndproc (HWND h, UINT msg, WPARAM wp, LPARAM lp)
{
    switch(msg) {
        case WM_CLOSE:
            // The 3D window is the standalone sim's UI - closing it shuts the simulator down so the socket
            // drops and the connected sender (ioSender) sees the controller go away. exit() runs the atexit
            // hooks (e.g. the EEPROM save). running=0 first lets the render thread unwind cleanly.
            running = 0;
            exit(0);
            return 0;
        case WM_LBUTTONDOWN:
            dragging = 1; last_mx = (short)LOWORD(lp); last_my = (short)HIWORD(lp); SetCapture(h);
            return 0;

        case WM_COMMAND:
            switch(LOWORD(wp)) {
                case ID_SHOWLOG:
                    log_window_show();
                    return 0;
                case ID_FORMAT:
                    if(MessageBoxA(h, "Wipe the simulator filesystem (littlefs) and restart?\n\n"
                                      "Uploaded macros and ATC state will be cleared.",
                                   "Format filesystem", MB_OKCANCEL | MB_ICONWARNING) == IDOK)
                        sim_request_format_reboot();
                    return 0;
                case ID_SETTINGS:
                    settings_dialog_show();
                    return 0;
            }
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

    // Menu bar: top-level command items (no sub-menus) - clicking each sends WM_COMMAND.
    hmenu = CreateMenu();
    AppendMenuA(hmenu, MF_STRING, ID_SETTINGS, "Settings");
    AppendMenuA(hmenu, MF_STRING, ID_FORMAT,   "Format");
    AppendMenuA(hmenu, MF_STRING, ID_SHOWLOG,  "Show Log");
    SetMenu(hwnd, hmenu);

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

    // Build a bitmap font for the 2D overlay (display lists 0..255 for the current DC font).
    HFONT hfont = CreateFontA(-15, 0, 0, 0, FW_NORMAL, FALSE, FALSE, FALSE, ANSI_CHARSET,
                              OUT_TT_PRECIS, CLIP_DEFAULT_PRECIS, CLEARTYPE_QUALITY,
                              FIXED_PITCH | FF_MODERN, "Consolas");
    SelectObject(hdc, hfont);
    font_base = glGenLists(256);
    wglUseFontBitmaps(hdc, 0, 256, font_base);
    TEXTMETRICA tm;
    if(GetTextMetricsA(hdc, &tm)) { char_w = tm.tmAveCharWidth; char_h = tm.tmHeight; }

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

    // Hide the console when we own it (e.g. launched from Explorer) - the action log is shown on demand
    // via the Show Log menu window. A console shared with a shell is left alone.
    {
        HWND con = GetConsoleWindow();
        DWORD pids[2];
        if(con && GetConsoleProcessList(pids, 2) == 1)
            ShowWindow(con, SW_HIDE);
    }

    while(running) {
        MSG m;
        while(PeekMessage(&m, NULL, 0, 0, PM_REMOVE)) {
            TranslateMessage(&m);
            DispatchMessage(&m);
        }
        render();
        if(log_hwnd && logdirty && IsWindowVisible(log_hwnd))   // live-update the log window if open
            log_refresh();
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
void sim_view_set_message (const char *s) { (void)s; }
void sim_view_log_append (const char *s) { (void)s; }

#endif

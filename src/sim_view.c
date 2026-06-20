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
#define ID_RESETSTOCK 1004
#define ID_FLIPX    1005
#define ID_FLIPY    1006

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

// ---- stock material-removal heightmap (dexel model) ----------------------------------------------
// The stock is a grid of Z-heights over its XY footprint. As the cutter passes through, cells within the
// tool radius are lowered to the cutter's bottom profile (flat / ball / V), giving a true 3-axis carve.
// The heightmap is owned entirely by the render thread: the grbl/realtime thread only publishes the live
// tool tip + cutter geometry (cheap stores under `lock`), and the render thread carves the swept path each
// frame. This keeps all carving cost off the realtime loop (which must stay responsive to keep the planner
// fed) and means the heightmap itself never needs locking.
static float *hmap = NULL;                               // heightmap (render thread only)
static unsigned char *hmap_tool = NULL;                 // per-cell cutter shape that last cut it (for colour)
static int    hm_nx = 0, hm_ny = 0;
static float  hm_x0, hm_y0, hm_cell, hm_top, hm_bot;
static int    geom_dirty = 0;                            // set by set_geometry (grbl), consumed by render
static int    stock_reset = 0;                           // set by reset_stock, consumed by render
static float  cut_dia = 0.0f, cut_vangle = 0.0f;         // active cutter geometry (published under lock)
static int    cut_shape = SIM_TOOL_FLAT, cut_tool = -1;
static float  carve_lx, carve_ly, carve_lz;              // render thread's carve cursor (last carved tip)
static int    carve_have_last = 0;
static long   carve_count = 0;                           // cells lowered (diagnostic)

// Render the stock through a GL display list rebuilt only when the heightmap changes, so a fine grid stays
// cheap: the geometry is compiled once (GPU-resident) and camera moves just replay it with glCallList;
// carving recompiles it, throttled to a few times/second.
static GLuint stock_list = 0;
static int    hm_render_dirty = 1, hm_render_frames = 999;

// ---- data feed (called from the grbl/realtime threads) -------------------------------------------

bool sim_view_active (void)
{
    return running != 0;
}

// Window title (from the setup "description", e.g. "Mega V XL") plus the build timestamp, so it is obvious
// at a glance which build is running. Stored even before the window exists so gl_create() can apply it;
// updated live when edited in the Settings dialog.
#define SIM_BUILD_STAMP "built " __DATE__ " " __TIME__
static char view_title[140] = "grblHAL_sim  -  " SIM_BUILD_STAMP;

void sim_view_set_title (const char *s)
{
    if(s && *s)
        snprintf(view_title, sizeof(view_title), "%s  -  %s", s, SIM_BUILD_STAMP);
    if(hwnd)
        SetWindowTextA(hwnd, view_title);
}

// ---- setters (grbl/realtime thread) - cheap publishes only; the render thread does the heavy carving ----

void sim_view_set_geometry (const sim_view_geometry_t *g)
{
    if(!running)
        return;
    EnterCriticalSection(&lock);
    geom = *g;
    geom_dirty = 1;                                     // the render thread (re)builds the heightmap
    LeaveCriticalSection(&lock);
}

void sim_view_reset_stock (void)
{
    if(!running)
        return;
    EnterCriticalSection(&lock);
    stock_reset = 1;                                    // the render thread restores an uncut block
    LeaveCriticalSection(&lock);
}

void sim_view_set_tool_geometry (float diameter, int shape, float vangle, int tool)
{
    if(!running)
        return;
    EnterCriticalSection(&lock);
    cut_dia = diameter; cut_shape = shape; cut_vangle = vangle; cut_tool = tool;
    LeaveCriticalSection(&lock);
}

void sim_view_set_tool (float x, float y, float z)
{
    if(!running)
        return;
    EnterCriticalSection(&lock);
    tool[0] = x; tool[1] = y; tool[2] = z;              // just publish the tip; carving happens in render()
    LeaveCriticalSection(&lock);
}

// ---- heightmap carve (render thread) -------------------------------------------------------------

// (Re)allocate the stock heightmap for the given fixture geometry and fill it with an uncut block. Cell
// size targets ~0.5 mm but the grid is capped so a large stock stays cheap to draw. Render thread only.
static void heightmap_alloc (const sim_view_geometry_t *g)
{
    float spanx = g->stock_max[0] - g->stock_min[0];
    float spany = g->stock_max[1] - g->stock_min[1];
    if(spanx <= 0.0f || spany <= 0.0f) {
        free(hmap); hmap = NULL; hm_nx = hm_ny = 0;
        return;
    }
    // Cell size from the setup (Settings dialog), clamped to a sane range; 0 -> 1 mm default. The grid is
    // also capped (MAXGRID per axis) so a tiny cell on a big stock can't explode the cell count.
    const int MAXGRID = 600;
    float cell = g->cell_size > 0.0f ? g->cell_size : 1.0f;
    if(cell < 0.2f) cell = 0.2f;
    if(cell > 20.0f) cell = 20.0f;
    float mincell = fmaxf(spanx, spany) / MAXGRID;      // smallest cell that keeps both axes <= MAXGRID
    if(cell < mincell) cell = mincell;
    int nx = (int)ceilf(spanx / cell), ny = (int)ceilf(spany / cell);
    if(nx < 1) nx = 1; if(ny < 1) ny = 1;

    if(nx != hm_nx || ny != hm_ny) {                    // (re)size the buffers
        free(hmap); free(hmap_tool);
        hmap = (float *)malloc((size_t)nx * ny * sizeof(float));
        hmap_tool = (unsigned char *)malloc((size_t)nx * ny);
    }
    hm_nx = nx; hm_ny = ny;
    hm_x0 = g->stock_min[0]; hm_y0 = g->stock_min[1];
    hm_cell = cell;
    hm_top = g->stock_max[2]; hm_bot = g->stock_min[2];

    if(hmap)
        for(int i = 0; i < nx * ny; i++)
            hmap[i] = hm_top;
    if(hmap_tool)
        memset(hmap_tool, 0, (size_t)nx * ny);
    hm_render_dirty = 1;
}

// Lower the heightmap cells the cutter (centred at x,y, tip at z; radius r) currently overlaps.
static void carve_at (float x, float y, float z, float r, int shape, float tanhalf)
{
    if(z >= hm_top)
        return;

    int ix0 = (int)floorf((x - r - hm_x0) / hm_cell), ix1 = (int)ceilf((x + r - hm_x0) / hm_cell);
    int iy0 = (int)floorf((y - r - hm_y0) / hm_cell), iy1 = (int)ceilf((y + r - hm_y0) / hm_cell);
    if(ix0 < 0) ix0 = 0; if(iy0 < 0) iy0 = 0;
    if(ix1 >= hm_nx) ix1 = hm_nx - 1; if(iy1 >= hm_ny) iy1 = hm_ny - 1;

    for(int iy = iy0; iy <= iy1; iy++) {
        float cy = hm_y0 + (iy + 0.5f) * hm_cell;
        for(int ix = ix0; ix <= ix1; ix++) {
            float cx = hm_x0 + (ix + 0.5f) * hm_cell;
            float d2 = (cx - x) * (cx - x) + (cy - y) * (cy - y);
            if(d2 > r * r)
                continue;
            float th = z;                               // flat endmill: bottom is a plane at z
            if(shape == SIM_TOOL_BALL)
                th = z + (r - sqrtf(r * r - d2));        // ball-nose: bottom curves up off-axis
            else if(tanhalf > 0.0f)
                th = z + sqrtf(d2) / tanhalf;            // V-bit cone
            if(th < hm_bot) th = hm_bot;                 // never cut below the spoilboard
            int idx = iy * hm_nx + ix;
            if(th < hmap[idx]) {
                hmap[idx] = th;
                if(hmap_tool) hmap_tool[idx] = (unsigned char)shape;   // colour by which cutter removed it
                carve_count++;
                hm_render_dirty = 1;
            }
        }
    }
}

// Carve the cutter's swept path from the last carved tip to the current one. Render thread, each frame.
static void heightmap_advance (float x, float y, float z, float dia, int shape, float vangle)
{
    float r = dia * 0.5f;
    if(hmap == NULL || r <= 0.0f) {
        carve_lx = x; carve_ly = y; carve_lz = z; carve_have_last = 1;
        return;
    }

    float tanhalf = 0.0f;                               // V-bit: tip rises by d/tan(halfangle) off-axis
    if(shape == SIM_TOOL_VBIT) {
        float half = vangle * 0.5f;
        if(half > 1.0f && half < 89.0f)
            tanhalf = tanf(half * (3.14159265f / 180.0f));
    }

    if(carve_have_last && !(carve_lz >= hm_top && z >= hm_top)) {   // skip rapids entirely above the stock
        float dx = x - carve_lx, dy = y - carve_ly, dz = z - carve_lz;
        float dist = sqrtf(dx * dx + dy * dy);
        int steps = (int)(dist / (hm_cell * 0.5f)) + 1;
        if(steps > 20000) steps = 20000;
        for(int s = 1; s <= steps; s++) {
            float fr = (float)s / steps;
            carve_at(carve_lx + dx * fr, carve_ly + dy * fr, carve_lz + dz * fr, r, shape, tanhalf);
        }
    } else
        carve_at(x, y, z, r, shape, tanhalf);

    carve_lx = x; carve_ly = y; carve_lz = z; carve_have_last = 1;
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

// Colour a stock cell: tan where uncut; otherwise by which cutter removed it (flat/rough = grey, ball/fine
// = blue, V-bit = green), darkening toward near-black at the full cut depth so depth stays readable.
static void stock_color_rgb (float h, int shape, float *rgb)
{
    if(h >= hm_top - 0.02f) {
        rgb[0] = 0.82f; rgb[1] = 0.68f; rgb[2] = 0.42f;            // uncut stock (tan)
        return;
    }
    float span = hm_top - hm_bot;
    float frac = span > 0.0f ? (hm_top - h) / span : 1.0f;        // 0 just-cut .. 1 full depth
    if(frac > 1.0f) frac = 1.0f;
    float s = 1.0f - frac;                                        // 1 shallow .. 0 deep
    if(shape == SIM_TOOL_BALL) {                                  // ball / finish cut - blue
        rgb[0] = 0.08f + 0.18f * s; rgb[1] = 0.16f + 0.30f * s; rgb[2] = 0.26f + 0.42f * s;
    } else if(shape == SIM_TOOL_VBIT) {                           // V-bit - green
        rgb[0] = 0.08f + 0.18f * s; rgb[1] = 0.18f + 0.34f * s; rgb[2] = 0.10f + 0.18f * s;
    } else {                                                      // flat / rough cut - grey toward black
        rgb[0] = 0.10f + 0.28f * s; rgb[1] = 0.10f + 0.25f * s; rgb[2] = 0.12f + 0.22f * s;
    }
}

// Emit one flat-shaded quad (colour + normal + 4 verts). Must be called inside a glBegin(GL_QUADS).
static void gl_quad (float ax, float ay, float az, float bx, float by, float bz,
                     float cx, float cy, float cz, float dx, float dy, float dz,
                     float nx, float ny, float nz, const float *rgb)
{
    glColor3fv(rgb);
    glNormal3f(nx, ny, nz);
    glVertex3f(ax, ay, az); glVertex3f(bx, by, bz); glVertex3f(cx, cy, cz); glVertex3f(dx, dy, dz);
}

// Flip the stock over for double-sided machining (cut one side, flip, cut the other). The side already
// machined goes to the underside (not modelled by the single top heightmap) and a fresh flat top comes up
// to machine the second side - so this resets the surface to an uncut block. For a rectangular stock the
// result is the same whichever axis is chosen; the axis just records which way the operator flipped.
// Render thread (called from the menu handler, same thread). axis_y selects the X vs Y log label.
static void flip_stock (int axis_y)
{
    if(hmap == NULL || hm_nx == 0)
        return;
    for(int i = 0; i < hm_nx * hm_ny; i++)
        hmap[i] = hm_top;
    if(hmap_tool)
        memset(hmap_tool, 0, (size_t)hm_nx * hm_ny);
    carve_have_last = 0;
    hm_render_dirty = 1;
    sim_view_log_append(axis_y ? "stock: flipped on Y - fresh top for the second side"
                               : "stock: flipped on X - fresh top for the second side");
}

// Compile the carved stock into the display list: a flat top quad per cell, vertical walls where
// neighbouring cells differ (pocket walls), and an outer skirt down to the stock bottom. Carved areas are
// coloured by cutter + depth (stock_color_rgb). Called only when the heightmap changed (and throttled).
static void compile_stock_list (void)
{
    float c = hm_cell, xR = hm_x0 + hm_nx * c, yT = hm_y0 + hm_ny * c, col[3];
    #define TSHAPE(i) (hmap_tool ? (int)hmap_tool[i] : SIM_TOOL_FLAT)

    glNewList(stock_list, GL_COMPILE);
    glBegin(GL_QUADS);

    for(int iy = 0; iy < hm_ny; iy++) {                  // top surface
        float y0 = hm_y0 + iy * c, y1 = y0 + c;
        for(int ix = 0; ix < hm_nx; ix++) {
            int idx = iy * hm_nx + ix;
            float x0 = hm_x0 + ix * c, x1 = x0 + c, h = hmap[idx];
            stock_color_rgb(h, TSHAPE(idx), col);
            gl_quad(x0, y0, h, x1, y0, h, x1, y1, h, x0, y1, h, 0, 0, 1, col);
        }
    }
    for(int iy = 0; iy < hm_ny; iy++) {                  // interior pocket walls (colour from the lower cell)
        float y0 = hm_y0 + iy * c, y1 = y0 + c;
        for(int ix = 0; ix < hm_nx; ix++) {
            int idx = iy * hm_nx + ix;
            float x0 = hm_x0 + ix * c, x1 = x0 + c, h = hmap[idx];
            if(ix + 1 < hm_nx) {
                float hn = hmap[idx + 1];
                if(hn != h) { stock_color_rgb(fminf(h, hn), TSHAPE(h < hn ? idx : idx + 1), col);
                    gl_quad(x1, y0, h, x1, y1, h, x1, y1, hn, x1, y0, hn, 1, 0, 0, col); }
            }
            if(iy + 1 < hm_ny) {
                float hn = hmap[idx + hm_nx];
                if(hn != h) { stock_color_rgb(fminf(h, hn), TSHAPE(h < hn ? idx : idx + hm_nx), col);
                    gl_quad(x0, y1, h, x1, y1, h, x1, y1, hn, x0, y1, hn, 0, 1, 0, col); }
            }
        }
    }
    for(int ix = 0; ix < hm_nx; ix++) {                  // outer skirt down to the spoilboard
        float x0 = hm_x0 + ix * c, x1 = x0 + c;
        int bi = ix, ti = (hm_ny - 1) * hm_nx + ix;
        float hB = hmap[bi], hT = hmap[ti];
        stock_color_rgb(hB, TSHAPE(bi), col); gl_quad(x0, hm_y0, hm_bot, x1, hm_y0, hm_bot, x1, hm_y0, hB, x0, hm_y0, hB, 0, -1, 0, col);
        stock_color_rgb(hT, TSHAPE(ti), col); gl_quad(x0, yT, hT, x1, yT, hT, x1, yT, hm_bot, x0, yT, hm_bot, 0, 1, 0, col);
    }
    for(int iy = 0; iy < hm_ny; iy++) {
        float y0 = hm_y0 + iy * c, y1 = y0 + c;
        int li = iy * hm_nx, ri = iy * hm_nx + (hm_nx - 1);
        float hL = hmap[li], hR = hmap[ri];
        stock_color_rgb(hL, TSHAPE(li), col); gl_quad(hm_x0, y0, hL, hm_x0, y1, hL, hm_x0, y1, hm_bot, hm_x0, y0, hm_bot, -1, 0, 0, col);
        stock_color_rgb(hR, TSHAPE(ri), col); gl_quad(xR, y0, hm_bot, xR, y1, hm_bot, xR, y1, hR, xR, y0, hR, 1, 0, 0, col);
    }
    #undef TSHAPE

    glEnd();
    glEndList();
}

// Draw the carved stock. The display list is recompiled only when the heightmap changed, at most a few
// times per second; camera moves just replay it with glCallList, so a fine grid stays cheap.
static void draw_heightmap (void)
{
    if(hmap == NULL || hm_nx == 0)
        return;
    if(stock_list == 0)
        stock_list = glGenLists(1);
    if(stock_list && hm_render_dirty && ++hm_render_frames >= 3) {  // throttle (~20/s); carving stays full-rate
        compile_stock_list();
        hm_render_dirty = 0;
        hm_render_frames = 0;
    }
    if(stock_list)
        glCallList(stock_list);
}

static void render (void)
{
    sim_view_geometry_t g;
    float t[3], cd, cv;
    int cs, ct, gd, sr;
    char msg[160];
    EnterCriticalSection(&lock);                        // brief snapshot of the published state
    g = geom; t[0] = tool[0]; t[1] = tool[1]; t[2] = tool[2];
    cd = cut_dia; cs = cut_shape; cv = cut_vangle; ct = cut_tool;
    gd = geom_dirty; geom_dirty = 0;
    sr = stock_reset; stock_reset = 0;
    strncpy(msg, message, sizeof msg); msg[sizeof msg - 1] = '\0';
    LeaveCriticalSection(&lock);

    // Heightmap lives entirely on this (render) thread - (re)build on a geometry change, reset on request,
    // then carve the cutter's swept path up to the current tool position.
    if(gd) {
        if(g.have_fixtures)
            heightmap_alloc(&g);
        else { free(hmap); hmap = NULL; hm_nx = hm_ny = 0; }
        carve_have_last = 0;
    }
    if(sr && hmap) {
        for(int i = 0; i < hm_nx * hm_ny; i++) hmap[i] = hm_top;
        if(hmap_tool) memset(hmap_tool, 0, (size_t)hm_nx * hm_ny);
        carve_have_last = 0;
        hm_render_dirty = 1;
    }
    heightmap_advance(t[0], t[1], t[2], cd, cs, cv);
    int have_stock = hmap != NULL && hm_nx > 0;

    // Carve status: surface why the stock is / isn't being removed. Shown persistently in the overlay
    // (below) and also logged to the action log (Show Log) on change, so the cause is never a mystery.
    int verdict;
    if(!have_stock)        verdict = 0;    // no stock heightmap
    else if(cd <= 0.0f)    verdict = 1;    // no cutter geometry
    else {
        float xR = hm_x0 + hm_nx * hm_cell, yT = hm_y0 + hm_ny * hm_cell;
        int over = t[0] >= hm_x0 && t[0] <= xR && t[1] >= hm_y0 && t[1] <= yT;
        verdict = !over ? 2 : (t[2] >= hm_top ? 3 : 4);   // off-stock / above / cutting
    }
    static const char *verdict_msg[] = {
        "no stock defined (need a -setup fixture)",
        "no cutter geometry - add a (TOOL T=n D=.. TYPE=..) comment",
        "tool is not over the stock - check the job's G54 work offset",
        "tool above the stock top (air move) - no cut yet",
        "cutting - removing stock" };
    const char *shape_name = cs == SIM_TOOL_BALL ? "BALL" : cs == SIM_TOOL_VBIT ? "VBIT" : "FLAT";
    char tool_label[40];
    if(cd > 0.0f && ct >= 0)
        snprintf(tool_label, sizeof tool_label, "T%d %s D%.2f", ct, shape_name, cd);
    else
        snprintf(tool_label, sizeof tool_label, "no active tool");

    static int last_verdict = -1;
    if(verdict != last_verdict) {                          // log only on change (no spam)
        last_verdict = verdict;
        char line[220];
        snprintf(line, sizeof line, "carve: %s  [%s  pos=(%.1f,%.1f,%.1f) cells=%ld]",
                 verdict_msg[verdict], tool_label, t[0], t[1], t[2], carve_count);
        fprintf(stderr, "%s\n", line);
        sim_view_log_append(line);
    }

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
        glColor3f(0.82f, 0.68f, 0.42f);                 // tan stock (carved heightmap, or solid if none yet)
        if(have_stock)
            draw_heightmap();
        else
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

    // Carve status, top-left: green while cutting, red when it can't (with the reason + active tool).
    if(verdict == 4) glColor3f(0.0f, 0.45f, 0.0f); else glColor3f(0.70f, 0.10f, 0.10f);
    char cl[140];
    snprintf(cl, sizeof cl, "carve: %s  [%s]", verdict_msg[verdict], tool_label);
    text2d(10, h - char_h - 6, cl);

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
    { "Cell size (mm)",    offsetof(sim_setup_values_t, resolution_mm)     },
};
#define N_SETUP_FIELDS ((int)(sizeof(setup_fields) / sizeof(setup_fields[0])))
#define IDC_SAVE   200
#define IDC_CANCEL 201

static HWND cfg_hwnd = NULL, cfg_edit[N_SETUP_FIELDS], cfg_desc = NULL;
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
    int descrow = 1;                                    // a Description text field above the numeric rows
    int clienth = top + (N_SETUP_FIELDS + descrow) * rowh + 12 + 30 + pad;
    RECT r = { 0, 0, clientw, clienth };
    DWORD style = WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU;
    AdjustWindowRect(&r, style, FALSE);
    cfg_hwnd = CreateWindowExA(WS_EX_CONTROLPARENT | WS_EX_DLGMODALFRAME, "grblHALSimSettings",
                               "Fixture settings", style, CW_USEDEFAULT, CW_USEDEFAULT,
                               r.right - r.left, r.bottom - r.top, hwnd, NULL, hi, NULL);

    HFONT f = (HFONT)GetStockObject(DEFAULT_GUI_FONT);

    HWND dlbl = CreateWindowA("STATIC", "Description", WS_CHILD | WS_VISIBLE | SS_RIGHT,
                              pad, top + 3, lblw, 18, cfg_hwnd, NULL, hi, NULL);
    cfg_desc = CreateWindowA("EDIT", v.description, WS_CHILD | WS_VISIBLE | WS_BORDER | WS_TABSTOP | ES_AUTOHSCROLL,
                             pad + lblw + 8, top, editw, 20, cfg_hwnd, NULL, hi, NULL);
    SendMessageA(dlbl, WM_SETFONT, (WPARAM)f, TRUE);
    SendMessageA(cfg_desc, WM_SETFONT, (WPARAM)f, TRUE);

    for(int i = 0; i < N_SETUP_FIELDS; i++) {
        int y = top + (i + descrow) * rowh;
        HWND lbl = CreateWindowA("STATIC", setup_fields[i].label, WS_CHILD | WS_VISIBLE | SS_RIGHT,
                                 pad, y + 3, lblw, 18, cfg_hwnd, NULL, hi, NULL);
        char txt[32];
        snprintf(txt, sizeof txt, "%.3f", *(float *)((char *)&v + setup_fields[i].off));
        cfg_edit[i] = CreateWindowA("EDIT", txt, WS_CHILD | WS_VISIBLE | WS_BORDER | WS_TABSTOP | ES_AUTOHSCROLL,
                                    pad + lblw + 8, y, editw, 20, cfg_hwnd, NULL, hi, NULL);
        SendMessageA(lbl, WM_SETFONT, (WPARAM)f, TRUE);
        SendMessageA(cfg_edit[i], WM_SETFONT, (WPARAM)f, TRUE);
    }
    int by = top + (N_SETUP_FIELDS + descrow) * rowh + 8;
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
        GetWindowTextA(cfg_desc, v.description, sizeof v.description);
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

// ---- window placement persistence (HKCU\Software\grblHALSim) --------------------------------------

#define SIM_REG_KEY "Software\\grblHALSim"

// Save a window's screen rectangle (x,y,w,h as 4 DWORDs) so it reopens where the user left it.
static void save_window_pos (const char *name, HWND h)
{
    RECT r;
    if(!h || IsIconic(h) || IsZoomed(h) || !GetWindowRect(h, &r))   // skip minimised/maximised
        return;
    HKEY k;
    if(RegCreateKeyExA(HKEY_CURRENT_USER, SIM_REG_KEY, 0, NULL, 0, KEY_WRITE, NULL, &k, NULL) == ERROR_SUCCESS) {
        DWORD v[4] = { (DWORD)r.left, (DWORD)r.top, (DWORD)(r.right - r.left), (DWORD)(r.bottom - r.top) };
        RegSetValueExA(k, name, 0, REG_BINARY, (const BYTE *)v, sizeof v);
        RegCloseKey(k);
    }
}

// Load a saved window rectangle; returns 0 (caller uses defaults) unless a sane, on-screen rect is found.
static int load_window_pos (const char *name, int *x, int *y, int *w, int *h)
{
    HKEY k;
    DWORD v[4], sz = sizeof v, type = 0;
    int ok = 0;
    if(RegOpenKeyExA(HKEY_CURRENT_USER, SIM_REG_KEY, 0, KEY_READ, &k) == ERROR_SUCCESS) {
        if(RegQueryValueExA(k, name, NULL, &type, (BYTE *)v, &sz) == ERROR_SUCCESS && sz == sizeof v && type == REG_BINARY)
            ok = 1;
        RegCloseKey(k);
    }
    if(!ok)
        return 0;
    *x = (int)v[0]; *y = (int)v[1]; *w = (int)v[2]; *h = (int)v[3];
    if(*w < 200 || *h < 150 || *w > 10000 || *h > 10000)            // sanity
        return 0;
    // Reject a rect whose title bar is entirely off the virtual desktop (e.g. a monitor was removed).
    int vx = GetSystemMetrics(SM_XVIRTUALSCREEN), vy = GetSystemMetrics(SM_YVIRTUALSCREEN);
    int vw = GetSystemMetrics(SM_CXVIRTUALSCREEN), vh = GetSystemMetrics(SM_CYVIRTUALSCREEN);
    if(*x + *w < vx + 40 || *x > vx + vw - 40 || *y < vy - 4 || *y > vy + vh - 30)
        return 0;
    return 1;
}

static LRESULT CALLBACK logproc (HWND h, UINT msg, WPARAM wp, LPARAM lp)
{
    if(msg == WM_SIZE) { if(log_edit) MoveWindow(log_edit, 0, 0, LOWORD(lp), HIWORD(lp), TRUE); return 0; }
    if(msg == WM_EXITSIZEMOVE) { save_window_pos("LogWindow", h); return 0; }    // moved/resized
    if(msg == WM_CLOSE) { save_window_pos("LogWindow", h); ShowWindow(h, SW_HIDE); return 0; }   // hide, keep
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
        int lx = CW_USEDEFAULT, ly = CW_USEDEFAULT, lw = 760, lh = 480, sx, sy, sw, sh;
        if(load_window_pos("LogWindow", &sx, &sy, &sw, &sh)) { lx = sx; ly = sy; lw = sw; lh = sh; }
        log_hwnd = CreateWindowA("grblHALSimLog", "grblHAL_sim - action log", WS_OVERLAPPEDWINDOW,
                                 lx, ly, lw, lh, NULL, NULL, wc.hInstance, NULL);
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
        case WM_EXITSIZEMOVE:
            save_window_pos("MainWindow", h);           // remember where the user put / sized it
            return 0;
        case WM_CLOSE:
            // The 3D window is the standalone sim's UI - closing it shuts the simulator down so the socket
            // drops and the connected sender (ioSender) sees the controller go away. exit() runs the atexit
            // hooks (e.g. the EEPROM save). running=0 first lets the render thread unwind cleanly.
            save_window_pos("MainWindow", h);
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
                case ID_RESETSTOCK:
                    sim_view_reset_stock();             // restore an uncut block (e.g. before a re-run)
                    return 0;
                case ID_FLIPX:
                    flip_stock(0);                      // flip about X (front/back) for the second side
                    return 0;
                case ID_FLIPY:
                    flip_stock(1);                      // flip about Y (left/right)
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

    int wx = CW_USEDEFAULT, wy = CW_USEDEFAULT, ww = 960, wh = 720, sx, sy, sw, sh;
    if(load_window_pos("MainWindow", &sx, &sy, &sw, &sh)) { wx = sx; wy = sy; ww = sw; wh = sh; }
    hwnd = CreateWindowA("grblHALSimView", view_title,
                         WS_OVERLAPPEDWINDOW | WS_VISIBLE, wx, wy, ww, wh, NULL, NULL, wc.hInstance, NULL);
    if(!hwnd)
        return 0;

    // Menu bar: top-level command items (no sub-menus) - clicking each sends WM_COMMAND.
    hmenu = CreateMenu();
    AppendMenuA(hmenu, MF_STRING, ID_SETTINGS,    "Settings");
    AppendMenuA(hmenu, MF_STRING, ID_RESETSTOCK,  "Reset Stock");
    AppendMenuA(hmenu, MF_STRING, ID_FLIPX,       "Flip Stock on X");
    AppendMenuA(hmenu, MF_STRING, ID_FLIPY,       "Flip Stock on Y");
    AppendMenuA(hmenu, MF_STRING, ID_FORMAT,      "Format LittleFS");
    AppendMenuA(hmenu, MF_STRING, ID_SHOWLOG,     "Show Log");
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
    sim_view_log_append("grblHAL_sim 3D view  -  " SIM_BUILD_STAMP);   // first Show Log line = build version
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
void sim_view_set_tool_geometry (float diameter, int shape, float vangle, int tool) { (void)diameter; (void)shape; (void)vangle; (void)tool; }
void sim_view_reset_stock (void) {}
void sim_view_set_title (const char *s) { (void)s; }

#endif

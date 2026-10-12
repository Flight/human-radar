/**
 * @file radar_ui.c
 * @brief Green phosphor radar scope on the 720x720 panel (LVGL)
 *
 * The static grid (range rings, the threat and breach zones, the sensor's
 * field of view, bearing ticks, crosshair) is rendered once at start-up into
 * three full-screen canvases, one per outer ring colour (green, amber, red),
 * because LVGL's software arcs are far too slow to redraw every frame. On top
 * sits a transparent full-screen object with a custom draw callback for the
 * moving parts: the sweep line with its afterglow (thin triangles fading
 * behind the line) and the blips that light up where the sweep paints a
 * person and fade until the next pass. An LVGL timer advances the sweep by
 * the clock, reads the shared target list and the scene state, updates the
 * text and triggers the contact blips. Everything runs inside the LVGL task,
 * so the draw callback only reads state and the timer owns all changes.
 */

#include <sdkconfig.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <esp_log.h>
#include <lvgl.h>
#include "bsp/esp-bsp.h"

#include "targets/targets.h"
#include "sound/sound.h"
#include "scene/scene.h"
#include "radar_ui.h"

static const char *TAG = "radar_ui";

/* Geometry (pixels) */
#define SCREEN_SIZE      720
#define SCOPE_CENTER_X   360
#define SCOPE_CENTER_Y   360
#define SCOPE_RADIUS     296 /* outer range ring */
#define RANGE_RINGS      4
#define BEARING_LABEL_R  322 /* centre of the bearing numbers, outside the ring */
#define TICK_LEN_MINOR   5
#define TICK_LEN_MAJOR   11
#define BLIP_RADIUS      7
#define BLIP_HALO_RADIUS 13
#define TRAIL_STEPS      12 /* triangles in the afterglow */
#define TEXT_MARGIN      12
#define SENSOR_FOV_DEG   60 /* the LD2450 sees +/-60 degrees */
#define RADAR_PI         3.14159265f

/* Timing */
#define STATUS_BLINK_MS   500
#define PANIC_BLINK_MS    250
#define FPS_LOG_PERIOD_MS 5000

/* Phosphor palette */
#define COLOR_BG       lv_color_hex(0x020A02)
#define COLOR_PHOSPHOR lv_color_hex(0x39FF14)
#define COLOR_GRID     lv_color_hex(0x1E8A1E)
#define COLOR_GRID_DIM lv_color_hex(0x0E450E)
#define COLOR_TEXT     lv_color_hex(0x5CFF5C)
#define COLOR_AMBER    lv_color_hex(0xFFB000)
#define COLOR_RED      lv_color_hex(0xFF3030)

#define GRID_VARIANTS 3 /* outer ring green / amber / red */

/* A blip stays where the sweep painted it, like phosphor on a CRT */
typedef struct {
    uint32_t lit_at_ms;  /* lv_tick when the sweep crossed the target, 0 = dark */
    int32_t distance_mm; /* distance at that moment */
    int32_t x, y;        /* screen position at that moment */
} blip_t;

static lv_obj_t *s_grid[GRID_VARIANTS];
static int s_grid_shown = -1;
static lv_obj_t *s_scope;
static lv_obj_t *s_status;
static lv_obj_t *s_footer;
static lv_obj_t *s_banner;
static lv_obj_t *s_blip_labels[TARGETS_MAX];
static lv_obj_t *s_sound_button;
static lv_obj_t *s_sound_label;
static lv_timer_t *s_timer;

static int32_t s_sweep_deg10;     /* sweep bearing in tenths of a degree, clockwise from straight ahead */
static uint32_t s_sweep_epoch_ms; /* the sweep angle is derived from the time since this instant */
static lv_area_t s_sweep_area;    /* bounding box of the sweep and its trail in the last frame */
static blip_t s_blips[TARGETS_MAX];
static target_frame_t s_frame;
static scene_state_t s_scene;
static bool s_status_visible = true;
static bool s_banner_visible;
static char s_footer_text[80];
static uint32_t s_fps_frames;
static uint32_t s_fps_mark_ms;

/* --- geometry helpers ------------------------------------------------------------------------ */

static void polar_to_screen_f(float bearing_deg, int32_t radius, lv_point_t *out)
{
    float a = bearing_deg * RADAR_PI / 180.0f;
    out->x = SCOPE_CENTER_X + (int32_t)lroundf((float)radius * sinf(a));
    out->y = SCOPE_CENTER_Y - (int32_t)lroundf((float)radius * cosf(a));
}

static void polar_to_screen(int32_t bearing_deg, int32_t radius, lv_point_t *out)
{
    polar_to_screen_f((float)bearing_deg, radius, out);
}

static int32_t mm_to_px(int32_t mm)
{
    return mm * SCOPE_RADIUS / CONFIG_RADAR_RANGE_MM;
}

static void area_include(lv_area_t *area, int32_t x, int32_t y)
{
    area->x1 = LV_MIN(area->x1, x);
    area->y1 = LV_MIN(area->y1, y);
    area->x2 = LV_MAX(area->x2, x);
    area->y2 = LV_MAX(area->y2, y);
}

/* Bounding box of the sweep line and its trail; the arc between samples bulges well under a pixel */
static void sweep_bounds(float sweep_deg, lv_area_t *area)
{
    *area = (lv_area_t){ SCOPE_CENTER_X, SCOPE_CENTER_Y, SCOPE_CENTER_X, SCOPE_CENTER_Y };
    float start = sweep_deg - CONFIG_RADAR_TRAIL_DEGREES;
    for (int i = 0; i <= 8; i++) {
        lv_point_t p;
        polar_to_screen_f(start + (sweep_deg - start) * (float)i / 8.0f, SCOPE_RADIUS + 2, &p);
        area_include(area, p.x, p.y);
    }
    area->x1 -= 3;
    area->y1 -= 3;
    area->x2 += 3;
    area->y2 += 3;
}

static void square_around(int32_t x, int32_t y, int32_t radius, lv_area_t *area)
{
    area->x1 = x - radius;
    area->y1 = y - radius;
    area->x2 = x + radius;
    area->y2 = y + radius;
}

/* All angles in tenths of a degree, 0..3599 */
static bool sweep_crossed(int32_t prev, int32_t cur, int32_t bearing)
{
    if (prev == cur) {
        return false;
    }
    if (cur > prev) {
        return bearing > prev && bearing <= cur;
    }
    return bearing > prev || bearing <= cur; /* the sweep wrapped past 359.9 */
}

static void set_hidden(lv_obj_t *obj, bool *visible, bool show)
{
    if (show == *visible) {
        return;
    }
    if (show) {
        lv_obj_remove_flag(obj, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(obj, LV_OBJ_FLAG_HIDDEN);
    }
    *visible = show;
}

/* --- colours per stage ----------------------------------------------------------------------- */

static const lv_color_t *grid_colors(void)
{
    static lv_color_t colors[GRID_VARIANTS];
    colors[0] = COLOR_PHOSPHOR;
    colors[1] = COLOR_AMBER;
    colors[2] = COLOR_RED;
    return colors;
}

static int ring_variant(void)
{
    switch (s_scene.stage) {
        case SCENE_BREACH: return 2;
        case SCENE_THREAT: return 1;
        default: return 0;
    }
}

/* --- drawing (read-only, runs in LV_EVENT_DRAW_MAIN or once into the canvases) -------------- */

static void draw_line(lv_layer_t *layer, lv_draw_line_dsc_t *line, lv_point_t a, lv_point_t b)
{
    line->p1.x = a.x;
    line->p1.y = a.y;
    line->p2.x = b.x;
    line->p2.y = b.y;
    lv_draw_line(layer, line);
}

static void draw_grid(lv_layer_t *layer, lv_color_t outer_color)
{
    lv_point_t center = { SCOPE_CENTER_X, SCOPE_CENTER_Y };

    /* The breach zone: a faint red disc right around you */
    lv_draw_fill_dsc_t fill;
    lv_draw_fill_dsc_init(&fill);
    fill.radius = LV_RADIUS_CIRCLE;
    fill.color = COLOR_RED;
    fill.opa = LV_OPA_20;
    lv_area_t breach_area;
    square_around(SCOPE_CENTER_X, SCOPE_CENTER_Y, mm_to_px(CONFIG_SCENE_BREACH_RANGE_MM), &breach_area);
    lv_draw_fill(layer, &fill, &breach_area);

    lv_draw_arc_dsc_t arc;
    lv_draw_arc_dsc_init(&arc);
    arc.center = center;
    arc.start_angle = 0;
    arc.end_angle = 360;
    for (int i = 1; i <= RANGE_RINGS; i++) {
        bool outer = (i == RANGE_RINGS);
        arc.radius = SCOPE_RADIUS * i / RANGE_RINGS;
        arc.width = outer ? 3 : 1;
        arc.color = outer ? outer_color : COLOR_GRID;
        lv_draw_arc(layer, &arc);
    }
    /* Zone rings: threat in amber, breach in red */
    arc.width = 1;
    arc.radius = mm_to_px(CONFIG_SCENE_THREAT_RANGE_MM);
    arc.color = COLOR_AMBER;
    arc.opa = LV_OPA_60;
    lv_draw_arc(layer, &arc);
    arc.radius = mm_to_px(CONFIG_SCENE_BREACH_RANGE_MM);
    arc.color = COLOR_RED;
    lv_draw_arc(layer, &arc);

    lv_draw_line_dsc_t line;
    lv_draw_line_dsc_init(&line);
    line.width = 1;
    line.color = COLOR_GRID_DIM;
    /* Crosshair */
    draw_line(layer, &line, (lv_point_t){ SCOPE_CENTER_X - SCOPE_RADIUS, SCOPE_CENTER_Y },
              (lv_point_t){ SCOPE_CENTER_X + SCOPE_RADIUS, SCOPE_CENTER_Y });
    draw_line(layer, &line, (lv_point_t){ SCOPE_CENTER_X, SCOPE_CENTER_Y - SCOPE_RADIUS },
              (lv_point_t){ SCOPE_CENTER_X, SCOPE_CENTER_Y + SCOPE_RADIUS });

    /* The sensor's field of view, dashed */
    line.color = COLOR_GRID;
    line.dash_width = 6;
    line.dash_gap = 6;
    for (int sign = -1; sign <= 1; sign += 2) {
        lv_point_t edge;
        polar_to_screen(sign * SENSOR_FOV_DEG, SCOPE_RADIUS, &edge);
        draw_line(layer, &line, center, edge);
    }
    line.dash_width = 0;
    line.dash_gap = 0;

    /* Bearing ticks outside the ring: every 10 degrees, longer every 30 */
    for (int deg = 0; deg < 360; deg += 10) {
        int len = (deg % 30 == 0) ? TICK_LEN_MAJOR : TICK_LEN_MINOR;
        lv_point_t a, b;
        polar_to_screen(deg, SCOPE_RADIUS + 3, &a);
        polar_to_screen(deg, SCOPE_RADIUS + 3 + len, &b);
        draw_line(layer, &line, a, b);
    }
}

static void draw_blips(lv_layer_t *layer, uint32_t now_ms)
{
    lv_draw_fill_dsc_t fill;
    lv_draw_fill_dsc_init(&fill);
    fill.radius = LV_RADIUS_CIRCLE;
    fill.color = COLOR_PHOSPHOR;

    for (int i = 0; i < TARGETS_MAX; i++) {
        const blip_t *b = &s_blips[i];
        if (b->lit_at_ms == 0) {
            continue;
        }
        uint32_t age = now_ms - b->lit_at_ms;
        if (age >= CONFIG_RADAR_BLIP_DECAY_MS) {
            continue;
        }
        float k = 1.0f - (float)age / CONFIG_RADAR_BLIP_DECAY_MS;
        lv_area_t area;
        fill.opa = (lv_opa_t)(90.0f * k);
        square_around(b->x, b->y, BLIP_HALO_RADIUS, &area);
        lv_draw_fill(layer, &fill, &area);
        fill.opa = (lv_opa_t)(255.0f * k);
        square_around(b->x, b->y, BLIP_RADIUS, &area);
        lv_draw_fill(layer, &fill, &area);
    }
}

static void draw_sweep(lv_layer_t *layer, lv_color_t color)
{
    float sweep_deg = (float)s_sweep_deg10 / 10.0f;
    float step = (float)CONFIG_RADAR_TRAIL_DEGREES / TRAIL_STEPS;

    /* Afterglow: thin triangles from the centre to the ring, dimmest first, fading behind the line.
     * Triangles use cheap line masks; a filled arc wedge costs ten times more in software. */
    lv_draw_triangle_dsc_t tri;
    lv_draw_triangle_dsc_init(&tri);
    tri.color = color;
    tri.p[0].x = SCOPE_CENTER_X;
    tri.p[0].y = SCOPE_CENTER_Y;
    for (int i = TRAIL_STEPS - 1; i >= 0; i--) {
        float end_bearing = sweep_deg - (float)i * step;
        lv_point_t a, b;
        polar_to_screen_f(end_bearing - step, SCOPE_RADIUS - 2, &a);
        polar_to_screen_f(end_bearing, SCOPE_RADIUS - 2, &b);
        tri.p[1].x = a.x;
        tri.p[1].y = a.y;
        tri.p[2].x = b.x;
        tri.p[2].y = b.y;
        float k = 1.0f - (float)i / TRAIL_STEPS;
        tri.opa = (lv_opa_t)(150.0f * k * k);
        lv_draw_triangle(layer, &tri);
    }

    /* The sweep line itself */
    lv_point_t tip;
    polar_to_screen_f(sweep_deg, SCOPE_RADIUS, &tip);
    lv_draw_line_dsc_t line;
    lv_draw_line_dsc_init(&line);
    line.color = color;
    line.width = 3;
    line.round_end = 1;
    draw_line(layer, &line, (lv_point_t){ SCOPE_CENTER_X, SCOPE_CENTER_Y }, tip);
}

static void scope_draw_cb(lv_event_t *e)
{
    lv_layer_t *layer = lv_event_get_layer(e);
    lv_color_t color = grid_colors()[ring_variant()];
    draw_blips(layer, lv_tick_get());
    draw_sweep(layer, color);
}

/* --- state updates (run in the LVGL timer) --------------------------------------------------- */

/* Only the moving parts are redrawn: the blip's halo box, and the sweep's box from this and the last frame */
static void invalidate_blip(const blip_t *b)
{
    lv_area_t area;
    square_around(b->x, b->y, BLIP_HALO_RADIUS + 1, &area);
    lv_obj_invalidate_area(s_scope, &area);
}

static void invalidate_sweep(void)
{
    lv_area_t now;
    sweep_bounds((float)s_sweep_deg10 / 10.0f, &now);
    lv_area_t both = now;
    area_include(&both, s_sweep_area.x1, s_sweep_area.y1);
    area_include(&both, s_sweep_area.x2, s_sweep_area.y2);
    lv_obj_invalidate_area(s_scope, &both);
    s_sweep_area = now;
}

static void paint_blip(int index, int32_t distance_mm, int32_t bearing_deg, uint32_t now_ms)
{
    blip_t *b = &s_blips[index];
    lv_point_t p;
    polar_to_screen(bearing_deg, mm_to_px(distance_mm), &p);
    b->x = p.x;
    b->y = p.y;
    b->distance_mm = distance_mm;
    b->lit_at_ms = (now_ms != 0) ? now_ms : 1;
    invalidate_blip(b);

    lv_obj_t *label = s_blip_labels[index];
    lv_label_set_text_fmt(label, "%d.%dm", (int)(distance_mm / 1000), (int)((distance_mm % 1000) / 100));
    lv_obj_set_pos(label, p.x + BLIP_HALO_RADIUS + 2, p.y - 8);
    lv_obj_set_style_text_opa(label, LV_OPA_COVER, 0);
    lv_obj_remove_flag(label, LV_OBJ_FLAG_HIDDEN);

    sound_play_contact(distance_mm);
}

static void fade_blips(uint32_t now_ms)
{
    for (int i = 0; i < TARGETS_MAX; i++) {
        blip_t *b = &s_blips[i];
        if (b->lit_at_ms == 0) {
            continue;
        }
        uint32_t age = now_ms - b->lit_at_ms;
        invalidate_blip(b); /* its brightness changes every frame */
        if (age >= CONFIG_RADAR_BLIP_DECAY_MS) {
            b->lit_at_ms = 0;
            lv_obj_add_flag(s_blip_labels[i], LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_opa_t opa = (lv_opa_t)(255 - 255 * age / CONFIG_RADAR_BLIP_DECAY_MS);
            lv_obj_set_style_text_opa(s_blip_labels[i], opa, 0);
        }
    }
}

static void show_grid(int variant)
{
    if (variant == s_grid_shown) {
        return;
    }
    if (s_grid_shown >= 0) {
        lv_obj_add_flag(s_grid[s_grid_shown], LV_OBJ_FLAG_HIDDEN);
    }
    lv_obj_remove_flag(s_grid[variant], LV_OBJ_FLAG_HIDDEN);
    s_grid_shown = variant;
}

static void update_text(uint32_t now_ms, int contacts, int32_t nearest_mm, bool nearest_valid)
{
    lv_color_t stage_color = grid_colors()[ring_variant()];
    bool slow_blink = (now_ms / STATUS_BLINK_MS) % 2 == 0;
    bool fast_blink = (now_ms / PANIC_BLINK_MS) % 2 == 0;

    /* Top right: the stage, blinking when things get serious */
    switch (s_scene.stage) {
        case SCENE_CLEAR:
            lv_label_set_text(s_status, "SCANNING");
            set_hidden(s_status, &s_status_visible, slow_blink);
            break;
        case SCENE_CONTACT:
            lv_label_set_text(s_status, "CONTACT");
            set_hidden(s_status, &s_status_visible, true);
            break;
        case SCENE_THREAT:
            lv_label_set_text(s_status, "THREAT DETECTED");
            set_hidden(s_status, &s_status_visible, slow_blink);
            break;
        case SCENE_BREACH:
            lv_label_set_text(s_status, s_scene.test ? "BREACH (TEST)" : "BREACH");
            set_hidden(s_status, &s_status_visible, fast_blink);
            break;
    }
    lv_obj_set_style_text_color(s_status, stage_color, 0);

    /* Big banner for the people standing behind you */
    if (s_scene.stage == SCENE_BREACH) {
        lv_label_set_text(s_banner, "GET OUT");
        lv_obj_set_style_text_color(s_banner, COLOR_RED, 0);
        set_hidden(s_banner, &s_banner_visible, fast_blink);
    } else if (s_scene.stage == SCENE_THREAT) {
        lv_label_set_text(s_banner, "THREAT DETECTED");
        lv_obj_set_style_text_color(s_banner, COLOR_AMBER, 0);
        set_hidden(s_banner, &s_banner_visible, true);
    } else {
        set_hidden(s_banner, &s_banner_visible, false);
    }

    /* Bottom: contact summary, rewritten only when it changes */
    char nearest[24];
    if (nearest_valid) {
        snprintf(nearest, sizeof(nearest), "%d.%dm", (int)(nearest_mm / 1000), (int)((nearest_mm % 1000) / 100));
    } else {
        strcpy(nearest, "---");
    }
    char text[sizeof(s_footer_text)];
    snprintf(text, sizeof(text), "CONTACTS %d  NEAREST %s  %s%s", contacts, nearest,
             scene_stage_name(s_scene.stage), s_scene.fog_on ? "  FOG" : "");
    if (strcmp(text, s_footer_text) != 0) {
        strcpy(s_footer_text, text);
        lv_label_set_text(s_footer, text);
        lv_obj_set_style_text_color(s_footer, (s_scene.stage >= SCENE_THREAT) ? stage_color : COLOR_TEXT, 0);
    }
}

static void radar_tick(lv_timer_t *timer)
{
    uint32_t now_ms = lv_tick_get();

    /* The sweep angle follows the clock, so a slow frame never slows the rotation or drifts it */
    int32_t prev = s_sweep_deg10;
    uint32_t phase_ms = (now_ms - s_sweep_epoch_ms) % CONFIG_RADAR_SWEEP_PERIOD_MS;
    s_sweep_deg10 = (int32_t)(phase_ms * 3600 / CONFIG_RADAR_SWEEP_PERIOD_MS);
    int32_t cur = s_sweep_deg10;
#ifdef CONFIG_SOUND_SWEEP_PING
    if (cur < prev) {
        sound_play(SOUND_SWEEP);
    }
#endif

    /* Paint everyone the sweep passed since the last frame */
    targets_get(&s_frame);
    scene_get(&s_scene);
    int contacts = 0;
    int32_t nearest_mm = 0;
    bool nearest_valid = false;
    for (int i = 0; i < s_frame.count && i < TARGETS_MAX; i++) {
        const target_t *t = &s_frame.list[i];
        if (!t->valid) {
            continue;
        }
        int32_t distance_mm = target_distance_mm(t);
        if (distance_mm > CONFIG_RADAR_RANGE_MM) {
            continue;
        }
        contacts++;
        if (!nearest_valid || distance_mm < nearest_mm) {
            nearest_valid = true;
            nearest_mm = distance_mm;
        }
        int32_t bearing_deg = target_bearing_deg(t);
        if (sweep_crossed(prev, cur, bearing_deg * 10)) {
            paint_blip(i, distance_mm, bearing_deg, now_ms);
        }
    }
    fade_blips(now_ms);

    update_text(now_ms, contacts, nearest_mm, nearest_valid);
    show_grid(ring_variant());
    invalidate_sweep();

#ifdef CONFIG_RADAR_LOG_FPS
    s_fps_frames++;
    if (now_ms - s_fps_mark_ms >= FPS_LOG_PERIOD_MS) {
        ESP_LOGI(TAG, "Scope %u fps, %d contact(s), %s", (unsigned)(s_fps_frames * 1000 / (now_ms - s_fps_mark_ms)),
                 contacts, scene_stage_name(s_scene.stage));
        s_fps_frames = 0;
        s_fps_mark_ms = now_ms;
    }
#endif
}

static void scope_click_cb(lv_event_t *e)
{
    ESP_LOGI(TAG, "Scope tapped: test breach");
    scene_test_breach();
}

/* --- screen construction --------------------------------------------------------------------- */

static lv_obj_t *make_label(lv_obj_t *parent, const char *text, const lv_font_t *font, lv_color_t color)
{
    lv_obj_t *label = lv_label_create(parent);
    lv_label_set_text(label, text);
    lv_obj_set_style_text_font(label, font, 0);
    lv_obj_set_style_text_color(label, color, 0);
    return label;
}

/* Render the static grid once into a full-screen RGB565 canvas (1 MB, lands in PSRAM) */
static lv_obj_t *make_grid_canvas(lv_obj_t *parent, lv_color_t outer_color)
{
    lv_draw_buf_t *buf = lv_draw_buf_create(SCREEN_SIZE, SCREEN_SIZE, LV_COLOR_FORMAT_RGB565, 0);
    if (buf == NULL) {
        ESP_LOGE(TAG, "Grid canvas allocation failed");
        return NULL;
    }
    lv_obj_t *canvas = lv_canvas_create(parent);
    lv_canvas_set_draw_buf(canvas, buf);
    lv_obj_set_pos(canvas, 0, 0);
    lv_canvas_fill_bg(canvas, COLOR_BG, LV_OPA_COVER);

    lv_layer_t layer;
    lv_canvas_init_layer(canvas, &layer);
    draw_grid(&layer, outer_color);
    lv_canvas_finish_layer(canvas, &layer);

    lv_obj_add_flag(canvas, LV_OBJ_FLAG_HIDDEN);
    return canvas;
}

static esp_err_t build_grids(lv_obj_t *screen)
{
    for (int i = 0; i < GRID_VARIANTS; i++) {
        s_grid[i] = make_grid_canvas(screen, grid_colors()[i]);
        if (s_grid[i] == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }
    show_grid(0);
    return ESP_OK;
}

static void build_scope(lv_obj_t *screen)
{
    s_scope = lv_obj_create(screen);
    lv_obj_remove_style_all(s_scope);
    lv_obj_set_size(s_scope, SCREEN_SIZE, SCREEN_SIZE);
    lv_obj_set_pos(s_scope, 0, 0);
    lv_obj_add_flag(s_scope, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(s_scope, scope_draw_cb, LV_EVENT_DRAW_MAIN, NULL);
    lv_obj_add_event_cb(s_scope, scope_click_cb, LV_EVENT_CLICKED, NULL);

    /* Bearing numbers around the ring */
    for (int deg = 0; deg < 360; deg += 30) {
        char text[4];
        snprintf(text, sizeof(text), "%03d", deg);
        lv_obj_t *label = make_label(screen, text, &lv_font_unscii_16, COLOR_GRID);
        lv_point_t p;
        polar_to_screen(deg, BEARING_LABEL_R, &p);
        lv_obj_align(label, LV_ALIGN_CENTER, p.x - SCOPE_CENTER_X, p.y - SCOPE_CENTER_Y);
    }

    /* Range labels just above each ring, left of the vertical axis */
    for (int i = 1; i <= RANGE_RINGS; i++) {
        int32_t mm = CONFIG_RADAR_RANGE_MM * i / RANGE_RINGS;
        char text[16];
        snprintf(text, sizeof(text), "%d.%dm", (int)(mm / 1000), (int)((mm % 1000) / 100));
        lv_obj_t *label = make_label(screen, text, &lv_font_unscii_16, COLOR_GRID);
        lv_obj_update_layout(label);
        lv_obj_set_pos(label, SCOPE_CENTER_X - 6 - lv_obj_get_width(label),
                       SCOPE_CENTER_Y - SCOPE_RADIUS * i / RANGE_RINGS + 4);
    }

    /* Where things are: the door ahead of the sensor, you at the centre */
    lv_obj_t *door = make_label(screen, "DOOR", &lv_font_unscii_16, COLOR_GRID);
    lv_obj_set_pos(door, SCOPE_CENTER_X + 8, SCOPE_CENTER_Y - SCOPE_RADIUS + 4);
    lv_obj_t *you = make_label(screen, "YOU", &lv_font_unscii_16, COLOR_GRID);
    lv_obj_set_pos(you, SCOPE_CENTER_X + 8, SCOPE_CENTER_Y + 6);

    /* Distance tags next to the blips, shown while a blip is lit */
    for (int i = 0; i < TARGETS_MAX; i++) {
        s_blip_labels[i] = make_label(screen, "", &lv_font_unscii_16, COLOR_PHOSPHOR);
        lv_obj_add_flag(s_blip_labels[i], LV_OBJ_FLAG_HIDDEN);
    }
}

static void build_text(lv_obj_t *screen)
{
    lv_obj_t *header = make_label(screen, "HUMAN RADAR", &lv_font_unscii_16, COLOR_TEXT);
    lv_obj_align(header, LV_ALIGN_TOP_LEFT, TEXT_MARGIN, TEXT_MARGIN);

    s_status = make_label(screen, "SCANNING", &lv_font_unscii_16, COLOR_TEXT);
    lv_obj_align(s_status, LV_ALIGN_TOP_RIGHT, -TEXT_MARGIN, TEXT_MARGIN);

    s_footer = make_label(screen, "", &lv_font_unscii_16, COLOR_TEXT);
    lv_obj_align(s_footer, LV_ALIGN_BOTTOM_LEFT, TEXT_MARGIN, -TEXT_MARGIN);

    s_banner = make_label(screen, "", &lv_font_montserrat_32, COLOR_RED);
    lv_obj_align(s_banner, LV_ALIGN_CENTER, 0, SCOPE_RADIUS / 2);
    lv_obj_add_flag(s_banner, LV_OBJ_FLAG_HIDDEN);
}

static void update_sound_button(void)
{
    bool on = sound_is_enabled();
    lv_label_set_text(s_sound_label, on ? "SOUND ON" : "SOUND OFF");
    lv_obj_set_style_text_color(s_sound_label, on ? COLOR_PHOSPHOR : COLOR_GRID, 0);
    lv_obj_set_style_border_color(s_sound_button, on ? COLOR_PHOSPHOR : COLOR_GRID, 0);
}

static void sound_button_cb(lv_event_t *e)
{
    bool on = !sound_is_enabled();
    sound_set_enabled(on);
    if (on) {
        sound_play(SOUND_CONTACT_MID); /* a blip so you hear that it works */
    }
    update_sound_button();
}

static void build_sound_button(lv_obj_t *screen)
{
    s_sound_button = lv_button_create(screen);
    lv_obj_remove_style_all(s_sound_button);
    lv_obj_set_size(s_sound_button, 150, 40);
    lv_obj_align(s_sound_button, LV_ALIGN_TOP_LEFT, TEXT_MARGIN, TEXT_MARGIN + 26);
    lv_obj_set_style_bg_color(s_sound_button, COLOR_BG, 0);
    lv_obj_set_style_bg_opa(s_sound_button, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(s_sound_button, COLOR_GRID_DIM, LV_STATE_PRESSED);
    lv_obj_set_style_border_width(s_sound_button, 2, 0);
    lv_obj_set_style_radius(s_sound_button, 0, 0);
    lv_obj_add_event_cb(s_sound_button, sound_button_cb, LV_EVENT_CLICKED, NULL);

    s_sound_label = make_label(s_sound_button, "", &lv_font_unscii_16, COLOR_PHOSPHOR);
    lv_obj_center(s_sound_label);
    update_sound_button();
}

esp_err_t radar_ui_start(void)
{
    if (!bsp_display_lock(1000)) {
        ESP_LOGE(TAG, "Could not take the display lock");
        return ESP_ERR_TIMEOUT;
    }

    lv_obj_t *screen = lv_screen_active();
    lv_obj_set_style_bg_color(screen, COLOR_BG, 0);
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, 0);
    lv_obj_remove_flag(screen, LV_OBJ_FLAG_SCROLLABLE);

    if (build_grids(screen) != ESP_OK) {
        bsp_display_unlock();
        return ESP_ERR_NO_MEM;
    }
    build_scope(screen);
    build_text(screen);
    build_sound_button(screen);

    s_sweep_epoch_ms = lv_tick_get();
    s_fps_mark_ms = s_sweep_epoch_ms;
    s_sweep_area = (lv_area_t){ 0, 0, SCREEN_SIZE - 1, SCREEN_SIZE - 1 };
    s_timer = lv_timer_create(radar_tick, CONFIG_RADAR_FRAME_PERIOD_MS, NULL);

    bsp_display_unlock();
    ESP_LOGI(TAG, "Scope ready: range %d mm, sweep %d ms, threat %d mm, breach %d mm", CONFIG_RADAR_RANGE_MM,
             CONFIG_RADAR_SWEEP_PERIOD_MS, CONFIG_SCENE_THREAT_RANGE_MM, CONFIG_SCENE_BREACH_RANGE_MM);
    return ESP_OK;
}

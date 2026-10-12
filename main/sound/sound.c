/**
 * @file sound.c
 * @brief Radar sounds: synthesised effects plus voice and scream clips, mixed into the speaker path
 *
 * Effects are short lists of tone segments (frequency, optional glide, length,
 * level, ping-style decay) rendered on the fly. Clips are 16-bit mono WAV files
 * from the "storage" SPIFFS partition (the WAV files in assets/, flashed with
 * the app),
 * loaded into PSRAM at start-up; a missing clip falls back to a synthesised
 * sound. A mixer task renders 20 ms blocks of 16-bit mono PCM at 22050 Hz,
 * summing up to MAX_VOICES voices, and streams them to the ES8311 through the
 * BSP. Silence is streamed while idle or muted, so the amplifier never pops and
 * the I2S DMA never runs dry.
 */

#include <sdkconfig.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <esp_log.h>
#include <esp_check.h>
#include <esp_heap_caps.h>

#include "sound.h"

#ifdef CONFIG_SOUND_ENABLED

#include "bsp/esp-bsp.h"

static const char *TAG = "sound";

#define SAMPLE_RATE_HZ 22050 /* the BSP's default I2S clock */
#define BLOCK_SAMPLES  441   /* 20 ms */
#define MAX_VOICES     4
#define FADE_SAMPLES   44    /* 2 ms ramps at segment edges: no clicks */
#define QUEUE_LENGTH   8
#define TWO_PI         6.28318530f
#define FULL_SCALE     32767.0f
#define CLIP_LEVEL     0.9f  /* clips play at 90 % so they can mix with the effects */

typedef struct {
    uint16_t freq_hz;     /**< Tone frequency, 0 for a pause */
    uint16_t freq_end_hz; /**< Glide to this frequency by the end of the segment, 0 = no glide */
    uint16_t ms;          /**< Segment length */
    uint8_t level;        /**< 0..100 % of full scale */
    bool decay;           /**< Exponential decay (ping) instead of a flat tone */
} tone_seg_t;

typedef struct {
    const tone_seg_t *segs;
    int count;
} sound_def_t;

#define SOUND_DEF(segs) { segs, sizeof(segs) / sizeof((segs)[0]) }

static const tone_seg_t s_startup_segs[] = {
    { 300, 1300, 350, 55, false }, /* rising chirp: radar powering up */
    { 0, 0, 60, 0, false },
    { 1300, 0, 180, 55, true },
};
static const tone_seg_t s_sweep_segs[] = {
    { 760, 0, 450, 40, true }, /* sonar ping... */
    { 0, 0, 180, 0, false },
    { 760, 0, 300, 15, true }, /* ...and its echo */
};
static const tone_seg_t s_contact_far_segs[] = {
    { 1000, 0, 120, 50, true },
};
static const tone_seg_t s_contact_mid_segs[] = {
    { 1400, 0, 120, 65, true },
};
static const tone_seg_t s_contact_near_segs[] = {
    { 1900, 0, 90, 85, true }, /* double blip: someone is close */
    { 0, 0, 40, 0, false },
    { 1900, 0, 90, 85, true },
};
static const tone_seg_t s_alert_segs[] = {
    { 620, 0, 170, 90, false }, { 930, 0, 170, 90, false },
    { 620, 0, 170, 90, false }, { 930, 0, 170, 90, false },
};
static const tone_seg_t s_threat_segs[] = {
    { 440, 0, 160, 80, false }, { 0, 0, 60, 0, false },
    { 660, 0, 160, 80, false }, { 0, 0, 60, 0, false },
    { 880, 0, 320, 85, true },
};
static const tone_seg_t s_scream_segs[] = {
    { 500, 1400, 300, 90, false }, { 1400, 900, 250, 90, false },
    { 900, 1600, 300, 90, false }, { 1600, 300, 600, 90, true },
};

static const sound_def_t s_sounds[SOUND_COUNT] = {
    [SOUND_STARTUP]      = SOUND_DEF(s_startup_segs),
    [SOUND_SWEEP]        = SOUND_DEF(s_sweep_segs),
    [SOUND_CONTACT_FAR]  = SOUND_DEF(s_contact_far_segs),
    [SOUND_CONTACT_MID]  = SOUND_DEF(s_contact_mid_segs),
    [SOUND_CONTACT_NEAR] = SOUND_DEF(s_contact_near_segs),
    [SOUND_ALERT]        = SOUND_DEF(s_alert_segs),
    [SOUND_THREAT]       = SOUND_DEF(s_threat_segs),
    [SOUND_SCREAM]       = SOUND_DEF(s_scream_segs),
};

typedef struct {
    const char *file;
    sound_id_t fallback;
    int16_t *pcm;     /**< mono 16-bit PCM in PSRAM, NULL when the clip is missing */
    uint32_t samples;
} clip_t;

static clip_t s_clips[SAMPLE_COUNT] = {
    [SAMPLE_THREAT]  = { .file = BSP_SPIFFS_MOUNT_POINT "/threat.wav", .fallback = SOUND_THREAT },
    [SAMPLE_SCREAM]  = { .file = BSP_SPIFFS_MOUNT_POINT "/scream.wav", .fallback = SOUND_SCREAM },
    [SAMPLE_GET_OUT] = { .file = BSP_SPIFFS_MOUNT_POINT "/getout.wav", .fallback = SOUND_SCREAM },
};

typedef enum { REQUEST_TONE, REQUEST_CLIP } request_kind_t;

typedef struct {
    request_kind_t kind;
    int id;
} request_t;

/* A voice plays either a tone list or a clip */
typedef struct {
    const sound_def_t *def;
    const clip_t *clip;
    int seg;          /* tone: current segment */
    uint32_t seg_pos; /* tone: samples rendered in the segment */
    uint32_t seg_len; /* tone: samples in the segment */
    float phase;      /* tone: oscillator phase, continuous across segments */
    uint32_t pos;     /* clip: next sample */
} voice_t;

static QueueHandle_t s_queue;
static esp_codec_dev_handle_t s_codec;
static voice_t s_voices[MAX_VOICES];
#ifdef CONFIG_SOUND_ON_AT_BOOT
static volatile bool s_enabled = true;
#else
static volatile bool s_enabled = false;
#endif

static uint32_t segment_samples(const tone_seg_t *seg)
{
    return (uint32_t)seg->ms * SAMPLE_RATE_HZ / 1000;
}

static voice_t *free_voice(void)
{
    for (int i = 0; i < MAX_VOICES; i++) {
        if (s_voices[i].def == NULL && s_voices[i].clip == NULL) {
            return &s_voices[i];
        }
    }
    return NULL;
}

static void voice_start_tone(sound_id_t id)
{
    voice_t *v = free_voice();
    if (v == NULL) {
        ESP_LOGD(TAG, "All voices busy, dropping sound %d", id);
        return;
    }
    *v = (voice_t){ .def = &s_sounds[id], .seg_len = segment_samples(&s_sounds[id].segs[0]) };
}

static void voice_start_clip(sample_id_t id)
{
    const clip_t *clip = &s_clips[id];
    if (clip->pcm == NULL) {
        voice_start_tone(clip->fallback);
        return;
    }
    voice_t *v = free_voice();
    if (v == NULL) {
        ESP_LOGD(TAG, "All voices busy, dropping clip %d", id);
        return;
    }
    *v = (voice_t){ .clip = clip };
}

/* Short ramps at the start and end keep the speaker from clicking */
static float edge_fade(uint32_t pos, uint32_t len)
{
    float k = 1.0f;
    if (pos < FADE_SAMPLES) {
        k *= (float)pos / FADE_SAMPLES;
    }
    if (len - pos < FADE_SAMPLES) {
        k *= (float)(len - pos) / FADE_SAMPLES;
    }
    return k;
}

/* One sample of a voice; frees the voice when it ends */
static int32_t voice_next_sample(voice_t *v)
{
    if (v->clip != NULL) {
        float sample = v->clip->pcm[v->pos] * CLIP_LEVEL * edge_fade(v->pos, v->clip->samples);
        if (++v->pos >= v->clip->samples) {
            v->clip = NULL;
        }
        return (int32_t)sample;
    }

    const tone_seg_t *seg = &v->def->segs[v->seg];
    float t = (float)v->seg_pos / (float)v->seg_len; /* 0..1 within the segment */
    float amp = seg->level / 100.0f * FULL_SCALE * edge_fade(v->seg_pos, v->seg_len);
    if (seg->decay) {
        amp *= expf(-5.0f * t);
    }

    int32_t sample = 0;
    if (seg->freq_hz != 0) {
        float freq = seg->freq_hz;
        if (seg->freq_end_hz != 0) {
            freq += ((float)seg->freq_end_hz - (float)seg->freq_hz) * t;
        }
        sample = (int32_t)(amp * sinf(v->phase));
        v->phase += TWO_PI * freq / SAMPLE_RATE_HZ;
        if (v->phase >= TWO_PI) {
            v->phase -= TWO_PI;
        }
    }

    if (++v->seg_pos >= v->seg_len) {
        v->seg++;
        v->seg_pos = 0;
        if (v->seg >= v->def->count) {
            v->def = NULL;
        } else {
            v->seg_len = segment_samples(&v->def->segs[v->seg]);
        }
    }
    return sample;
}

static void sound_task(void *arg)
{
    static int16_t block[BLOCK_SAMPLES];

    while (true) {
        request_t request;
        while (xQueueReceive(s_queue, &request, 0) == pdTRUE) {
            if (!s_enabled) {
                continue;
            }
            if (request.kind == REQUEST_CLIP) {
                voice_start_clip(request.id);
            } else {
                voice_start_tone(request.id);
            }
        }
        if (!s_enabled) {
            /* Muted: silence whatever was still playing, keep streaming zeros */
            memset(s_voices, 0, sizeof(s_voices));
        }

        for (int n = 0; n < BLOCK_SAMPLES; n++) {
            int32_t mix = 0;
            for (int i = 0; i < MAX_VOICES; i++) {
                if (s_voices[i].def != NULL || s_voices[i].clip != NULL) {
                    mix += voice_next_sample(&s_voices[i]);
                }
            }
            if (mix > 32767) {
                mix = 32767;
            } else if (mix < -32768) {
                mix = -32768;
            }
            block[n] = (int16_t)mix;
        }

        /* Blocks until the I2S DMA accepts the block, which paces this loop in real time */
        int err = esp_codec_dev_write(s_codec, block, sizeof(block));
        if (err != ESP_CODEC_DEV_OK) {
            ESP_LOGW(TAG, "Codec write failed (%d)", err);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

static uint32_t le32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static uint16_t le16(const uint8_t *p)
{
    return (uint16_t)(p[0] | (p[1] << 8));
}

/* Minimal RIFF/WAVE reader: 16-bit PCM, mono or stereo (mixed down), loaded into PSRAM */
static esp_err_t load_wav(clip_t *clip)
{
    FILE *f = fopen(clip->file, "rb");
    if (f == NULL) {
        return ESP_ERR_NOT_FOUND;
    }
    uint8_t riff[12];
    if (fread(riff, 1, sizeof(riff), f) != sizeof(riff) || memcmp(riff, "RIFF", 4) != 0 || memcmp(riff + 8, "WAVE", 4) != 0) {
        fclose(f);
        return ESP_ERR_INVALID_RESPONSE;
    }

    esp_err_t err = ESP_ERR_NOT_FOUND;
    uint16_t channels = 0;
    uint16_t bits = 0;
    uint32_t rate = 0;
    uint8_t chunk[8];
    while (fread(chunk, 1, sizeof(chunk), f) == sizeof(chunk)) {
        uint32_t size = le32(chunk + 4);
        if (memcmp(chunk, "fmt ", 4) == 0) {
            uint8_t fmt[16];
            if (size < sizeof(fmt) || fread(fmt, 1, sizeof(fmt), f) != sizeof(fmt)) {
                break;
            }
            channels = le16(fmt + 2);
            rate = le32(fmt + 4);
            bits = le16(fmt + 14);
            if (le16(fmt) != 1 || bits != 16 || (channels != 1 && channels != 2)) {
                err = ESP_ERR_NOT_SUPPORTED;
                break;
            }
            fseek(f, (long)(size - sizeof(fmt)), SEEK_CUR);
        } else if (memcmp(chunk, "data", 4) == 0) {
            if (bits != 16) {
                err = ESP_ERR_INVALID_STATE;
                break;
            }
            int16_t *pcm = heap_caps_malloc(size, MALLOC_CAP_SPIRAM);
            if (pcm == NULL) {
                err = ESP_ERR_NO_MEM;
                break;
            }
            if (fread(pcm, 1, size, f) != size) {
                free(pcm);
                err = ESP_FAIL;
                break;
            }
            uint32_t frames = size / 2 / channels;
            if (channels == 2) {
                for (uint32_t i = 0; i < frames; i++) {
                    pcm[i] = (int16_t)(((int32_t)pcm[2 * i] + pcm[2 * i + 1]) / 2);
                }
            }
            if (rate != SAMPLE_RATE_HZ) {
                ESP_LOGW(TAG, "%s is %u Hz, played at %d Hz", clip->file, (unsigned)rate, SAMPLE_RATE_HZ);
            }
            clip->pcm = pcm;
            clip->samples = frames;
            err = ESP_OK;
            break;
        } else {
            fseek(f, (long)((size + 1) & ~1u), SEEK_CUR); /* chunks are word aligned */
        }
    }
    fclose(f);
    return err;
}

static void load_clips(void)
{
    esp_err_t err = bsp_spiffs_mount();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Clip partition not mounted (%s): using synthesised fallbacks", esp_err_to_name(err));
        return;
    }
    for (int i = 0; i < SAMPLE_COUNT; i++) {
        err = load_wav(&s_clips[i]);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Loaded %s (%.1f s)", s_clips[i].file, (float)s_clips[i].samples / SAMPLE_RATE_HZ);
        } else {
            ESP_LOGW(TAG, "%s not loaded (%s): using the synthesised fallback", s_clips[i].file, esp_err_to_name(err));
        }
    }
}

esp_err_t sound_init(void)
{
    s_codec = bsp_audio_codec_speaker_init();
    ESP_RETURN_ON_FALSE(s_codec != NULL, ESP_FAIL, TAG, "Speaker codec init failed");

    esp_codec_dev_sample_info_t fs = {
        .bits_per_sample = 16,
        .channel = 1,
        .channel_mask = 0,
        .sample_rate = SAMPLE_RATE_HZ,
        .mclk_multiple = 0,
    };
    int err = esp_codec_dev_open(s_codec, &fs);
    ESP_RETURN_ON_FALSE(err == ESP_CODEC_DEV_OK, ESP_FAIL, TAG, "Codec open failed (%d)", err);
    esp_codec_dev_set_out_vol(s_codec, CONFIG_SOUND_VOLUME);
    esp_codec_dev_set_out_mute(s_codec, !s_enabled);

    load_clips();

    s_queue = xQueueCreate(QUEUE_LENGTH, sizeof(request_t));
    ESP_RETURN_ON_FALSE(s_queue != NULL, ESP_ERR_NO_MEM, TAG, "Sound queue allocation failed");
    /* Above the LVGL task: a 20 ms block is cheap, but it must never be late */
    BaseType_t ok = xTaskCreatePinnedToCore(&sound_task, "Sound", 4096, NULL, 7, NULL, 1);
    ESP_RETURN_ON_FALSE(ok == pdPASS, ESP_FAIL, TAG, "Sound task creation failed");

    ESP_LOGI(TAG, "Speaker ready: %d Hz mono, volume %d %%, sound %s", SAMPLE_RATE_HZ, CONFIG_SOUND_VOLUME,
             s_enabled ? "on" : "off");
    return ESP_OK;
}

static void queue_request(request_kind_t kind, int id)
{
    if (s_queue == NULL || !s_enabled) {
        return;
    }
    request_t request = { .kind = kind, .id = id };
    if (xQueueSend(s_queue, &request, 0) != pdTRUE) {
        ESP_LOGD(TAG, "Sound queue full, dropping request %d/%d", kind, id);
    }
}

void sound_play(sound_id_t id)
{
    if (id < SOUND_COUNT) {
        queue_request(REQUEST_TONE, id);
    }
}

void sound_play_sample(sample_id_t id)
{
    if (id < SAMPLE_COUNT) {
        queue_request(REQUEST_CLIP, id);
    }
}

void sound_play_contact(int32_t distance_mm)
{
    if (distance_mm <= CONFIG_SCENE_BREACH_RANGE_MM) {
        sound_play(SOUND_CONTACT_NEAR);
    } else if (distance_mm <= CONFIG_SCENE_THREAT_RANGE_MM) {
        sound_play(SOUND_CONTACT_MID);
    } else {
        sound_play(SOUND_CONTACT_FAR);
    }
}

void sound_set_volume(int percent)
{
    if (s_codec != NULL) {
        esp_codec_dev_set_out_vol(s_codec, percent);
    }
}

void sound_set_enabled(bool enabled)
{
    s_enabled = enabled;
    if (s_codec != NULL) {
        esp_codec_dev_set_out_mute(s_codec, !enabled);
    }
    ESP_LOGI(TAG, "Sound %s", enabled ? "on" : "off");
}

bool sound_is_enabled(void)
{
    return s_enabled;
}

#else /* CONFIG_SOUND_ENABLED */

esp_err_t sound_init(void) { return ESP_OK; }
void sound_play(sound_id_t id) { (void)id; }
void sound_play_sample(sample_id_t id) { (void)id; }
void sound_play_contact(int32_t distance_mm) { (void)distance_mm; }
void sound_set_volume(int percent) { (void)percent; }
void sound_set_enabled(bool enabled) { (void)enabled; }
bool sound_is_enabled(void) { return false; }

#endif /* CONFIG_SOUND_ENABLED */

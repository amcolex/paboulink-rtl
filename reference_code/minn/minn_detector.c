#include "minn_detector.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <complex.h>
#include <stddef.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define MINN_EPSILON 1e-12f

static inline float complex_abs_sq(complex_t x) {
    float re = crealf(x);
    float im = cimagf(x);
    return re * re + im * im;
}

static inline complex_t conj_complex(complex_t x) {
    return conjf(x);
}

static void minn_detector_clear_state(minn_detector_t *det) {
    det->ring_head = 0;
    det->ring_count = 0;
    det->total_samples = 0;
    det->oldest_index = 0;

    memset(det->corr_c1, 0, sizeof(det->corr_c1));
    memset(det->corr_c2, 0, sizeof(det->corr_c2));
    memset(det->energy_q1, 0, sizeof(det->energy_q1));
    memset(det->energy_q2, 0, sizeof(det->energy_q2));
    memset(det->energy_q3, 0, sizeof(det->energy_q3));
    det->sums_initialized = false;
    det->current_window_start = 0;
    det->next_metric_index = 0;

    det->metric_window_head = 0;
    det->metric_window_count = 0;
    det->metric_window_sum = 0.0f;
    det->norm_initialized = false;
    det->norm_noise_estimate = 0.0f;

    det->detection_triggered = false;
    det->detection_index = 0;
    det->cfo_valid = false;
    det->last_cfo_estimate_hz = 0.0f;
    det->gate_active = false;
    det->gate_start_index = 0;
    det->gate_peak_index = 0;
    det->gate_peak_metric = 0.0f;
    det->gate_peak_total_c = 0.0f + 0.0f * I;
    det->gate_peak_queue_pos = SIZE_MAX;
    det->pending.size = 0;
    det->pending.head = 0;
    det->pending.tail = 0;
    det->history.size = 0;
    det->history.head = 0;
}

static inline size_t ring_base_index(const minn_detector_t *det) {
    return (det->ring_head + det->ring_capacity - det->ring_count) % det->ring_capacity;
}

static complex_t ring_get(const minn_detector_t *det, int antenna, uint64_t sample_index) {
    if (sample_index < det->oldest_index || sample_index >= det->total_samples) {
        return 0.0f + 0.0f * I;
    }
    size_t offset = (size_t)(sample_index - det->oldest_index);
    size_t base = ring_base_index(det);
    size_t pos = (base + offset) % det->ring_capacity;
    return det->ring[antenna][pos];
}

static void ring_push(minn_detector_t *det, complex_t sample0, complex_t sample1) {
    det->ring[0][det->ring_head] = sample0;
    det->ring[1][det->ring_head] = sample1;
    det->ring_head = (det->ring_head + 1) % det->ring_capacity;

    if (det->ring_count < det->ring_capacity) {
        det->ring_count++;
    } else {
        det->oldest_index++;
    }

    det->total_samples++;
}

static inline size_t ring_pos_from_offset(const minn_detector_t *det, size_t base, size_t offset) {
    const size_t capacity = det->ring_capacity;
    if (capacity == 0) {
        return 0;
    }
    if (offset >= capacity) {
        offset -= capacity;
    }
    size_t pos = base + offset;
    if (pos >= capacity) {
        pos -= capacity;
    }
    return pos;
}

static inline size_t ring_increment(size_t index, size_t capacity) {
    index++;
    if (index >= capacity) {
        index = 0;
    }
    return index;
}

static void compute_initial_window(minn_detector_t *det, uint64_t start_index) {
    const size_t Q = det->quarter_len;
    const size_t capacity = det->ring_capacity;
    const size_t base = ring_base_index(det);
    const uint64_t oldest = det->oldest_index;
    size_t start_offset = 0;

    if (start_index >= oldest) {
        start_offset = (size_t)(start_index - oldest);
    }

    for (int ant = 0; ant < 2; ++ant) {
        complex_t c1 = 0.0f + 0.0f * I;
        complex_t c2 = 0.0f + 0.0f * I;
        float e1 = 0.0f;
        float e2 = 0.0f;
        float e3 = 0.0f;

        size_t pos0 = ring_pos_from_offset(det, base, start_offset);
        size_t pos1 = ring_pos_from_offset(det, base, start_offset + Q);
        size_t pos2 = ring_pos_from_offset(det, base, start_offset + 2 * Q);
        size_t pos3 = ring_pos_from_offset(det, base, start_offset + 3 * Q);

        for (size_t i = 0; i < Q; ++i) {
            const complex_t q0 = det->ring[ant][pos0];
            const complex_t q1 = det->ring[ant][pos1];
            const complex_t q2 = det->ring[ant][pos2];
            const complex_t q3 = det->ring[ant][pos3];

            c1 += q0 * conj_complex(q1);
            c2 += q2 * conj_complex(q3);
            e1 += complex_abs_sq(q1);
            e2 += complex_abs_sq(q2);
            e3 += complex_abs_sq(q3);

            pos0 = ring_increment(pos0, capacity);
            pos1 = ring_increment(pos1, capacity);
            pos2 = ring_increment(pos2, capacity);
            pos3 = ring_increment(pos3, capacity);
        }

        det->corr_c1[ant] = c1;
        det->corr_c2[ant] = c2;
        det->energy_q1[ant] = e1;
        det->energy_q2[ant] = e2;
        det->energy_q3[ant] = e3;
    }
    det->current_window_start = start_index;
    det->sums_initialized = true;
}

static void slide_window_forward(minn_detector_t *det, uint64_t new_start) {
    const uint64_t prev_start = det->current_window_start;
    if (new_start <= prev_start) {
        return;
    }
    const size_t Q = det->quarter_len;
    const size_t capacity = det->ring_capacity;
    const size_t base = ring_base_index(det);
    const uint64_t oldest = det->oldest_index;
    const uint64_t steps = new_start - prev_start;

    for (uint64_t step = 0; step < steps; ++step) {
        const uint64_t sample_idx = prev_start + step;
        if (sample_idx < oldest) {
            continue;
        }
        const size_t offset_s = (size_t)(sample_idx - oldest);

        const size_t pos_s = ring_pos_from_offset(det, base, offset_s);
        const size_t pos_sQ = ring_pos_from_offset(det, base, offset_s + Q);
        const size_t pos_s2Q = ring_pos_from_offset(det, base, offset_s + 2 * Q);
        const size_t pos_s3Q = ring_pos_from_offset(det, base, offset_s + 3 * Q);
        const size_t pos_s4Q = ring_pos_from_offset(det, base, offset_s + 4 * Q);

        const complex_t x0_s = det->ring[0][pos_s];
        const complex_t x0_sQ = det->ring[0][pos_sQ];
        const complex_t x0_s2Q = det->ring[0][pos_s2Q];
        const complex_t x0_s3Q = det->ring[0][pos_s3Q];
        const complex_t x0_s4Q = det->ring[0][pos_s4Q];

        det->corr_c1[0] -= x0_s * conj_complex(x0_sQ);
        det->corr_c1[0] += x0_sQ * conj_complex(x0_s2Q);

        det->corr_c2[0] -= x0_s2Q * conj_complex(x0_s3Q);
        det->corr_c2[0] += x0_s3Q * conj_complex(x0_s4Q);

        det->energy_q1[0] -= complex_abs_sq(x0_sQ);
        det->energy_q1[0] += complex_abs_sq(x0_s2Q);

        det->energy_q2[0] -= complex_abs_sq(x0_s2Q);
        det->energy_q2[0] += complex_abs_sq(x0_s3Q);

        det->energy_q3[0] -= complex_abs_sq(x0_s3Q);
        det->energy_q3[0] += complex_abs_sq(x0_s4Q);

        const complex_t x1_s = det->ring[1][pos_s];
        const complex_t x1_sQ = det->ring[1][pos_sQ];
        const complex_t x1_s2Q = det->ring[1][pos_s2Q];
        const complex_t x1_s3Q = det->ring[1][pos_s3Q];
        const complex_t x1_s4Q = det->ring[1][pos_s4Q];

        det->corr_c1[1] -= x1_s * conj_complex(x1_sQ);
        det->corr_c1[1] += x1_sQ * conj_complex(x1_s2Q);

        det->corr_c2[1] -= x1_s2Q * conj_complex(x1_s3Q);
        det->corr_c2[1] += x1_s3Q * conj_complex(x1_s4Q);

        det->energy_q1[1] -= complex_abs_sq(x1_sQ);
        det->energy_q1[1] += complex_abs_sq(x1_s2Q);

        det->energy_q2[1] -= complex_abs_sq(x1_s2Q);
        det->energy_q2[1] += complex_abs_sq(x1_s3Q);

        det->energy_q3[1] -= complex_abs_sq(x1_s3Q);
        det->energy_q3[1] += complex_abs_sq(x1_s4Q);
    }

    det->current_window_start = new_start;
}

static float update_metric_smoothing(minn_detector_t *det, float metric) {
    if (det->smooth_window == 0 || !det->metric_window) {
        return metric;
    }

    if (det->metric_window_count == det->smooth_window) {
        det->metric_window_sum -= det->metric_window[det->metric_window_head];
    } else {
        det->metric_window_count++;
    }

    det->metric_window[det->metric_window_head] = metric;
    det->metric_window_sum += metric;
    det->metric_window_head = (det->metric_window_head + 1) % det->smooth_window;

    return det->metric_window_sum / (float)det->metric_window_count;
}

static float apply_adaptive_normalization(minn_detector_t *det, float smoothed_metric) {
    if (!det->normalize_enabled) {
        return smoothed_metric;
    }

    const float floor = det->norm_floor > MINN_EPSILON ? det->norm_floor : MINN_EPSILON;

    if (!det->norm_initialized) {
        det->norm_noise_estimate = fmaxf(smoothed_metric, floor);
        det->norm_initialized = true;
    } else {
        float capped = smoothed_metric;
        if (det->norm_learn_limit > 0.0f && det->norm_noise_estimate > 0.0f) {
            const float limit = det->norm_noise_estimate * det->norm_learn_limit;
            if (limit > 0.0f && capped > limit) {
                capped = limit;
            }
        }

        float alpha = det->norm_alpha;
        if (!(alpha > 0.0f && alpha <= 1.0f)) {
            alpha = 0.01f;
        }

        float target = fmaxf(capped, floor);

        float applied_alpha = alpha;
        if (det->gate_active && target > det->norm_noise_estimate) {
            applied_alpha = alpha * 0.1f;
            if (applied_alpha < 1e-5f) {
                applied_alpha = alpha;
            }
        }

        det->norm_noise_estimate =
            (1.0f - applied_alpha) * det->norm_noise_estimate + applied_alpha * target;
    }

    const float denom = smoothed_metric + det->norm_noise_estimate + floor;
    if (denom <= MINN_EPSILON) {
        return 0.0f;
    }

    float normalized = smoothed_metric / denom;
    if (normalized < 0.0f) normalized = 0.0f;
    if (normalized > 1.0f) normalized = 1.0f;
    return normalized;
}

static int pending_ensure_capacity(minn_detector_t *det, size_t needed) {
    if (needed <= det->pending.capacity) {
        return 0;
    }

    size_t new_cap = det->pending.capacity ? det->pending.capacity : det->window_len;
    if (new_cap == 0) {
        new_cap = det->window_len ? det->window_len : 1;
    }
    while (new_cap < needed) {
        new_cap *= 2;
    }

    float *new_metrics = (float*)malloc(new_cap * sizeof(float));
    if (!new_metrics) return -1;
    complex_t *new_ch0 = (complex_t*)malloc(new_cap * sizeof(complex_t));
    if (!new_ch0) { free(new_metrics); return -1; }
    complex_t *new_ch1 = (complex_t*)malloc(new_cap * sizeof(complex_t));
    if (!new_ch1) { free(new_metrics); free(new_ch0); return -1; }
    uint8_t *new_flags = (uint8_t*)malloc(new_cap * sizeof(uint8_t));
    if (!new_flags) { free(new_metrics); free(new_ch0); free(new_ch1); return -1; }
    float *new_cfo = (float*)malloc(new_cap * sizeof(float));
    if (!new_cfo) { free(new_metrics); free(new_ch0); free(new_ch1); free(new_flags); return -1; }

    const size_t old_cap = det->pending.capacity;
    const size_t size = det->pending.size;
    for (size_t i = 0; i < size; ++i) {
        size_t idx = 0;
        if (old_cap != 0) {
            idx = (det->pending.head + i) % old_cap;
        }
        new_metrics[i] = det->pending.metrics ? det->pending.metrics[idx] : 0.0f;
        new_ch0[i] = det->pending.ch0 ? det->pending.ch0[idx] : 0.0f + 0.0f * I;
        new_ch1[i] = det->pending.ch1 ? det->pending.ch1[idx] : 0.0f + 0.0f * I;
        new_flags[i] = det->pending.flags ? det->pending.flags[idx] : 0;
        new_cfo[i] = det->pending.cfo ? det->pending.cfo[idx] : 0.0f;
    }

    free(det->pending.metrics);
    free(det->pending.ch0);
    free(det->pending.ch1);
    free(det->pending.flags);
    free(det->pending.cfo);

    det->pending.metrics = new_metrics;
    det->pending.ch0 = new_ch0;
    det->pending.ch1 = new_ch1;
    det->pending.flags = new_flags;
    det->pending.cfo = new_cfo;
    det->pending.capacity = new_cap;
    det->pending.head = 0;
    det->pending.tail = size % new_cap;
    return 0;
}

static inline size_t pending_relative_to_phys(const minn_detector_t *det, size_t relative_index) {
    if (det->pending.capacity == 0) {
        return 0;
    }
    return (det->pending.head + relative_index) % det->pending.capacity;
}

static void history_push(minn_detector_t *det,
                         float smoothed_metric,
                         float normalized_metric) {
    if (!det || !det->history.smoothed || det->history.capacity == 0) {
        return;
    }

    if (det->history.size < det->history.capacity) {
        size_t idx = (det->history.head + det->history.size) % det->history.capacity;
        det->history.smoothed[idx] = smoothed_metric;
        if (det->history.normalized) {
            det->history.normalized[idx] = normalized_metric;
        }
        det->history.size++;
    } else {
        size_t idx = det->history.head;
        det->history.smoothed[idx] = smoothed_metric;
        if (det->history.normalized) {
            det->history.normalized[idx] = normalized_metric;
        }
        det->history.head = (det->history.head + 1) % det->history.capacity;
    }
}

static int pending_push(minn_detector_t *det,
                        complex_t sample0,
                        complex_t sample1,
                        float metric,
                        uint8_t flag,
                        float cfo_hz,
                        size_t *out_pos)
{
    if (pending_ensure_capacity(det, det->pending.size + 1) != 0) {
        return -1;
    }
    const size_t relative_pos = det->pending.size;
    const size_t pos = det->pending.tail;
    det->pending.ch0[pos] = sample0;
    det->pending.ch1[pos] = sample1;
    det->pending.metrics[pos] = metric;
    det->pending.flags[pos] = flag;
    det->pending.cfo[pos] = cfo_hz;
    det->pending.size++;
    det->pending.tail = (det->pending.tail + 1) % det->pending.capacity;
    if (out_pos) {
        *out_pos = relative_pos;
    }
    return 0;
}

static bool pending_can_pop(const minn_detector_t *det) {
    if (det->gate_active) {
        return false;
    }
    return det->pending.size > det->output_delay;
}

static bool pending_pop(minn_detector_t *det,
                        complex_t *out_ch0,
                        complex_t *out_ch1,
                        float *out_metric,
                        uint8_t *out_flag,
                        float *out_cfo)
{
    if (det->pending.size == 0) {
        return false;
    }
    const size_t idx = det->pending.head;
    if (out_ch0) *out_ch0 = det->pending.ch0[idx];
    if (out_ch1) *out_ch1 = det->pending.ch1[idx];
    if (out_metric) *out_metric = det->pending.metrics[idx];
    if (out_flag) *out_flag = det->pending.flags[idx];
    if (out_cfo) *out_cfo = det->pending.cfo[idx];

    det->pending.head = (det->pending.head + 1) % det->pending.capacity;
    det->pending.size--;
    if (det->pending.size == 0) {
        det->pending.head = 0;
        det->pending.tail = 0;
    }
    return true;
}

int minn_detector_init(minn_detector_t *detector,
                       const mconfig_t *config) {
    if (!detector || !config) {
        return -1;
    }
    if (config->ofdm.derived.fft_size <= 0 || (config->ofdm.derived.fft_size % 4) != 0) {
        return -2;
    }

    memset(detector, 0, sizeof(*detector));
    detector->config = config;
    detector->fft_size = (size_t)config->ofdm.derived.fft_size;
    detector->window_len = detector->fft_size;
    detector->quarter_len = detector->fft_size / 4;
    detector->ring_capacity = detector->window_len + 1;
    const minn_detector_config_params_t *minn_params = &config->minn.params;
    detector->smooth_window = minn_params->smooth_window > 0 ? minn_params->smooth_window : 1;
    detector->gate_threshold = minn_params->gate_threshold;
    if (detector->gate_threshold < 0.0f) detector->gate_threshold = 0.0f;
    if (detector->gate_threshold > 1.0f) detector->gate_threshold = 1.0f;
    detector->min_metric = minn_params->min_metric > 0.0f ? minn_params->min_metric : 0.0f;
    detector->timing_offset = minn_params->timing_offset;
    detector->normalize_enabled = minn_params->enable_adaptive_norm;
    if (detector->normalize_enabled) {
        const float alpha = minn_params->norm_alpha;
        detector->norm_alpha = (alpha > 0.0f && alpha <= 1.0f) ? alpha : 0.01f;
        const float floor = minn_params->norm_floor;
        detector->norm_floor = floor > MINN_EPSILON ? floor : 1e-5f;
        const float clamp = minn_params->norm_learn_limit;
        detector->norm_learn_limit = clamp >= 1.0f ? clamp : 1.0f;
    } else {
        detector->norm_alpha = 0.0f;
        detector->norm_floor = 0.0f;
        detector->norm_learn_limit = 0.0f;
    }

    detector->ring[0] = calloc(detector->ring_capacity, sizeof(complex_t));
    detector->ring[1] = calloc(detector->ring_capacity, sizeof(complex_t));
    detector->metric_window = calloc(detector->smooth_window, sizeof(float));

    if (!detector->ring[0] || !detector->ring[1] || !detector->metric_window) {
        minn_detector_free(detector);
        return -3;
    }

    detector->output_delay = detector->window_len;
    if (detector->output_delay == 0) detector->output_delay = 1;
    detector->pending.capacity = detector->output_delay + detector->smooth_window + 4;
    detector->pending.metrics = (float*)calloc(detector->pending.capacity, sizeof(float));
    detector->pending.ch0 = (complex_t*)calloc(detector->pending.capacity, sizeof(complex_t));
    detector->pending.ch1 = (complex_t*)calloc(detector->pending.capacity, sizeof(complex_t));
    detector->pending.flags = (uint8_t*)calloc(detector->pending.capacity, sizeof(uint8_t));
    detector->pending.cfo = (float*)calloc(detector->pending.capacity, sizeof(float));
    if (!detector->pending.metrics || !detector->pending.ch0 ||
        !detector->pending.ch1 || !detector->pending.flags || !detector->pending.cfo) {
        minn_detector_free(detector);
        return -3;
    }

    size_t history_cap = detector->window_len * 8;
    if (history_cap < detector->window_len + detector->smooth_window + 64) {
        history_cap = detector->window_len + detector->smooth_window + 64;
    }
    if (history_cap < 1024) {
        history_cap = 1024;
    }
    detector->history.capacity = history_cap;
    detector->history.smoothed = (float*)calloc(history_cap, sizeof(float));
    detector->history.normalized = (float*)calloc(history_cap, sizeof(float));
    if (!detector->history.smoothed || !detector->history.normalized) {
        minn_detector_free(detector);
        return -3;
    }
    detector->history.size = 0;
    detector->history.head = 0;

    minn_detector_clear_state(detector);
    return 0;
}

void minn_detector_reset(minn_detector_t *detector) {
    if (!detector) {
        return;
    }
    minn_detector_clear_state(detector);
}

void minn_detector_free(minn_detector_t *detector) {
    if (!detector) {
        return;
    }
    if (detector->ring[0]) {
        free(detector->ring[0]);
        detector->ring[0] = NULL;
    }
    if (detector->ring[1]) {
        free(detector->ring[1]);
        detector->ring[1] = NULL;
    }
    if (detector->metric_window) {
        free(detector->metric_window);
        detector->metric_window = NULL;
    }
    if (detector->pending.metrics) {
        free(detector->pending.metrics);
        detector->pending.metrics = NULL;
    }
    if (detector->pending.ch0) {
        free(detector->pending.ch0);
        detector->pending.ch0 = NULL;
    }
    if (detector->pending.ch1) {
        free(detector->pending.ch1);
        detector->pending.ch1 = NULL;
    }
    if (detector->pending.flags) {
        free(detector->pending.flags);
        detector->pending.flags = NULL;
    }
    if (detector->pending.cfo) {
        free(detector->pending.cfo);
        detector->pending.cfo = NULL;
    }
    if (detector->history.smoothed) {
        free(detector->history.smoothed);
        detector->history.smoothed = NULL;
    }
    if (detector->history.normalized) {
        free(detector->history.normalized);
        detector->history.normalized = NULL;
    }
    detector->history.capacity = 0;
    detector->history.size = 0;
    detector->history.head = 0;
    detector->ring_capacity = 0;
    detector->smooth_window = 0;
    detector->pending.capacity = 0;
    minn_detector_clear_state(detector);
}

bool minn_detector_has_detection(const minn_detector_t *detector) {
    if (!detector) {
        return false;
    }
    return detector->detection_triggered;
}

uint64_t minn_detector_get_detection_index(const minn_detector_t *detector) {
    if (!detector || !detector->detection_triggered) {
        return 0;
    }
    return detector->detection_index;
}

static size_t minn_detector_process_one(minn_detector_t *detector,
                                        complex_t sample_ch0,
                                        complex_t sample_ch1,
                                        minn_detector_output_t *out) {
    if (!detector) {
        return 0;
    }

    ring_push(detector, sample_ch0, sample_ch1);

    while (detector->next_metric_index + detector->window_len <= detector->total_samples) {
        uint64_t start_idx = detector->next_metric_index;

        if (!detector->sums_initialized) {
            compute_initial_window(detector, start_idx);
        } else {
            slide_window_forward(detector, start_idx);
        }

        complex_t total_c = (detector->corr_c1[0] + detector->corr_c2[0]) +
                            (detector->corr_c1[1] + detector->corr_c2[1]);

        float total_energy = (detector->energy_q1[0] + detector->energy_q2[0] + detector->energy_q3[0]) +
                             (detector->energy_q1[1] + detector->energy_q2[1] + detector->energy_q3[1]);

        float aligned_real = fmaxf(crealf(total_c), 0.0f);
        float denom = fmaxf(total_energy, MINN_EPSILON);
        float metric = (aligned_real * aligned_real) / (denom * denom);
        float positive_metric = metric > 0.0f ? metric : 0.0f;
        float smoothed = update_metric_smoothing(detector, positive_metric);
        float gating_metric = apply_adaptive_normalization(detector, smoothed);
        float peak_metric = smoothed;

        history_push(detector, smoothed, gating_metric);

        complex_t sample0 = ring_get(detector, 0, start_idx);
        complex_t sample1 = ring_get(detector, 1, start_idx);
        const bool above_threshold = (gating_metric >= detector->gate_threshold) &&
                                     (gating_metric >= detector->min_metric);
        
        // Store smoothed Minn metric for downstream analysis/visualization
        float queue_metric = smoothed;

        // Push sample into pending queue (flag/cfo will be updated later if this becomes the detection peak)
        size_t queue_pos = 0;
        if (pending_push(detector, sample0, sample1, queue_metric, 0, 0.0f, &queue_pos) != 0) {
            return false;
        }

        if (!detector->detection_triggered) {
            if (detector->gate_active) {
                if (above_threshold) {
                    if (peak_metric > detector->gate_peak_metric) {
                        detector->gate_peak_metric = peak_metric;
                        detector->gate_peak_index = start_idx;
                        detector->gate_peak_total_c = total_c;
                        detector->gate_peak_queue_pos = queue_pos;
                    }
                } else {
                    detector->gate_active = false;
                    detector->detection_triggered = true;
                    
                    // Apply timing offset to detection index
                    int64_t shifted_idx = (int64_t)detector->gate_peak_index + (int64_t)detector->timing_offset;
                    if (shifted_idx < 0) shifted_idx = 0;
                    detector->detection_index = (uint64_t)shifted_idx;

                    const float angle = atan2f(cimagf(detector->gate_peak_total_c), crealf(detector->gate_peak_total_c));
                    if (detector->quarter_len > 0 && detector->config->ofdm.derived.sample_rate > 0.0f) {
                        const float denom_q = (float)detector->quarter_len;
                        detector->last_cfo_estimate_hz = -angle * detector->config->ofdm.derived.sample_rate / (2.0f * (float)M_PI * denom_q);
                        detector->cfo_valid = true;
                    } else {
                        detector->last_cfo_estimate_hz = 0.0f;
                        detector->cfo_valid = false;
                    }

                    // Find the queue position for the shifted detection index
                    // The queue_pos was for gate_peak_index, we need to adjust for timing offset
                    int64_t flag_pos = (int64_t)detector->gate_peak_queue_pos + (int64_t)detector->timing_offset;
                    
                    if (flag_pos >= 0 && (size_t)flag_pos < detector->pending.size) {
                        size_t idx = pending_relative_to_phys(detector, (size_t)flag_pos);
                        detector->pending.flags[idx] = 1;
                        detector->pending.cfo[idx] = detector->cfo_valid ? detector->last_cfo_estimate_hz : 0.0f;
                    }
                    detector->gate_peak_queue_pos = SIZE_MAX;
                }
            }

            if (!detector->gate_active && !detector->detection_triggered && above_threshold) {
                detector->gate_active = true;
                detector->gate_start_index = start_idx;
                detector->gate_peak_metric = peak_metric;
                detector->gate_peak_index = start_idx;
                detector->gate_peak_total_c = total_c;
                detector->gate_peak_queue_pos = queue_pos;
            }
        }

        if (detector->detection_triggered && detector->gate_peak_queue_pos == SIZE_MAX && start_idx == detector->detection_index) {
            // Detection determined in this iteration for the just-pushed sample
            size_t idx = pending_relative_to_phys(detector, queue_pos);
            detector->pending.flags[idx] = 1;
            detector->pending.cfo[idx] = detector->cfo_valid ? detector->last_cfo_estimate_hz : 0.0f;
        }

        detector->next_metric_index++;
    }

    if (!pending_can_pop(detector)) {
        return 0;
    }

    minn_detector_output_t local_out;
    minn_detector_output_t *dst = out ? out : &local_out;
    if (!pending_pop(detector,
                     &dst->sample_ch0,
                     &dst->sample_ch1,
                     &dst->metric,
                     &dst->flag,
                     &dst->cfo_hz)) {
        return 0;
    }

    return 1;
}

bool minn_detector_process(minn_detector_t *detector,
                           complex_t sample_ch0,
                           complex_t sample_ch1,
                           complex_t *out_ch0,
                           complex_t *out_ch1,
                           uint8_t *out_flag,
                           float *out_metric,
                           float *out_cfo_hz) {
    minn_detector_output_t result;
    size_t produced = minn_detector_process_one(detector,
                                                sample_ch0,
                                                sample_ch1,
                                                &result);
    if (produced == 0) {
        return false;
    }

    if (out_ch0) *out_ch0 = result.sample_ch0;
    if (out_ch1) *out_ch1 = result.sample_ch1;
    if (out_metric) *out_metric = result.metric;
    if (out_flag) *out_flag = result.flag;
    if (out_cfo_hz) *out_cfo_hz = result.cfo_hz;

    return true;
}

size_t minn_detector_process_block(minn_detector_t *detector,
                                   const complex_t *samples_ch0,
                                   const complex_t *samples_ch1,
                                   size_t count,
                                   minn_detector_output_t *outputs,
                                   size_t max_outputs) {
    if (!detector || !samples_ch0 || !samples_ch1 || count == 0) {
        return 0;
    }

    size_t emitted = 0;
    for (size_t i = 0; i < count; ++i) {
        minn_detector_output_t temp;
        minn_detector_output_t *slot = NULL;
        if (outputs && emitted < max_outputs) {
            slot = &outputs[emitted];
        }
        size_t produced = minn_detector_process_one(detector,
                                                    samples_ch0[i],
                                                    samples_ch1[i],
                                                    slot ? slot : &temp);
        if (produced) {
            emitted += produced;
        }
    }

    return emitted;
}

int minn_detector_get_snapshot(const minn_detector_t *detector,
                                minn_detector_snapshot_t *snapshot) {
    if (!detector || !snapshot) {
        return -1;
    }
    
    memset(snapshot, 0, sizeof(*snapshot));
    
    const size_t history_count = detector->history.size;
    if (history_count > 0 && detector->history.smoothed) {
        float *raw_metrics = (float*)malloc(history_count * sizeof(float));
        if (!raw_metrics) {
            return -2;
        }

        float *normalized_metrics = NULL;
        if (detector->history.normalized) {
            normalized_metrics = (float*)malloc(history_count * sizeof(float));
            if (!normalized_metrics) {
                // Allocation failure for normalized metrics is non-fatal; continue with raw only.
                normalized_metrics = NULL;
            }
        }

        for (size_t i = 0; i < history_count; ++i) {
            size_t idx = (detector->history.head + i) % detector->history.capacity;
            raw_metrics[i] = detector->history.smoothed[idx];
            if (normalized_metrics) {
                normalized_metrics[i] = detector->history.normalized[idx];
            }
        }

        snapshot->raw_metrics = raw_metrics;
        snapshot->smoothed_metrics = normalized_metrics;
        snapshot->metric_count = history_count;
    }
    
    // Capture detection parameters
    snapshot->gate_threshold = detector->gate_threshold;
    snapshot->peak_metric = detector->gate_peak_metric;
    snapshot->cfo_estimate_hz = detector->last_cfo_estimate_hz;
    snapshot->adaptive_norm_enabled = detector->normalize_enabled;
    snapshot->noise_estimate = detector->norm_noise_estimate;
    
    // Find peak index within the snapshot
    snapshot->peak_index = 0;
    if (snapshot->raw_metrics && snapshot->metric_count > 0) {
        float max_val = snapshot->raw_metrics[0];
        for (size_t i = 1; i < snapshot->metric_count; ++i) {
            if (snapshot->raw_metrics[i] > max_val) {
                max_val = snapshot->raw_metrics[i];
                snapshot->peak_index = i;
            }
        }
    }
    
    return 0;
}

void minn_detector_free_snapshot(minn_detector_snapshot_t *snapshot) {
    if (!snapshot) {
        return;
    }
    
    if (snapshot->raw_metrics) {
        free(snapshot->raw_metrics);
        snapshot->raw_metrics = NULL;
    }
    
    if (snapshot->smoothed_metrics) {
        free(snapshot->smoothed_metrics);
        snapshot->smoothed_metrics = NULL;
    }
    
    memset(snapshot, 0, sizeof(*snapshot));
}

#ifndef MINN_DETECTOR_H
#define MINN_DETECTOR_H

#include "types.h"
#include "modem_config.h"
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Snapshot of Minn detector state at detection moment
 */
typedef struct {
    float *raw_metrics;           // Minn metric after smoothing (before adaptive normalization)
    float *smoothed_metrics;      // Adaptive-normalized metric values (optional)
    size_t metric_count;          // Number of metrics in the snapshot
    float gate_threshold;         // Threshold used for detection
    float peak_metric;            // Peak metric value
    size_t peak_index;            // Index of peak within snapshot
    float cfo_estimate_hz;        // CFO estimate at detection
    float noise_estimate;         // Adaptive noise estimate (if enabled)
    bool adaptive_norm_enabled;   // Whether adaptive normalization was used
} minn_detector_snapshot_t;

typedef struct {
    // Configuration
    const mconfig_t *config;
    size_t fft_size;
    size_t window_len;
    size_t quarter_len;

    // Sample history (ring buffer)
    complex_t *ring[2];
    size_t ring_capacity;
    size_t ring_head;
    size_t ring_count;
    uint64_t total_samples;
    uint64_t oldest_index;

    // Running correlation / energy statistics per branch
    complex_t corr_c1[2];
    complex_t corr_c2[2];
    float energy_q1[2];
    float energy_q2[2];
    float energy_q3[2];
    bool sums_initialized;
    uint64_t current_window_start;
    uint64_t next_metric_index;

    // Metric smoothing
    size_t smooth_window;
    float gate_threshold;
    float min_metric;
    int timing_offset;
    float *metric_window;
    size_t metric_window_head;
    size_t metric_window_count;
    float metric_window_sum;

    // Adaptive normalization
    bool normalize_enabled;
    bool norm_initialized;
    float norm_alpha;
    float norm_floor;
    float norm_learn_limit;
    float norm_noise_estimate;

    // Detection state
    bool detection_triggered;
    uint64_t detection_index;
    bool cfo_valid;
    float last_cfo_estimate_hz;
    bool gate_active;
    uint64_t gate_start_index;
    uint64_t gate_peak_index;
    float gate_peak_metric;
    complex_t gate_peak_total_c;
    size_t gate_peak_queue_pos;
    size_t output_delay;
    struct {
        size_t capacity;
        size_t size;
        size_t head;
        size_t tail;
        float *metrics;
        complex_t *ch0;
        complex_t *ch1;
        uint8_t *flags;
        float *cfo;
    } pending;
    struct {
        float *smoothed;          // Metric after smoothing but before normalization
        float *normalized;        // Metric after adaptive normalization (if enabled)
        size_t capacity;          // Total history capacity
        size_t size;              // Number of valid entries in history
        size_t head;              // Index of oldest entry
    } history;
} minn_detector_t;

typedef struct {
    complex_t sample_ch0;
    complex_t sample_ch1;
    float metric;
    uint8_t flag;
    float cfo_hz;
} minn_detector_output_t;

int minn_detector_init(minn_detector_t *detector,
                       const mconfig_t *config);

void minn_detector_reset(minn_detector_t *detector);

void minn_detector_free(minn_detector_t *detector);

bool minn_detector_has_detection(const minn_detector_t *detector);

uint64_t minn_detector_get_detection_index(const minn_detector_t *detector);

bool minn_detector_process(minn_detector_t *detector,
                           complex_t sample_ch0,
                           complex_t sample_ch1,
                           complex_t *out_ch0,
                           complex_t *out_ch1,
                           uint8_t *out_flag,
                           float *out_metric,
                           float *out_cfo_hz);

size_t minn_detector_process_block(minn_detector_t *detector,
                                   const complex_t *samples_ch0,
                                   const complex_t *samples_ch1,
                                   size_t count,
                                   minn_detector_output_t *outputs,
                                   size_t max_outputs);

/**
 * @brief Create a snapshot of the detector's metric history
 *
 * This captures the circular buffer state at the moment of detection,
 * useful for visualization and debugging.
 *
 * @param detector Pointer to detector
 * @param snapshot Pointer to snapshot structure to fill
 * @return 0 on success, negative on error
 */
int minn_detector_get_snapshot(const minn_detector_t *detector,
                                minn_detector_snapshot_t *snapshot);

/**
 * @brief Free resources allocated in a snapshot
 *
 * @param snapshot Pointer to snapshot to free
 */
void minn_detector_free_snapshot(minn_detector_snapshot_t *snapshot);

#ifdef __cplusplus
}
#endif

#endif // MINN_DETECTOR_H

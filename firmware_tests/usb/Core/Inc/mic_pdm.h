#ifndef MIC_PDM_H
#define MIC_PDM_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
  bool valid;
  bool overflow;
  bool dma_error;
  bool autodetect_failed;
  bool sai_enabled;
  bool dma_enabled;
  bool pdm_enabled;
  bool ckstr;
  uint8_t diag_stage;
  uint8_t sai_init_status;
  uint8_t dma_init_status;
  uint8_t rx_start_status;
  uint8_t sai_state;
  uint8_t dma_state;
  uint32_t capture_ms;
  uint32_t pdm_words;
  uint32_t pcm_samples;
  uint32_t block_count;
  uint32_t sai_clk_hz;
  uint32_t sai_error;
  uint32_t dma_cndtr;
  uint32_t sai_sr;
  uint32_t raw_zero_words;
  uint32_t raw_full_words;
  uint32_t raw_first_word;
  uint32_t raw_last_word;
  uint16_t peak_abs;
  uint16_t avg_abs;
  int32_t avg_signed;
  uint8_t raw_popcount_min;
  uint8_t raw_popcount_max;
  int16_t last_sample;
  uint32_t goertzel_bins_total;
  uint32_t goertzel_bins_active;
  uint32_t spike_clusters;
  uint8_t scan_peak_k;
  uint32_t scan_peak_mag;
  uint32_t scan_target_mag;
} mic_pdm_result_t;

bool mic_pdm_init(void);
bool mic_pdm_start(void);
void mic_pdm_request_stop(void);
void mic_pdm_force_stop(void);
void mic_pdm_task(void);
bool mic_pdm_is_active(void);
bool mic_pdm_take_result(mic_pdm_result_t *result);
void mic_pdm_set_bg_offset(int32_t offset);
int32_t mic_pdm_get_bg_offset(void);
void mic_pdm_start_goertzel_cal(void);
float mic_pdm_finish_goertzel_cal(void);
void mic_pdm_set_goertzel_threshold(float threshold);
float mic_pdm_get_goertzel_threshold(void);
bool mic_pdm_get_scan_averages(uint16_t *out_avg, uint16_t *out_bg, uint32_t count);
void mic_pdm_dma_irqhandler(void);

/* Accumulated FFT: 32 log-spaced bins from ~23 Hz to ~20 kHz.
 * Frequency of bin i: s_afft_k[i] * 46875 / 4096 Hz (see mic_pdm.c for k table).
 * Peak magnitude per bin accumulated over full capture duration.
 */
#define MIC_PDM_FFT_BINS 32u
bool mic_pdm_get_fft_spectrum(uint16_t *mags, uint32_t *count);

/* Debug envelope trace: EMA energy / background energy * 10 per spike window.
 * For current thresholds, value 15 = re-arm and 30 = detect.
 */
#define MIC_PDM_ENVELOPE_TRACE_BINS 384u
bool mic_pdm_get_envelope_trace(const uint16_t **trace, uint32_t *count);

#endif /* MIC_PDM_H */

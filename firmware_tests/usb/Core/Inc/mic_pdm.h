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
void mic_pdm_dma_irqhandler(void);

#endif /* MIC_PDM_H */

#ifndef MIC_PDM_H
#define MIC_PDM_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
  bool valid;
  bool overflow;
  bool dma_error;
  uint32_t capture_ms;
  uint32_t pdm_words;
  uint32_t pcm_samples;
  uint32_t block_count;
  uint16_t peak_abs;
  uint16_t avg_abs;
  uint32_t hf15_energy;
  int16_t last_sample;
} mic_pdm_result_t;

bool mic_pdm_init(void);
bool mic_pdm_start(void);
void mic_pdm_request_stop(void);
void mic_pdm_task(void);
bool mic_pdm_is_active(void);
bool mic_pdm_take_result(mic_pdm_result_t *result);
void mic_pdm_dma_irqhandler(void);

#endif /* MIC_PDM_H */

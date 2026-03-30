#ifndef DRIVERS_DISPLAY_GDEM0097T61_H_
#define DRIVERS_DISPLAY_GDEM0097T61_H_

#include <stdbool.h>
#include <stdint.h>

#include "main.h"

#define GDEM0097T61_WIDTH       88u
#define GDEM0097T61_HEIGHT      184u
#define GDEM0097T61_FRAME_BYTES ((GDEM0097T61_WIDTH * GDEM0097T61_HEIGHT) / 8u)

typedef enum
{
  GDEM0097T61_STATE_UNINITIALIZED = 0,
  GDEM0097T61_STATE_IDLE,
  GDEM0097T61_STATE_BUSY,
  GDEM0097T61_STATE_ERROR
} gdem0097t61_state_t;

bool gdem0097t61_init(SPI_HandleTypeDef *spi);
void gdem0097t61_task(void);
bool gdem0097t61_request_refresh(const uint8_t *framebuffer);
void gdem0097t61_on_busy_exti(void);
void gdem0097t61_on_spi_tx_complete(SPI_HandleTypeDef *hspi);
bool gdem0097t61_is_busy(void);
bool gdem0097t61_is_idle(void);
bool gdem0097t61_get_state(gdem0097t61_state_t *state);

#endif /* DRIVERS_DISPLAY_GDEM0097T61_H_ */

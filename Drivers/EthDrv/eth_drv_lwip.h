#ifndef ETHDRV_ETH_DRV_LWIP
#define ETHDRV_ETH_DRV_LWIP

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    bool init;
    bool up;
    uint16_t speed;
    bool duplex;
} LinkState;

#endif /* ETHDRV_ETH_DRV_LWIP */

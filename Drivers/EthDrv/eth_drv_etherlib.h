#ifndef ETHDRV_ETH_DRV_ETHERLIB
#define ETHDRV_ETH_DRV_ETHERLIB

#include <etherlib/packet.h>
#include <etherlib/msg_queue.h>

#include <stdint.h>

typedef struct {
    bool init;
    bool up;
    uint16_t speed;
    bool duplex;
} LinkState;

struct EthInterface_;
struct EthIODef_;

void ethdrv_init();
int ethdrv_input(RawPckt * pckt);
int ethdrv_output(const RawPckt * pckt);

int ethdrv_send(struct EthIODef_ * io, MsgQueue * mq);
struct EthIODef_ * ethdrv_get();

#endif /* ETHDRV_ETH_DRV_ETHERLIB */

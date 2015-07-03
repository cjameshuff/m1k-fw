
#ifndef _BULK_SAMPLING_H_
#define _BULK_SAMPLING_H_

void config_bulk_sampling(uint16_t period, uint16_t sync);

void bulk_set_interleave(bool interleave);

void enable_bulk_transfers(void);

void handle_bulk_transfers(void);

void poll_trigger(void);

#endif // _BULK_SAMPLING_H_

#ifndef JANUS_MSB_H_INCLUDED_
#define JANUS_MSB_H_INCLUDED_

#include <stdint.h>

#define BARR_ELBITS (8)
#define BARR_ARRAYSIZE(N) (((N) +BARR_ELBITS-1) / BARR_ELBITS)
#define BARR_ELNUM(N) ((N) / BARR_ELBITS)
#define BARR_BITNUM(N) ((N) % BARR_ELBITS)
#define BARR_SET(barr, N) (((char*)(barr))[BARR_ELNUM(N)] |= 0x080 >> BARR_BITNUM(N))
#define BARR_CLEAR(barr, N) (((char*)(barr))[BARR_ELNUM(N)] &= ~(0x080 >> BARR_BITNUM(N)))
#define BARR_FLIP(barr, N) (((char*)(barr))[BARR_ELNUM(N)] ^= 0x080 >> BARR_BITNUM(N))
#define BARR_TEST(barr, N) (((char*)(barr))[BARR_ELNUM(N)] & (0x080 >> BARR_BITNUM(N)))
#define SWAP32_BY_BITSIZE(data, bit_size) swap((data), ((bit_size) + 7) / 8)

int int2barr(char *barr, int offset, void *val, unsigned int l, unsigned int h);
int barr2int(char *barr, int offset, void *val, unsigned int l, unsigned int h);
int swap(int32_t from, size_t len);
void log_barr(char *barr, int len);

#endif

#include <stdlib.h>
#include <stdio.h>

#include <janus/interleave.h>

int
main(int argc, char** argv)
{
  unsigned dat_len = 0;
  unsigned res_len = 0;
  janus_uint8_t dat[1024] = {0};
  janus_uint8_t res[1024] = {0};
  FILE* ifd = 0;
  FILE* rfd = 0;
  int matches = 0;
  janus_uint8_t* out = 0;
  int rv = 0;
  unsigned i = 0;

  if (argc != 3)
  {
    fprintf(stderr, "ERROR: this program requires two file names as argument.");
    return 1;
  }

  // Read input.
  ifd = fopen(argv[1], "r");
  while(!feof(ifd))
    matches = fscanf(ifd, "%hhu\n", dat + dat_len++);
  fclose(ifd);
  
  // Read result.
  rfd = fopen(argv[2], "r");
  while(!feof(rfd))
    matches = fscanf(rfd, "%hhu\n", res + res_len++);
  fclose(rfd);
  
  if (res_len != dat_len)
  {
    fprintf(stderr, "ERROR: data set is invalid.\n");
    return 1;
  }
  
  out = (janus_uint8_t*)calloc(sizeof(janus_uint8_t), dat_len);
  janus_interleave(dat, dat_len, out);
  
  rv = 0;
  i = 0;
  for (i = 0; i < res_len; ++i)
  {
    if (res[i] != out[i])
    {
      fprintf(stderr, "ERROR: mismatch in line %d: %d != %d.\n", i, out[i], res[i]);
      rv = 1;
    }
  }

  return rv;
}

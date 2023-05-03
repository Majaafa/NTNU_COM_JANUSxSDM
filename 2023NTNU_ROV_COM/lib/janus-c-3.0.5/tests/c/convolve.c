#include <stdio.h>
#include <stdlib.h>
#include <janus/janus.h>

int
main(int argc, char** argv)
{
  if (argc != 3)
  {
    fprintf(stderr, "ERROR: this program requires two file names as argument.\n");
    return 1;
  }

  // Read input.
  {
    unsigned dat_len = 0;
    janus_uint8_t dat[1024] = {0};
    FILE* ifd = fopen(argv[1], "r");
    int matches = 0;
    while(!feof(ifd))
    {
      matches = fscanf(ifd, "%hhu\n", dat + dat_len);
      ++dat_len;
    }
    fclose(ifd);
  }
  
  // Read result.
  {
    unsigned res_len = 0;
    janus_uint8_t res[1024] = {0};
    FILE* rfd = fopen(argv[2], "r");
    int matches = 0;
    while(!feof(rfd))
    {
      matches = fscanf(rfd, "%hhu\n", res + res_len);
      ++res_len;
    }
    fclose(rfd);
  }

  return 0;
}


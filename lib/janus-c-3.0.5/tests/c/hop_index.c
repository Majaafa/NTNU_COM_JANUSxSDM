#include <stdlib.h>
#include <stdio.h>

#include <janus/hop_index.h>

#define ALPHA_MIN 2
#define ALPHA_MAX 3
#define BLOCK_SIZE_MIN 3
#define BLOCK_SIZE_MAX 16
#define M_MIN 144
#define M_MAX 1024

int
main(int argc, char** argv)
{
  char file[256];
  unsigned index = 0;
  unsigned cvalue = 0;
  unsigned tvalue = 0;

  unsigned alpha = 0;
  unsigned block_size = 0;
  unsigned m = 0;

  int matches = 0;

  for (alpha = ALPHA_MIN; alpha <= ALPHA_MAX; ++alpha)
  {
    for (block_size = BLOCK_SIZE_MIN; block_size <= BLOCK_SIZE_MAX; ++block_size)
    {
      for (m = M_MIN; m <= M_MAX; ++m)
      {
        FILE* fd = 0;
        index = 0;

        sprintf(file, "%s/hop_index-%04u-%04u-%04u.dat", argv[1], alpha, block_size, m);
        fd = fopen(file, "r");
        if (fd == 0)
        {
          fprintf(stderr, "ERROR: failed to open test file: %s\n", file);
          exit(1);
        }

        while (!feof(fd))
        {
          matches = fscanf(fd, "%u\n", &tvalue);
          cvalue = janus_hop_index(index, alpha, block_size);

          if (tvalue != cvalue)
          {
            fprintf(stderr, "ERROR: test failed: %s [%u]: %u != %u\n", file, index, cvalue, tvalue);
            exit(1);
          }
          
          ++index;
        }
        fclose(fd);
      }
    }
  }
  
  return 0;
}

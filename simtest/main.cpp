#include <stdio.h>

#include <datatypes.h>

int main(int argc, char ** argv)
{
    if (argc < 2) {
        fprintf(stderr, "Usage: %s WORLDFILE\n", argv[0]);
        return 1;
    }

    return 0;
}

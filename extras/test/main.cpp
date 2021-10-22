/*
   Parser tester

   MIT License
 */

#include <stdio.h>
#include <stdlib.h>

#include "parser.hpp"

int main(int argc, char ** argv)
{
    parse('$');
    parse('M');
    parse('<');
    parse(0);
    parse(215);

    return 0;
}

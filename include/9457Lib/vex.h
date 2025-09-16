#ifndef VEX_H
#define VEX_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <iostream>     // The standard input/output stream library - to use cout and cin
#include <vector>        // The vector functions - to call the standard vector functions
#include "v5.h"
#include "v5_vcs.h"

#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))

#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)

#endif // End of File //
// ===============================================================================================================================================================================
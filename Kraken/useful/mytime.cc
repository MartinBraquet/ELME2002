#include "mytime.h"
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <time.h>

/*! \brief set an angle in the in ]-pi;pi] range
 * 
 * \param[in] x angle to limit
 * \return angle limited in ]-pi;pi]
 */
double get_time()
{
    struct timespec spec;
    clock_gettime(CLOCK_REALTIME, &spec);

    double s  = (double) spec.tv_sec;
    double ns  = (double) spec.tv_nsec;

    return s + ns / 1e9;
}

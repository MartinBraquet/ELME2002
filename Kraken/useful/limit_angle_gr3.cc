#include "limit_angle_gr3.h"
#include <math.h>

#if ROBOTICS_COURSE
    NAMESPACE_INIT(ctrlGr3);
#endif

/*! \brief set an angle in the in ]-pi;pi] range
 * 
 * \param[in] x angle to limit
 * \return angle limited in ]-pi;pi]
 */
double limit_angle(double x)
{
	while (x <= -M_PI)
	{
		x += 2.0*M_PI;
	}
	while (x > M_PI)
	{
		x -= 2.0*M_PI;
	}

	return x;
}

#if ROBOTICS_COURSE
    NAMESPACE_CLOSE();
#endif

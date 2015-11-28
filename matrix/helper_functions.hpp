#pragma once

#include "math.hpp"
#include <math.h>


namespace matrix {
template<typename Type>
float wrap_pi(Type value)
{
	/* value is inf or NaN */
	if (!isfinite(value)) {
		return value;
	}

	int c = 0;

	while (value >= (Type)M_PI) {
		value -= (Type)(2.0 * M_PI);

		if (c++ > 3) {
			return NAN;
		}
	}

	c = 0;

	while (value < (Type)(-M_PI)) {
		value += (Type)(2.0 * M_PI);

		if (c++ > 3) {
			return NAN;
		}
	}

	return value;
}


};
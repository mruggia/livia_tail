
#pragma once

#include <cmath>
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>

class AP_BirdTail {
public:

    AP_BirdTail(void);
	
    CLASS_NO_COPY(AP_BirdTail);
	static AP_BirdTail *get_singleton(void) { return _singleton; }
    static const struct AP_Param::GroupInfo var_info[];
	bool arming_checks(size_t buflen, char *buffer) const { return true; }

	void update(void);
    
private:
	
    static AP_BirdTail *_singleton;
	
};

namespace AP {
    AP_BirdTail &birdtail();
};
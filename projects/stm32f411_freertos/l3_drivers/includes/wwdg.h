#pragma once

#include <stdint.h>

void wwdg__power_on(void);

void wwdg__enable(void);

/**
 * Set the maximum timeout before restart
 * This function will try to set the desired timeout, but it is limited by the APB1 clock frequency
 * and the allowed watchdog time base dividers. To know the limitations for your particular APB1 clock
 * frequency, see Chapter 16.4, Table 63
 * @param ms maximum timeout in ms
 **/ 
void wwdg__set_max_timeout(float ms);

/**
 * Set the window start
 * The start of the window is when check-in is allowed. If the application does not check-in between
 * this window start and the timeout, the controller will restart. It will also restart if the application
 * checks in too early (before this window start)
 * @param ms start of the window
 **/ 
void wwdg__set_window_start(float ms);

/**
 * Application check in
 **/ 
void wwdg__check_in();
#include "wwdg_test.h"
#include "wwdg.h"

#include "FreeRTOS.h"
#include "task.h"

#define DEBUG 1

#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

static void init_wwdg()
{
  wwdg__power_on();
  wwdg__set_max_timeout(50.0f);
  wwdg__set_window_start(20.0f);
  wwdg__enable();
}

void wwdg_test__task(void* parameter)
{
  PRINTF("SYSTEM RESTARTED\n");
  init_wwdg();

  while (1)
  {
    vTaskDelay(20);
    wwdg__check_in();
  }
}
#include <stdbool.h>
static bool test_inhibited;
static uint16_t Board_SetPWM(unsigned i, uint16_t duty) { (void)i; return test_inhibited ? 0 : duty; }
static bool Board_ReleaseOutputs(void) { test_inhibited = false; return true; }

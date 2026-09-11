/* sim stub */
#ifndef SIM_DEBUG_H
#define SIM_DEBUG_H
extern int sim_verbose;
#define DEBUGF(...) do { if (sim_verbose) { printf("  [ftl] " __VA_ARGS__); fflush(stdout); } } while (0)
#endif

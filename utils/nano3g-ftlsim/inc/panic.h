/* sim stub */
#ifndef SIM_PANIC_H
#define SIM_PANIC_H
void panicf(const char *fmt, ...) __attribute__((noreturn, format(printf, 1, 2)));
#endif

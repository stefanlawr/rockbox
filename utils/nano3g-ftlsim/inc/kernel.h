/* sim stub: single-threaded mutexes */
#ifndef SIM_KERNEL_H
#define SIM_KERNEL_H
struct mutex { int locked; };
static inline void mutex_init(struct mutex *m) { m->locked = 0; }
static inline void mutex_lock(struct mutex *m) { m->locked++; }
static inline void mutex_unlock(struct mutex *m) { m->locked--; }
#define HZ 100
extern long current_tick;
#endif

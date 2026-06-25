// Stub omp.h for dacpo_psr — compiled without OpenMP.
// All OMP functions return single-thread values.
// #pragma omp directives are ignored by the compiler without -fopenmp.
#ifndef OMP_H_STUB
#define OMP_H_STUB
#ifdef __cplusplus
extern "C" {
#endif
static inline int omp_get_max_threads(void) { return 1; }
static inline int omp_get_num_threads(void) { return 1; }
static inline int omp_get_thread_num(void) { return 0; }
static inline void omp_set_num_threads(int n) { (void)n; }
#ifdef __cplusplus
}
#endif
#endif

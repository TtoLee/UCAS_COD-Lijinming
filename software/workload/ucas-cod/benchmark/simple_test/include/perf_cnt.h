
#ifndef __PERF_CNT__
#define __PERF_CNT__

#ifdef __cplusplus
extern "C" {
#endif

typedef struct Result {
	int pass;
	unsigned int msec;
	unsigned int mem_c;
	unsigned int inst_c;
	unsigned int wait_c;
	unsigned int ld_c;
	unsigned int if_c;
	unsigned int iw_c;
	unsigned int memw_c;
	unsigned int rdw_c;
	unsigned int msec_overflow;
	unsigned int intr_c;
	unsigned int intr_s;
} Result;

void bench_prepare(Result *res);
void bench_done(Result *res);

#endif

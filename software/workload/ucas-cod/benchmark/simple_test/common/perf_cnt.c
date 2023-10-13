#include "perf_cnt.h"

volatile unsigned int* const cycle_cnt = (void *)0x60010000;
volatile unsigned int* const memory_cnt = (void *)0x60010008;
volatile unsigned int* const instruction_cnt = (void *)0x60011000;
volatile unsigned int* const wait_cnt = (void *)0x60011008;
volatile unsigned int* const load_cnt = (void *)0x60012000;
volatile unsigned int* const if_cnt = (void *)0x60012008;
volatile unsigned int* const iw_cnt = (void *)0x60013000;
volatile unsigned int* const memw_cnt = (void *)0x60013008;
volatile unsigned int* const rdw_cnt = (void *)0x60014000;
volatile unsigned int* const cycle_overflow_cnt = (void *)0x60014008;
volatile unsigned int* const intr_cnt = (void *)0x60015000;
volatile unsigned int* const intr_signal = (void *)0x60015008;

unsigned long _uptime() {
  // TODO [COD]
  //   You can use this function to access performance counter related with time or cycle.
  return *cycle_cnt;
}

void bench_prepare(Result *res) {
  // TODO [COD]
  //   Add preprocess code, record performance counters' initial states.
  //   You can communicate between bench_prepare() and bench_done() through
  //   static variables or add additional fields in `struct Result`
  res->msec = _uptime();
  res->mem_c = *memory_cnt;
  res->inst_c = *instruction_cnt;
  res->wait_c = *wait_cnt;
  res->ld_c = *load_cnt;
  res->if_c = *if_cnt;
  res->iw_c = *iw_cnt;
  res->memw_c = *memw_cnt;
  res->rdw_c = *rdw_cnt;
  res->msec_overflow = *cycle_overflow_cnt;
  res->intr_c = *intr_cnt;
  res->intr_s = *intr_signal;
}

void bench_done(Result *res) {
  // TODO [COD]
  //  Add postprocess code, record performance counters' current states.
  res->msec = _uptime() - res->msec;
  res->mem_c = *memory_cnt - res->mem_c;
  res->inst_c = *instruction_cnt - res->inst_c;
  res->wait_c = *wait_cnt - res->wait_c;
  res->ld_c = *load_cnt - res->ld_c;
  res->if_c = *if_cnt - res->if_c;
  res->iw_c = *iw_cnt - res->iw_c;
  res->memw_c = *memw_cnt - res->memw_c;
  res->rdw_c = *rdw_cnt - res->rdw_c;
  res->msec_overflow = *cycle_overflow_cnt - res->msec_overflow;
  res->intr_c = *intr_cnt - res->intr_c;
  res->intr_s = *intr_signal - res->intr_s;

}
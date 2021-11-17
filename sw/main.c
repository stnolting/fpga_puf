// #################################################################################################
// # << fpga_puf: Physical Unclonable Function (PUF) Test Program >>                               #
// # The PUF is implemented as Custom Functions Unit (CFS) within the NEORV32 Processor.           #
// # ********************************************************************************************* #
// # BSD 3-Clause License                                                                          #
// #                                                                                               #
// # Copyright (c) 2021, Stephan Nolting. All rights reserved.                                     #
// #                                                                                               #
// # Redistribution and use in source and binary forms, with or without modification, are          #
// # permitted provided that the following conditions are met:                                     #
// #                                                                                               #
// # 1. Redistributions of source code must retain the above copyright notice, this list of        #
// #    conditions and the following disclaimer.                                                   #
// #                                                                                               #
// # 2. Redistributions in binary form must reproduce the above copyright notice, this list of     #
// #    conditions and the following disclaimer in the documentation and/or other materials        #
// #    provided with the distribution.                                                            #
// #                                                                                               #
// # 3. Neither the name of the copyright holder nor the names of its contributors may be used to  #
// #    endorse or promote products derived from this software without specific prior written      #
// #    permission.                                                                                #
// #                                                                                               #
// # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS   #
// # OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF               #
// # MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE    #
// # COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,     #
// # EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE #
// # GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED    #
// # AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING     #
// # NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED  #
// # OF THE POSSIBILITY OF SUCH DAMAGE.                                                            #
// # ********************************************************************************************* #
// # fpga_puf - https://github.com/stnolting/fpga_puf                          (c) Stephan Nolting #
// #################################################################################################

#include <neorv32.h>
#include "fpga_puf_neorv32_cfs.h"


/**********************************************************************//**
 * User configuration
 **************************************************************************/
#define BAUD_RATE 19200 // UART baud rate
#define NUM_RUNS      8 // number of PUF ID determination runs
#define SAMPLES    4096 // number of PUF ID samples per run
#define HYST_HI   (SAMPLES-(SAMPLES/32)) // min number of times bit x in all samples has to be set to be identified as '1'
#define HYST_LO   (SAMPLES/32) // max number of times bit x in all samples has to be clear to be identified as '0'


/**********************************************************************//**
 * Prototypes
 **************************************************************************/
void get_puf_id(puf_data_t *puf_data);
uint32_t hamming_dist(puf_data_t id_a, puf_data_t id_b);

void simple_test(void);
void long_test_raw(void);


/**********************************************************************//**
 * Main function
 **************************************************************************/
int main() {

  // initialize the neorv32 runtime environment
  neorv32_rte_setup();

  // setup UART0 at default baud rate, no parity bits, ho hw flow control
  neorv32_uart0_setup(BAUD_RATE, PARITY_NONE, FLOW_CONTROL_RTSCTS);

  neorv32_uart0_printf("Physical Unclonable Functions <fpga_puf> Test\n");
  neorv32_uart0_printf("PUF implemented as NEORV32 Custom Functions Subsystem (CFS)\n\n");

  if (fpga_puf_available() == 0) {
    neorv32_uart0_printf("ERROR! CFS (PUF) not implemented!\n\n");
    return 1;
  }

  while(1) {
    neorv32_uart0_printf("Select test mode:\n");
    neorv32_uart0_printf(" a - run quick test (execute %u runs with %u ID samples each).\n", NUM_RUNS, SAMPLES);
    neorv32_uart0_printf(" b - compute hamming distance of raw IDs\n");

    char c = neorv32_uart0_getc();
    if ((c == 'a') || (c == 'b')) {
      neorv32_uart0_printf("starting test...\n");
    }
    else {
       neorv32_uart0_printf("invalid test selected!\n");
       continue;
    }

    if (c == 'a') {
      simple_test();
    }
    else if (c == 'b') {
      long_test_raw();
    }
  }

  neorv32_uart0_printf("Test completed.\n");

  return 0;
}


/**********************************************************************//**
 * Get 96-bit PUF ID.
 *
 * @note This function also performs "post-processing" of the raw PUF data.
 *
 * @param[in,out] puf_data 3x 32-bit array for the 96-bit raw ID (#puf_data_t);
 **************************************************************************/
void get_puf_id(puf_data_t *puf_data) {

  uint32_t x, y;
  puf_data_t puf_raw;
  uint32_t cnt[96];
  uint32_t tmp;
  uint32_t res[3];
  uint32_t val[3];

  for (x=0; x<96; x++) {
    cnt[x] = 0;
  }

  // count how often each bit of the 96-bit ID across all samples
  for (x=0; x<(uint32_t)SAMPLES; x++) {

    // get 96-bit raw ID sample
    fpga_puf_get_raw(&puf_raw);

    // test every single bit if set or cleared
    for (y=0; y<96; y++) {

      if (y>=64) {
        tmp = puf_raw.id[2];
      }
      else if (y>=32) {
        tmp = puf_raw.id[1];
      }
      else {
        tmp = puf_raw.id[0];
      }

      tmp >>= (y % 32);

      // increment "set-bits" counter
      if (tmp & 1) {
        cnt[y]++;
      }
    }
  }

  // construct final ID
  res[0] = 0; res[1] = 0; res[2] = 0;
  val[0] = 0; val[1] = 0; val[2] = 0;

  // compute each bit of the final ID independently
  for (x=0; x<96; x++) {

    // bit value - majority decision
    if (cnt[x] >= (SAMPLES/2)) {
      tmp = 1; // bit is considered '1' if more than half of all samples had this bit set
    }
    else {
      tmp = 0; // bit is considered '0' if less than half of all samples had this bit cleared
    }
    tmp <<= (x % 32);;
    if (x>=64) {
      res[0] |= tmp;
    }
    else if (x>=32) {
      res[1] |= tmp;
    }
    else {
      res[2] |= tmp;
    }

    // bit valid - hysteresis decision
    // bit is only valid if it is "set most of the times" or "cleared most of the times"
    if ((cnt[x] >= (uint32_t)HYST_HI) || (cnt[x] <= (uint32_t)HYST_LO)) {
      tmp = 1; // bit valid
    }
    else {
      tmp = 0; // bit invalid
    }
    tmp <<= (x % 32);;
    if (x>=64) {
      val[0] |= tmp;
    }
    else if (x>=32) {
      val[1] |= tmp;
    }
    else {
      val[2] |= tmp;
    }
  }

  // final ID, unstable/noisy bits are masked out
  puf_data->id[0] = res[0] & val[0];
  puf_data->id[1] = res[1] & val[1];
  puf_data->id[2] = res[2] & val[2];
}


/**********************************************************************//**
 * Compute Hamming distance of two IDs.
 *
 * @param[int] id_a 3x 32-bit array for first ID (#puf_data_t);
 * @param[int] id_b 3x 32-bit array for second ID (#puf_data_t);
 * @param return Hamming distance (0..96)
 **************************************************************************/
uint32_t hamming_dist(puf_data_t id_a, puf_data_t id_b) {

  uint32_t i, tmp;
  uint32_t cnt = 0;

  for (i=0; i<96; i++) {
    if (i>=64) {
      tmp = id_a.id[2] ^ id_b.id[2];
    }
    else if (i>=32) {
      tmp = id_a.id[1] ^ id_b.id[1];
    }
    else {
      tmp = id_a.id[0] ^ id_b.id[0];
    }
    tmp >>= (i % 32);
    if (tmp & 1) {
      cnt++;
    }
  }

  return cnt;
}


/**********************************************************************//**
 * Simple test: Compute NUM_RUNS pre-processed IDs.
 **************************************************************************/
void simple_test(void) {

  int i;

  for(i=0; i<NUM_RUNS; i++) {
    puf_data_t puf_data;
    get_puf_id(&puf_data);
    neorv32_uart0_printf("Run %u ID: 0x%x%x%x\n", i, puf_data.id[0], puf_data.id[1], puf_data.id[2]);
  }
}


/**********************************************************************//**
 * Long test: Compute Hamming distance of two consecutive raw IDs
 **************************************************************************/
void long_test_raw(void) {

  int i;
  puf_data_t id_init, id_a, id_b;
  uint32_t h_rel_tmp;
  uint32_t h_rel_max = 0;
  uint32_t h_abs_tmp;
  uint32_t h_abs_max = 0;
  puf_data_t noisy_bits;
  puf_data_t zero_id;
  uint32_t popcount;

  fpga_puf_get_raw(&id_init); // initial ID

  zero_id.id[0] = 0;
  zero_id.id[1] = 0;
  zero_id.id[2] = 0;

  i = 0;
  noisy_bits.id[0] = 0;
  noisy_bits.id[1] = 0;
  noisy_bits.id[2] = 0;

  while(1){
     if (neorv32_uart0_char_received()) {
       return;
     }

    fpga_puf_get_raw(&id_a);
    fpga_puf_get_raw(&id_b);

    // compute Hamming distance against initial sample
    h_abs_tmp = hamming_dist(id_a, id_init);
    if (h_abs_tmp > h_abs_max) {
      h_abs_max = h_abs_tmp;
    }

    // compute Hamming distance of two consecutive pre-processed ID samples
    h_rel_tmp = hamming_dist(id_a, id_b);
    if (h_rel_tmp > h_rel_max) {
      h_rel_max = h_rel_tmp;
    }

    // accumulate noisy bits
    noisy_bits.id[0] |= (id_init.id[0] ^ id_a.id[0]) | (id_init.id[0] ^ id_b.id[0]);
    noisy_bits.id[1] |= (id_init.id[1] ^ id_a.id[1]) | (id_init.id[1] ^ id_b.id[1]);
    noisy_bits.id[2] |= (id_init.id[2] ^ id_a.id[2]) | (id_init.id[2] ^ id_b.id[2]);
    popcount = hamming_dist(noisy_bits, zero_id);

    neorv32_uart0_printf("Run %i: ", i);
    neorv32_uart0_printf("I=0x%x%x%x, ", id_init.id[2], id_init.id[1], id_init.id[0]);
    neorv32_uart0_printf("A=0x%x%x%x, ", id_a.id[2], id_a.id[1], id_a.id[0]);
    neorv32_uart0_printf("B=0x%x%x%x - ", id_b.id[2], id_b.id[1], id_b.id[0]);
    neorv32_uart0_printf("F=0x%x%x%x (%u) - ", noisy_bits.id[2], noisy_bits.id[1], noisy_bits.id[0], popcount);
    neorv32_uart0_printf("H(A,B)=%u, H_max(A,B)=%u - ", h_rel_tmp, h_rel_max);
    neorv32_uart0_printf("H(I,A)=%u, H_max(I,A)=%u\n", h_abs_tmp, h_abs_max);
    i++;
  }
}


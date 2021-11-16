// #################################################################################################
// # << fpga_puf_neorv32_cfs.c - Physical Unclonable Function (PUF) CFS HW Driver >>               #
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
 * Check if PUF CFS unit was synthesized.
 *
 * @return 0 if PUF CFS was not synthesized, 1 if PUF CFS is available.
 **************************************************************************/
int fpga_puf_available(void) {

  if (NEORV32_SYSINFO.SOC & (1 << SYSINFO_SOC_IO_CFS)) {
    return 1;
  }
  else {
    return 0;
  }
}


/**********************************************************************//**
 * Get 96-bit PUF ID raw (!) data.
 *
 * @warning This function returns the RAW PUF ID. Since some bits might be very noisy
 * an additional post-processing is required to ensure an ID that is reliable
 * (static over time and operating conditions).
 *
 * @param[in,out] puf_data 3x 32-bit array for the 96-bit raw ID (#puf_data_t);
 **************************************************************************/
void fpga_puf_get_raw(puf_data_t *puf_data) {

  FPGA_PUF_CTRL = 0; // reset

  // enable PUF and trigger sampling
  FPGA_PUF_CTRL = (1<<PUF_CTRL_SAMPLE) | (1<<PUF_CTRL_EN);

  // wait for sampling to complete, takes about ~102 clock cycles (deterministic)
  while(FPGA_PUF_CTRL & (1<<PUF_CTRL_SAMPLE));

  FPGA_PUF_CTRL = 0; // shutdown

  // get 96-bit PUF ID
  puf_data->id[0] = FPGA_PUF_ID0;
  puf_data->id[1] = FPGA_PUF_ID1;
  puf_data->id[2] = FPGA_PUF_ID2;
}

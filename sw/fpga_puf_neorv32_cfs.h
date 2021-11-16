// #################################################################################################
// # << fpga_puf_neorv32_cfs.h - Physical Unclonable Function (PUF) CFS HW Driver >>               #
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

#ifndef fpga_puf_neorv32_cfs_h
#define fpga_puf_neorv32_cfs_h

// CFS registers
#define FPGA_PUF_CTRL NEORV32_CFS.REG[0] // control register
#define FPGA_PUF_ID0  NEORV32_CFS.REG[1] // puf id bits 31:0
#define FPGA_PUF_ID1  NEORV32_CFS.REG[2] // puf id bits 63:32
#define FPGA_PUF_ID2  NEORV32_CFS.REG[3] // puf id bits 95:64

// control register bits
#define PUF_CTRL_EN     0 // module enable/reset
#define PUF_CTRL_SAMPLE 1 // trigger sampling / busy flag

// PUF ID type
typedef struct puf_data_t { uint32_t id[3]; } puf_data_t;

// prototypes
int fpga_puf_available(void);
void fpga_puf_get_raw(puf_data_t *puf_data);

#endif // fpga_puf_neorv32_cfs_h

-- #################################################################################################
-- # << Technology-Agnostic Physical Unclonable Function (PUF) >>                                  #
-- # ********************************************************************************************* #
-- # The PUF is based on single-bit ring oscillators, which are individually reset and enabled to  #
-- # ensure the synthesis tool does not remove ("optimize-away") the cells (this concept is taken  #
-- # from the NEORV32 TRNG -> https://github.com/stnolting/neoTRNG).                               #
-- #                                                                                               #
-- # The oscillators are enabled and for one clock cycle. After that, their current state is       #
-- # frozen and sampled to get another bit of the final PUF ID. The final state of an oscillator   #
-- # is defined by chip-specific manufacturing fluctuations (wire capacitance due to oxide         #
-- # thickness variations and stuff like that). For a large amount of samples (over time) this     #
-- # state seems to be reproducible (see exemplary results in fpga_puf documentation).             #
-- #                                                                                               #
-- # NOTE: Some bits in the PUF ID might be very noisy. Hence, an appropriate post-processing is   #
-- # is required to determine an ID that is stable over time and operation conditions.             #
-- # ********************************************************************************************* #
-- # BSD 3-Clause License                                                                          #
-- #                                                                                               #
-- # Copyright (c) 2022, Stephan Nolting. All rights reserved.                                     #
-- #                                                                                               #
-- # Redistribution and use in source and binary forms, with or without modification, are          #
-- # permitted provided that the following conditions are met:                                     #
-- #                                                                                               #
-- # 1. Redistributions of source code must retain the above copyright notice, this list of        #
-- #    conditions and the following disclaimer.                                                   #
-- #                                                                                               #
-- # 2. Redistributions in binary form must reproduce the above copyright notice, this list of     #
-- #    conditions and the following disclaimer in the documentation and/or other materials        #
-- #    provided with the distribution.                                                            #
-- #                                                                                               #
-- # 3. Neither the name of the copyright holder nor the names of its contributors may be used to  #
-- #    endorse or promote products derived from this software without specific prior written      #
-- #    permission.                                                                                #
-- #                                                                                               #
-- # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS   #
-- # OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF               #
-- # MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE    #
-- # COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,     #
-- # EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE #
-- # GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED    #
-- # AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING     #
-- # NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED  #
-- # OF THE POSSIBILITY OF SUCH DAMAGE.                                                            #
-- # ********************************************************************************************* #
-- # fpga_puf - https://github.com/stnolting/fpga_puf                          (c) Stephan Nolting #
-- #################################################################################################

library ieee;
use ieee.std_logic_1164.all;

entity fpga_puf is
  generic (
    ID_SIZE : natural := 96 -- size of PUF ID in bits
  );
  port (
    clk_i  : in  std_ulogic; -- global clock line
    rstn_i : in  std_ulogic; -- SYNC reset, low-active
    trig_i : in  std_ulogic; -- set high for one clock to trigger ID sampling
    busy_o : out std_ulogic; -- busy when set (sampling ID)
    id_o   : out std_ulogic_vector(ID_SIZE-1 downto 0) -- PUF ID (valid after sampling is done)
  );
end fpga_puf;

architecture fpga_puf_rtl of fpga_puf is

  -- arbiter --
  type state_t is (S_IDLE, S_RUN, S_SAMPLE);
  type arbiter_t is record
    state  : state_t;
    sreg   : std_ulogic_vector(ID_SIZE downto 0);
    sample : std_ulogic;
  end record;
  signal arbiter : arbiter_t;

  -- component: 1-bit PUF sell --
  component fpga_puf_cell
    port (
      clk_i    : in  std_ulogic;
      reset_i  : in  std_ulogic;
      latch_i  : in  std_ulogic;
      sample_i : in  std_ulogic;
      data_o   : out std_ulogic 
    );
  end component;

begin

  -- Control Arbiter ------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  sampling_arbiter: process(clk_i)
  begin
    if rising_edge(clk_i) then
      -- latch control SREG: control reset and transparent mode --
      arbiter.sreg(arbiter.sreg'left downto 1) <= arbiter.sreg(arbiter.sreg'left-1 downto 0);
      arbiter.sreg(0) <= '0'; -- default

      -- sampling FSM --
      if (rstn_i = '0') then -- reset
        arbiter.state <= S_IDLE;
      else
        case arbiter.state is

          when S_IDLE => -- wait for trigger
            if (trig_i = '1') then
              arbiter.sreg(0) <= '1';
              arbiter.state   <= S_RUN;
            end if;

          when S_RUN => -- reset & open latches for one cycle - one by one
            if (arbiter.sreg(arbiter.sreg'left) = '1') then
              arbiter.state <= S_SAMPLE;
            end if;

          when S_SAMPLE => -- sample latch states
            arbiter.state <= S_IDLE;

          when others => -- undefined
            arbiter.state <= S_IDLE;

        end case;
      end if;
    end if;
  end process sampling_arbiter;

  -- sample PUF data --
  arbiter.sample <= '1' when (arbiter.state = S_SAMPLE) else '0';

  -- busy flag --
  busy_o <= '0' when (arbiter.state = S_IDLE) else '1';


  -- PUF Cells ------------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  fpga_puf_cell_inst:
  for i in 0 to ID_SIZE-1 generate
    fpga_puf_cell_inst_i: fpga_puf_cell
    port map (
      clk_i    => clk_i,
      reset_i  => arbiter.sreg(i),   -- reset once cycle before opening latch
      latch_i  => arbiter.sreg(i+1), -- open latch for one cycle
      sample_i => arbiter.sample,
      data_o   => id_o(i)
    );
  end generate;


end fpga_puf_rtl;


-- ############################################################################################################################
-- ############################################################################################################################


-- #################################################################################################
-- # << Technology-Agnostic Physical Unclonable Function (PUF) - 1-bit PUF Cell >>                 #
-- # ********************************************************************************************* #
-- # Based on a simple 1-bit oscillator decoupled by a latch (also providing reset).               #
-- # ********************************************************************************************* #
-- # BSD 3-Clause License                                                                          #
-- #                                                                                               #
-- # Copyright (c) 2022, Stephan Nolting. All rights reserved.                                     #
-- #                                                                                               #
-- # Redistribution and use in source and binary forms, with or without modification, are          #
-- # permitted provided that the following conditions are met:                                     #
-- #                                                                                               #
-- # 1. Redistributions of source code must retain the above copyright notice, this list of        #
-- #    conditions and the following disclaimer.                                                   #
-- #                                                                                               #
-- # 2. Redistributions in binary form must reproduce the above copyright notice, this list of     #
-- #    conditions and the following disclaimer in the documentation and/or other materials        #
-- #    provided with the distribution.                                                            #
-- #                                                                                               #
-- # 3. Neither the name of the copyright holder nor the names of its contributors may be used to  #
-- #    endorse or promote products derived from this software without specific prior written      #
-- #    permission.                                                                                #
-- #                                                                                               #
-- # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS   #
-- # OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF               #
-- # MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE    #
-- # COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,     #
-- # EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE #
-- # GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED    #
-- # AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING     #
-- # NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED  #
-- # OF THE POSSIBILITY OF SUCH DAMAGE.                                                            #
-- # ********************************************************************************************* #
-- # fpga_puf - https://github.com/stnolting/fpga_puf                          (c) Stephan Nolting #
-- #################################################################################################

library ieee;
use ieee.std_logic_1164.all;

entity fpga_puf_cell is
  port (
    clk_i    : in  std_ulogic;
    reset_i  : in  std_ulogic;
    latch_i  : in  std_ulogic;
    sample_i : in  std_ulogic;
    data_o   : out std_ulogic 
  );
end fpga_puf_cell;

architecture fpga_puf_cell_rtl of fpga_puf_cell is

  signal osc : std_ulogic;

begin

  -- Oscillator Cell ------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  oscillator: process(osc, reset_i, latch_i)
  begin
    if (reset_i = '1') then -- reset oscillator to defined state
      osc <= '0';
    elsif (latch_i = '1') then -- enable ring-oscillator; keep current state when disabled
      osc <= not osc;
    end if;
  end process oscillator;


  -- Output Capture Register ----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  cap_reg: process(clk_i)
  begin
    if rising_edge(clk_i) then
      if (sample_i = '1') then
        data_o <= osc;
      end if;
    end if;
  end process cap_reg;


end fpga_puf_cell_rtl;

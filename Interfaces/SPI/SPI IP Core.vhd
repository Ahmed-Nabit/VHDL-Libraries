-- ============================================================================
-- ATmega16-compatible SPI Master/Slave Controller
-- ============================================================================
-- Notes:
--   * clk must be faster than the SPI clock.
--   * wr/rd are assumed to be one-cycle strobes.
--   * This module implements the SPI engine only.
--   * Exact AVR port override behavior is completed by connecting
--     mosi_ddr / miso_ddr / sck_ddr from the GPIO DDR model.
--   * SS is not automatically driven by the Master, per the datasheet.
--   * External inputs are synchronized. For maximum SCK rates, I/O
--     constraints and timing closure are required.
-- ============================================================================
-- Copyright © 2024-2026 Ahmed Nabit [Lazrdo@gmail.com](mailto:Lazrdo@gmail.com)
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at:
--     http://www.apache.org/licenses/LICENSE-2.0
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.
-- ============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity spi_atmega16 is
  port (
    -------------------------------------------------------------------------
    -- System
    -------------------------------------------------------------------------
    clk           : in  std_logic;
    reset         : in  std_logic;  -- synchronous reset, active high

    -------------------------------------------------------------------------
    -- CPU / register interface
    -------------------------------------------------------------------------
    cs_n          : in  std_logic;
    addr          : in  std_logic_vector(1 downto 0);
                  -- "00" SPCR
                  -- "01" SPSR
                  -- "10" SPDR
    wr            : in  std_logic;
    rd            : in  std_logic;
    din           : in  std_logic_vector(7 downto 0);
    dout          : out std_logic_vector(7 downto 0);
    irq           : out std_logic;
    irq_ack       : in  std_logic;  -- interrupt vector acknowledge, pulse high
                                    -- tie low if not used

    -------------------------------------------------------------------------
    -- SPI pins
    -------------------------------------------------------------------------
    mosi_i        : in  std_logic;
    mosi_o        : out std_logic;
    mosi_oe       : out std_logic;

    miso_i        : in  std_logic;
    miso_o        : out std_logic;
    miso_oe       : out std_logic;

    sck_i         : in  std_logic;
    sck_o         : out std_logic;
    sck_oe        : out std_logic;

    ss_i          : in  std_logic;
    ss_o          : out std_logic;
    ss_oe         : out std_logic;

    -------------------------------------------------------------------------
    -- Direction / override inputs
    -------------------------------------------------------------------------
    ss_input_mode : in  std_logic;  -- '1' = SS input, '0' = SS output
    mosi_ddr      : in  std_logic;  -- '1' = user DDR allows MOSI output
    miso_ddr      : in  std_logic;  -- '1' = user DDR allows MISO output
    sck_ddr       : in  std_logic   -- '1' = user DDR allows SCK output
  );
end entity spi_atmega16;

architecture rtl of spi_atmega16 is

  ---------------------------------------------------------------------------
  -- Helper functions
  ---------------------------------------------------------------------------
  function first_bit(
    v    : std_logic_vector(7 downto 0);
    dord : std_logic
  ) return std_logic is
  begin
    if dord = '0' then
      return v(7);
    else
      return v(0);
    end if;
  end function;

  function shift_next(
    v    : std_logic_vector(7 downto 0);
    sin  : std_logic;
    dord : std_logic
  ) return std_logic_vector is
  begin
    if dord = '0' then
      return v(6 downto 0) & sin;
    else
      return sin & v(7 downto 1);
    end if;
  end function;

  function half_period_value(
    spi2x : std_logic;
    spr1  : std_logic;
    spr0  : std_logic
  ) return integer is
  begin
    if spi2x = '0' then
      case spr1 & spr0 is
        when "00" => return 2;    -- fosc/4
        when "01" => return 8;    -- fosc/16
        when "10" => return 32;   -- fosc/64
        when "11" => return 64;   -- fosc/128
        when others => return 2;
      end case;
    else
      case spr1 & spr0 is
        when "00" => return 1;    -- fosc/2
        when "01" => return 4;    -- fosc/8
        when "10" => return 16;   -- fosc/32
        when "11" => return 32;   -- fosc/64
        when others => return 1;
      end case;
    end if;
  end function;

  ---------------------------------------------------------------------------
  -- CPU-visible registers / register fragments
  ---------------------------------------------------------------------------
  signal spcr_q  : std_logic_vector(7 downto 0) := (others => '0');
  signal spi2x_q : std_logic := '0';
  signal spif_q  : std_logic := '0';
  signal wcol_q  : std_logic := '0';

  signal tx_buf : std_logic_vector(7 downto 0) := (others => '0');
  signal rx_buf : std_logic_vector(7 downto 0) := (others => '0');

  ---------------------------------------------------------------------------
  -- Flag clear sequence arms
  ---------------------------------------------------------------------------
  signal arm_spif_q : std_logic := '0';
  signal arm_wcol_q : std_logic := '0';

  ---------------------------------------------------------------------------
  -- SPSR read value
  ---------------------------------------------------------------------------
  signal spsr : std_logic_vector(7 downto 0);

  ---------------------------------------------------------------------------
  -- SPCR aliases
  ---------------------------------------------------------------------------
  alias SPIE : std_logic is spcr_q(7);
  alias SPE  : std_logic is spcr_q(6);
  alias DORD : std_logic is spcr_q(5);
  alias MSTR : std_logic is spcr_q(4);
  alias CPOL : std_logic is spcr_q(3);
  alias CPHA : std_logic is spcr_q(2);
  alias SPR1 : std_logic is spcr_q(1);
  alias SPR0 : std_logic is spcr_q(0);

  ---------------------------------------------------------------------------
  -- Synchronizers
  ---------------------------------------------------------------------------
  signal ss_sync   : std_logic_vector(1 downto 0) := (others => '1');
  signal sck_sync  : std_logic_vector(1 downto 0) := (others => '1');
  signal mosi_sync : std_logic_vector(1 downto 0) := (others => '1');
  signal miso_sync : std_logic_vector(1 downto 0) := (others => '1');

  signal sck_prev : std_logic := '1';

  ---------------------------------------------------------------------------
  -- Mode fault
  ---------------------------------------------------------------------------
  signal mode_fault_cond : std_logic;

  ---------------------------------------------------------------------------
  -- Master datapath
  ---------------------------------------------------------------------------
  type master_state_t is (IDLE, RUNNING);
  signal master_state : master_state_t := IDLE;
  signal master_busy  : std_logic := '0';

  signal master_pending_start : std_logic := '0';

  signal shift_reg : std_logic_vector(7 downto 0) := (others => '0');
  signal bit_cnt   : integer range 0 to 7 := 0;
  signal sample_q  : std_logic := '0';

  signal half_period_cmd : integer range 1 to 64;
  signal clk_limit       : integer range 1 to 64 := 2;
  signal half_cnt        : integer range 0 to 63 := 0;

  signal sck_q : std_logic := '0';

  ---------------------------------------------------------------------------
  -- Slave datapath
  ---------------------------------------------------------------------------
  type slave_state_t is (IDLE, ACTIVE);
  signal slave_state : slave_state_t := IDLE;

  signal slave_shift     : std_logic_vector(7 downto 0) := (others => '0');
  signal slave_bit_cnt   : integer range 0 to 7 := 0;
  signal slave_sample    : std_logic := '0';
  signal slave_xfer_busy : std_logic := '0';

  ---------------------------------------------------------------------------
  -- Internal pin drive
  ---------------------------------------------------------------------------
  signal mosi_out_int : std_logic := '0';
  signal miso_out_int : std_logic := '0';

  signal mosi_oe_int : std_logic := '0';
  signal miso_oe_int : std_logic := '0';
  signal sck_oe_int  : std_logic := '0';

  ---------------------------------------------------------------------------
  -- Read mux selector
  ---------------------------------------------------------------------------
  signal read_sel : std_logic_vector(3 downto 0);

begin

  ---------------------------------------------------------------------------
  -- Concurrent helpers
  ---------------------------------------------------------------------------
  spsr <= spif_q & wcol_q & "00000" & spi2x_q;

  half_period_cmd <= half_period_value(spi2x_q, SPR1, SPR0);

  mode_fault_cond <= '1' when (SPE = '1' and
                               MSTR = '1' and
                               ss_input_mode = '1' and
                               ss_sync(1) = '0')
                     else '0';

  read_sel <= cs_n & rd & addr;

  with read_sel select
    dout <=
      spcr_q          when "0100",
      spsr            when "0101",
      rx_buf          when "0110",
      (others => '0') when others;

  irq <= spif_q and SPIE;

  ---------------------------------------------------------------------------
  -- Pin outputs
  ---------------------------------------------------------------------------
  mosi_o <= mosi_out_int;
  miso_o <= miso_out_int;
  sck_o  <= sck_q;

  -- The datasheet says Master mode has no automatic SS control.
  -- Keep SPI-owned SS inactive. Use GPIO/software for SS.
  ss_o <= '1';

  ---------------------------------------------------------------------------
  -- Output enables, gated by SPE and user DDR
  ---------------------------------------------------------------------------
  mosi_oe <= (mosi_oe_int and mosi_ddr) when SPE = '1' else '0';
  miso_oe <= (miso_oe_int and miso_ddr) when SPE = '1' else '0';
  sck_oe  <= (sck_oe_int  and sck_ddr)  when SPE = '1' else '0';

  -- SPI core does not own automatic Master SS output.
  ss_oe <= '0';

  ---------------------------------------------------------------------------
  -- Input synchronizers
  ---------------------------------------------------------------------------
  sync_proc : process(clk)
  begin
    if rising_edge(clk) then
      if reset = '1' then
        ss_sync   <= (others => '1');
        sck_sync  <= (others => '1');
        mosi_sync <= (others => '1');
        miso_sync <= (others => '1');
      else
        ss_sync   <= ss_sync(0)   & ss_i;
        sck_sync  <= sck_sync(0)  & sck_i;
        mosi_sync <= mosi_sync(0) & mosi_i;
        miso_sync <= miso_sync(0) & miso_i;
      end if;
    end if;
  end process sync_proc;

  ---------------------------------------------------------------------------
  -- Main SPI process
  ---------------------------------------------------------------------------
  main_proc : process(clk) is
    variable v_spif_set          : std_logic;
    variable v_spif_clear        : std_logic;
    variable v_wcol_set          : std_logic;
    variable v_wcol_clear        : std_logic;
    variable v_spif_next         : std_logic;
    variable v_wcol_next         : std_logic;
    variable v_arm_spif_next     : std_logic;
    variable v_arm_wcol_next     : std_logic;

    variable v_spsr_read         : std_logic;
    variable v_spdr_access       : std_logic;

    variable v_tx_updated        : std_logic;
    variable v_tx_data           : std_logic_vector(7 downto 0);

    variable v_master_start      : std_logic;

    variable v_spcr_next         : std_logic_vector(7 downto 0);

    variable v_master_sck_next   : std_logic;
    variable v_master_tick       : boolean;
    variable v_master_leading    : boolean;
    variable v_master_trailing   : boolean;
    variable v_master_completing : boolean;

    variable v_sck_leading       : boolean;
    variable v_sck_trailing      : boolean;
    variable v_slave_completing  : boolean;

    variable v_next              : std_logic_vector(7 downto 0);
    variable v_slave_reload_data : std_logic_vector(7 downto 0);

    variable v_SPE_eff           : std_logic;
    variable v_MSTR_eff          : std_logic;
    variable v_DORD_eff          : std_logic;
    variable v_CPOL_eff          : std_logic;
    variable v_CPHA_eff          : std_logic;
  begin
    if rising_edge(clk) then

      -----------------------------------------------------------------------
      -- Per-cycle variable defaults
      -----------------------------------------------------------------------
      v_spif_set          := '0';
      v_spif_clear        := '0';
      v_wcol_set          := '0';
      v_wcol_clear        := '0';

      v_spsr_read         := '0';
      v_spdr_access       := '0';

      v_tx_updated        := '0';
      v_tx_data           := din;

      v_master_start      := '0';

      v_spcr_next         := spcr_q;

      v_master_sck_next   := sck_q;
      v_master_tick       := false;
      v_master_leading    := false;
      v_master_trailing   := false;
      v_master_completing := false;

      v_sck_leading       := false;
      v_sck_trailing      := false;
      v_slave_completing  := false;

      v_next              := shift_reg;
      v_slave_reload_data := tx_buf;

      -----------------------------------------------------------------------
      -- Synchronous reset
      -----------------------------------------------------------------------
      if reset = '1' then
        spcr_q             <= (others => '0');
        spi2x_q            <= '0';
        spif_q             <= '0';
        wcol_q             <= '0';

        tx_buf             <= (others => '0');
        rx_buf             <= (others => '0');

        arm_spif_q         <= '0';
        arm_wcol_q         <= '0';

        sck_prev           <= '1';

        master_state       <= IDLE;
        master_busy        <= '0';
        master_pending_start <= '0';

        shift_reg          <= (others => '0');
        bit_cnt            <= 0;
        sample_q           <= '0';

        clk_limit          <= 2;
        half_cnt           <= 0;
        sck_q              <= '0';

        slave_state        <= IDLE;
        slave_shift        <= (others => '0');
        slave_bit_cnt      <= 0;
        slave_sample       <= '0';
        slave_xfer_busy    <= '0';

        mosi_out_int       <= '0';
        miso_out_int       <= '0';

        mosi_oe_int        <= '0';
        miso_oe_int        <= '0';
        sck_oe_int         <= '0';

      else

        ---------------------------------------------------------------------
        -- Previous-value updates for edge detection
        ---------------------------------------------------------------------
        sck_prev <= sck_sync(1);

        ---------------------------------------------------------------------
        -- Precompute Master SCK tick and completion status.
        -- This is used for write-collision window handling.
        ---------------------------------------------------------------------
        if SPE = '1' and MSTR = '1' and mode_fault_cond = '0' and
           master_busy = '1' and master_state = RUNNING and
           half_cnt = clk_limit - 1 then

          v_master_tick     := true;
          v_master_sck_next := not sck_q;

          if (CPOL = '0' and sck_q = '0' and v_master_sck_next = '1') or
             (CPOL = '1' and sck_q = '1' and v_master_sck_next = '0') then
            v_master_leading := true;
          else
            v_master_trailing := true;
          end if;

          if v_master_trailing and bit_cnt = 7 then
            v_master_completing := true;
          end if;
        end if;

        ---------------------------------------------------------------------
        -- Precompute external SCK edges for Slave mode.
        ---------------------------------------------------------------------
        if CPOL = '0' then
          v_sck_leading  := (sck_sync(1) = '1' and sck_prev = '0');
          v_sck_trailing := (sck_sync(1) = '0' and sck_prev = '1');
        else
          v_sck_leading  := (sck_sync(1) = '0' and sck_prev = '1');
          v_sck_trailing := (sck_sync(1) = '1' and sck_prev = '0');
        end if;

        if SPE = '1' and MSTR = '0' and
           slave_state = ACTIVE and
           v_sck_trailing and
           slave_bit_cnt = 7 then
          v_slave_completing := true;
        end if;

        ---------------------------------------------------------------------
        -- CPU writes
        ---------------------------------------------------------------------
        if cs_n = '0' and wr = '1' then
          case addr is

            -----------------------------------------------------------------
            -- SPCR
            -----------------------------------------------------------------
            when "00" =>
              v_spcr_next := din;

            -----------------------------------------------------------------
            -- SPSR: only SPI2X writable
            -----------------------------------------------------------------
            when "01" =>
              spi2x_q <= din(0);

            -----------------------------------------------------------------
            -- SPDR write:
            --   * write to TX buffer
            --   * start Master transfer if enabled
            --   * set WCOL if write during active transfer
            -----------------------------------------------------------------
            when "10" =>
              v_spdr_access := '1';

              if (master_busy = '1' and not v_master_completing) or
                 (slave_xfer_busy = '1' and not v_slave_completing) then
                v_wcol_set := '1';
              else
                tx_buf       <= din;
                v_tx_updated := '1';
                v_tx_data    := din;

                if SPE = '1' and MSTR = '1' and mode_fault_cond = '0' then
                  if master_busy = '0' then
                    v_master_start := '1';
                  else
                    master_pending_start <= '1';
                  end if;
                end if;
              end if;

            when others =>
              null;
          end case;
        end if;

        ---------------------------------------------------------------------
        -- CPU reads: side effects only.
        -- dout is driven combinationally.
        ---------------------------------------------------------------------
        if cs_n = '0' and rd = '1' then
          case addr is
            when "01" =>
              v_spsr_read := '1';

            when "10" =>
              v_spdr_access := '1';

            when others =>
              null;
          end case;
        end if;

        ---------------------------------------------------------------------
        -- Mode fault:
        -- If Master, SS input, and SS low: clear MSTR and set SPIF.
        -- This is level-checked against both the current register state and
        -- the next SPCR value, so attempted MSTR writes during fault are
        -- also blocked.
        ---------------------------------------------------------------------
        if mode_fault_cond = '1' or
           (v_spcr_next(6) = '1' and
            v_spcr_next(4) = '1' and
            ss_input_mode = '1' and
            ss_sync(1) = '0') then

          v_spcr_next(4) := '0';
          v_spif_set     := '1';
        end if;

        ---------------------------------------------------------------------
        -- Effective control bits after mode-fault handling
        ---------------------------------------------------------------------
        v_SPE_eff  := v_spcr_next(6);
        v_MSTR_eff := v_spcr_next(4);
        v_DORD_eff := v_spcr_next(5);
        v_CPOL_eff := v_spcr_next(3);
        v_CPHA_eff := v_spcr_next(2);

        ---------------------------------------------------------------------
        -- Master datapath
        ---------------------------------------------------------------------
        if v_SPE_eff = '1' and v_MSTR_eff = '1' then

          -- In Master mode, SCK and MOSI are driven if user DDR allows it.
          sck_oe_int  <= '1';
          mosi_oe_int <= '1';
          miso_oe_int <= '0';

          if v_master_start = '1' then
            -----------------------------------------------------------------
            -- Start Master transfer immediately
            -----------------------------------------------------------------
            master_busy          <= '1';
            master_state         <= RUNNING;
            shift_reg            <= v_tx_data;
            bit_cnt              <= 0;
            sample_q             <= '0';
            half_cnt             <= 0;
            clk_limit            <= half_period_cmd;
            sck_q                <= v_CPOL_eff;
            mosi_out_int         <= first_bit(v_tx_data, v_DORD_eff);
            master_pending_start <= '0';

          elsif master_pending_start = '1' and master_busy = '0' then
            -----------------------------------------------------------------
            -- Start pending Master transfer.
            -- This covers an accepted SPDR write that occurred in the same
            -- cycle as the previous transfer's completion.
            -----------------------------------------------------------------
            master_busy          <= '1';
            master_state         <= RUNNING;
            shift_reg            <= tx_buf;
            bit_cnt              <= 0;
            sample_q             <= '0';
            half_cnt             <= 0;
            clk_limit            <= half_period_cmd;
            sck_q                <= v_CPOL_eff;
            mosi_out_int         <= first_bit(tx_buf, v_DORD_eff);
            master_pending_start <= '0';

          elsif master_busy = '1' and master_state = RUNNING then
            -----------------------------------------------------------------
            -- Generate SCK and perform edge actions
            -----------------------------------------------------------------
            if v_master_tick then
              half_cnt <= 0;
              sck_q    <= v_master_sck_next;

              -----------------------------------------------------------------
              -- Leading edge
              -----------------------------------------------------------------
              if v_master_leading then
                if v_CPHA_eff = '1' then
                  -- CPHA=1: setup on leading edge
                  mosi_out_int <= first_bit(shift_reg, v_DORD_eff);
                else
                  -- CPHA=0: sample on leading edge
                  sample_q <= miso_sync(1);
                end if;
              end if;

              -----------------------------------------------------------------
              -- Trailing edge
              -----------------------------------------------------------------
              if v_master_trailing then
                if v_CPHA_eff = '0' then
                  -- CPHA=0: shift on trailing edge, output next bit
                  v_next := shift_next(shift_reg, sample_q, v_DORD_eff);

                  if bit_cnt = 7 then
                    rx_buf       <= v_next;
                    shift_reg    <= v_next;
                    v_spif_set   := '1';
                    master_busy  <= '0';
                    master_state <= IDLE;
                  else
                    shift_reg    <= v_next;
                    mosi_out_int <= first_bit(v_next, v_DORD_eff);
                    bit_cnt      <= bit_cnt + 1;
                  end if;

                else
                  -- CPHA=1: sample on trailing edge, do NOT change MOSI here
                  v_next := shift_next(shift_reg, miso_sync(1), v_DORD_eff);

                  if bit_cnt = 7 then
                    rx_buf       <= v_next;
                    shift_reg    <= v_next;
                    v_spif_set   := '1';
                    master_busy  <= '0';
                    master_state <= IDLE;
                  else
                    shift_reg <= v_next;
                    bit_cnt   <= bit_cnt + 1;
                  end if;
                end if;
              end if;

            else
              half_cnt <= half_cnt + 1;
            end if;

          else
            -----------------------------------------------------------------
            -- Master idle
            -----------------------------------------------------------------
            master_busy  <= '0';
            master_state <= IDLE;
            sck_q        <= v_CPOL_eff;
            half_cnt     <= 0;
            clk_limit    <= half_period_cmd;
            mosi_out_int <= '0';
          end if;

        else
          -------------------------------------------------------------------
          -- Not Master-enabled: disable Master-driven outputs
          -------------------------------------------------------------------
          master_busy          <= '0';
          master_state         <= IDLE;
          master_pending_start <= '0';

          sck_oe_int           <= '0';
          mosi_oe_int          <= '0';
          miso_oe_int          <= '0';

          sck_q                <= '0';
          half_cnt             <= 0;
          mosi_out_int         <= '0';
        end if;

        ---------------------------------------------------------------------
        -- Slave datapath
        ---------------------------------------------------------------------
        if v_SPE_eff = '1' and v_MSTR_eff = '0' then

          if slave_state = ACTIVE then

            -----------------------------------------------------------------
            -- If software updates TX data while no byte is currently
            -- shifting, allow it to affect the next byte.
            -----------------------------------------------------------------
            if v_tx_updated = '1' and
               slave_xfer_busy = '0' and
               slave_bit_cnt = 0 then

              slave_shift <= v_tx_data;

              if v_CPHA_eff = '0' then
                miso_out_int <= first_bit(v_tx_data, v_DORD_eff);
                miso_oe_int  <= '1';
              end if;
            end if;

            -----------------------------------------------------------------
            -- SCK leading edge
            -----------------------------------------------------------------
            if v_sck_leading and ss_sync(1) = '0' then
              slave_xfer_busy <= '1';

              if v_CPHA_eff = '1' then
                -- CPHA=1: setup on leading edge
                if v_tx_updated = '1' and
                   slave_xfer_busy = '0' and
                   slave_bit_cnt = 0 then
                  miso_out_int <= first_bit(v_tx_data, v_DORD_eff);
                else
                  miso_out_int <= first_bit(slave_shift, v_DORD_eff);
                end if;

                miso_oe_int <= '1';

              else
                -- CPHA=0: sample on leading edge
                slave_sample <= mosi_sync(1);
              end if;
            end if;

            -----------------------------------------------------------------
            -- SCK trailing edge
            -----------------------------------------------------------------
            if v_sck_trailing then

              -- Allow final trailing edge to complete even if synchronized SS
              -- has just gone high. Partial transfers are still aborted by SS.
              if ss_sync(1) = '0' or slave_bit_cnt = 7 then

                if v_CPHA_eff = '0' then
                  -- CPHA=0: shift on trailing edge, output next bit
                  v_next := shift_next(slave_shift, slave_sample, v_DORD_eff);

                  if slave_bit_cnt = 7 then
                    rx_buf          <= v_next;
                    v_spif_set      := '1';
                    slave_xfer_busy <= '0';
                    slave_bit_cnt   <= 0;

                    -- Continuous multi-byte support:
                    -- immediately load and present next byte if possible.
                    v_slave_reload_data := tx_buf;
                    if v_tx_updated = '1' then
                      v_slave_reload_data := v_tx_data;
                    end if;

                    slave_shift  <= v_slave_reload_data;
                    miso_out_int <= first_bit(v_slave_reload_data, v_DORD_eff);
                    miso_oe_int  <= '1';

                  else
                    slave_shift    <= v_next;
                    miso_out_int   <= first_bit(v_next, v_DORD_eff);
                    slave_bit_cnt  <= slave_bit_cnt + 1;
                  end if;

                else
                  -- CPHA=1: sample on trailing edge, do NOT change MISO here
                  v_next := shift_next(slave_shift, mosi_sync(1), v_DORD_eff);

                  if slave_bit_cnt = 7 then
                    rx_buf          <= v_next;
                    v_spif_set      := '1';
                    slave_xfer_busy <= '0';
                    slave_bit_cnt   <= 0;

                    -- Load next byte, but drive its first bit on next leading edge.
                    v_slave_reload_data := tx_buf;
                    if v_tx_updated = '1' then
                      v_slave_reload_data := v_tx_data;
                    end if;

                    slave_shift <= v_slave_reload_data;

                  else
                    slave_shift   <= v_next;
                    slave_bit_cnt <= slave_bit_cnt + 1;
                  end if;
                end if;

              end if;
            end if;

          end if;

          -------------------------------------------------------------------
          -- SS high: Slave passive and reset.
          -- SS low: activate Slave when IDLE.
          -------------------------------------------------------------------
          if ss_sync(1) = '1' then
            slave_state     <= IDLE;
            slave_bit_cnt   <= 0;
            miso_oe_int     <= '0';
            slave_xfer_busy <= '0';

          else
            if slave_state = IDLE then
              ---------------------------------------------------------------
              -- Activate Slave
              ---------------------------------------------------------------
              slave_state     <= ACTIVE;
              slave_bit_cnt   <= 0;
              slave_xfer_busy <= '0';

              v_slave_reload_data := tx_buf;
              if v_tx_updated = '1' then
                v_slave_reload_data := v_tx_data;
              end if;

              slave_shift <= v_slave_reload_data;

              if v_CPHA_eff = '0' then
                -- CPHA=0: first bit must be presented before first leading edge
                miso_out_int <= first_bit(v_slave_reload_data, v_DORD_eff);
                miso_oe_int  <= '1';
              else
                -- CPHA=1: first bit is driven on first leading edge
                miso_out_int <= first_bit(v_slave_reload_data, v_DORD_eff);
                miso_oe_int  <= '0';
              end if;
            end if;
          end if;

        else
          -------------------------------------------------------------------
          -- Not Slave-enabled
          -------------------------------------------------------------------
          slave_state     <= IDLE;
          slave_bit_cnt   <= 0;
          miso_oe_int     <= '0';
          slave_xfer_busy <= '0';
        end if;

        ---------------------------------------------------------------------
        -- Flag clear by SPDR access
        -- Datasheet: read SPSR with flag set, then access SPDR.
        -- Access means read OR write.
        ---------------------------------------------------------------------
        if v_spdr_access = '1' then
          if arm_spif_q = '1' then
            v_spif_clear := '1';
          end if;

          if arm_wcol_q = '1' then
            v_wcol_clear := '1';
          end if;
        end if;

        ---------------------------------------------------------------------
        -- Hardware interrupt-vector clear
        ---------------------------------------------------------------------
        if irq_ack = '1' then
          v_spif_clear := '1';
        end if;

        ---------------------------------------------------------------------
        -- Flag next-state computation
        -- Set has priority over clear to avoid losing a new event.
        ---------------------------------------------------------------------
        v_spif_next := spif_q;
        if v_spif_clear = '1' then
          v_spif_next := '0';
        end if;
        if v_spif_set = '1' then
          v_spif_next := '1';
        end if;

        v_wcol_next := wcol_q;
        if v_wcol_clear = '1' then
          v_wcol_next := '0';
        end if;
        if v_wcol_set = '1' then
          v_wcol_next := '1';
        end if;

        ---------------------------------------------------------------------
        -- Flag-clear sequence arm update
        ---------------------------------------------------------------------
        v_arm_spif_next := arm_spif_q;
        v_arm_wcol_next := arm_wcol_q;

        -- SPDR access completes the sequence.
        if v_spdr_access = '1' then
          v_arm_spif_next := '0';
          v_arm_wcol_next := '0';
        end if;

        -- Reading SPSR arms the flags that are currently set.
        if v_spsr_read = '1' then
          v_arm_spif_next := spif_q;
          v_arm_wcol_next := wcol_q;
        end if;

        -- If a flag is not set, it cannot remain armed.
        if v_spif_next = '0' then
          v_arm_spif_next := '0';
        end if;

        if v_wcol_next = '0' then
          v_arm_wcol_next := '0';
        end if;

        ---------------------------------------------------------------------
        -- Register updates
        ---------------------------------------------------------------------
        spcr_q     <= v_spcr_next;
        spif_q     <= v_spif_next;
        wcol_q     <= v_wcol_next;
        arm_spif_q <= v_arm_spif_next;
        arm_wcol_q <= v_arm_wcol_next;

      end if;
    end if;
  end process main_proc;

end architecture rtl;
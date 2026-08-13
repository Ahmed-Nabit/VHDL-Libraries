-- =====================================================================
-- USART - Fixed, production-oriented implementation
-- =====================================================================
-- Coding constraints honored:
--   * No natural or real numbers
--   * No concurrent signal assignments
--   * Variables are used for internal state and computations
--   * All signal assignments are inside one process
--
-- Fixes included:
--   * Atomic UDR read/write side effects
--   * RXC / UDRE / TXC update timing made consistent
--   * TXC no longer falsely set when a new UDR write occurs at frame end
--   * DOR / receive-shift-register third-buffer corner cases handled
--   * Synchronous back-to-back transmission gap removed
--   * Shared UBRRH/UCSRC read sequence preserved
--   * Added tx_oe for proper TxD pin override / production integration
-- =====================================================================
-- Copyright © 2024-2026 Ahmed Nabit <Lazrdo@gmail.com>
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at:
--     http://www.apache.org/licenses/LICENSE-2.0
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

library ieee;
use ieee.std_logic_1164.all;

entity usart is
  port (
    clk        : in  std_logic;
    reset      : in  std_logic;  -- asynchronous active-high reset

    -- Serial I/O
    rx_in      : in  std_logic;
    tx_out     : out std_logic;
    tx_oe      : out std_logic;

    -- Synchronous clock I/O
    xck_in     : in  std_logic;
    xck_out    : out std_logic;
    xck_oe     : out std_logic;
    ddr_xck    : in  std_logic;  -- 1 = master (output), 0 = slave (input)

    -- Register bus
    addr       : in  std_logic_vector(3 downto 0);
    din        : in  std_logic_vector(7 downto 0);
    dout       : out std_logic_vector(7 downto 0);
    we         : in  std_logic;
    re         : in  std_logic;

    -- Interrupts
    rx_irq     : out std_logic;
    tx_irq     : out std_logic;
    udre_irq   : out std_logic;

    -- Transmit-complete interrupt acknowledge.
    -- Drive to '0' if unused.
    tx_irq_ack : in  std_logic
  );
end entity usart;

architecture rtl of usart is

  -- ===================================================================
  -- Register addresses
  -- ===================================================================
  constant ADDR_UDR    : std_logic_vector(3 downto 0) := "0000";
  constant ADDR_UCSRA  : std_logic_vector(3 downto 0) := "0001";
  constant ADDR_UCSRB  : std_logic_vector(3 downto 0) := "0010";
  constant ADDR_SHARED : std_logic_vector(3 downto 0) := "0011"; -- UBRRH / UCSRC
  constant ADDR_UBRRL  : std_logic_vector(3 downto 0) := "0100";

  -- ===================================================================
  -- Types
  -- ===================================================================
  type tx_state_t is (TX_IDLE, TX_START, TX_DATA, TX_PARITY, TX_STOP1, TX_STOP2);
  type rx_state_t is (RX_IDLE, RX_START, RX_DATA, RX_PARITY, RX_STOP);

  type fifo_entry_t is record
    data : std_logic_vector(8 downto 0);
    fe   : std_logic;
    dor  : std_logic;
    pe   : std_logic;
  end record;

  type fifo_array_t is array (integer range 0 to 1) of fifo_entry_t;

  -- ===================================================================
  -- Helper functions (no natural / real, no to_integer)
  -- ===================================================================
  function parity_even(data : std_logic_vector; width : integer) return std_logic is
    variable p : std_logic;
  begin
    p := '0';
    for i in 0 to width - 1 loop
      p := p xor data(i);
    end loop;
    return p;
  end function;

  function parity_odd(data : std_logic_vector; width : integer) return std_logic is
  begin
    return not parity_even(data, width);
  end function;

  function slv_to_int(v : std_logic_vector) return integer is
    variable r : integer range 0 to 4095;
  begin
    r := 0;
    for i in v'range loop
      r := r * 2;
      if v(i) = '1' then
        r := r + 1;
      end if;
    end loop;
    return r;
  end function;

  function int_to_slv8(v : integer) return std_logic_vector is
    variable r : std_logic_vector(7 downto 0);
    variable x : integer;
  begin
    x := v;
    r := (others => '0');
    for i in 0 to 7 loop
      if (x rem 2) = 1 then
        r(i) := '1';
      else
        r(i) := '0';
      end if;
      x := x / 2;
    end loop;
    return r;
  end function;

  function int_to_slv4(v : integer) return std_logic_vector is
    variable r : std_logic_vector(3 downto 0);
    variable x : integer;
  begin
    x := v;
    r := (others => '0');
    for i in 0 to 3 loop
      if (x rem 2) = 1 then
        r(i) := '1';
      else
        r(i) := '0';
      end if;
      x := x / 2;
    end loop;
    return r;
  end function;

begin

  -- ===================================================================
  -- Single process containing all state and behavior
  -- ===================================================================
  main_proc : process (clk, reset) is

    -------------------------------------------------------------------
    -- Control registers
    -------------------------------------------------------------------
    variable u2x_q   : std_logic := '0';
    variable mpcm_q  : std_logic := '0';

    variable rxcie_q : std_logic := '0';
    variable txcie_q : std_logic := '0';
    variable udrie_q : std_logic := '0';
    variable rxen_q  : std_logic := '0';
    variable txen_q  : std_logic := '0';
    variable ucsz2_q : std_logic := '0';
    variable txb8_q  : std_logic := '0';

    variable umsel_q : std_logic := '0';
    variable upm1_q  : std_logic := '0';
    variable upm0_q  : std_logic := '0';
    variable usbs_q  : std_logic := '0';
    variable ucsz1_q : std_logic := '1';
    variable ucsz0_q : std_logic := '1';
    variable ucpol_q : std_logic := '0';

    variable ubrr_q : integer range 0 to 4095 := 0;

    -------------------------------------------------------------------
    -- Shared register read sequence
    -------------------------------------------------------------------
    variable prev_shared_q : boolean := false;

    -------------------------------------------------------------------
    -- Bus / output register
    -------------------------------------------------------------------
    variable dout_q : std_logic_vector(7 downto 0) := (others => '0');

    -------------------------------------------------------------------
    -- Transmitter state
    -------------------------------------------------------------------
    variable tx_state_q       : tx_state_t := TX_IDLE;
    variable tx_out_q         : std_logic := '1';
    variable txc_q            : std_logic := '0';
    variable udre_q           : std_logic := '1';
    variable tx_buf_full_q    : std_logic := '0';
    variable tx_buf_q         : std_logic_vector(8 downto 0) := (others => '0');
    variable tx_shift_q       : std_logic_vector(8 downto 0) := (others => '0');
    variable tx_bit_cnt_q     : integer range 0 to 8 := 0;
    variable tx_timer_q       : integer range 0 to 65535 := 0;
    variable tx_sync_first_q  : std_logic := '0';

    variable tx_char_bits_q   : integer range 5 to 9 := 8;
    variable tx_parity_en_q   : boolean := false;
    variable tx_odd_q         : std_logic := '0';
    variable tx_stop2_q       : std_logic := '0';
    variable tx_parity_bit_q  : std_logic := '0';

    -------------------------------------------------------------------
    -- Receiver state
    -------------------------------------------------------------------
    variable rx_state_q       : rx_state_t := RX_IDLE;

    variable rx_meta_q        : std_logic := '0';
    variable rx_sync_q        : std_logic := '0';
    variable rx_sync_d_q      : std_logic := '0';

    variable rx_shift_q       : std_logic_vector(8 downto 0) := (others => '0');
    variable rx_bit_cnt_q     : integer range 0 to 8 := 0;
    variable rx_s0_q          : std_logic := '0';
    variable rx_s1_q          : std_logic := '0';
    variable rx_pe_latched_q  : std_logic := '0';

    variable rx_shift_full_q  : std_logic := '0';
    variable rx_dor_pending_q : std_logic := '0';

    variable rx_char_bits_q   : integer range 5 to 9 := 8;
    variable rx_parity_en_q   : boolean := false;
    variable rx_odd_q         : std_logic := '0';
    variable rx_mpcm_q        : std_logic := '0';

    variable rx_pending_data_q : std_logic_vector(8 downto 0) := (others => '0');
    variable rx_pending_fe_q   : std_logic := '0';
    variable rx_pending_dor_q  : std_logic := '0';
    variable rx_pending_pe_q   : std_logic := '0';

    -------------------------------------------------------------------
    -- Asynchronous sample generator
    -------------------------------------------------------------------
    variable baud_cnt_q       : integer range 0 to 4095 := 0;
    variable rx_sample_pos_q  : integer range 0 to 15 := 0;

    -------------------------------------------------------------------
    -- Receive FIFO
    -------------------------------------------------------------------
    variable fifo_q       : fifo_array_t;
    variable fifo_wr_q    : integer range 0 to 1 := 0;
    variable fifo_rd_q    : integer range 0 to 1 := 0;
    variable fifo_cnt_q   : integer range 0 to 2 := 0;

    variable rxc_q        : std_logic := '0';
    variable head_data_q  : std_logic_vector(8 downto 0) := (others => '0');
    variable head_fe_q    : std_logic := '0';
    variable head_dor_q   : std_logic := '0';
    variable head_pe_q    : std_logic := '0';
    variable rxb8_q       : std_logic := '0';

    -------------------------------------------------------------------
    -- Synchronous clock state
    -------------------------------------------------------------------
    variable xck_meta_q    : std_logic := '0';
    variable xck_sync_q    : std_logic := '0';
    variable xck_sync_d_q  : std_logic := '0';

    variable xck_q         : std_logic := '0';
    variable xck_cnt_q     : integer range 0 to 4095 := 0;
    variable xck_out_q     : std_logic := '0';
    variable xck_oe_q      : std_logic := '0';

    -------------------------------------------------------------------
    -- Local per-cycle variables
    -------------------------------------------------------------------
    variable v_char_bits       : integer range 5 to 9;
    variable v_parity_en       : boolean;
    variable v_samples_per_bit : integer range 8 to 16;
    variable v_sample_first    : integer range 0 to 15;
    variable v_sample_center   : integer range 0 to 15;
    variable v_sample_last     : integer range 0 to 15;
    variable v_ubrr_val        : integer range 0 to 4095;

    variable v_rxd             : std_logic;
    variable v_rx_prev         : std_logic;

    variable v_xck_sync        : std_logic;
    variable v_xck_sync_prev   : std_logic;

    variable v_slave_rising    : boolean;
    variable v_slave_falling   : boolean;

    variable v_master_run      : boolean;
    variable v_master_edge     : boolean;
    variable v_master_rising   : boolean;
    variable v_master_falling  : boolean;

    variable v_xck_change      : boolean;
    variable v_xck_sample      : boolean;

    variable v_xck_q_old       : std_logic;

    variable v_fifo            : fifo_array_t;
    variable v_wr              : integer range 0 to 1;
    variable v_rd              : integer range 0 to 1;
    variable v_cnt             : integer range 0 to 2;

    variable v_prev_shared_next : boolean;

    variable v_udr_write       : boolean;
    variable v_udr_write_data  : std_logic_vector(7 downto 0);
    variable v_txc_clear       : boolean;
    variable v_ubrrl_write     : boolean;

    variable v_tx_tick         : boolean;
    variable v_period          : integer range 0 to 65536;

    variable v_rx_enabled      : boolean;

    variable v_rx_sample_tick  : boolean;
    variable v_rx_counter_reset: boolean;

    variable v_rx_state        : rx_state_t;
    variable v_shift_full      : std_logic;
    variable v_dor_pending     : std_logic;

    variable v_complete        : boolean;
    variable v_stop_err        : std_logic;
    variable v_pe_err          : std_logic;
    variable v_frame_type      : std_logic;
    variable v_bit             : std_logic;
    variable v_majority        : std_logic;
    variable v_parity_expected : std_logic;
    variable v_frame_accept    : boolean;

    variable v_rxb8_read       : std_logic;
    variable v_tx_oe           : std_logic;

  begin

    -------------------------------------------------------------------
    -- Asynchronous reset
    -------------------------------------------------------------------
    if reset = '1' then

      u2x_q   := '0';
      mpcm_q  := '0';

      rxcie_q := '0';
      txcie_q := '0';
      udrie_q := '0';
      rxen_q  := '0';
      txen_q  := '0';
      ucsz2_q := '0';
      txb8_q  := '0';

      umsel_q := '0';
      upm1_q  := '0';
      upm0_q  := '0';
      usbs_q  := '0';
      ucsz1_q := '1';
      ucsz0_q := '1';
      ucpol_q := '0';

      ubrr_q := 0;

      prev_shared_q := false;
      dout_q := (others => '0');

      tx_state_q      := TX_IDLE;
      tx_out_q        := '1';
      txc_q           := '0';
      udre_q          := '1';
      tx_buf_full_q   := '0';
      tx_buf_q        := (others => '0');
      tx_shift_q      := (others => '0');
      tx_bit_cnt_q    := 0;
      tx_timer_q      := 0;
      tx_sync_first_q := '0';

      tx_char_bits_q  := 8;
      tx_parity_en_q  := false;
      tx_odd_q        := '0';
      tx_stop2_q      := '0';
      tx_parity_bit_q := '0';

      rx_state_q       := RX_IDLE;
      rx_meta_q        := '0';
      rx_sync_q        := '0';
      rx_sync_d_q      := '0';
      rx_shift_q       := (others => '0');
      rx_bit_cnt_q     := 0;
      rx_s0_q          := '0';
      rx_s1_q          := '0';
      rx_pe_latched_q  := '0';
      rx_shift_full_q  := '0';
      rx_dor_pending_q := '0';

      rx_char_bits_q   := 8;
      rx_parity_en_q   := false;
      rx_odd_q         := '0';
      rx_mpcm_q        := '0';

      rx_pending_data_q := (others => '0');
      rx_pending_fe_q   := '0';
      rx_pending_dor_q  := '0';
      rx_pending_pe_q   := '0';

      baud_cnt_q      := 0;
      rx_sample_pos_q := 0;

      fifo_wr_q  := 0;
      fifo_rd_q  := 0;
      fifo_cnt_q := 0;
      rxc_q      := '0';

      head_data_q := (others => '0');
      head_fe_q   := '0';
      head_dor_q  := '0';
      head_pe_q   := '0';
      rxb8_q      := '0';

      for i in 0 to 1 loop
        fifo_q(i).data := (others => '0');
        fifo_q(i).fe   := '0';
        fifo_q(i).dor  := '0';
        fifo_q(i).pe   := '0';
      end loop;

      xck_meta_q   := '0';
      xck_sync_q   := '0';
      xck_sync_d_q := '0';
      xck_q        := '0';
      xck_cnt_q    := 0;
      xck_out_q    := '0';
      xck_oe_q     := '0';

    -------------------------------------------------------------------
    -- Clock edge
    -------------------------------------------------------------------
    elsif rising_edge(clk) then

      -----------------------------------------------------------------
      -- Capture old synchronized inputs for edge/data usage
      -----------------------------------------------------------------
      v_rxd           := rx_sync_q;
      v_rx_prev       := rx_sync_d_q;
      v_xck_sync      := xck_sync_q;
      v_xck_sync_prev := xck_sync_d_q;

      -----------------------------------------------------------------
      -- Update input synchronizers
      -----------------------------------------------------------------
      rx_sync_d_q := rx_sync_q;
      rx_sync_q   := rx_meta_q;
      rx_meta_q   := rx_in;

      xck_sync_d_q := xck_sync_q;
      xck_sync_q   := xck_meta_q;
      xck_meta_q   := xck_in;

      -----------------------------------------------------------------
      -- Local FIFO working copies
      -----------------------------------------------------------------
      v_fifo := fifo_q;
      v_wr   := fifo_wr_q;
      v_rd   := fifo_rd_q;
      v_cnt  := fifo_cnt_q;

      v_prev_shared_next := false;

      -----------------------------------------------------------------
      -- Read accesses
      -----------------------------------------------------------------
      if re = '1' then
        case addr is

          when ADDR_UDR =>
            if v_cnt > 0 then
              dout_q := v_fifo(v_rd).data(7 downto 0);

              v_fifo(v_rd).data := (others => '0');
              v_fifo(v_rd).fe   := '0';
              v_fifo(v_rd).dor  := '0';
              v_fifo(v_rd).pe   := '0';

              if v_rd = 0 then
                v_rd := 1;
              else
                v_rd := 0;
              end if;

              v_cnt := v_cnt - 1;
            else
              dout_q := (others => '0');
            end if;

          when ADDR_UCSRA =>
            if v_cnt > 0 then
              dout_q := '1' & txc_q & udre_q &
                        v_fifo(v_rd).fe & v_fifo(v_rd).dor & v_fifo(v_rd).pe &
                        u2x_q & mpcm_q;
            else
              dout_q := '0' & txc_q & udre_q &
                        '0' & '0' & '0' &
                        u2x_q & mpcm_q;
            end if;

          when ADDR_UCSRB =>
            v_rxb8_read := '0';
            if v_cnt > 0 then
              v_rxb8_read := v_fifo(v_rd).data(8);
            end if;

            dout_q := rxcie_q & txcie_q & udrie_q &
                      rxen_q & txen_q & ucsz2_q &
                      v_rxb8_read & txb8_q;

          when ADDR_SHARED =>
            if prev_shared_q then
              dout_q := '1' & umsel_q & upm1_q & upm0_q &
                        usbs_q & ucsz1_q & ucsz0_q & ucpol_q;
            else
              dout_q := "0000" & int_to_slv4(ubrr_q / 256);
            end if;
            v_prev_shared_next := true;

          when ADDR_UBRRL =>
            dout_q := int_to_slv8(ubrr_q mod 256);

          when others =>
            dout_q := (others => '0');

        end case;
      end if;

      -----------------------------------------------------------------
      -- Write accesses
      -----------------------------------------------------------------
      v_udr_write      := false;
      v_udr_write_data := (others => '0');
      v_txc_clear      := false;
      v_ubrrl_write    := false;

      if we = '1' then
        case addr is

          when ADDR_UDR =>
            if txen_q = '1' and udre_q = '1' then
              v_udr_write      := true;
              v_udr_write_data := din;
            end if;

          when ADDR_UCSRA =>
            u2x_q  := din(1);
            mpcm_q := din(0);

            if din(6) = '1' then
              v_txc_clear := true;
            end if;

          when ADDR_UCSRB =>
            rxcie_q := din(7);
            txcie_q := din(6);
            udrie_q := din(5);
            rxen_q  := din(4);
            txen_q  := din(3);
            ucsz2_q := din(2);
            txb8_q  := din(0);

          when ADDR_SHARED =>
            if din(7) = '1' then
              umsel_q := din(6);
              upm1_q  := din(5);
              upm0_q  := din(4);
              usbs_q  := din(3);
              ucsz1_q := din(2);
              ucsz0_q := din(1);
              ucpol_q := din(0);
            else
              ubrr_q := slv_to_int(din(3 downto 0)) * 256 + (ubrr_q mod 256);
            end if;

          when ADDR_UBRRL =>
            ubrr_q         := (ubrr_q / 256) * 256 + slv_to_int(din);
            v_ubrrl_write  := true;

          when others =>
            null;

        end case;
      end if;

      prev_shared_q := v_prev_shared_next;

      -----------------------------------------------------------------
      -- Derived frame / baud configuration
      -----------------------------------------------------------------
      if ucsz2_q = '0' and ucsz1_q = '0' and ucsz0_q = '0' then
        v_char_bits := 5;
      elsif ucsz2_q = '0' and ucsz1_q = '0' and ucsz0_q = '1' then
        v_char_bits := 6;
      elsif ucsz2_q = '0' and ucsz1_q = '1' and ucsz0_q = '0' then
        v_char_bits := 7;
      elsif ucsz2_q = '0' and ucsz1_q = '1' and ucsz0_q = '1' then
        v_char_bits := 8;
      elsif ucsz2_q = '1' and ucsz1_q = '1' and ucsz0_q = '1' then
        v_char_bits := 9;
      else
        v_char_bits := 8;
      end if;

      v_parity_en := (upm1_q = '1');

      if u2x_q = '1' and umsel_q = '0' then
        v_samples_per_bit := 8;
      else
        v_samples_per_bit := 16;
      end if;

      if v_samples_per_bit = 8 then
        v_sample_first  := 3;
        v_sample_center := 4;
        v_sample_last   := 5;
      else
        v_sample_first  := 7;
        v_sample_center := 8;
        v_sample_last   := 9;
      end if;

      v_ubrr_val := ubrr_q;

      -----------------------------------------------------------------
      -- Receiver disabled: immediate flush and state clear
      -----------------------------------------------------------------
      if rxen_q = '0' then
        v_rx_enabled := false;

        rx_state_q       := RX_IDLE;
        rx_shift_full_q  := '0';
        rx_dor_pending_q := '0';
        rx_pe_latched_q  := '0';
        rx_s0_q          := '0';
        rx_s1_q          := '0';
        rx_shift_q       := (others => '0');
        rx_bit_cnt_q     := 0;

        rx_char_bits_q := 8;
        rx_parity_en_q := false;
        rx_odd_q       := '0';
        rx_mpcm_q      := '0';

        rx_pending_data_q := (others => '0');
        rx_pending_fe_q   := '0';
        rx_pending_dor_q  := '0';
        rx_pending_pe_q   := '0';

        baud_cnt_q      := 0;
        rx_sample_pos_q := 0;

        fifo_wr_q := 0;
        fifo_rd_q := 0;
        fifo_cnt_q := 0;
        rxc_q      := '0';

        head_data_q := (others => '0');
        head_fe_q   := '0';
        head_dor_q  := '0';
        head_pe_q   := '0';
        rxb8_q      := '0';

        for i in 0 to 1 loop
          fifo_q(i).data := (others => '0');
          fifo_q(i).fe   := '0';
          fifo_q(i).dor  := '0';
          fifo_q(i).pe   := '0';
        end loop;

      else
        v_rx_enabled := true;
      end if;

      -----------------------------------------------------------------
      -- Synchronous clock generation / edge detection
      -----------------------------------------------------------------
      v_slave_rising  := (v_xck_sync = '1' and v_xck_sync_prev = '0');
      v_slave_falling := (v_xck_sync = '0' and v_xck_sync_prev = '1');

      v_master_run := (umsel_q = '1' and ddr_xck = '1' and
                       (txen_q = '1' or rxen_q = '1' or
                        tx_state_q /= TX_IDLE or
                        tx_buf_full_q = '1' or
                        rx_state_q /= RX_IDLE));

      v_master_edge    := false;
      v_master_rising  := false;
      v_master_falling := false;

      xck_oe_q := '0';

      if umsel_q = '1' and ddr_xck = '1' then
        xck_oe_q := '1';

        if v_master_run then

          if v_ubrrl_write then
            xck_cnt_q := 0;
          end if;

          v_xck_q_old := xck_q;

          if xck_cnt_q >= v_ubrr_val then
            v_master_edge := true;

            if v_xck_q_old = '0' then
              v_master_rising := true;
            else
              v_master_falling := true;
            end if;

            xck_q     := not v_xck_q_old;
            xck_out_q := not v_xck_q_old;
            xck_cnt_q := 0;
          else
            xck_out_q := xck_q;
            xck_cnt_q := xck_cnt_q + 1;
          end if;

        else
          xck_cnt_q := 0;

          if ucpol_q = '1' then
            xck_q     := '1';
            xck_out_q := '1';
          else
            xck_q     := '0';
            xck_out_q := '0';
          end if;
        end if;

      else
        xck_oe_q  := '0';
        xck_out_q := '0';
        xck_cnt_q := 0;

        if ucpol_q = '1' then
          xck_q := '1';
        else
          xck_q := '0';
        end if;
      end if;

      v_xck_change := false;
      v_xck_sample := false;

      if umsel_q = '1' then
        if ddr_xck = '1' then
          if v_master_edge then
            if ucpol_q = '0' then
              if v_master_rising then
                v_xck_change := true;
              elsif v_master_falling then
                v_xck_sample := true;
              end if;
            else
              if v_master_falling then
                v_xck_change := true;
              elsif v_master_rising then
                v_xck_sample := true;
              end if;
            end if;
          end if;
        else
          if ucpol_q = '0' then
            if v_slave_rising then
              v_xck_change := true;
            elsif v_slave_falling then
              v_xck_sample := true;
            end if;
          else
            if v_slave_falling then
              v_xck_change := true;
            elsif v_slave_rising then
              v_xck_sample := true;
            end if;
          end if;
        end if;
      end if;

      -----------------------------------------------------------------
      -- TXC clear sources
      -----------------------------------------------------------------
      if v_txc_clear or tx_irq_ack = '1' then
        txc_q := '0';
      end if;

      -----------------------------------------------------------------
      -- Accept UDR write into transmit buffer
      -----------------------------------------------------------------
      if v_udr_write and txen_q = '1' and tx_buf_full_q = '0' then
        tx_buf_q      := txb8_q & v_udr_write_data;
        tx_buf_full_q := '1';
        udre_q        := '0';
      end if;

      -----------------------------------------------------------------
      -- Asynchronous transmitter bit timer
      -----------------------------------------------------------------
      v_tx_tick := false;

      if umsel_q = '0' and tx_state_q /= TX_IDLE then
        v_period := (v_ubrr_val + 1) * v_samples_per_bit;

        if tx_timer_q >= v_period - 1 then
          tx_timer_q := 0;
          v_tx_tick  := true;
        else
          tx_timer_q := tx_timer_q + 1;
        end if;
      else
        tx_timer_q := 0;
      end if;

      -----------------------------------------------------------------
      -- Transmit state machine
      -----------------------------------------------------------------
      if (umsel_q = '0' and v_tx_tick) or
         (umsel_q = '1' and v_xck_change) then

        case tx_state_q is

          when TX_IDLE =>
            null;

          when TX_START =>
            if umsel_q = '1' then
              if tx_sync_first_q = '1' then
                tx_out_q        := '0';
                tx_sync_first_q := '0';
              else
                tx_out_q     := tx_shift_q(0);
                tx_bit_cnt_q := 0;
                tx_state_q   := TX_DATA;
              end if;
            else
              tx_out_q     := tx_shift_q(0);
              tx_bit_cnt_q := 0;
              tx_state_q   := TX_DATA;
            end if;

          when TX_DATA =>
            if tx_bit_cnt_q = tx_char_bits_q - 1 then
              if tx_parity_en_q then
                tx_out_q   := tx_parity_bit_q;
                tx_state_q := TX_PARITY;
              else
                tx_out_q   := '1';
                tx_state_q := TX_STOP1;
              end if;
            else
              tx_bit_cnt_q := tx_bit_cnt_q + 1;
              tx_out_q     := tx_shift_q(tx_bit_cnt_q + 1);
            end if;

          when TX_PARITY =>
            tx_out_q   := '1';
            tx_state_q := TX_STOP1;

          when TX_STOP1 =>
            if tx_stop2_q = '1' then
              tx_out_q   := '1';
              tx_state_q := TX_STOP2;
            else
              if tx_buf_full_q = '1' then
                -- Back-to-back frame: start immediately, no sync gap.
                tx_shift_q     := tx_buf_q;
                tx_buf_full_q  := '0';
                udre_q         := '1';
                tx_bit_cnt_q   := 0;
                tx_char_bits_q := v_char_bits;
                tx_parity_en_q := v_parity_en;
                tx_odd_q       := upm0_q;
                tx_stop2_q     := usbs_q;

                if upm0_q = '0' then
                  tx_parity_bit_q := parity_even(tx_buf_q, v_char_bits);
                else
                  tx_parity_bit_q := parity_odd(tx_buf_q, v_char_bits);
                end if;

                tx_out_q        := '0';
                tx_sync_first_q := '0';
                tx_state_q      := TX_START;
                tx_timer_q      := 0;
              else
                tx_state_q := TX_IDLE;
                tx_out_q   := '1';
                txc_q      := '1';
              end if;
            end if;

          when TX_STOP2 =>
            if tx_buf_full_q = '1' then
              -- Back-to-back frame: start immediately, no sync gap.
              tx_shift_q     := tx_buf_q;
              tx_buf_full_q  := '0';
              udre_q         := '1';
              tx_bit_cnt_q   := 0;
              tx_char_bits_q := v_char_bits;
              tx_parity_en_q := v_parity_en;
              tx_odd_q       := upm0_q;
              tx_stop2_q     := usbs_q;

              if upm0_q = '0' then
                tx_parity_bit_q := parity_even(tx_buf_q, v_char_bits);
              else
                tx_parity_bit_q := parity_odd(tx_buf_q, v_char_bits);
              end if;

              tx_out_q        := '0';
              tx_sync_first_q := '0';
              tx_state_q      := TX_START;
              tx_timer_q      := 0;
            else
              tx_state_q := TX_IDLE;
              tx_out_q   := '1';
              txc_q      := '1';
            end if;

          when others =>
            tx_state_q := TX_IDLE;

        end case;
      end if;

      -----------------------------------------------------------------
      -- Start a new frame when idle and buffer is full
      -----------------------------------------------------------------
      if tx_state_q = TX_IDLE and tx_buf_full_q = '1' then
        tx_shift_q     := tx_buf_q;
        tx_buf_full_q  := '0';
        udre_q         := '1';
        tx_bit_cnt_q   := 0;
        tx_char_bits_q := v_char_bits;
        tx_parity_en_q := v_parity_en;
        tx_odd_q       := upm0_q;
        tx_stop2_q     := usbs_q;

        if upm0_q = '0' then
          tx_parity_bit_q := parity_even(tx_buf_q, v_char_bits);
        else
          tx_parity_bit_q := parity_odd(tx_buf_q, v_char_bits);
        end if;

        if umsel_q = '1' then
          tx_out_q        := '1';
          tx_sync_first_q := '1';
        else
          tx_out_q        := '0';
          tx_sync_first_q := '0';
        end if;

        tx_state_q := TX_START;
        tx_timer_q := 0;
      end if;

      -----------------------------------------------------------------
      -- Idle line high when truly idle
      -----------------------------------------------------------------
      if tx_state_q = TX_IDLE and tx_buf_full_q = '0' then
        tx_out_q := '1';
      end if;

      -----------------------------------------------------------------
      -- Receiver enabled path
      -----------------------------------------------------------------
      if v_rx_enabled then

        -----------------------------------------------------------------
        -- Asynchronous sample tick generator
        -----------------------------------------------------------------
        v_rx_sample_tick   := false;
        v_rx_counter_reset := false;

        if umsel_q = '0' and not v_ubrrl_write then
          if baud_cnt_q >= v_ubrr_val then
            v_rx_sample_tick := true;
            baud_cnt_q       := 0;

            if rx_sample_pos_q >= v_samples_per_bit - 1 then
              rx_sample_pos_q := 0;
            else
              rx_sample_pos_q := rx_sample_pos_q + 1;
            end if;
          else
            baud_cnt_q := baud_cnt_q + 1;
          end if;
        else
          baud_cnt_q      := 0;
          rx_sample_pos_q := 0;
        end if;

        -----------------------------------------------------------------
        -- Receiver local working copies
        -----------------------------------------------------------------
        v_rx_state     := rx_state_q;
        v_shift_full   := rx_shift_full_q;
        v_dor_pending  := rx_dor_pending_q;

        v_complete     := false;
        v_stop_err     := '0';
        v_pe_err       := '0';
        v_frame_type   := '0';
        v_bit          := '0';
        v_majority     := '0';

        -----------------------------------------------------------------
        -- Move pending shift-register frame into FIFO if space exists
        -----------------------------------------------------------------
        if v_shift_full = '1' and v_cnt < 2 then
          v_fifo(v_wr).data := rx_pending_data_q;
          v_fifo(v_wr).fe   := rx_pending_fe_q;
          v_fifo(v_wr).dor  := rx_pending_dor_q or v_dor_pending;
          v_fifo(v_wr).pe   := rx_pending_pe_q;

          if v_wr = 0 then
            v_wr := 1;
          else
            v_wr := 0;
          end if;

          v_cnt := v_cnt + 1;

          v_shift_full  := '0';
          v_dor_pending := '0';
        end if;

        -----------------------------------------------------------------
        -- Asynchronous or synchronous receiver state machine
        -----------------------------------------------------------------
        if umsel_q = '0' then

          if v_rx_sample_tick then
            case v_rx_state is

              when RX_IDLE =>
                null;

              when RX_START =>
                if rx_sample_pos_q = v_sample_first then
                  rx_s0_q := v_rxd;
                elsif rx_sample_pos_q = v_sample_center then
                  rx_s1_q := v_rxd;
                elsif rx_sample_pos_q = v_sample_last then
                  v_majority := (rx_s0_q and rx_s1_q) or
                                (rx_s0_q and v_rxd) or
                                (rx_s1_q and v_rxd);

                  if v_majority = '1' then
                    v_rx_state := RX_IDLE;
                  else
                    v_rx_state   := RX_DATA;
                    rx_bit_cnt_q := 0;
                  end if;
                end if;

              when RX_DATA =>
                if rx_sample_pos_q = v_sample_first then
                  rx_s0_q := v_rxd;
                elsif rx_sample_pos_q = v_sample_center then
                  rx_s1_q := v_rxd;
                elsif rx_sample_pos_q = v_sample_last then
                  v_bit := (rx_s0_q and rx_s1_q) or
                           (rx_s0_q and v_rxd) or
                           (rx_s1_q and v_rxd);

                  rx_shift_q(rx_bit_cnt_q) := v_bit;

                  if rx_bit_cnt_q = rx_char_bits_q - 1 then
                    if rx_parity_en_q then
                      v_rx_state := RX_PARITY;
                    else
                      v_rx_state := RX_STOP;
                    end if;
                  else
                    rx_bit_cnt_q := rx_bit_cnt_q + 1;
                  end if;
                end if;

              when RX_PARITY =>
                if rx_sample_pos_q = v_sample_first then
                  rx_s0_q := v_rxd;
                elsif rx_sample_pos_q = v_sample_center then
                  rx_s1_q := v_rxd;
                elsif rx_sample_pos_q = v_sample_last then
                  v_bit := (rx_s0_q and rx_s1_q) or
                           (rx_s0_q and v_rxd) or
                           (rx_s1_q and v_rxd);

                  if rx_odd_q = '0' then
                    v_parity_expected := parity_even(rx_shift_q, rx_char_bits_q);
                  else
                    v_parity_expected := parity_odd(rx_shift_q, rx_char_bits_q);
                  end if;

                  rx_pe_latched_q := v_parity_expected xor v_bit;
                  v_rx_state      := RX_STOP;
                end if;

              when RX_STOP =>
                if rx_sample_pos_q = v_sample_first then
                  rx_s0_q := v_rxd;
                elsif rx_sample_pos_q = v_sample_center then
                  rx_s1_q := v_rxd;
                elsif rx_sample_pos_q = v_sample_last then
                  v_bit := (rx_s0_q and rx_s1_q) or
                           (rx_s0_q and v_rxd) or
                           (rx_s1_q and v_rxd);

                  v_complete := true;
                  v_stop_err := not v_bit;

                  if rx_parity_en_q then
                    v_pe_err := rx_pe_latched_q;
                  else
                    v_pe_err := '0';
                  end if;

                  if rx_char_bits_q = 9 then
                    v_frame_type := rx_shift_q(8);
                  else
                    v_frame_type := v_bit;
                  end if;

                  v_rx_state := RX_IDLE;
                end if;

              when others =>
                v_rx_state := RX_IDLE;

            end case;
          end if;

        else

          if v_xck_sample then
            case v_rx_state is

              when RX_IDLE =>
                if v_rxd = '0' then
                  if v_shift_full = '1' then
                    v_dor_pending := '1';
                    v_shift_full  := '0';
                  end if;

                  rx_char_bits_q := v_char_bits;
                  rx_parity_en_q := v_parity_en;
                  rx_odd_q       := upm0_q;
                  rx_mpcm_q      := mpcm_q;

                  rx_shift_q      := (others => '0');
                  rx_bit_cnt_q    := 0;
                  rx_pe_latched_q := '0';

                  v_rx_state := RX_DATA;
                end if;

              when RX_DATA =>
                rx_shift_q(rx_bit_cnt_q) := v_rxd;

                if rx_bit_cnt_q = rx_char_bits_q - 1 then
                  if rx_parity_en_q then
                    v_rx_state := RX_PARITY;
                  else
                    v_rx_state := RX_STOP;
                  end if;
                else
                  rx_bit_cnt_q := rx_bit_cnt_q + 1;
                end if;

              when RX_PARITY =>
                if rx_odd_q = '0' then
                  v_parity_expected := parity_even(rx_shift_q, rx_char_bits_q);
                else
                  v_parity_expected := parity_odd(rx_shift_q, rx_char_bits_q);
                end if;

                rx_pe_latched_q := v_parity_expected xor v_rxd;
                v_rx_state      := RX_STOP;

              when RX_STOP =>
                v_complete := true;
                v_stop_err := not v_rxd;

                if rx_parity_en_q then
                  v_pe_err := rx_pe_latched_q;
                else
                  v_pe_err := '0';
                end if;

                if rx_char_bits_q = 9 then
                  v_frame_type := rx_shift_q(8);
                else
                  v_frame_type := v_rxd;
                end if;

                v_rx_state := RX_IDLE;

              when others =>
                v_rx_state := RX_IDLE;

            end case;
          end if;

        end if;

        -----------------------------------------------------------------
        -- Frame completion handling
        -----------------------------------------------------------------
        if v_complete then
          v_frame_accept := (rx_mpcm_q = '0') or (v_frame_type = '1');

          if v_frame_accept then
            if v_cnt < 2 then
              v_fifo(v_wr).data := rx_shift_q;
              v_fifo(v_wr).fe   := v_stop_err;
              v_fifo(v_wr).dor  := v_dor_pending;
              v_fifo(v_wr).pe   := v_pe_err;

              if v_wr = 0 then
                v_wr := 1;
              else
                v_wr := 0;
              end if;

              v_cnt := v_cnt + 1;

              v_dor_pending := '0';
            else
              if v_shift_full = '1' then
                v_dor_pending := '1';
              end if;

              rx_pending_data_q := rx_shift_q;
              rx_pending_fe_q   := v_stop_err;
              rx_pending_dor_q  := v_dor_pending;
              rx_pending_pe_q   := v_pe_err;

              v_shift_full  := '1';
              v_dor_pending := '0';
            end if;
          end if;
        end if;

        -----------------------------------------------------------------
        -- Asynchronous start-bit detection
        -- This occurs after completion handling so a just-completed
        -- frame can be treated as pending before a new start bit.
        -----------------------------------------------------------------
        if umsel_q = '0' then
          if v_rx_state = RX_IDLE and v_rxd = '0' and v_rx_prev = '1' then
            if v_shift_full = '1' then
              v_dor_pending := '1';
              v_shift_full  := '0';
            end if;

            rx_char_bits_q := v_char_bits;
            rx_parity_en_q := v_parity_en;
            rx_odd_q       := upm0_q;
            rx_mpcm_q      := mpcm_q;

            rx_shift_q      := (others => '0');
            rx_bit_cnt_q    := 0;
            rx_s0_q         := '0';
            rx_s1_q         := '0';
            rx_pe_latched_q := '0';

            v_rx_state        := RX_START;
            v_rx_counter_reset := true;
          end if;
        end if;

        -----------------------------------------------------------------
        -- Commit receiver local state
        -----------------------------------------------------------------
        rx_state_q       := v_rx_state;
        rx_shift_full_q  := v_shift_full;
        rx_dor_pending_q := v_dor_pending;

        if v_rx_counter_reset then
          baud_cnt_q      := 0;
          rx_sample_pos_q := 0;
        end if;

        -----------------------------------------------------------------
        -- Commit FIFO state and buffered status
        -----------------------------------------------------------------
        fifo_q     := v_fifo;
        fifo_wr_q  := v_wr;
        fifo_rd_q  := v_rd;
        fifo_cnt_q := v_cnt;

        if v_cnt > 0 then
          rxc_q      := '1';
          head_data_q := v_fifo(v_rd).data;
          head_fe_q   := v_fifo(v_rd).fe;
          head_dor_q  := v_fifo(v_rd).dor;
          head_pe_q   := v_fifo(v_rd).pe;
          rxb8_q      := v_fifo(v_rd).data(8);
        else
          rxc_q      := '0';
          head_data_q := (others => '0');
          head_fe_q   := '0';
          head_dor_q  := '0';
          head_pe_q   := '0';
          rxb8_q      := '0';
        end if;

      end if;

    end if;

    -------------------------------------------------------------------
    -- Drive outputs from variables
    -------------------------------------------------------------------
    tx_out  <= tx_out_q;
    xck_out <= xck_out_q;
    xck_oe  <= xck_oe_q;
    dout    <= dout_q;

    if txen_q = '1' or tx_buf_full_q = '1' or tx_state_q /= TX_IDLE then
      v_tx_oe := '1';
    else
      v_tx_oe := '0';
    end if;

    tx_oe <= v_tx_oe;

    rx_irq   <= rxc_q and rxcie_q;
    tx_irq   <= txc_q and txcie_q;
    udre_irq <= udre_q and udrie_q;

  end process main_proc;

end architecture rtl;
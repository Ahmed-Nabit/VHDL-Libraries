-- fixed_mod_pipe.vhd - Pipelined Q32.32 modulo reduction (64-bit input, 32-bit output)
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
use ieee.numeric_std.all;
use work.fixed_pkg.all;          -- provides q32_32, has_x

entity fixed_mod_pipe is
    generic(
        MODULUS       : q32_32 := signed'(x"00000006487ED511")   -- 2π in Q32.32
    );
    port(
        clk          : in  std_logic;
        rst          : in  std_logic;                     -- synchronous reset
        start        : in  std_logic;                     -- start new operation
        busy         : out std_logic;                      -- high while operating
        done         : out std_logic;                      -- high until next start
        angle_in     : in  q32_32;                         -- 64-bit input (Q32.32)
        angle_out    : out q32_32;                         -- wrapped to [0, MODULUS) Q32.32
        modulo_q     : out q32_32;                         -- modulus value (constant)
        overflow     : out std_logic;                      -- set if result invalid
        timeout      : out std_logic;                      -- set if divider timed out
        abort        : in  std_logic                       -- synchronous abort
    );
end entity fixed_mod_pipe;

architecture rtl of fixed_mod_pipe is

    constant TIMEOUT_CYCLES : positive := 500;   -- max cycles for divider

    -- Component declaration for fixed_div (external)
    component fixed_div is
        port(
            clk       : in  std_logic;
            rst       : in  std_logic;
            start     : in  std_logic;
            busy      : out std_logic;
            done      : out std_logic;
            dividend  : in  q32_32;
            divisor   : in  q32_32;
            quotient  : out q32_32;
            div_zero  : out std_logic;
            overflow  : out std_logic;
            timeout   : out std_logic;
            abort     : in  std_logic
        );
    end component;

    -- State machine
    type state_t is (IDLE, WAIT_DIV, WAIT_MUL, FINISH);
    signal state_reg : state_t := IDLE;

    -- Divider interface signals
    signal div_start      : std_logic;
    signal div_busy       : std_logic;
    signal div_done       : std_logic;
    signal div_quotient   : q32_32;
    signal div_zero       : std_logic;
    signal div_overflow   : std_logic;
    signal div_timeout    : std_logic;
    signal internal_abort : std_logic;                     -- internal abort to divider

    -- Internal registers
    signal angle_reg      : q32_32;
    signal quotient_reg   : q32_32;                         -- divider result (Q32.32)
    signal product_full   : signed(127 downto 0);           -- 128-bit product (integer * MODULUS)
    signal product_shifted: q32_32;                         -- product after shift (Q32.32)
    signal remainder_reg  : q32_32;                          -- final modulo result
    signal overflow_reg   : std_logic;
    signal timeout_reg    : std_logic;
    signal done_reg       : std_logic;
    signal busy_reg       : std_logic;

    -- Timeout counter
    signal timeout_cnt    : integer range 0 to TIMEOUT_CYCLES;

begin

    -- Instantiate the fixed_div divider with internal abort ORed with external abort
    u_div : fixed_div
        port map(
            clk       => clk,
            rst       => rst,
            start     => div_start,
            busy      => div_busy,
            done      => div_done,
            dividend  => angle_reg,
            divisor   => MODULUS,
            quotient  => div_quotient,
            div_zero  => div_zero,
            overflow  => div_overflow,
            timeout   => div_timeout,
            abort     => abort or internal_abort
        );

    -- Output assignments
    busy      <= busy_reg;
    done      <= done_reg;
    angle_out <= remainder_reg;
    modulo_q  <= MODULUS;
    overflow  <= overflow_reg;
    timeout   <= timeout_reg;

    --------------------------------------------------------------------------
    -- Main control and datapath (clocked process)
    --------------------------------------------------------------------------
    process(clk)
        variable v_state        : state_t;
        variable v_div_start    : std_logic;
        variable v_remainder    : signed(63 downto 0);
        variable v_overflow     : std_logic;
        variable v_timeout      : std_logic;
        variable v_done         : std_logic;
        variable v_busy         : std_logic;
        variable v_internal_abort: std_logic;
        variable v_quotient_int : signed(63 downto 0);      -- integer part as Q32.32 (frac=0)
        variable v_product_full : signed(127 downto 0);
        variable v_shifted_96   : signed(95 downto 0);
    begin
        if rising_edge(clk) then
            if rst = '1' or abort = '1' then
                state_reg      <= IDLE;
                angle_reg      <= (others => '0');
                quotient_reg   <= (others => '0');
                product_full   <= (others => '0');
                product_shifted<= (others => '0');
                remainder_reg  <= (others => '0');
                overflow_reg   <= '0';
                timeout_reg    <= '0';
                done_reg       <= '0';
                busy_reg       <= '0';
                div_start      <= '0';
                internal_abort <= '0';
                timeout_cnt    <= 0;
            else
                -- Default assignments
                v_div_start     := '0';
                v_overflow      := overflow_reg;
                v_timeout       := timeout_reg;
                v_done          := '0';
                v_busy          := '1';
                v_remainder     := remainder_reg;
                v_internal_abort:= internal_abort;

                ------------------------------------------------------------------
                -- Timeout counter (only active in WAIT_DIV)
                ------------------------------------------------------------------
                if state_reg = WAIT_DIV and div_busy = '1' and div_done = '0' then
                    if timeout_cnt < TIMEOUT_CYCLES then
                        timeout_cnt <= timeout_cnt + 1;
                    end if;
                else
                    timeout_cnt <= 0;
                end if;

                ------------------------------------------------------------------
                -- State machine
                ------------------------------------------------------------------
                v_state := state_reg;

                case state_reg is
                    when IDLE =>
                        v_internal_abort := '0';          -- clear abort flag
                        if start = '1' and not has_x(angle_in) then
                            v_done          := '0';
                            v_overflow      := '0';
                            v_timeout       := '0';
                            v_div_start     := '1';
                            angle_reg       <= angle_in;
                            v_state         := WAIT_DIV;
                        else
                            v_busy := '0';
                        end if;

                    when WAIT_DIV =>
                        if div_done = '1' then
                            -- Divider finished normally
                            if div_overflow = '1' or div_zero = '1' then
                                v_overflow := '1';
                                v_remainder := (others => '0');
                                v_state := FINISH;
                            else
                                quotient_reg <= div_quotient;  -- store quotient
                                -- Extract integer part (bits 63..32) into Q32.32 with zero fraction
                                v_quotient_int := (others => '0');
                                v_quotient_int(63 downto 32) := div_quotient(63 downto 32);
                                -- Compute full 128-bit product (integer * MODULUS)
                                v_product_full := v_quotient_int * MODULUS;
                                product_full <= v_product_full;
                                v_state := WAIT_MUL;
                            end if;
                            if div_timeout = '1' then
                                v_timeout := '1';
                            end if;
                        elsif timeout_cnt = TIMEOUT_CYCLES then
                            -- Timeout: abort divider and finish
                            v_overflow := '1';
                            v_timeout := '1';
                            v_internal_abort := '1';
                            v_remainder := (others => '0');
                            v_state := FINISH;
                        end if;

                    when WAIT_MUL =>
                        -- Shift product right by 32 to obtain Q32.32
                        v_shifted_96 := product_full(127 downto 32);   -- 96 bits
                        -- Check overflow in the top 32 bits of the shifted value
                        if (v_shifted_96(95) = '0' and v_shifted_96(95 downto 64) /= x"00000000") or
                           (v_shifted_96(95) = '1' and v_shifted_96(95 downto 64) /= x"FFFFFFFF") then
                            v_overflow := '1';
                            product_shifted <= (others => '0');   -- saturate to zero (or could set max)
                        else
                            product_shifted <= v_shifted_96(63 downto 0);
                        end if;
                        v_state := FINISH;

                    when FINISH =>
                        if v_overflow = '1' or v_timeout = '1' then
                            v_remainder := (others => '0');
                        else
                            v_remainder := angle_reg - product_shifted;
                            if v_remainder < 0 then
                                v_remainder := v_remainder + MODULUS;
                            end if;
                            -- Final check: remainder must be in [0, MODULUS)
                            if v_remainder < 0 or v_remainder >= MODULUS then
                                v_overflow := '1';
                                v_remainder := (others => '0');
                            end if;
                        end if;
                        v_done  := '1';
                        v_busy  := '0';
                        v_state := IDLE;

                    when others =>
                        v_state := IDLE;

                end case;

                -- Update registers
                state_reg     <= v_state;
                div_start     <= v_div_start;
                remainder_reg <= v_remainder;
                overflow_reg  <= v_overflow;
                timeout_reg   <= v_timeout;
                done_reg      <= v_done;
                busy_reg      <= v_busy;
                internal_abort<= v_internal_abort;
            end if;
        end if;
    end process;

end architecture rtl;
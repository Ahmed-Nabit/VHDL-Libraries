-- fixed_mul_64x64.vhd - 4-stage pipelined Q32.32 x Q32.32 multiplier with overflow detection and saturation
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
use work.fixed_pkg.all;

entity fixed_mul_64x64 is
    port(
        clk       : in  std_logic;
        rst       : in  std_logic;
        start     : in  std_logic;
        busy      : out std_logic;
        done      : out std_logic;
        a         : in  q32_32;
        b         : in  q32_32;
        result    : out q32_32;
        overflow  : out std_logic;
        timeout   : out std_logic;
        abort     : in  std_logic
    );
end entity fixed_mul_64x64;

architecture rtl of fixed_mul_64x64 is

    constant PIPE_DEPTH : integer := 4;   -- 4 pipeline stages
    constant ROUND_VAL  : signed(127 downto 0) := shift_left(to_signed(1, 128), 31);  -- 2^31

    type pipe_stage_t is record
        valid     : std_logic;
        product   : signed(127 downto 0);
        overflow  : std_logic;
    end record;
    
    type pipe_array_t is array(0 to PIPE_DEPTH-1) of pipe_stage_t;
    
    constant PIPE_DEFAULT : pipe_stage_t := (
        valid    => '0',
        product  => (others => '0'),
        overflow => '0'
    );
    
    signal pipe : pipe_array_t := (others => PIPE_DEFAULT);

    signal busy_r     : std_logic := '0';
    signal done_r     : std_logic := '0';
    signal result_r   : q32_32 := (others => '0');
    signal overflow_r : std_logic := '0';

begin

    timeout <= '0';   -- not used

    busy    <= busy_r;
    done    <= done_r;
    result  <= result_r;
    overflow <= overflow_r;

    process(clk)
        variable pipe_v      : pipe_array_t;
        variable busy_v      : std_logic;
        variable done_v      : std_logic;
        variable result_v    : q32_32;
        variable overflow_v  : std_logic;
        variable product_full: signed(127 downto 0);
        variable product_round: signed(127 downto 0);
        -- Constants for comparison (avoid aggregates in comparisons)
        constant ZERO_33  : signed(32 downto 0) := (others => '0');
        constant ONES_33  : signed(32 downto 0) := (others => '1');
    begin
        if rising_edge(clk) then
            if rst = '1' or abort = '1' then
                pipe      <= (others => PIPE_DEFAULT);
                busy_r    <= '0';
                done_r    <= '0';
                result_r  <= (others => '0');
                overflow_r<= '0';
            else
                -- Default control signals from registered values
                busy_v    := busy_r;
                done_v    := done_r;
                result_v  := result_r;
                overflow_v:= overflow_r;
                done_v    := '0';  -- will be set to '1' if output is valid this cycle
                ------------------------------------------------------------------
                -- 1. Initialize next pipeline state to all invalid (bubbles)
                ------------------------------------------------------------------
                for i in 0 to PIPE_DEPTH-1 loop
                    pipe_v(i) := PIPE_DEFAULT;
                end loop;

                ------------------------------------------------------------------
                -- 2. Shift existing valid stages forward (i → i+1)
                ------------------------------------------------------------------
                -- Stage 0 → Stage 1 (Rounding)
                if pipe(0).valid = '1' then
                    if pipe(0).overflow = '1' then
                        pipe_v(1) := (valid => '1', overflow => '1', product => (others => '0'));
                    else
                        if has_x(pipe(0).product) then
                            pipe_v(1) := (valid => '1', overflow => '1', product => (others => '0'));
                        else
                            -- Correct rounding: add for positive, subtract for negative
                            if pipe(0).product(127) = '0' then
                                product_round := pipe(0).product + ROUND_VAL;
                            else
                                product_round := pipe(0).product - ROUND_VAL;
                            end if;
                            pipe_v(1) := (valid => '1', overflow => '0', product => product_round);
                        end if;
                    end if;
                end if;

                -- Stage 1 → Stage 2 (Overflow detection) - FIXED RANGE and COMPARISON
                if pipe(1).valid = '1' then
                    if pipe(1).overflow = '1' then
                        pipe_v(2) := (valid => '1', overflow => '1', product => (others => '0'));
                    else
                        if has_x(pipe(1).product) then
                            pipe_v(2) := (valid => '1', overflow => '1', product => (others => '0'));
                        else
                            -- Determine overflow based on sign: check bits 127:95 (33 bits)
                            if pipe(1).product(127) = '0' then
                                -- Positive: bits 127:95 must be all 0
                                -- Use slice comparison instead of aggregate to avoid warnings
                                if pipe(1).product(127 downto 95) /= ZERO_33 then
                                    pipe_v(2).overflow := '1';
                                else
                                    pipe_v(2).overflow := '0';
                                end if;
                            else
                                -- Negative: bits 127:95 must be all 1
                                if pipe(1).product(127 downto 95) /= ONES_33 then
                                    pipe_v(2).overflow := '1';
                                else
                                    pipe_v(2).overflow := '0';
                                end if;
                            end if;
                            pipe_v(2).valid   := '1';
                            pipe_v(2).product := pipe(1).product;
                        end if;
                    end if;
                end if;

                -- Stage 2 → Stage 3 (Shift to 64‑bit)
                if pipe(2).valid = '1' then
                    if pipe(2).overflow = '1' then
                        pipe_v(3).valid    := '1';
                        pipe_v(3).overflow := '1';
                        -- Zero‑fill upper 64 bits, keep shifted product in lower 64
                        -- Use concatenation instead of aggregate to avoid warnings
                        pipe_v(3).product(127 downto 64) := (others => '0');
                        pipe_v(3).product(63 downto 0)   := pipe(2).product(95 downto 32);
                    else
                        if has_x(pipe(2).product) then
                            pipe_v(3) := (valid => '1', overflow => '1', product => (others => '0'));
                        else
                            pipe_v(3).valid    := '1';
                            pipe_v(3).overflow := pipe(2).overflow;
                            -- Zero‑fill upper 64 bits, keep shifted product in lower 64
                            pipe_v(3).product(127 downto 64) := (others => '0');
                            pipe_v(3).product(63 downto 0)   := pipe(2).product(95 downto 32);
                        end if;
                    end if;
                end if;

                ------------------------------------------------------------------
                -- 3. New input (overwrites stage 0) - FIXED HANDSHAKE (use busy_r)
                ------------------------------------------------------------------
                if start = '1' then
                    if has_x(a) or has_x(b) then
                        pipe_v(0) := (valid => '1', overflow => '1', product => (others => '0'));
                    else
                        product_full := a * b;
                        pipe_v(0) := (valid => '1', overflow => '0', product => product_full);
                    end if;
                    done_v := '0';
                end if;

                ------------------------------------------------------------------
                -- 4. Output stage (uses registered pipe(3))
                ------------------------------------------------------------------
                if pipe(3).valid = '1' then
                    if pipe(3).overflow = '1' then
                        overflow_v := '1';
                        -- Saturation based on sign of the (shifted) product
                        if pipe(3).product(63) = '0' then
                            result_v := x"7FFFFFFFFFFFFFFF";  -- Max positive
                        else
                            result_v := x"8000000000000000";  -- Max negative
                        end if;
                    else
                        overflow_v := '0';
                        result_v   := pipe(3).product(63 downto 0);
                    end if;
                    done_v := '1';
                else
                    done_v := '0';
                end if;

                ------------------------------------------------------------------
                -- 5. Compute busy flag from next pipeline state
                ------------------------------------------------------------------
                busy_v := '0';
                for i in 0 to PIPE_DEPTH-1 loop
                    if pipe_v(i).valid = '1' then
                        busy_v := '1';
                        exit;
                    end if;
                end loop;

                ------------------------------------------------------------------
                -- 6. Update registers
                ------------------------------------------------------------------
                for i in 0 to PIPE_DEPTH-1 loop
                    pipe(i) <= pipe_v(i);
                end loop;
                busy_r     <= busy_v;
                done_r     <= done_v;
                result_r   <= result_v;
                overflow_r <= overflow_v;

            end if;
        end if;
    end process;

end architecture rtl;
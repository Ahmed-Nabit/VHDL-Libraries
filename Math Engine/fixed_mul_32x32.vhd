-- fixed_mul_32x32.vhd - 3-stage pipelined Q16.16 x Q16.16 multiplier with overflow saturation
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
use work.fixed_pkg.all;                 -- provides q16_16 type

entity fixed_mul_32x32 is
    port(
        clk       : in  std_logic;
        rst       : in  std_logic;
        start     : in  std_logic;
        busy      : out std_logic;
        done      : out std_logic;
        a         : in  q16_16;
        b         : in  q16_16;
        result    : out q16_16;
        overflow  : out std_logic;
        timeout   : out std_logic;       -- tied low in this implementation
        abort     : in  std_logic
    );
end entity fixed_mul_32x32;

architecture rtl of fixed_mul_32x32 is

    constant PIPE_DEPTH : integer := 3;              -- actual pipeline stages
    -- rounding constant = 2^15 (half of the LSB of the final fractional part)
    constant ROUND_VAL  : signed(63 downto 0) := to_signed(32768, 64);

    type pipe_stage_t is record
        valid     : std_logic;
        product   : signed(63 downto 0);
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
    signal result_r   : q16_16 := (others => '0');
    signal overflow_r : std_logic := '0';

    -- Helper: detect 'X' or 'U' in a signed vector (simulation only)
    -- Synthesis tools will ignore this as the condition is always false.
    function has_x(s : signed) return boolean is
    begin
        for i in s'range loop
            if s(i) = 'X' or s(i) = 'U' then
                return true;
            end if;
        end loop;
        return false;
    end function;

begin

    timeout <= '0';                       -- not used in this basic version

    busy    <= busy_r;
    done    <= done_r;
    result  <= result_r;
    overflow <= overflow_r;

    process(clk)
        variable new_pipe      : pipe_array_t;
        variable product_round : signed(63 downto 0);
        variable overflow_tmp  : std_logic;
    begin
        if rising_edge(clk) then
            if rst = '1' or abort = '1' then
                pipe      <= (others => PIPE_DEFAULT);
                busy_r    <= '0';
                done_r    <= '0';
                result_r  <= (others => '0');
                overflow_r<= '0';
            else
                -- start with a completely fresh pipeline (all stages invalid)
                new_pipe := (others => PIPE_DEFAULT);

                ------------------------------------------------------------------
                -- Stage 2 (output) – uses old pipe(1)
                ------------------------------------------------------------------
                if pipe(1).valid = '1' then
                    if pipe(1).overflow = '1' then
                        overflow_r <= '1';
                        -- saturation using sign from original product
                        if pipe(1).product(63) = '0' then
                            result_r <= x"7FFFFFFF";   -- max positive
                        else
                            result_r <= x"80000000";   -- max negative
                        end if;
                    else
                        overflow_r <= '0';
                        result_r <= pipe(1).product(47 downto 16);   -- Q16.16
                    end if;
                    done_r <= '1';
                    busy_r <= '0';                      -- operation finished
                else
                    done_r <= '0';
                    overflow_r <= '0';                   -- clear stale overflow
                    -- busy_r is not changed here; may be updated by start later
                end if;

                ------------------------------------------------------------------
                -- Stage 1 (rounding & overflow) – from old pipe(0) to new_pipe(1)
                ------------------------------------------------------------------
                if pipe(0).valid = '1' then
                    if pipe(0).overflow = '1' then
                        -- preserve sign for correct saturation in stage 2
                        new_pipe(1) := (valid    => '1',
                                        overflow => '1',
                                        product  => (63 => pipe(0).product(63),
                                                     others => '0'));
                    else
                        if has_x(pipe(0).product) then
                            new_pipe(1) := (valid    => '1',
                                            overflow => '1',
                                            product  => (others => '0'));
                        else
                            -- rounding (add half of LSB, with sign adjustment)
                            if pipe(0).product(63) = '0' then
                                product_round := pipe(0).product + ROUND_VAL;
                            else
                                product_round := pipe(0).product - ROUND_VAL;
                            end if;

                            -- overflow detection after rounding
                            if product_round(63) = '0' then
                                -- positive: bits 63..48 must be zero
                                overflow_tmp := '0' when product_round(63 downto 48) = 0 else '1';
                            else
                                -- negative: bits 63..48 must be all ones
                                overflow_tmp := '0' when product_round(63 downto 48) = x"FFFF" else '1';
                            end if;

                            new_pipe(1) := (valid    => '1',
                                            overflow => overflow_tmp,
                                            product  => product_round);
                        end if;
                    end if;
                end if;

                ------------------------------------------------------------------
                -- Stage 0 (input acceptance) – from external inputs to new_pipe(0)
                ------------------------------------------------------------------
                if start = '1' and busy_r = '0' then
                    if has_x(a) or has_x(b) then
                        new_pipe(0) := (valid    => '1',
                                        overflow => '1',
                                        product  => (others => '0'));
                    else
                        new_pipe(0) := (valid    => '1',
                                        overflow => '0',
                                        product  => a * b);
                    end if;
                    busy_r <= '1';          -- unit now busy
                end if;

                ------------------------------------------------------------------
                -- Update pipeline registers
                ------------------------------------------------------------------
                pipe <= new_pipe;

            end if; -- rst/abort
        end if; -- rising_edge
    end process;

end architecture rtl;
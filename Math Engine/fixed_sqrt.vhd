-- fixed_sqrt.vhd - 49-stage pipelined radix-2 restoring square root for Q32.32 fixed-point
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
use work.fixed_pkg.all;          -- provides q32_32 type (signed)

entity fixed_sqrt is
    port(
        clk      : in  std_logic;
        rst      : in  std_logic;
        start    : in  std_logic;
        busy     : out std_logic;
        done     : out std_logic;
        radicand : in  q32_32;
        result   : out q32_32;
        invalid  : out std_logic;
        timeout  : out std_logic;   -- stub (not implemented)
        abort    : in  std_logic
    );
end entity;

architecture rtl of fixed_sqrt is

    constant ITERATIONS : integer := 48;               -- 48 iterations → 48‑bit internal root
    constant PIPE_DEPTH : integer := ITERATIONS + 1;   -- 49 stages: input + 48 iter + output

    -- Pipeline stage record
    type pipe_stage_t is record
        valid      : std_logic;
        x          : unsigned(95 downto 0);   -- working radicand (scaled, 96 bits)
        q          : unsigned(47 downto 0);   -- quotient (48 bits, internal root)
        remi       : unsigned(97 downto 0);   -- remainder (98 bits)
        invalid    : std_logic;
        norm_shift : unsigned(5 downto 0);    -- right shift for denormalization (0..31)
    end record;

    type pipe_array_t is array(0 to PIPE_DEPTH-1) of pipe_stage_t;

    constant PIPE_DEFAULT : pipe_stage_t := (
        valid      => '0',
        x          => (others => '0'),
        q          => (others => '0'),
        remi       => (others => '0'),
        invalid    => '0',
        norm_shift => (others => '0')
    );

    signal pipe      : pipe_array_t := (others => PIPE_DEFAULT);
    signal busy_r    : std_logic := '0';
    signal done_r    : std_logic := '0';
    signal result_r  : q32_32 := (others => '0');
    signal invalid_r : std_logic := '0';

    -- Abort synchronizer (two‑flop)
    signal abort_meta : std_logic := '0';
    signal abort_sync : std_logic := '0';

    -- Constants
    constant ZERO_Q_LOCAL : q32_32 := (others => '0');

    -- -------------------------------------------------------------------------
    -- Simulation‑only function to detect unknown bits.
    -- In synthesis the function reduces to "return false;" due to translate_off.
    -- -------------------------------------------------------------------------
    function has_x(s : std_logic_vector) return boolean is
    begin
        -- synthesis translate_off
        for i in s'range loop
            if s(i) = 'X' or s(i) = 'U' then
                return true;
            end if;
        end loop;
        -- synthesis translate_on
        return false;
    end function;

    -- -------------------------------------------------------------------------
    -- Count leading zeros in a 64‑bit unsigned number.
    -- Returns 64 for all zeros.
    -- -------------------------------------------------------------------------
    function count_leading_zeros(v : unsigned(63 downto 0)) return integer is
    begin
        for i in 63 downto 0 loop
            if v(i) = '1' then
                return 63 - i;
            end if;
        end loop;
        return 64;  -- all zeros
    end function;

begin

    timeout <= '0';   -- stub (kept as requested)

    busy    <= busy_r;
    done    <= done_r;
    result  <= result_r;
    invalid <= invalid_r;

    process(clk)
        variable pipe_next    : pipe_array_t;
        variable busy_v       : std_logic;
        variable done_v       : std_logic;
        variable result_v     : q32_32;
        variable invalid_v    : std_logic;

        -- Working variables
        variable rad_abs      : unsigned(63 downto 0) := (others => '0');
        variable lz           : integer range 0 to 64;
        variable lz_even      : integer range 0 to 64;
        variable norm_shift   : unsigned(5 downto 0);
        variable total_shift  : integer range 0 to 96;
        variable x_scaled     : unsigned(95 downto 0) := (others => '0');
        variable next_bits    : unsigned(1 downto 0)  := (others => '0');
        variable trial_remi   : unsigned(99 downto 0) := (others => '0');
        variable q_shifted    : unsigned(49 downto 0) := (others => '0');   -- q << 2 (50 bits)
        variable candidate    : unsigned(99 downto 0) := (others => '0');
        variable diff         : unsigned(99 downto 0) := (others => '0');
        variable new_q        : unsigned(47 downto 0) := (others => '0');
        variable new_remi     : unsigned(97 downto 0) := (others => '0');
        variable new_x        : unsigned(95 downto 0) := (others => '0');

        -- Output denormalization
        variable q_val        : unsigned(47 downto 0);
        variable shift_amt    : integer range 0 to 31;
        variable q_denorm     : unsigned(63 downto 0);

    begin
        if rising_edge(clk) then
            -- -----------------------------------------------------------------
            -- Synchronize abort (two‑flop synchronizer)
            -- -----------------------------------------------------------------
            abort_meta <= abort;
            abort_sync <= abort_meta;

            if rst = '1' or abort_sync = '1' then
                -- Reset pipeline
                for i in 0 to PIPE_DEPTH-1 loop
                    pipe(i) <= PIPE_DEFAULT;
                end loop;
                abort_meta <= '0';
                abort_sync <= '0';
                busy_r    <= '0';
                done_r    <= '0';
                result_r  <= (others => '0');
                invalid_r <= '0';
            else
                -- -----------------------------------------------------------------
                -- Initialise next pipeline state (all stages invalid)
                -- -----------------------------------------------------------------
                for i in 0 to PIPE_DEPTH-1 loop
                    pipe_next(i) := PIPE_DEFAULT;
                end loop;

                -- Default outputs
                done_v    := '0';
                invalid_v := invalid_r;
                result_v  := result_r;

                -- -----------------------------------------------------------------
                -- Propagate existing pipeline stages (iterations)
                -- -----------------------------------------------------------------
                for i in 0 to ITERATIONS-1 loop
                    if pipe(i).valid = '1' then
                        if pipe(i).invalid = '0' then
                            -- Normal iteration: compute one new bit
                            next_bits := pipe(i).x(95 downto 94);   -- two bits of radicand

                            -- trial_remi = (remi << 2) + next_bits
                            trial_remi := shift_left(resize(pipe(i).remi, 100), 2)
                                         + resize(next_bits, 100);

                            q_shifted := pipe(i).q & "00";   -- q << 2 (for candidate)

                            -- Radix-2 digit selection: test d=1, then d=0
                            candidate := resize(q_shifted, 100) + to_unsigned(1, 100);  -- candidate = q<<2 + 1
                            if trial_remi >= candidate then
                                -- d=1
                                new_q := shift_left(pipe(i).q, 1);
                                new_q(0) := '1';                    -- q << 1 + 1
                                diff  := trial_remi - candidate;
                                new_remi := diff(97 downto 0);
                            else
                                -- d=0
                                new_q := shift_left(pipe(i).q, 1); -- q << 1
                                diff  := trial_remi;
                                new_remi := diff(97 downto 0);
                            end if;

                            new_x := shift_left(pipe(i).x, 2);    -- consume two bits of radicand

                            pipe_next(i+1).valid   := '1';
                            pipe_next(i+1).invalid := '0';
                            pipe_next(i+1).q       := new_q;
                            pipe_next(i+1).remi    := new_remi;
                            pipe_next(i+1).x       := new_x;
                            pipe_next(i+1).norm_shift := pipe(i).norm_shift;  -- propagate
                        else
                            -- Invalid input: propagate invalid flag
                            pipe_next(i+1).valid   := '1';
                            pipe_next(i+1).invalid := '1';
                            pipe_next(i+1).x       := pipe(i).x;
                            pipe_next(i+1).q       := pipe(i).q;
                            pipe_next(i+1).remi    := pipe(i).remi;
                            pipe_next(i+1).norm_shift := pipe(i).norm_shift;
                        end if;
                    end if;
                end loop;

                -- -----------------------------------------------------------------
                -- Output stage (source = ITERATIONS)
                -- Denormalize: result = q >> (norm_shift)
                -- -----------------------------------------------------------------
                if pipe(ITERATIONS).valid = '1' then
                    if pipe(ITERATIONS).invalid = '1' then
                        result_v  := (others => '0');
                        invalid_v := '1';
                    else
                        q_val := pipe(ITERATIONS).q;
                        shift_amt := to_integer(pipe(ITERATIONS).norm_shift);
                        q_denorm := resize(q_val srl shift_amt, 64);
                        result_v := signed(q_denorm);
                        invalid_v := '0';
                    end if;
                    done_v := '1';
                end if;

                -- -----------------------------------------------------------------
                -- Stage 0 : Input acceptance with dynamic normalization
                -- -----------------------------------------------------------------
                if start = '1' and not has_x(std_logic_vector(radicand)) then
                    if radicand < ZERO_Q_LOCAL then
                        -- Negative input -> invalid
                        pipe_next(0).valid   := '1';
                        pipe_next(0).invalid := '1';
                        pipe_next(0).norm_shift := (others => '0');
                    elsif radicand = ZERO_Q_LOCAL then
                        -- Zero input -> result zero
                        pipe_next(0).valid   := '1';
                        pipe_next(0).invalid := '0';
                        pipe_next(0).x       := (others => '0');
                        pipe_next(0).q       := (others => '0');
                        pipe_next(0).remi    := (others => '0');
                        pipe_next(0).norm_shift := (others => '0');
                    else
                        -- Positive input: normalize
                        rad_abs := unsigned(radicand);                -- safe because radicand > 0
                        lz := count_leading_zeros(rad_abs);

                        -- Make the total shift (lz + 32) even to allow integer denormalization
                        if lz mod 2 = 0 then
                            lz_even := lz;
                        else
                            lz_even := lz - 1;
                        end if;

                        norm_shift := to_unsigned(lz_even / 2, 6);    -- right shift after sqrt
                        total_shift := lz_even + 32;                  -- left shift before algorithm

                        -- Shift radicand left by total_shift and place into 96‑bit register
                        x_scaled := shift_left(resize(rad_abs, 96), total_shift);

                        pipe_next(0).valid   := '1';
                        pipe_next(0).invalid := '0';
                        pipe_next(0).x       := x_scaled;
                        pipe_next(0).norm_shift := norm_shift;
                        -- q and remi default to zero
                    end if;
                end if;

                -- -----------------------------------------------------------------
                -- Compute final busy flag from the complete next pipeline state
                -- -----------------------------------------------------------------
                busy_v := '0';
                for i in 0 to PIPE_DEPTH-1 loop
                    if pipe_next(i).valid = '1' then
                        busy_v := '1';
                        exit;
                    end if;
                end loop;

                -- -----------------------------------------------------------------
                -- Update registers
                -- -----------------------------------------------------------------
                pipe      <= pipe_next;
                busy_r    <= busy_v;
                done_r    <= done_v;
                result_r  <= result_v;
                invalid_r <= invalid_v;

            end if;
        end if;
    end process;

end architecture;
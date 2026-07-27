-- fixed_div.vhd - 97-stage pipelined Q32.32 signed divider with rounding, saturation and abort
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

entity fixed_div is
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
        timeout   : out std_logic;                 -- tied to '0' (placeholder)
        abort     : in  std_logic
    );
end entity fixed_div;

architecture rtl of fixed_div is

    -- Constants for Q32.32 range
    constant MAX_POS_MAG : unsigned(63 downto 0) := x"7FFFFFFF_FFFFFFFF";  --  2^63 - 1
    constant MAX_NEG_MAG : unsigned(63 downto 0) := x"80000000_00000000";  --  2^63
    constant MAX_POS     : q32_32   := x"7FFFFFFF_FFFFFFFF";
    constant MAX_NEG     : q32_32   := x"80000000_00000000";
    constant ZERO_64     : q32_32   := (others => '0');

    constant ITERATIONS : integer := 96;          -- number of quotient bits
    constant PIPE_DEPTH : integer := ITERATIONS + 1;  -- 97 stages (0..96)

    -- Pipeline stage record (iter field removed)
    type pipe_stage_t is record
        valid          : std_logic;                        -- stage contains valid data
        dividend_scaled : unsigned(95 downto 0);           -- |dividend| << 32 (shifts left each step)
        divisor_abs    : unsigned(63 downto 0);            -- |divisor|
        remainder      : unsigned(64 downto 0);            -- 65‑bit remainder (initial 0)
        quotient       : unsigned(95 downto 0);            -- 96‑bit raw quotient
        sign_neg       : std_logic;                         -- result sign (1 = negative)
        div_zero       : std_logic;                         -- division by zero detected
        bad_input      : std_logic;                         -- X or Z detected on inputs
    end record;

    type pipe_array_t is array(0 to PIPE_DEPTH-1) of pipe_stage_t;

    constant PIPE_DEFAULT : pipe_stage_t := (
        valid          => '0',
        dividend_scaled => (others => '0'),
        divisor_abs    => (others => '0'),
        remainder      => (others => '0'),
        quotient       => (others => '0'),
        sign_neg       => '0',
        div_zero       => '0',
        bad_input      => '0'
    );

    -- Pipeline registers
    signal pipe_r     : pipe_array_t := (others => PIPE_DEFAULT);
    signal pipe_next  : pipe_array_t;

    -- Control registers
    signal busy_r          : std_logic := '0';
    signal done_r          : std_logic := '0';
    signal quotient_r      : q32_32 := (others => '0');
    signal div_zero_r      : std_logic := '0';
    signal overflow_r      : std_logic := '0';
    signal start_accepted_r : std_logic := '0';    -- prevents double acceptance of start
    signal abort_pending_r : std_logic := '0';     -- abort has occurred, block new starts

    -- Abort synchroniser (two‑flop)
    signal abort_meta : std_logic := '0';
    signal abort_sync : std_logic := '0';

    -- Next state signals
    signal busy_next          : std_logic;
    signal done_next          : std_logic;
    signal quotient_next      : q32_32;
    signal div_zero_next      : std_logic;
    signal overflow_next      : std_logic;
    signal start_accepted_next : std_logic;
    signal abort_pending_next : std_logic;

    -- Helper functions
    function has_x(vec : std_logic_vector) return boolean is
    begin
        for i in vec'range loop
            if vec(i) = 'X' or vec(i) = 'U' or vec(i) = 'Z' or vec(i) = '-' then
                return true;
            end if;
        end loop;
        return false;
    end function;

    function abs_unsigned_64(a : signed(63 downto 0)) return unsigned is
        variable r : unsigned(63 downto 0);
    begin
        if a(63) = '1' then
            r := unsigned(-a);
        else
            r := unsigned(a);
        end if;
        return r;
    end function;

begin

    timeout <= '0';   -- stub (kept per original specification)

    busy    <= busy_r;
    done    <= done_r;
    quotient <= quotient_r;
    div_zero <= div_zero_r;
    overflow <= overflow_r;

    -- =========================================================================
    -- Combinatorial pipeline and control logic
    -- =========================================================================
    process(pipe_r, start, dividend, divisor, busy_r, done_r, quotient_r,
            div_zero_r, overflow_r, abort_sync, abort_pending_r, start_accepted_r)
        variable pipe_v          : pipe_array_t;
        variable busy_v          : std_logic;
        variable done_v          : std_logic;
        variable quotient_v      : q32_32;
        variable div_zero_v      : std_logic;
        variable overflow_v      : std_logic;
        variable start_accepted_v : std_logic;
        variable abort_pending_v : std_logic;
        variable start_processed : std_logic;
        variable abort_rising    : std_logic;
        variable abort_active    : std_logic;

        -- Division step variables
        variable temp_rem        : unsigned(65 downto 0);   -- 66‑bit shifted remainder + new bit
        variable new_rem         : unsigned(65 downto 0);   -- 66‑bit after possible subtraction
        variable quotient_bit    : std_logic;
        variable divisor_66      : unsigned(65 downto 0);   -- divisor extended to 66 bits

        -- Final stage variables
        variable q96             : unsigned(95 downto 0);
        variable q64             : unsigned(63 downto 0);
        variable round_up        : std_logic;

    begin
        -- Default assignments (hold current values)
        busy_v          := busy_r;
        done_v          := '0';   -- default to not done, set to '1' in final stage when appropriate    
        quotient_v      := quotient_r;
        div_zero_v      := div_zero_r;
        overflow_v      := overflow_r;
        start_accepted_v := start_accepted_r;
        abort_pending_v := abort_pending_r;

        -- Default pipeline: all bubbles
        for i in 0 to PIPE_DEPTH-1 loop
            pipe_v(i) := PIPE_DEFAULT;
        end loop;

        ------------------------------------------------------------------------
        -- Abort detection (rising edge of synchronised abort)
        ------------------------------------------------------------------------
        abort_rising := abort_sync and not abort_pending_r;
        if abort_rising = '1' then
            -- Abort current operation: clear pipeline, set overflow, zero quotient
            busy_v          := '0';
            done_v          := '0';
            start_accepted_v := '0';
            abort_pending_v := '1';
            overflow_v      := '1';
            quotient_v      := (others => '0');   -- clear stale data
        end if;

        abort_active := abort_sync or abort_pending_v;

        ------------------------------------------------------------------------
        -- Stage 0: Input acceptance (only if no abort pending)
        ------------------------------------------------------------------------
        start_processed := '0';
        if abort_active = '0' then
            if start = '1' and busy_v = '0' and start_accepted_v = '0' then
                start_accepted_v := '1';
                start_processed := '1';
                pipe_v(0).valid := '1';

                -- Input validation
                if has_x(std_logic_vector(dividend)) or has_x(std_logic_vector(divisor)) then
                    pipe_v(0).bad_input := '1';
                elsif divisor = ZERO_64 then
                    pipe_v(0).div_zero := '1';
                else
                    pipe_v(0).dividend_scaled := abs_unsigned_64(dividend) & x"00000000";
                    pipe_v(0).divisor_abs     := abs_unsigned_64(divisor);
                    pipe_v(0).sign_neg        := dividend(63) xor divisor(63);
                    -- remainder and quotient already zero from PIPE_DEFAULT
                end if;
                done_v := '0';
            end if;
        end if;

        ------------------------------------------------------------------------
        -- Pipeline stages 0 .. ITERATIONS-1 (division steps)
        ------------------------------------------------------------------------
        if abort_active = '0' then
            for i in 0 to ITERATIONS-1 loop
                if pipe_r(i).valid = '1' then
                    -- If any error flag is set, propagate it without computation
                    if pipe_r(i).div_zero = '1' or pipe_r(i).bad_input = '1' then
                        pipe_v(i+1).valid      := '1';
                        pipe_v(i+1).div_zero   := pipe_r(i).div_zero;
                        pipe_v(i+1).bad_input  := pipe_r(i).bad_input;
                        pipe_v(i+1).sign_neg   := pipe_r(i).sign_neg;
                        pipe_v(i+1).divisor_abs := pipe_r(i).divisor_abs;
                    else
                        -- Restoring division step
                        temp_rem := pipe_r(i).remainder & pipe_r(i).dividend_scaled(95);
                        divisor_66 := "00" & pipe_r(i).divisor_abs;   -- extend to 66 bits

                        if temp_rem >= divisor_66 then
                            new_rem := temp_rem - divisor_66;
                            pipe_v(i+1).remainder := new_rem(64 downto 0);   -- lower 65 bits (MSB=0)
                            quotient_bit := '1';
                        else
                            pipe_v(i+1).remainder := temp_rem(64 downto 0);
                            quotient_bit := '0';
                        end if;

                        -- Shift dividend and update quotient
                        pipe_v(i+1).dividend_scaled := pipe_r(i).dividend_scaled(94 downto 0) & '0';
                        pipe_v(i+1).quotient := pipe_r(i).quotient(94 downto 0) & quotient_bit;

                        -- Propagate other fields
                        pipe_v(i+1).divisor_abs := pipe_r(i).divisor_abs;
                        pipe_v(i+1).sign_neg    := pipe_r(i).sign_neg;
                        pipe_v(i+1).valid       := '1';
                    end if;
                end if;
            end loop;
        end if;

        ------------------------------------------------------------------------
        -- Final stage (ITERATIONS) – Output generation
        ------------------------------------------------------------------------
        if abort_active = '0' then
            if pipe_r(ITERATIONS).valid = '1' then
                -- Error cases take precedence
                if pipe_r(ITERATIONS).bad_input = '1' then
                    quotient_v := (others => '0');
                    div_zero_v := '0';
                    overflow_v := '1';
                    done_v     := '1';               -- FIX #1: assert done
                elsif pipe_r(ITERATIONS).div_zero = '1' then
                    quotient_v := (others => '0');
                    div_zero_v := '1';
                    overflow_v := '0';
                    done_v     := '1';               -- FIX #1: assert done
                else
                    q96 := pipe_r(ITERATIONS).quotient;

                    -- Check if the high 32 bits of the raw quotient are non‑zero
                    if q96(95 downto 64) /= x"00000000" then
                        -- Magnitude too large, saturate
                        overflow_v := '1';
                        if pipe_r(ITERATIONS).sign_neg = '0' then
                            quotient_v := MAX_POS;
                        else
                            quotient_v := MAX_NEG;
                        end if;
                    else
                        q64 := q96(63 downto 0);   -- magnitude of result

                        -- Preliminary overflow check (before rounding)
                        if pipe_r(ITERATIONS).sign_neg = '0' then
                            if q64 > MAX_POS_MAG then
                                overflow_v := '1';
                                quotient_v := MAX_POS;
                            else
                                overflow_v := '0';
                            end if;
                        else
                            if q64 > MAX_NEG_MAG then
                                overflow_v := '1';
                                quotient_v := MAX_NEG;
                            else
                                overflow_v := '0';
                            end if;
                        end if;

                        -- Rounding (if no overflow yet)
                        if overflow_v = '0' then
                            -- Condition: (remainder << 1) >= divisor_abs
                            -- Use full remainder width for robustness (FIX #3)
                            round_up := '0';
                            if (pipe_r(ITERATIONS).remainder & '0') >= ("00" & pipe_r(ITERATIONS).divisor_abs) then
                                round_up := '1';
                                q64 := q64 + 1;
                            end if;

                            if round_up = '1' then
                                -- Re‑check overflow after rounding
                                if pipe_r(ITERATIONS).sign_neg = '0' then
                                    if q64 > MAX_POS_MAG then
                                        overflow_v := '1';
                                        quotient_v := MAX_POS;
                                    else
                                        quotient_v := signed(q64);
                                    end if;
                                else
                                    if q64 > MAX_NEG_MAG then
                                        overflow_v := '1';
                                        quotient_v := MAX_NEG;
                                    else
                                        quotient_v := -signed(q64);
                                    end if;
                                end if;
                            else
                                -- No rounding, apply sign
                                if pipe_r(ITERATIONS).sign_neg = '0' then
                                    quotient_v := signed(q64);
                                else
                                    quotient_v := -signed(q64);
                                end if;
                            end if;
                        end if;
                    end if;

                    div_zero_v := '0';
                    done_v := '1';
                end if;
            end if;
        end if;

        ------------------------------------------------------------------------
        -- Compute busy flag from next pipeline state (any valid stage)
        ------------------------------------------------------------------------
        busy_v := '0';
        for i in 0 to ITERATIONS loop
            if pipe_v(i).valid = '1' then
                busy_v := '1';
                exit;
            end if;
        end loop;

        ------------------------------------------------------------------------
        -- Clear acceptance and abort flags when appropriate
        ------------------------------------------------------------------------
        if start = '0' and start_processed = '0' then
            start_accepted_v := '0';
        end if;

        if abort_sync = '0' and busy_v = '0' then
            abort_pending_v := '0';
        end if;

        ------------------------------------------------------------------------
        -- Assign next state
        ------------------------------------------------------------------------
        pipe_next <= pipe_v;
        busy_next <= busy_v;
        done_next <= done_v;
        quotient_next <= quotient_v;
        div_zero_next <= div_zero_v;
        overflow_next <= overflow_v;
        start_accepted_next <= start_accepted_v;
        abort_pending_next <= abort_pending_v;

    end process;

    -- =========================================================================
    -- Registered pipeline with reset and abort synchronisation
    -- =========================================================================
    process(clk)
    begin
        if rising_edge(clk) then
            -- Abort synchroniser (two‑flop)
            abort_meta <= abort;
            abort_sync <= abort_meta;

            if rst = '1' then
                -- Reset all registers
                for i in 0 to PIPE_DEPTH-1 loop
                    pipe_r(i) <= PIPE_DEFAULT;
                end loop;
                busy_r          <= '0';
                done_r          <= '0';
                quotient_r      <= (others => '0');
                div_zero_r      <= '0';
                overflow_r      <= '0';
                start_accepted_r <= '0';
                abort_pending_r <= '0';
                -- Also reset synchroniser flops (optional but safe)
                abort_meta      <= '0';
                abort_sync      <= '0';
            else
                pipe_r          <= pipe_next;
                busy_r          <= busy_next;
                done_r          <= done_next;
                quotient_r      <= quotient_next;
                div_zero_r      <= div_zero_next;
                overflow_r      <= overflow_next;
                start_accepted_r <= start_accepted_next;
                abort_pending_r <= abort_pending_next;
            end if;
        end if;
    end process;

end architecture rtl;
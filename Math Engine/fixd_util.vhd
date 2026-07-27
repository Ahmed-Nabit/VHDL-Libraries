-- fixd_util.vhd - Fixed-point utility functions: unit conversions, angle limiters and inverse operations
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

entity fixd_util is
    port(
        clk         : in  std_logic;
        rst         : in  std_logic;
        start       : in  std_logic;
        busy        : out std_logic;
        done        : out std_logic;
        timeout     : out std_logic;
        abort       : in  std_logic;
        
        opcode      : in  unsigned(3 downto 0);
        operand_a   : in  q32_32;
        operand_b   : in  q32_32;
        
        result      : out q32_32;
        overflow    : out std_logic;
        div_by_zero : out std_logic;
        invalid_op  : out std_logic
    );
end entity fixd_util;

architecture rtl of fixd_util is

    -- Constants
    constant ZERO_Q       : q32_32 := (others => '0');
    constant ONE_Q        : q32_32 := x"0000000100000000";
    constant MINUS_ONE_Q  : q32_32 := x"FFFFFFFF00000000";
    constant DEG_TO_RAD_Q : q32_32 := x"000000000477D1A9";  -- ?/180
    constant RAD_TO_DEG_Q : q32_32 := x"000000394BB834C6";  -- 180/?
    constant TWO_PI_Q     : q32_32 := x"00000006487ED511";  -- 2?
    constant PI_Q         : q32_32 := x"00000003243F6A89";  -- ?

    constant TIMEOUT_MAX   : integer := 10000000;
    constant STALL_THRESHOLD : integer := 5000000;
    constant MAX_EXPONENT   : integer := 63;

    -- Opcodes
    constant OP_TO_RAD    : unsigned(3 downto 0) := "0000";
    constant OP_TO_DEG    : unsigned(3 downto 0) := "0001";
    constant OP_SCALED_INT_TO : unsigned(3 downto 0) := "0010";
    constant OP_SCALED_INT_FROM : unsigned(3 downto 0) := "0011";
    constant OP_INV       : unsigned(3 downto 0) := "0100";
    constant OP_POW       : unsigned(3 downto 0) := "0101";
    constant OP_LIMIT_0_2PI : unsigned(3 downto 0) := "0110";
    constant OP_LIMIT_0_1   : unsigned(3 downto 0) := "0111";
    constant OP_LIMIT_PI_PI : unsigned(3 downto 0) := "1000";
    constant OP_LIMIT_0_PI  : unsigned(3 downto 0) := "1001";

    -- FSM states
    type state_t is (
        ST_IDLE,
        ST_CAPTURE,
        ST_MUL_START, ST_WAIT_MUL,
        ST_DIV_START, ST_WAIT_DIV,
        ST_INV_DIV_START, ST_INV_WAIT_DIV,
        ST_POW_CHECK,
        ST_POW_MUL_START, ST_POW_WAIT_MUL,
        ST_POW_NEG_START, ST_POW_NEG_WAIT_DIV,
        ST_POW_DONE,
        -- New division-free limit states
        ST_LIMIT_RAW,          -- capture raw operand
        ST_LIMIT_360,          -- bring into (-360�,360�]
        ST_LIMIT_SIGNED,       -- bring into (-180�,180�]
        ST_LIMIT_FINAL,        -- map to required range
        ST_LIMIT_0_1_START,    -- special for modulus 1
        ST_LIMIT_0_1_ADJ,
        ST_TIMEOUT, ST_ABORT, ST_RESET_WAIT
    );
    signal state      : state_t := ST_IDLE;
    signal wait_cnt   : integer range 0 to 10 := 0;
    signal timeout_cnt: integer range 0 to TIMEOUT_MAX := 0;
    signal reset_complete : unsigned(3 downto 0) := (others => '0');

    -- Operation registers
    signal op_reg          : unsigned(3 downto 0) := (others => '0');
    signal a_reg           : q32_32 := (others => '0');
    signal b_reg           : q32_32 := (others => '0');
    signal result_reg      : q32_32 := (others => '0');
    signal overflow_reg    : std_logic := '0';
    signal div_zero_reg    : std_logic := '0';
    signal invalid_op_reg  : std_logic := '0';

    -- Internal control for q_pow
    signal pow_base        : q32_32 := (others => '0');
    signal pow_exp         : integer range -MAX_EXPONENT to MAX_EXPONENT := 0;
    signal pow_result      : q32_32 := (others => '0');
    signal pow_iter        : integer range 0 to MAX_EXPONENT := 0;

    -- Limit pipeline registers
    signal raw_val         : q32_32 := (others => '0');
    signal limited_360     : q32_32 := (others => '0');
    signal signed_val      : q32_32 := (others => '0');
    signal final_val       : q32_32 := (others => '0');

    -- Math engine components (unchanged)
    component fixed_mul_64x64 is
        port(
            clk      : in  std_logic;
            rst      : in  std_logic;
            start    : in  std_logic;
            busy     : out std_logic;
            done     : out std_logic;
            a        : in  q32_32;
            b        : in  q32_32;
            result   : out q32_32;
            overflow : out std_logic;
            timeout  : out std_logic;
            abort    : in  std_logic
        );
    end component;

    component fixed_div is
        port(
            clk      : in  std_logic;
            rst      : in  std_logic;
            start    : in  std_logic;
            busy     : out std_logic;
            done     : out std_logic;
            dividend : in  q32_32;
            divisor  : in  q32_32;
            quotient : out q32_32;
            div_zero : out std_logic;
            overflow : out std_logic;
            timeout  : out std_logic;
            abort    : in  std_logic
        );
    end component;

    -- Math engine signals (unchanged)
    signal mul_start         : std_logic := '0';
    signal mul_start_pending : std_logic := '0';
    signal mul_busy          : std_logic;
    signal mul_done          : std_logic;
    signal mul_a_reg         : q32_32 := (others => '0');
    signal mul_b_reg         : q32_32 := (others => '0');
    signal mul_result        : q32_32;
    signal mul_overflow      : std_logic;
    signal mul_timeout       : std_logic;

    signal div_start         : std_logic := '0';
    signal div_start_pending : std_logic := '0';
    signal div_busy          : std_logic;
    signal div_done          : std_logic;
    signal div_dividend_reg  : q32_32 := (others => '0');
    signal div_divisor_reg   : q32_32 := (others => '0');
    signal div_quotient      : q32_32;
    signal div_zero          : std_logic;
    signal div_overflow      : std_logic;
    signal div_timeout       : std_logic;

    -- Stall counters
    signal mul_stall_cnt     : integer range 0 to STALL_THRESHOLD := 0;
    signal div_stall_cnt     : integer range 0 to STALL_THRESHOLD := 0;

    -- Control signals
    signal busy_r            : std_logic := '0';
    signal done_r            : std_logic := '0';
    signal timeout_r         : std_logic := '0';

    -- Helper functions
    function q_to_int(val : q32_32) return integer is
        variable tmp : signed(63 downto 0) := val;
    begin
        return to_integer(tmp(63 downto 32));
    end function;

    function int_to_q(val : integer) return q32_32 is
        variable tmp : signed(63 downto 0) := (others => '0');
    begin
        tmp(63 downto 32) := to_signed(val, 32);
        return tmp;
    end function;

begin

    mul_inst : fixed_mul_64x64
        port map(
            clk      => clk,
            rst      => rst,
            start    => mul_start,
            busy     => mul_busy,
            done     => mul_done,
            a        => mul_a_reg,
            b        => mul_b_reg,
            result   => mul_result,
            overflow => mul_overflow,
            timeout  => mul_timeout,
            abort    => abort
        );

    div_inst : fixed_div
        port map(
            clk      => clk,
            rst      => rst,
            start    => div_start,
            busy     => div_busy,
            done     => div_done,
            dividend => div_dividend_reg,
            divisor  => div_divisor_reg,
            quotient => div_quotient,
            div_zero => div_zero,
            overflow => div_overflow,
            timeout  => div_timeout,
            abort    => abort
        );

    busy    <= busy_r;
    done    <= done_r;
    timeout <= timeout_r;
    result  <= result_reg;
    overflow <= overflow_reg;
    div_by_zero <= div_zero_reg;
    invalid_op <= invalid_op_reg;

    process(clk)
        variable v_state          : state_t;
        variable v_wait_cnt       : integer range 0 to 10;
        variable v_timeout_cnt    : integer range 0 to TIMEOUT_MAX;
        variable v_busy           : std_logic;
        variable v_done           : std_logic;
        variable v_timeout        : std_logic;

        variable v_op_reg         : unsigned(3 downto 0);
        variable v_a_reg          : q32_32;
        variable v_b_reg          : q32_32;
        variable v_result_reg     : q32_32;
        variable v_overflow_reg   : std_logic;
        variable v_div_zero_reg   : std_logic;
        variable v_invalid_op_reg : std_logic;

        variable v_mul_start      : std_logic;
        variable v_mul_a          : q32_32;
        variable v_mul_b          : q32_32;
        variable v_div_start      : std_logic;
        variable v_div_dividend   : q32_32;
        variable v_div_divisor    : q32_32;

        variable v_mul_stall_cnt  : integer range 0 to STALL_THRESHOLD;
        variable v_div_stall_cnt  : integer range 0 to STALL_THRESHOLD;

        variable v_pow_base       : q32_32;
        variable v_pow_exp        : integer range -MAX_EXPONENT to MAX_EXPONENT;
        variable v_pow_result     : q32_32;
        variable v_pow_iter       : integer range 0 to MAX_EXPONENT;

        variable v_raw_val        : q32_32;
        variable v_limited_360    : q32_32;
        variable v_signed_val     : q32_32;
        variable v_final_val      : q32_32;

        variable v_timeout_expired : boolean;

    begin
        if rising_edge(clk) then
            if rst = '1' or abort = '1' then
                -- Reset all registers (same as original)
                state <= ST_IDLE;
                wait_cnt <= 0;
                timeout_cnt <= 0;
                reset_complete <= (others => '0');
                op_reg <= (others => '0');
                a_reg <= (others => '0');
                b_reg <= (others => '0');
                result_reg <= (others => '0');
                overflow_reg <= '0';
                div_zero_reg <= '0';
                invalid_op_reg <= '0';
                busy_r <= '0';
                done_r <= '0';
                timeout_r <= '0';
                mul_start <= '0';
                mul_start_pending <= '0';
                mul_a_reg <= (others => '0');
                mul_b_reg <= (others => '0');
                mul_stall_cnt <= 0;
                div_start <= '0';
                div_start_pending <= '0';
                div_dividend_reg <= (others => '0');
                div_divisor_reg <= (others => '0');
                div_stall_cnt <= 0;
                pow_base <= (others => '0');
                pow_exp <= 0;
                pow_result <= (others => '0');
                pow_iter <= 0;
                raw_val <= (others => '0');
                limited_360 <= (others => '0');
                signed_val <= (others => '0');
                final_val <= (others => '0');
            else
                -- Default assignments
                v_state := state;
                v_wait_cnt := wait_cnt;
                v_timeout_cnt := timeout_cnt;
                v_busy := busy_r;
                v_done := '0';
                v_timeout := '0';

                v_op_reg := op_reg;
                v_a_reg := a_reg;
                v_b_reg := b_reg;
                v_result_reg := result_reg;
                v_overflow_reg := overflow_reg;
                v_div_zero_reg := div_zero_reg;
                v_invalid_op_reg := invalid_op_reg;

                v_mul_start := '0';
                v_mul_a := mul_a_reg;
                v_mul_b := mul_b_reg;
                v_div_start := '0';
                v_div_dividend := div_dividend_reg;
                v_div_divisor := div_divisor_reg;

                v_mul_stall_cnt := mul_stall_cnt;
                v_div_stall_cnt := div_stall_cnt;

                v_pow_base := pow_base;
                v_pow_exp := pow_exp;
                v_pow_result := pow_result;
                v_pow_iter := pow_iter;

                v_raw_val := raw_val;
                v_limited_360 := limited_360;
                v_signed_val := signed_val;
                v_final_val := final_val;

                v_timeout_expired := false;

                if state /= ST_IDLE then
                    if v_timeout_cnt < TIMEOUT_MAX then
                        v_timeout_cnt := v_timeout_cnt + 1;
                    else
                        v_timeout_cnt := 0;
                        v_timeout_expired := true;
                    end if;
                end if;

                -- Stall detection (only for mul/div states, not for limit states)
                case state is
                    when ST_WAIT_MUL | ST_POW_WAIT_MUL =>
                        if mul_busy = '0' and mul_done = '0' and mul_start = '0' then
                            v_mul_stall_cnt := v_mul_stall_cnt + 1;
                            if v_mul_stall_cnt > STALL_THRESHOLD then
                                v_state := ST_TIMEOUT;
                            end if;
                        else
                            v_mul_stall_cnt := 0;
                        end if;

                    when ST_WAIT_DIV | ST_INV_WAIT_DIV | ST_POW_NEG_WAIT_DIV =>
                        if div_busy = '0' and div_done = '0' and div_start = '0' then
                            v_div_stall_cnt := v_div_stall_cnt + 1;
                            if v_div_stall_cnt > STALL_THRESHOLD then
                                v_state := ST_TIMEOUT;
                            end if;
                        else
                            v_div_stall_cnt := 0;
                        end if;

                    when others => null;
                end case;

                -- Main FSM
                case state is
                    when ST_IDLE =>
                        v_busy := '0';
                        if start = '1' then
                            v_busy := '1';
                            v_state := ST_CAPTURE;
                            v_timeout_cnt := 0;
                            v_invalid_op_reg := '0';
                            v_overflow_reg := '0';
                            v_div_zero_reg := '0';
                        end if;

                    when ST_CAPTURE =>
                        v_op_reg := opcode;
                        v_a_reg := operand_a;
                        v_b_reg := operand_b;

                        case opcode is
                            when OP_TO_RAD | OP_TO_DEG | OP_SCALED_INT_TO |
                                 OP_SCALED_INT_FROM | OP_INV | OP_POW =>
                                v_invalid_op_reg := '0';
                                case opcode is
                                    when OP_TO_RAD | OP_TO_DEG =>
                                        v_state := ST_MUL_START;
                                    when OP_SCALED_INT_TO | OP_SCALED_INT_FROM | OP_INV =>
                                        v_state := ST_DIV_START;
                                    when OP_POW =>
                                        v_state := ST_POW_CHECK;
                                    when others =>
                                        v_state := ST_TIMEOUT;
                                end case;

                            -- New division-free limit opcodes
                            when OP_LIMIT_0_2PI | OP_LIMIT_PI_PI | OP_LIMIT_0_PI =>
                                v_invalid_op_reg := '0';
                                v_state := ST_LIMIT_RAW;

                            when OP_LIMIT_0_1 =>
                                v_invalid_op_reg := '0';
                                v_state := ST_LIMIT_0_1_START;

                            when others =>
                                v_invalid_op_reg := '1';
                                v_state := ST_TIMEOUT;
                        end case;

                    -- =========================================================
                    -- Division-free limit functions (0x6,0x8,0x9)
                    -- =========================================================
                    when ST_LIMIT_RAW =>
                        v_raw_val := v_a_reg;
                        v_state := ST_LIMIT_360;

                    when ST_LIMIT_360 =>
                        -- Bring into (-360�,360�] with one addition/subtraction
                        if v_raw_val > TWO_PI_Q then
                            v_limited_360 := v_raw_val - TWO_PI_Q;
                        elsif v_raw_val < -TWO_PI_Q then
                            v_limited_360 := v_raw_val + TWO_PI_Q;
                        else
                            v_limited_360 := v_raw_val;
                        end if;
                        v_state := ST_LIMIT_SIGNED;

                    when ST_LIMIT_SIGNED =>
                        -- Bring into (-180�,180�] (signed)
                        if v_limited_360 > PI_Q then
                            v_signed_val := v_limited_360 - TWO_PI_Q;
                        elsif v_limited_360 <= -PI_Q then
                            v_signed_val := v_limited_360 + TWO_PI_Q;
                        else
                            v_signed_val := v_limited_360;
                        end if;
                        v_state := ST_LIMIT_FINAL;

                    when ST_LIMIT_FINAL =>
                        case v_op_reg is
                            when OP_LIMIT_PI_PI =>
                                v_final_val := v_signed_val;   -- already in (-?,?]

                            when OP_LIMIT_0_2PI =>
                                -- map signed value to [0,2?)
                                if v_signed_val < 0 then
                                    v_final_val := v_signed_val + TWO_PI_Q;
                                else
                                    v_final_val := v_signed_val;
                                end if;

                            when OP_LIMIT_0_PI =>
                                -- map to [0,?)
                                if v_signed_val < 0 then
                                    v_final_val := v_signed_val + PI_Q;
                                else
                                    v_final_val := v_signed_val;
                                end if;
                                -- ensure result < ? (if exactly ?, subtract ?)
                                if v_final_val >= PI_Q then
                                    v_final_val := v_final_val - PI_Q;
                                end if;

                            when others =>
                                v_final_val := (others => '0');
                        end case;
                        v_result_reg := v_final_val;
                        v_done := '1';
                        v_busy := '0';
                        v_state := ST_IDLE;

                    -- =========================================================
                    -- Special for modulus 1 (OP_LIMIT_0_1)
                    -- =========================================================
                    when ST_LIMIT_0_1_START =>
                        v_raw_val := v_a_reg;
                        v_state := ST_LIMIT_0_1_ADJ;

                    when ST_LIMIT_0_1_ADJ =>
                        -- Bring into [0,1) by at most one addition/subtraction
                        if v_raw_val >= ONE_Q then
                            v_final_val := v_raw_val - ONE_Q;
                        elsif v_raw_val < 0 then
                            v_final_val := v_raw_val + ONE_Q;
                        else
                            v_final_val := v_raw_val;
                        end if;
                        v_result_reg := v_final_val;
                        v_done := '1';
                        v_busy := '0';
                        v_state := ST_IDLE;

                    -- =========================================================
                    -- Original multiplication/division/power logic (unchanged)
                    -- =========================================================
                    when ST_MUL_START =>
                        if v_op_reg = OP_TO_RAD then
                            v_mul_a := v_a_reg;
                            v_mul_b := DEG_TO_RAD_Q;
                        else -- OP_TO_DEG
                            v_mul_a := v_a_reg;
                            v_mul_b := RAD_TO_DEG_Q;
                        end if;
                        v_mul_start := '1';
                        v_state := ST_WAIT_MUL;

                    when ST_WAIT_MUL =>
                        if mul_done = '1' then
                            v_mul_start := '0';
                            if mul_overflow = '1' or mul_timeout = '1' then
                                v_overflow_reg := '1';
                                v_state := ST_TIMEOUT;
                            else
                                v_result_reg := mul_result;
                                v_done := '1';
                                v_busy := '0';
                                v_state := ST_IDLE;
                            end if;
                        elsif v_timeout_expired then
                            v_state := ST_TIMEOUT;
                        end if;

                    when ST_DIV_START =>
                        if v_op_reg = OP_INV then
                            v_div_dividend := ONE_Q;
                            v_div_divisor := v_a_reg;
                        elsif v_op_reg = OP_SCALED_INT_TO then
                            v_div_dividend := v_a_reg;
                            v_div_divisor := v_b_reg;
                        else -- OP_SCALED_INT_FROM
                            v_mul_a := v_a_reg;
                            v_mul_b := v_b_reg;
                            v_mul_start := '1';
                            v_state := ST_WAIT_MUL;
                        end if;

                        if v_op_reg /= OP_SCALED_INT_FROM then
                            if v_div_divisor = ZERO_Q then
                                v_div_zero_reg := '1';
                                v_state := ST_TIMEOUT;
                            else
                                v_div_start := '1';
                                v_state := ST_WAIT_DIV;
                            end if;
                        end if;

                    when ST_WAIT_DIV =>
                        if div_done = '1' then
                            v_div_start := '0';
                            if div_zero = '1' or div_overflow = '1' or div_timeout = '1' then
                                if div_zero = '1' then
                                    v_div_zero_reg := '1';
                                else
                                    v_overflow_reg := '1';
                                end if;
                                v_state := ST_TIMEOUT;
                            else
                                v_result_reg := div_quotient;
                                v_done := '1';
                                v_busy := '0';
                                v_state := ST_IDLE;
                            end if;
                        elsif v_timeout_expired then
                            v_state := ST_TIMEOUT;
                        end if;

                    -- q_pow (unchanged)
                    when ST_POW_CHECK =>
                        v_pow_base := v_a_reg;
                        v_pow_exp := q_to_int(v_b_reg);
                        if v_pow_exp = 0 then
                            v_result_reg := ONE_Q;
                            v_done := '1';
                            v_busy := '0';
                            v_state := ST_IDLE;
                        elsif v_pow_exp > 0 then
                            v_pow_result := ONE_Q;
                            v_pow_iter := 0;
                            v_state := ST_POW_MUL_START;
                        else
                            if v_pow_base = ZERO_Q then
                                v_div_zero_reg := '1';
                                v_state := ST_TIMEOUT;
                            else
                                v_pow_result := ONE_Q;
                                v_pow_iter := 0;
                                v_pow_exp := -v_pow_exp;
                                v_state := ST_POW_MUL_START;
                            end if;
                        end if;

                    when ST_POW_MUL_START =>
                        if v_pow_iter < v_pow_exp then
                            v_mul_a := v_pow_result;
                            v_mul_b := v_pow_base;
                            v_mul_start := '1';
                            v_state := ST_POW_WAIT_MUL;
                        else
                            if v_pow_exp < 0 then
                                v_state := ST_POW_NEG_START;
                            else
                                v_result_reg := v_pow_result;
                                v_done := '1';
                                v_busy := '0';
                                v_state := ST_IDLE;
                            end if;
                        end if;

                    when ST_POW_WAIT_MUL =>
                        if mul_done = '1' then
                            v_mul_start := '0';
                            if mul_overflow = '1' or mul_timeout = '1' then
                                v_overflow_reg := '1';
                                v_state := ST_TIMEOUT;
                            else
                                v_pow_result := mul_result;
                                v_pow_iter := v_pow_iter + 1;
                                v_state := ST_POW_MUL_START;
                            end if;
                        elsif v_timeout_expired then
                            v_state := ST_TIMEOUT;
                        end if;

                    when ST_POW_NEG_START =>
                        if v_pow_result = ZERO_Q then
                            v_div_zero_reg := '1';
                            v_state := ST_TIMEOUT;
                        else
                            v_div_dividend := ONE_Q;
                            v_div_divisor := v_pow_result;
                            v_div_start := '1';
                            v_state := ST_POW_NEG_WAIT_DIV;
                        end if;

                    when ST_POW_NEG_WAIT_DIV =>
                        if div_done = '1' then
                            v_div_start := '0';
                            if div_zero = '1' or div_overflow = '1' or div_timeout = '1' then
                                if div_zero = '1' then
                                    v_div_zero_reg := '1';
                                else
                                    v_overflow_reg := '1';
                                end if;
                                v_state := ST_TIMEOUT;
                            else
                                v_result_reg := div_quotient;
                                v_done := '1';
                                v_busy := '0';
                                v_state := ST_IDLE;
                            end if;
                        elsif v_timeout_expired then
                            v_state := ST_TIMEOUT;
                        end if;

                    -- =========================================================
                    -- Error handling
                    -- =========================================================
                    when ST_TIMEOUT =>
                        v_timeout := '1';
                        v_busy := '0';
                        v_done := '0';
                        v_state := ST_ABORT;

                    when ST_ABORT =>
                        v_state := ST_RESET_WAIT;
                        reset_complete <= (others => '0');

                    when ST_RESET_WAIT =>
                        reset_complete <= reset_complete + 1;
                        if reset_complete > 10 then
                            v_state := ST_IDLE;
                        end if;

                    when others =>
                        v_state := ST_IDLE;
                end case;

                -- Math engine start pulse pipelining (unchanged)
                if v_mul_start = '1' then
                    mul_a_reg <= v_mul_a;
                    mul_b_reg <= v_mul_b;
                    mul_start <= '0';
                    mul_start_pending <= '1';
                elsif mul_start_pending = '1' then
                    mul_start <= '1';
                    mul_start_pending <= '0';
                else
                    mul_start <= '0';
                end if;

                if v_div_start = '1' then
                    div_dividend_reg <= v_div_dividend;
                    div_divisor_reg <= v_div_divisor;
                    div_start <= '0';
                    div_start_pending <= '1';
                elsif div_start_pending = '1' then
                    div_start <= '1';
                    div_start_pending <= '0';
                else
                    div_start <= '0';
                end if;

                -- Register updates
                state <= v_state;
                wait_cnt <= v_wait_cnt;
                timeout_cnt <= v_timeout_cnt;
                op_reg <= v_op_reg;
                a_reg <= v_a_reg;
                b_reg <= v_b_reg;
                result_reg <= v_result_reg;
                overflow_reg <= v_overflow_reg;
                div_zero_reg <= v_div_zero_reg;
                invalid_op_reg <= v_invalid_op_reg;
                busy_r <= v_busy;
                done_r <= v_done;
                timeout_r <= v_timeout;
                mul_stall_cnt <= v_mul_stall_cnt;
                div_stall_cnt <= v_div_stall_cnt;
                pow_base <= v_pow_base;
                pow_exp <= v_pow_exp;
                pow_result <= v_pow_result;
                pow_iter <= v_pow_iter;
                raw_val <= v_raw_val;
                limited_360 <= v_limited_360;
                signed_val <= v_signed_val;
                final_val <= v_final_val;
            end if;
        end if;
    end process;

end architecture rtl;
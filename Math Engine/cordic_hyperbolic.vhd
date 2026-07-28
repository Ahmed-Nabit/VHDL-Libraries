-- cordic_hyperbolic.vhd (fully corrected)
-- Single‑shot CORDIC hyperbolic core for ln(x) and exp(x) (Q32.32)
-- Uses fixed_mul_64x64 for all multiplications. No combinational '*'.
-- Features:
--   - mode = '0' : ln(x)   (x > 0)
--   - mode = '1' : exp(x)  (any x)
--   - 40 CORDIC iterations (with repeats at i=4,13)
--   - Overflow detection for invalid input or result out of range
--   - Proper handshake: start, busy, done
--   - abort resets the operation
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
use work.fixed_pkg.all;               -- provides q32_32 type

entity cordic_hyperbolic is
    port(
        clk       : in  std_logic;
        rst       : in  std_logic;
        start     : in  std_logic;
        busy      : out std_logic;
        done      : out std_logic;
        mode      : in  std_logic;                     -- '0' = ln, '1' = exp
        x_in      : in  q32_32;
        result    : out q32_32;
        abort     : in  std_logic;
        overflow  : out std_logic;
        timeout   : out std_logic
    );
end entity;

architecture seq of cordic_hyperbolic is

    -- Q32.32 constants
    constant ONE_Q      : q32_32 := x"0000000100000000";
    constant LN2_Q      : q32_32 := x"00000000B17217F7";
    constant INV_HYPERB_GAIN : q32_32 := x"00000000D402407C";   -- 1/K_h
    constant INV_LN2    : q32_32 := x"0000000171547646";       -- 1/ln(2)

    -- ATANH table (40 entries, repeats at i=4,13)
    type atanh_table_t is array(0 to 39) of q32_32;
	constant ATANH_TABLE : atanh_table_t := (
		-- i=0,  s=1
		x"000000008CA6C1D2",
		-- i=1,  s=2
		x"000000004161E52E",
		-- i=2,  s=3
		x"000000002027C8B2",
		-- i=3,  s=4
		x"0000000010046F8F",
		-- i=4,  s=4 (repeat)
		x"0000000010046F8F",
		-- i=5,  s=5
		x"000000000800AAC4",
		-- i=6,  s=6
		x"0000000004001556",
		-- i=7,  s=7
		x"00000000020002AB",
		-- i=8,  s=8
		x"0000000001000055",
		-- i=9,  s=9
		x"000000000080000B",
		-- i=10, s=10
		x"0000000000400001",
		-- i=11, s=11
		x"0000000000200000",
		-- i=12, s=12
		x"0000000000100000",
		-- i=13, s=13
		x"0000000000080000",
		-- i=14, s=13 (repeat)
		x"0000000000080000",
		-- i=15, s=14
		x"0000000000040000",
		-- i=16, s=15
		x"0000000000020000",
		-- i=17, s=16
		x"0000000000010000",
		-- i=18, s=17
		x"0000000000008000",
		-- i=19, s=18
		x"0000000000004000",
		-- i=20, s=19
		x"0000000000002000",
		-- i=21, s=20
		x"0000000000001000",
		-- i=22, s=21
		x"0000000000000800",
		-- i=23, s=22
		x"0000000000000400",
		-- i=24, s=23
		x"0000000000000200",
		-- i=25, s=24
		x"0000000000000100",
		-- i=26, s=25
		x"0000000000000080",
		-- i=27, s=26
		x"0000000000000040",
		-- i=28, s=27
		x"0000000000000020",
		-- i=29, s=28
		x"0000000000000010",
		-- i=30, s=29
		x"0000000000000008",
		-- i=31, s=30
		x"0000000000000004",
		-- i=32, s=31
		x"0000000000000002",
		-- i=33, s=32
		x"0000000000000001",
		-- i=34, s=33 (rounded up from 0.5)
		x"0000000000000001",
		-- i=35, s=34
		x"0000000000000000",
		-- i=36, s=35
		x"0000000000000000",
		-- i=37, s=36
		x"0000000000000000",
		-- i=38, s=37
		x"0000000000000000",
		-- i=39, s=38
		x"0000000000000000"
	);

    -- Corrected shift table: repeat i=4 and i=13
    type shift_table_t is array(0 to 39) of integer range 0 to 38;
    constant SHIFT_TABLE : shift_table_t := (
        1,2,3,4,4,5,6,7,8,9,10,11,12,13,13,14,15,16,17,18,
        19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38
    );

    subtype q32_32_int is signed(65 downto 0);  -- 66-bit signed with 2 guard bits

    -- FSM states
    type state_t is (IDLE,
                     LN_NORM,           -- leading‑one detection (combinatorial, immediate)
                     LN_ITERATE,
                     LN_FINAL_MUL,
                     EXP_SPLIT_MUL1,
                     EXP_SPLIT_MUL2,
                     EXP_ITERATE,
                     EXP_GAIN_MUL,
                     EXP_FINAL_MUL,
                     FINISH);
    signal state : state_t := IDLE;

    -- CORDIC registers
    signal iter_cnt : integer range 0 to 39 := 0;
    signal x_reg, y_reg : q32_32_int := (others => '0');
    signal z_reg : q32_32 := (others => '0');
    signal mode_reg : std_logic := '0';
    signal ln_exp_reg : signed(31 downto 0) := (others => '0');
    signal overflow_reg : std_logic := '0';
    signal exp_r_result : q32_32 := (others => '0');
    signal exp_r_corrected : q32_32 := (others => '0');

    -- For ln normalization (combinatorial, but stored in registers)
    signal mantissa : q32_32 := (others => '0');
    signal exponent : signed(31 downto 0) := (others => '0');
    signal norm_ovf : std_logic := '0';

    -- For exp splitting
    signal k_int : signed(31 downto 0) := (others => '0');
    signal r_frac : q32_32 := (others => '0');
    signal split_ovf : std_logic := '0';

    -- Multiplier interface
    signal mul_start   : std_logic := '0';
    signal mul_busy    : std_logic;
    signal mul_done    : std_logic;
    signal mul_a, mul_b : q32_32 := (others => '0');
    signal mul_result  : q32_32;
    signal mul_overflow : std_logic;
    signal mul_timeout : std_logic;
    signal mul_abort   : std_logic := '0';

    -- Multiplication results storage
    signal mul1_result : q32_32;   -- x_in * INV_LN2
    signal mul2_result : q32_32;   -- k_tmp * LN2_Q
    signal mul3_result : q32_32;   -- ln_exp_reg * LN2_Q
    signal mul4_result : q32_32;   -- exp(r) * 2^k

    signal busy_r, done_r : std_logic := '0';
    signal result_r : q32_32 := (others => '0');
    signal overflow_r : std_logic := '0';

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

    -- Helper to convert a signed integer to Q32.32 (shift left by 32)
    function int_to_q32_32(v : signed(31 downto 0)) return q32_32 is
    begin
        return resize(shift_left(v, 32), 64);
    end function;

begin

    timeout <= '0';
    busy <= busy_r;
    done <= done_r;
    result <= result_r;
    overflow <= overflow_r;

    mul_inst : fixed_mul_64x64
        port map(
            clk      => clk,
            rst      => rst,
            start    => mul_start,
            busy     => mul_busy,
            done     => mul_done,
            a        => mul_a,
            b        => mul_b,
            result   => mul_result,
            overflow => mul_overflow,
            timeout  => mul_timeout,
            abort    => mul_abort
        );

    -- Leading‑one detector for ln (combinatorial, but only shifts)
    process(x_in)
        variable x_abs : unsigned(63 downto 0);
        variable pos : integer range -1 to 63;
    begin
        norm_ovf <= '0';
        mantissa <= (others => '0');
        exponent <= (others => '0');
        if x_in <= 0 then
            norm_ovf <= '1';
            return;
        end if;
        x_abs := unsigned(x_in);
        pos := -1;
        for i in 63 downto 0 loop
            if x_abs(i) = '1' then
                pos := i;
                exit;
            end if;
        end loop;
        if pos >= 32 then
            mantissa <= signed(shift_right(x_abs, pos - 32));
            exponent <= to_signed(pos - 32, 32);
        else
            mantissa <= signed(shift_left(x_abs, 32 - pos));
            exponent <= to_signed(pos - 32, 32);
        end if;
    end process;

    -- Function to compute 2^k in Q32.32 (synthesizable, handles negative k)
    function two_pow_k_q32(k : signed(31 downto 0)) return q32_32 is
        variable shift_amt : integer;
        variable res : signed(63 downto 0);
    begin
        shift_amt := to_integer(k);
        if shift_amt >= 0 then
            if shift_amt > 31 then
                return x"7FFFFFFFFFFFFFFF";   -- saturate to max positive
            else
                return shift_left(to_signed(1, 64), shift_amt + 32);
            end if;
        else
            if shift_amt <= -32 then
                return (others => '0');       -- underflow to 0
            else
                return shift_left(to_signed(1, 64), 32 + shift_amt);
            end if;
        end if;
    end function;

    -- Main FSM
    process(clk)
        -- Variables for temporary calculations
        variable v_k : signed(31 downto 0);
        variable v_r : q32_32;
        variable v_overflow : std_logic;
        variable v_sum : signed(127 downto 0);
        variable v_round_bit : std_logic;
        variable v_q96 : signed(95 downto 0);
        variable v_sum66 : signed(65 downto 0);  -- for x+y
    begin
        if rising_edge(clk) then
            if rst = '1' or abort = '1' then
                state <= IDLE;
                busy_r <= '0';
                done_r <= '0';
                result_r <= (others => '0');
                overflow_r <= '0';
                iter_cnt <= 0;
                mul_start <= '0';
                mul_abort <= '1';
            else
                mul_abort <= '0';
                case state is
                    when IDLE =>
                        busy_r <= '0';
                        done_r <= '0';
                        if start = '1' then
                            busy_r <= '1';
                            if mode = '0' then
                                -- ln(x)
                                if norm_ovf = '1' then
                                    overflow_reg <= '1';
                                    state <= FINISH;
                                else
                                    mode_reg <= '0';
                                    ln_exp_reg <= exponent;
                                    overflow_reg <= '0';
                                    x_reg <= resize(mantissa + ONE_Q, 66);
                                    y_reg <= resize(mantissa - ONE_Q, 66);
                                    z_reg <= (others => '0');
                                    state <= LN_ITERATE;
                                    iter_cnt <= 0;
                                end if;
                            else
                                -- exp(x): start multiplier for x_in * INV_LN2
                                mul_a <= x_in;
                                mul_b <= INV_LN2;
                                mul_start <= '1';
                                state <= EXP_SPLIT_MUL1;
                            end if;
                        end if;

                    -- ========== EXP splitting: first multiplication ==========
                    when EXP_SPLIT_MUL1 =>
                        mul_start <= '0';
                        if mul_busy = '1' then
                            -- wait
                        elsif mul_done = '1' then
                            mul1_result <= mul_result;
                            v_k := mul_result(63 downto 32);  -- integer part
                            -- Start second multiplication: k * LN2_Q
                            mul_a <= int_to_q32_32(v_k);
                            mul_b <= LN2_Q;
                            mul_start <= '1';
                            state <= EXP_SPLIT_MUL2;
                        end if;

                    when EXP_SPLIT_MUL2 =>
                        mul_start <= '0';
                        if mul_busy = '1' then
                            -- wait
                        elsif mul_done = '1' then
                            mul2_result <= mul_result;
                            -- Compute r = x_in - (k * LN2_Q)
                            v_r := x_in - mul_result;
                            v_k := mul1_result(63 downto 32);  -- recover k
                            if v_r < 0 then
                                v_r := v_r + LN2_Q;
                                v_k := v_k - 1;
                            elsif v_r >= LN2_Q then
                                v_r := v_r - LN2_Q;
                                v_k := v_k + 1;
                            end if;
                            k_int <= v_k;
                            r_frac <= v_r;
                            -- Overflow if k >= 31 (2^31 out of signed Q32.32 range)
                            if v_k >= 31 then
                                v_overflow := '1';
                            else
                                v_overflow := '0';
                            end if;
                            split_ovf <= v_overflow;
                            if v_overflow = '1' then
                                overflow_reg <= '1';
                                state <= FINISH;
                            else
                                mode_reg <= '1';
                                ln_exp_reg <= v_k;
                                overflow_reg <= '0';
                                x_reg <= resize(ONE_Q, 66);
                                y_reg <= (others => '0');
                                z_reg <= v_r;
                                state <= EXP_ITERATE;
                                iter_cnt <= 0;
                            end if;
                        end if;

                    -- ========== CORDIC iterations (common for both modes) ==========
                    when LN_ITERATE | EXP_ITERATE =>
                        if iter_cnt = 40 then
                            if mode_reg = '0' then
                                -- ln: need final multiplication ln_exp_reg * LN2_Q
                                mul_a <= int_to_q32_32(ln_exp_reg);
                                mul_b <= LN2_Q;
                                mul_start <= '1';
                                state <= LN_FINAL_MUL;
                            else
                                -- exp: compute exp(r) from x+y (which is K_h*exp(r))
                                v_sum66 := x_reg + y_reg;
                                exp_r_result <= v_sum66(63 downto 0);
                                -- Multiply by 1/K_h to correct gain
                                mul_a <= exp_r_result;
                                mul_b <= INV_HYPERB_GAIN;
                                mul_start <= '1';
                                state <= EXP_GAIN_MUL;
                            end if;
                        else
                            -- Perform one CORDIC iteration with CORRECTED signs
                            --   For rotation (exp):  if z>=0 then d=+1 else d=-1
                            --   For vectoring (ln):  if y>=0 then d=-1 else d=+1
                            -- In both cases, x and y updates use the same d.
                            if mode_reg = '1' then   -- rotation (exp)
                                if z_reg >= 0 then   -- d = +1
                                    x_reg <= x_reg + shift_right(y_reg, SHIFT_TABLE(iter_cnt));
                                    y_reg <= y_reg + shift_right(x_reg, SHIFT_TABLE(iter_cnt));
                                    z_reg <= z_reg - ATANH_TABLE(iter_cnt);
                                else                 -- d = -1
                                    x_reg <= x_reg - shift_right(y_reg, SHIFT_TABLE(iter_cnt));
                                    y_reg <= y_reg - shift_right(x_reg, SHIFT_TABLE(iter_cnt));
                                    z_reg <= z_reg + ATANH_TABLE(iter_cnt);
                                end if;
                            else                       -- vectoring (ln)
                                if y_reg >= 0 then   -- d = -1
                                    x_reg <= x_reg - shift_right(y_reg, SHIFT_TABLE(iter_cnt));
                                    y_reg <= y_reg - shift_right(x_reg, SHIFT_TABLE(iter_cnt));
                                    z_reg <= z_reg + ATANH_TABLE(iter_cnt);
                                else                 -- d = +1
                                    x_reg <= x_reg + shift_right(y_reg, SHIFT_TABLE(iter_cnt));
                                    y_reg <= y_reg + shift_right(x_reg, SHIFT_TABLE(iter_cnt));
                                    z_reg <= z_reg - ATANH_TABLE(iter_cnt);
                                end if;
                            end if;
                            iter_cnt <= iter_cnt + 1;
                        end if;

                    -- ========== LN final multiplication ==========
                    when LN_FINAL_MUL =>
                        mul_start <= '0';
                        if mul_busy = '1' then
                            -- wait
                        elsif mul_done = '1' then
                            mul3_result <= mul_result;
                            -- Result = 2*z + (ln_exp * LN2_Q)
                            v_q96 := resize(z_reg, 96);
                            v_q96 := shift_left(v_q96, 1);
                            v_q96 := v_q96 + resize(mul3_result, 96);
                            -- Check overflow of the 96-bit sum
                            if (v_q96(95 downto 64) /= (95 downto 64 => v_q96(63))) then
                                overflow_r <= '1';
                            else
                                overflow_r <= '0';
                            end if;
                            -- OR with multiplier overflow (if any)
                            overflow_r <= overflow_r or mul_overflow;
                            -- take lower 64 bits (Q32.32)
                            result_r <= v_q96(63 downto 0);
                            state <= FINISH;
                        end if;

                    -- ========== EXP gain correction ==========
                    when EXP_GAIN_MUL =>
                        mul_start <= '0';
                        if mul_busy = '1' then
                            -- wait
                        elsif mul_done = '1' then
                            exp_r_corrected <= mul_result;
                            -- Start final scaling: exp(r) * 2^k
                            mul_a <= exp_r_corrected;
                            mul_b <= two_pow_k_q32(ln_exp_reg);
                            mul_start <= '1';
                            state <= EXP_FINAL_MUL;
                        end if;

                    -- ========== EXP final multiplication ==========
                    when EXP_FINAL_MUL =>
                        mul_start <= '0';
                        if mul_busy = '1' then
                            -- wait
                        elsif mul_done = '1' then
                            result_r <= mul_result;
                            overflow_r <= mul_overflow or overflow_reg;
                            state <= FINISH;
                        end if;

                    -- ========== Finish ==========
                    when FINISH =>
                        done_r <= '1';
                        busy_r <= '0';
                        overflow_r <= overflow_reg;   -- propagate overflow from early termination
                        state <= IDLE;

                    when others =>
                        state <= IDLE;
                end case;
            end if;
        end if;
    end process;

end architecture seq;

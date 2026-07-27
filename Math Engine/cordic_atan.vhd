-- cordic_atan.vhd - 18-stage CORDIC vectoring mode pipeline computing Q32.32 arctangent
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
use work.fixed_pkg.all;          -- provides q32_32 type and has_x function

entity cordic_atan is
    port(
        clk       : in  std_logic;
        rst       : in  std_logic;
        start     : in  std_logic;
        busy      : out std_logic;
        done      : out std_logic;
        y         : in  q32_32;   -- 32.32 fixedâ€‘point
        x         : in  q32_32;   -- 32.32 fixedâ€‘point
        angle_out : out q32_32;   -- 32.32 fixedâ€‘point result
        timeout   : out std_logic;   -- Stub (kept per Ahmed's instruction)
        abort     : in  std_logic
    );
end entity;

architecture rtl of cordic_atan is

    -- FIXED: Increased from 18 to 24 iterations to match cordic_sincos precision.
    -- Residual drops from ~0.00022 deg to ~0.0000034 deg, satisfying SPA +-0.0003 deg spec.
    constant ITER : integer := 32;
    constant PIPE_DEPTH : integer := ITER + 3;   -- input + ITER + quadrant + output (now 27)

    -- Q32.32 constants (scaled by 2^32)
    constant PI_Q       : q32_32 := signed'(x"00000003243F6A89");      -- pi * 2^32 (corrected)
    constant PI_NEG_Q   : q32_32 := signed'(x"FFFFFFFCDBC09577");      -- -pi * 2^32 (two's complement of PI_Q)
    constant PI_2_Q     : q32_32 := signed'(x"00000001921FB544");      -- pi/2 * 2^32 (correct, half of PI_Q)

    type atan_table_t is array(0 to ITER-1) of q32_32;
    constant ATAN_TABLE : atan_table_t := (
        signed'(x"00000000C90FDAA2"), -- atan(2^-0) = 0.785398163397
        signed'(x"0000000076B19C15"), -- atan(2^-1) = 0.463647609001
        signed'(x"000000003EB6EBF2"), -- atan(2^-2) = 0.244978663127
        signed'(x"000000001FD5BA9A"), -- atan(2^-3) = 0.124354994547
        signed'(x"000000000FFAADE0"), -- atan(2^-4) = 0.062418809996
        signed'(x"0000000007FF556B"), -- atan(2^-5) = 0.031239833430
        signed'(x"0000000003FFEAB7"), -- atan(2^-6) = 0.015623728620
        signed'(x"0000000001FFFD55"), -- atan(2^-7) = 0.007812341060
        signed'(x"0000000000FFFFAB"), -- atan(2^-8) = 0.003906230131
        signed'(x"00000000007FFFF5"), -- atan(2^-9) = 0.001953122516
        signed'(x"00000000003FFFFF"), -- atan(2^-10) = 0.000976562189
        signed'(x"0000000000200000"), -- atan(2^-11) = 0.000488281219 (rounded)
        signed'(x"0000000000100000"), -- atan(2^-12) = 0.000244140658 (rounded)
        signed'(x"0000000000080000"), -- atan(2^-13) = 0.000122070331 (rounded)
        signed'(x"0000000000040000"), -- atan(2^-14) = 0.000061035166 (rounded)
        signed'(x"0000000000020000"), -- atan(2^-15) = 0.000030517583 (rounded)
        signed'(x"0000000000010000"), -- atan(2^-16) = 0.000015258791 (rounded)
        signed'(x"0000000000008000"), -- atan(2^-17) = 0.000007629395
        signed'(x"0000000000004000"), -- atan(2^-18) ~= 0.000003814697
        signed'(x"0000000000002000"), -- atan(2^-19) ~= 0.000001907349
        signed'(x"0000000000001000"), -- atan(2^-20) ~= 0.000000953674
        signed'(x"0000000000000800"), -- atan(2^-21) ~= 0.000000476837
        signed'(x"0000000000000400"), -- atan(2^-22) ~= 0.000000238419
        signed'(x"0000000000000200"), -- atan(2^-23) ~= 0.000000119209
        signed'(x"0000000000000100"), -- atan(2^-24) ~= 0.000000059604
        signed'(x"0000000000000080"), -- atan(2^-25) ~= 0.000000029802
        signed'(x"0000000000000040"), -- atan(2^-26) ~= 0.000000014901
        signed'(x"0000000000000020"), -- atan(2^-27) ~= 0.000000007451
        signed'(x"0000000000000010"), -- atan(2^-28) ~= 0.000000003725
        signed'(x"0000000000000008"), -- atan(2^-29) ~= 0.000000001863
        signed'(x"0000000000000004"), -- atan(2^-30) ~= 0.000000000931
        signed'(x"0000000000000002")  -- atan(2^-31) ~= 0.000000000466
    );

    -- Internal data path widened to 66 bits (33 integer + 32 fractional + sign)
    -- This safely accommodates the CORDIC gain (~1.646) without overflow.
    subtype q32_32_int is signed(65 downto 0);

    type stage_t is record
        valid    : std_logic;
        x        : q32_32_int;          -- widened to 66 bits
        y        : q32_32_int;          -- widened to 66 bits
        z        : q32_32;               -- angle accumulator (64 bits)
        quadrant : unsigned(1 downto 0);
        bypass   : std_logic;            -- '1' for special cases where iterations are skipped
    end record;

    type pipe_t is array(0 to PIPE_DEPTH-1) of stage_t;

    constant STAGE_DEFAULT : stage_t := (
        valid    => '0',
        x        => (others=>'0'),       -- 66 bits
        y        => (others=>'0'),       -- 66 bits
        z        => (others=>'0'),
        quadrant => "00",
        bypass   => '0'
    );

    signal pipe      : pipe_t := (others=>STAGE_DEFAULT);

    signal busy_r    : std_logic := '0';
    signal done_r    : std_logic := '0';
    signal angle_r   : q32_32 := (others => '0');

    -- Abort synchronizer (twoâ€‘flop, no reset â€“ gives clean level)
    signal abort_meta : std_logic := '0';
    signal abort_sync : std_logic := '0';

    -- -------------------------------------------------------------------------
    -- Function to guard has_x from synthesis (simulation only)
    -- In synthesis, it always returns true.
    -- -------------------------------------------------------------------------
    function is_valid(x, y : q32_32) return boolean is
    begin
        -- synthesis translate_off
        if has_x(std_logic_vector(x)) or has_x(std_logic_vector(y)) then
            return false;
        end if;
        -- synthesis translate_on
        return true;
    end function;

begin

    busy    <= busy_r;
    done    <= done_r;
    angle_out <= angle_r;
    timeout <= '0';   -- Stub kept per Ahmed's instruction (not implemented)

    -- -------------------------------------------------------------------------
    -- Abort synchronizer (asynchronous input, synchronous reset)
    -- No reset on the synchronizer itself â€“ it runs continuously.
    -- -------------------------------------------------------------------------
    process(clk)
    begin
        if rising_edge(clk) then
            abort_meta <= abort;
            abort_sync <= abort_meta;
        end if;
    end process;

    -- -------------------------------------------------------------------------
    -- Main pipeline process
    -- -------------------------------------------------------------------------
    process(clk)
        variable pipe_v      : pipe_t;
        variable busy_v      : std_logic;
        variable done_v      : std_logic;
        variable angle_v     : q32_32;
        variable x_norm_int, y_norm_int : q32_32_int;    -- 66â€‘bit widened
        variable quad        : unsigned(1 downto 0);
        variable y_zero_flag : boolean;
        variable x_zero_flag : boolean;
        variable shift_val   : integer;
        variable x_shift, y_shift : q32_32_int;          -- 66â€‘bit shifted values
    begin
        if rising_edge(clk) then
            -- -----------------------------------------------------------------
            -- Synchronous reset (highest priority) or abort reset
            -- -----------------------------------------------------------------
            if rst = '1' or abort_sync = '1' then
                pipe    <= (others => STAGE_DEFAULT);
                busy_r  <= '0';
                done_r  <= '0';
                angle_r <= (others => '0');
            else
                -- -----------------------------------------------------------------
                -- Initialize next pipeline state (all stages invalid)
                -- -----------------------------------------------------------------
                pipe_v := (others => STAGE_DEFAULT);
                angle_v := angle_r;   -- hold previous output unless updated
                done_v := '0';

                -- -----------------------------------------------------------------
                -- Propagate existing pipeline stages (shift right)
                -- For i = 0 .. PIPE_DEPTH-2, move pipe(i) -> pipe_v(i+1)
                -- -----------------------------------------------------------------
                for i in 0 to PIPE_DEPTH-2 loop
                    if pipe(i).valid = '1' then
                        if pipe(i).bypass = '1' then
                            -- Bypass: just copy data forward (no arithmetic)
                            pipe_v(i+1) := pipe(i);
                            pipe_v(i+1).valid := '1';
                        else
                            -- Normal CORDIC iteration for stages 1..ITER
                            -- Stage indices: 0 = input, 1..ITER = iterations,
                            -- ITER+1 = quadrant correction, ITER+2 = output.
                            if i >= 1 and i <= ITER then
                                shift_val := i - 1;   -- shift amount for this iteration
                                x_shift := shift_right(pipe(i).x, shift_val);
                                y_shift := shift_right(pipe(i).y, shift_val);

                                if pipe(i).y >= 0 then
                                    pipe_v(i+1).x := pipe(i).x + y_shift;
                                    pipe_v(i+1).y := pipe(i).y - x_shift;
                                    pipe_v(i+1).z := pipe(i).z + ATAN_TABLE(i-1);
                                else
                                    pipe_v(i+1).x := pipe(i).x - y_shift;
                                    pipe_v(i+1).y := pipe(i).y + x_shift;
                                    pipe_v(i+1).z := pipe(i).z - ATAN_TABLE(i-1);
                                end if;
                                pipe_v(i+1).quadrant := pipe(i).quadrant;
                                pipe_v(i+1).bypass   := '0';   -- bypass cleared (normal path)
                                pipe_v(i+1).valid    := '1';

                            elsif i = ITER+1 then
                                -- Quadrant correction stage
                                if pipe(i).bypass = '1' then
                                    pipe_v(i+1).z := pipe(i).z;   -- already final
                                else
                                    case pipe(i).quadrant is
                                        when "00" =>  -- Quadrant I: angle = z
                                            pipe_v(i+1).z := pipe(i).z;
                                        when "01" =>  -- Quadrant II: angle = د€ - z
                                            pipe_v(i+1).z := PI_Q - pipe(i).z;
                                        when "10" =>  -- Quadrant III: angle = -د€ + z
                                            pipe_v(i+1).z := PI_NEG_Q + pipe(i).z;
                                        when others => -- Quadrant IV: angle = -z
                                            pipe_v(i+1).z := -pipe(i).z;
                                    end case;
                                end if;
                                pipe_v(i+1).x        := pipe(i).x;
                                pipe_v(i+1).y        := pipe(i).y;
                                pipe_v(i+1).quadrant := pipe(i).quadrant;
                                pipe_v(i+1).bypass   := pipe(i).bypass;
                                pipe_v(i+1).valid    := '1';

                            else
                                -- For stage 0 or other nonâ€‘iteration stages, just copy
                                pipe_v(i+1) := pipe(i);
                                pipe_v(i+1).valid := '1';
                            end if;
                        end if;
                    end if;
                end loop;

                -- -----------------------------------------------------------------
                -- Output stage: consume data from last stage (index PIPE_DEPTH-1)
                -- -----------------------------------------------------------------
                if pipe(PIPE_DEPTH-1).valid = '1' then
                    -- Saturation on 64â€‘bit value
                    if pipe(PIPE_DEPTH-1).z > PI_Q then
                        angle_v := PI_Q;
                    elsif pipe(PIPE_DEPTH-1).z < PI_NEG_Q then
                        angle_v := PI_NEG_Q;
                    else
                        angle_v := pipe(PIPE_DEPTH-1).z;
                    end if;
                    done_v := '1';
                end if;

                -- -----------------------------------------------------------------
                -- Stage 0: New input acceptance (always possible after propagation)
                -- -----------------------------------------------------------------
                if start = '1' and is_valid(x, y) then   -- guarded has_x
                    -- Widen to 66 bits first to avoid overflow during quadrant mapping
                    x_norm_int := resize(x, 66);
                    y_norm_int := resize(y, 66);

                    -- Quadrant normalization on 66-bit values
                    -- Map any (x,y) to first quadrant (x>=0, y>=0) and record original quadrant
                    if x_norm_int > 0 and y_norm_int >= 0 then          -- Quadrant I
                        -- x and y already non-negative
                        quad := "00";
                    elsif x_norm_int <= 0 and y_norm_int > 0 then       -- Quadrant II
                        x_norm_int := -x_norm_int;   -- x becomes positive
                        quad := "01";
                    elsif x_norm_int < 0 and y_norm_int <= 0 then       -- Quadrant III
                        x_norm_int := -x_norm_int;
                        y_norm_int := -y_norm_int;
                        quad := "10";
                    else                                                -- Quadrant IV (x > 0, y < 0)
                        y_norm_int := -y_norm_int;   -- y becomes positive
                        quad := "11";
                    end if;

                    -- Check zero conditions for bypass
                    y_zero_flag := (y_norm_int = 0);
                    x_zero_flag := (x_norm_int = 0);

                    if y_zero_flag then
                        -- y = 0 (point lies on xâ€‘axis): set angle directly
                        if x = 0 and y = 0 then
                            -- (0,0) is undefined; return 0
                            pipe_v(0).z := (others => '0');
                        elsif x > 0 then
                            pipe_v(0).z := (others => '0');
                        else
                            pipe_v(0).z := PI_Q;   -- x < 0  -> angle = د€
                        end if;
                        pipe_v(0).valid    := '1';
                        pipe_v(0).bypass   := '1';
                        pipe_v(0).x        := x_norm_int;
                        pipe_v(0).y        := y_norm_int;
                        pipe_v(0).quadrant := quad;

                    elsif x_zero_flag then
                        -- x = 0 (point lies on yâ€‘axis): set angle directly to آ±د€/2
                        if quad = "01" then          -- original y > 0
                            pipe_v(0).z := PI_2_Q;
                        else                          -- quad = "11" (original y < 0)
                            pipe_v(0).z := -PI_2_Q;
                        end if;
                        pipe_v(0).valid    := '1';
                        pipe_v(0).bypass   := '1';
                        pipe_v(0).x        := x_norm_int;
                        pipe_v(0).y        := y_norm_int;
                        pipe_v(0).quadrant := quad;

                    else
                        -- Normal case: start CORDIC iterations
                        pipe_v(0).valid    := '1';
                        pipe_v(0).bypass   := '0';
                        pipe_v(0).x        := x_norm_int;
                        pipe_v(0).y        := y_norm_int;
                        pipe_v(0).z        := (others => '0');
                        pipe_v(0).quadrant := quad;
                    end if;
                end if;

                -- -----------------------------------------------------------------
                -- Compute busy flag from next pipeline state
                -- -----------------------------------------------------------------
                busy_v := '0';
                for i in 0 to PIPE_DEPTH-1 loop
                    if pipe_v(i).valid = '1' then
                        busy_v := '1';
                        exit;
                    end if;
                end loop;

                -- -----------------------------------------------------------------
                -- Update registers
                -- -----------------------------------------------------------------
                pipe    <= pipe_v;
                busy_r  <= busy_v;
                done_r  <= done_v;
                angle_r <= angle_v;

            end if;   -- end of rst/abort handling
        end if;       -- end of rising_edge
    end process;

end architecture rtl;
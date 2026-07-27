-- cordic_sincos.vhd - 18-stage CORDIC rotation mode pipeline computing Q32.32 sine and cosine
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
use work.fixed_pkg.all;               -- for q32_32 types and has_x functions

entity cordic_sincos is
    port(
        clk       : in  std_logic;
        rst       : in  std_logic;
        start     : in  std_logic;
        busy      : out std_logic;
        done      : out std_logic;
        angle     : in  q32_32;
        sin_out   : out q32_32;
        cos_out   : out q32_32;
        abort     : in  std_logic;
        timeout   : out std_logic
    );
end entity;

architecture rtl of cordic_sincos is

    constant ITER        : integer := 32;
    constant QUEUE_DEPTH : integer := 4;
    constant PIPE_DEPTH  : integer := ITER + 3;   -- 35 stages (0 to 34)

    -- Constants in Q32.32
    constant PI_Q       : q32_32 := signed'(x"00000003243F6A89");      -- pi * 2^32
    constant HALF_PI_Q   : q32_32 := signed'(x"00000001921FB545");      -- pi/2 * 2^32
    constant TWO_PI_Q    : q32_32 := signed'(x"00000006487ED511");      -- 2π * 2^32 (rounded)

    -- CORDIC gain (K = 1.64676, 1/K = 0.60725)
    constant INV_GAIN    : q32_32 := signed'(x"000000009B74EDA8");  -- 0.607252935 * 2^32
    constant CORDIC_GAIN : q32_32 := signed'(x"00000001A5922B18");   -- 1.646760258 * 2^32 (not used in final output, but kept for reference)
    constant ONE_Q       : q32_32 := signed'(x"0000000100000000");   -- 2**32 (1.0 in Q32.32)

    -- Arctan Table (all entries Q32.32) - Extended to 32 iterations for full Q32.32 precision
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

    type angle_reduce_t is record
        angle    : q32_32;
        quadrant : unsigned(1 downto 0);
    end record;

    type angle_queue_t is array(0 to QUEUE_DEPTH-1) of q32_32;

    type pipe_stage_t is record
        valid    : std_logic;
        x        : q32_32_int;
        y        : q32_32_int;
        z        : q32_32;
        quadrant : unsigned(1 downto 0);
    end record;

    type pipe_array_t is array(0 to PIPE_DEPTH-1) of pipe_stage_t;

    constant PIPE_DEFAULT : pipe_stage_t := (
        valid    => '0',
        x        => (others => '0'),
        y        => (others => '0'),
        z        => (others => '0'),
        quadrant => "00"
    );

    signal pipe : pipe_array_t := (others => PIPE_DEFAULT);

    signal busy_r      : std_logic := '0';
    signal done_r      : std_logic := '0';
    signal sin_r       : q32_32 := (others => '0');
    signal cos_r       : q32_32 := (others => '0');

    -- Abort synchronizer
    signal abort_meta : std_logic := '0';
    signal abort_sync : std_logic := '0';

    -- Main state machine
    type main_state_t is (
        ST_IDLE,
        ST_MODULO_START,
        ST_MODULO_WAIT,
        ST_CORDIC_RUNNING,
        ST_DRAIN
    );
    signal main_state : main_state_t := ST_IDLE;

    -- Queue management
    signal angle_queue : angle_queue_t := (others => (others => '0'));
    signal queue_wr_ptr : integer range 0 to QUEUE_DEPTH-1 := 0;
    signal queue_rd_ptr : integer range 0 to QUEUE_DEPTH-1 := 0;
    signal queue_count  : integer range 0 to QUEUE_DEPTH := 0;

    -- Modulo instance signals
    signal mod_start    : std_logic := '0';
    signal mod_busy     : std_logic;
    signal mod_done     : std_logic;
    signal mod_overflow : std_logic;
    signal mod_angle    : q32_32;
    signal angle_to_mod : signed(63 downto 0);

    -- Stored angle for processing
    signal current_angle : q32_32 := (others => '0');

    -- Start edge detection
    signal start_d      : std_logic := '0';
    signal start_pulse  : std_logic;

    -- X detection (simulation only, guarded for synthesis)
    function has_x(s : signed) return boolean is
    begin
        -- synthesis translate_off
        for i in s'range loop
            if s(i) = 'X' or s(i) = 'U' then
                return true;
            end if;
        end loop;
        return false;
        -- synthesis translate_on
        return false;   -- synthesis will see only this line
    end function;

    -- Safe quadrant reduction
    function get_quadrant(a : q32_32) return angle_reduce_t is
        variable r : angle_reduce_t;
        variable a_pos : q32_32;
    begin
        if a < 0 then
            a_pos := a + TWO_PI_Q;
            if a_pos < 0 then
                a_pos := (others => '0');
            end if;
        else
            a_pos := a;
        end if;

        if a_pos <= HALF_PI_Q then
            r.angle := a_pos;
            r.quadrant := "00";
        elsif a_pos <= PI_Q then
            r.angle := PI_Q - a_pos;
            r.quadrant := "01";
        elsif a_pos <= PI_Q + HALF_PI_Q then
            r.angle := a_pos - PI_Q;
            r.quadrant := "10";
        else
            r.angle := TWO_PI_Q - a_pos;
            r.quadrant := "11";
        end if;
        return r;
    end function;

    -- Safe queue pointer increment
    function inc_ptr(ptr : integer; max : integer) return integer is
        variable result : integer range 0 to max-1;
    begin
        if ptr = max-1 then
            result := 0;
        else
            result := ptr + 1;
        end if;
        return result;
    end function;

    -- Reset signal for modulo (combines top‑level rst and abort)
    signal mod_rst : std_logic;

    component fixed_mod_pipe is
        generic(
            MODULUS       : q32_32 := signed'(x"00000006487ED511")   -- 2π in Q32.32
        );
        port(
            clk          : in  std_logic;
            rst          : in  std_logic;
            start        : in  std_logic;
            busy         : out std_logic;
            done         : out std_logic;
            angle_in     : in  q32_32;
            angle_out    : out q32_32;
            modulo_q     : out q32_32;
            overflow     : out std_logic;
            timeout      : out std_logic;
            abort        : in  std_logic
        );
    end component;
begin

    timeout <= '0';   -- Stub (kept per Ahmed's instruction)
    busy    <= busy_r;
    done    <= done_r;
    sin_out <= sin_r;
    cos_out <= cos_r;

    start_pulse <= start and not start_d;
    angle_to_mod <= resize(current_angle, 64);
    mod_rst <= rst or abort_sync;   -- ensure modulo resets on abort

    -- Modulo instance (now reset together with top‑level on abort)
    mod_inst: entity work.fixed_mod_pipe
        generic map(
            MODULUS  => TWO_PI_Q
        )
        port map(
            clk       => clk,
            rst       => mod_rst,                -- combined reset
            start     => mod_start,
            busy      => mod_busy,
            done      => mod_done,
            angle_in  => angle_to_mod,
            angle_out => mod_angle,
            modulo_q  => open,
            overflow  => mod_overflow,
            timeout   => open,
            abort     => abort_sync              -- still connected, but reset already covers it
        );

    process(clk)
        variable pipe_v        : pipe_array_t;
        variable reduced       : angle_reduce_t;
        variable queue_wr_next, queue_rd_next : integer range 0 to QUEUE_DEPTH-1;
        variable queue_count_next : integer range 0 to QUEUE_DEPTH;
        variable x_rot, y_rot   : q32_32_int;
        variable z_rot          : q32_32;
        variable done_v         : std_logic;
        variable all_idle       : boolean;
        -- Queue operation flags (for simultaneous read/write handling)
        variable write_en       : boolean;
        variable read_en        : boolean;
    begin
        if rising_edge(clk) then
            -- -----------------------------------------------------------------
            -- Synchronize abort (two‑flop synchronizer)
            -- -----------------------------------------------------------------
            abort_meta <= abort;
            abort_sync <= abort_meta;

            if rst = '1' or abort_sync = '1' then
                -- Reset all signals
                for i in 0 to PIPE_DEPTH-1 loop
                    pipe(i) <= PIPE_DEFAULT;
                end loop;
                busy_r     <= '0';
                done_r     <= '0';
                sin_r      <= (others => '0');
                cos_r      <= (others => '0');

                for i in 0 to QUEUE_DEPTH-1 loop
                    angle_queue(i) <= (others => '0');
                end loop;
                queue_wr_ptr <= 0;
                queue_rd_ptr <= 0;
                queue_count  <= 0;

                mod_start   <= '0';
                main_state  <= ST_IDLE;
                start_d     <= '0';
                current_angle <= (others => '0');
            else
                -- -----------------------------------------------------------------
                -- Step 1: Determine if pipeline is completely idle (all stages invalid)
                -- -----------------------------------------------------------------
                all_idle := true;
                for i in 0 to PIPE_DEPTH-1 loop
                    if pipe(i).valid = '1' then
                        all_idle := false;
                        exit;
                    end if;
                end loop;

                -- -----------------------------------------------------------------
                -- Step 2: Initialize next pipeline state (all invalid)
                -- -----------------------------------------------------------------
                pipe_v := (others => PIPE_DEFAULT);

                -- Default values
                start_d <= start;
                queue_wr_next := queue_wr_ptr;
                queue_rd_next := queue_rd_ptr;
                queue_count_next := queue_count;
                done_v := '0';

                -- -----------------------------------------------------------------
                -- Step 3: Propagate existing pipeline stages (shift right)
                -- -----------------------------------------------------------------
                for i in 0 to PIPE_DEPTH-2 loop
                    if pipe(i).valid = '1' then
                        if i = 0 then
                            -- First rotation stage (shift 0)
                            if has_x(pipe(i).x) or has_x(pipe(i).y) then
                                pipe_v(1).valid := '1';
                                pipe_v(1).x     := (others => '0');
                                pipe_v(1).y     := (others => '0');
                                pipe_v(1).z     := pipe(i).z;
                                pipe_v(1).quadrant := pipe(i).quadrant;
                            else
                                if pipe(i).z >= 0 then
                                    x_rot := pipe(i).x - shift_right(pipe(i).y, 0);
                                    y_rot := pipe(i).y + shift_right(pipe(i).x, 0);
                                    z_rot := pipe(i).z - ATAN_TABLE(0);
                                else
                                    x_rot := pipe(i).x + shift_right(pipe(i).y, 0);
                                    y_rot := pipe(i).y - shift_right(pipe(i).x, 0);
                                    z_rot := pipe(i).z + ATAN_TABLE(0);
                                end if;
                                pipe_v(1).x := x_rot;
                                pipe_v(1).y := y_rot;
                                pipe_v(1).z := z_rot;
                                pipe_v(1).quadrant := pipe(i).quadrant;
                                pipe_v(1).valid := '1';
                            end if;
                        elsif i >= 1 and i <= ITER-1 then
                            -- Subsequent rotation stages (shift i)
                            if has_x(pipe(i).x) or has_x(pipe(i).y) then
                                pipe_v(i+1).valid := '1';
                                pipe_v(i+1).x     := (others => '0');
                                pipe_v(i+1).y     := (others => '0');
                                pipe_v(i+1).z     := pipe(i).z;
                                pipe_v(i+1).quadrant := pipe(i).quadrant;
                            else
                                if pipe(i).z >= 0 then
                                    x_rot := pipe(i).x - shift_right(pipe(i).y, i);
                                    y_rot := pipe(i).y + shift_right(pipe(i).x, i);
                                    z_rot := pipe(i).z - ATAN_TABLE(i);
                                else
                                    x_rot := pipe(i).x + shift_right(pipe(i).y, i);
                                    y_rot := pipe(i).y - shift_right(pipe(i).x, i);
                                    z_rot := pipe(i).z + ATAN_TABLE(i);
                                end if;
                                pipe_v(i+1).x := x_rot;
                                pipe_v(i+1).y := y_rot;
                                pipe_v(i+1).z := z_rot;
                                pipe_v(i+1).quadrant := pipe(i).quadrant;
                                pipe_v(i+1).valid := '1';
                            end if;
                        elsif i = ITER then
                            -- Quadrant correction stage
                            if has_x(pipe(i).x) or has_x(pipe(i).y) then
                                pipe_v(i+1).x := (others => '0');
                                pipe_v(i+1).y := (others => '0');
                            else
                                case pipe(i).quadrant is
                                    when "00" =>  -- Quadrant I
                                        pipe_v(i+1).x :=  pipe(i).x;
                                        pipe_v(i+1).y :=  pipe(i).y;
                                    when "01" =>  -- Quadrant II
                                        pipe_v(i+1).x := -pipe(i).x;
                                        pipe_v(i+1).y :=  pipe(i).y;
                                    when "10" =>  -- Quadrant III
                                        pipe_v(i+1).x := -pipe(i).x;
                                        pipe_v(i+1).y := -pipe(i).y;
                                    when others => -- Quadrant IV
                                        pipe_v(i+1).x :=  pipe(i).x;
                                        pipe_v(i+1).y := -pipe(i).y;
                                end case;
                            end if;
                            pipe_v(i+1).z := pipe(i).z;
                            pipe_v(i+1).quadrant := pipe(i).quadrant;
                            pipe_v(i+1).valid := '1';
                        elsif i = ITER+1 then
                            -- Final pipeline register (pass-through)
                            pipe_v(i+1) := pipe(i);
                            pipe_v(i+1).valid := '1';
                        else
                            -- Should not happen, but for safety copy
                            pipe_v(i+1) := pipe(i);
                            pipe_v(i+1).valid := '1';
                        end if;
                    end if;
                end loop;

                -- -----------------------------------------------------------------
                -- Step 4: Handle output from last stage (index PIPE_DEPTH-1)
                -- -----------------------------------------------------------------
                if pipe(PIPE_DEPTH-1).valid = '1' then
                    if has_x(pipe(PIPE_DEPTH-1).x) or has_x(pipe(PIPE_DEPTH-1).y) then
                        cos_r <= (others => '0');
                        sin_r <= (others => '0');
                    else
                        -- FIX #1: Output pipeline values directly (no multiplication by CORDIC_GAIN)
                        sin_r <= resize(pipe(PIPE_DEPTH-1).y, 64);
                        cos_r <= resize(pipe(PIPE_DEPTH-1).x, 64);
                    end if;
                    done_v := '1';
                end if;

                -- -----------------------------------------------------------------
                -- Step 5: Queue management (write new angles) - FIXED for simultaneous RW
                -- -----------------------------------------------------------------
                -- FIX #4: Include has_x check in write enable to keep pointers consistent
                write_en := (start_pulse = '1') and (queue_count < QUEUE_DEPTH) and not has_x(angle);
                read_en  := (main_state = ST_IDLE) and (queue_count > 0) and all_idle;

                if write_en then
                    angle_queue(queue_wr_ptr) <= angle;
                    queue_wr_next := inc_ptr(queue_wr_ptr, QUEUE_DEPTH);
                end if;

                if read_en then
                    queue_rd_next := inc_ptr(queue_rd_ptr, QUEUE_DEPTH);
                end if;

                -- Update queue count: net change = write? +1 : 0 minus read? 1 : 0
                queue_count_next := queue_count;
                if write_en then
                    queue_count_next := queue_count_next + 1;
                end if;
                if read_en then
                    queue_count_next := queue_count_next - 1;
                end if;

                -- -----------------------------------------------------------------
                -- Step 6: Main state machine (handles modulo and new input)
                -- -----------------------------------------------------------------
                mod_start <= '0';  -- default

                case main_state is
                    when ST_IDLE =>
                        if read_en then
                            -- Get next angle from queue
                            current_angle <= angle_queue(queue_rd_ptr);
                            main_state <= ST_MODULO_START;
                        end if;

                    when ST_MODULO_START =>
                        -- Start modulo only if it's ready
                        if mod_busy = '0' then
                            mod_start <= '1';
                            main_state <= ST_MODULO_WAIT;
                        end if;

                    when ST_MODULO_WAIT =>
                        if mod_done = '1' then
                            -- Process modulo result into pipeline
                            if mod_overflow = '1' or has_x(mod_angle) then
                                -- Invalid angle: push zero result directly to output stage?
                                pipe_v(ITER).valid    := '1';
                                pipe_v(ITER).x        := (others => '0');
                                pipe_v(ITER).y        := (others => '0');
                                pipe_v(ITER).z        := (others => '0');
                                pipe_v(ITER).quadrant := "00";
                            else
                                reduced := get_quadrant(mod_angle);
                                -- SPECIAL CASE: If reduced angle is zero, bypass CORDIC rotation
                                if reduced.angle = 0 then
                                    -- FIX #3: Use ONE_Q (2^32) for cos=1, sin=0
                                    pipe_v(ITER).valid    := '1';
                                    pipe_v(ITER).x        := resize(ONE_Q, 66);  -- cos(0) = 1.0 in Q32.32
                                    pipe_v(ITER).y        := (others => '0');
                                    pipe_v(ITER).z        := (others => '0');
                                    pipe_v(ITER).quadrant := reduced.quadrant;
                                else
                                    -- Normal case: load into first pipeline stage
                                    pipe_v(0).valid    := '1';
                                    pipe_v(0).x        := resize(INV_GAIN, 66);
                                    pipe_v(0).y        := to_signed(0, 66);
                                    pipe_v(0).z        := reduced.angle;
                                    pipe_v(0).quadrant := reduced.quadrant;
                                end if;
                            end if;
                            main_state <= ST_CORDIC_RUNNING;
                        end if;

                    when ST_CORDIC_RUNNING =>
                        -- Wait for output
                        if done_v = '1' then
                            main_state <= ST_DRAIN;   -- Result taken, now drain pipeline
                        end if;

                    when ST_DRAIN =>
                        -- Wait until pipeline is completely empty
                        if all_idle then
                            main_state <= ST_IDLE;
                        end if;

                    when others =>
                        main_state <= ST_IDLE;
                end case;

                -- -----------------------------------------------------------------
                -- Step 7: Compute busy flag from next pipeline state and state machine
                -- -----------------------------------------------------------------
                busy_r <= '0';
                for i in 0 to PIPE_DEPTH-1 loop
                    if pipe_v(i).valid = '1' then
                        busy_r <= '1';
                        exit;
                    end if;
                end loop;

                if main_state /= ST_IDLE then
                    busy_r <= '1';  -- Modulo or draining, so we're busy
                end if;

                -- -----------------------------------------------------------------
                -- Step 8: Update registers
                -- -----------------------------------------------------------------
                pipe           <= pipe_v;
                done_r         <= done_v;
                queue_wr_ptr   <= queue_wr_next;
                queue_rd_ptr   <= queue_rd_next;
                queue_count    <= queue_count_next;

            end if;
        end if;
    end process;

end architecture rtl;
-- ============================================================================
-- Bit Timing – TQ generation, sample point, hard/soft resync,
-- with strict range validation per CAN spec (all segments 1..8, total 8..25).
-- BRP=0 handled by adjusting PHS1+1 and PHS2-1, with bounds enforcement.
-- ============================================================================
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
use work.can_pkg.all;

entity bit_timing is
    port (
        clk          : in  std_logic;
        reset_n      : in  std_logic;
        enable       : in  std_logic;
        rx           : in  std_logic;
        can_bt1      : in  std_logic_vector(7 downto 0);
        can_bt2      : in  std_logic_vector(7 downto 0);
        can_bt3      : in  std_logic_vector(7 downto 0);
        tq_clk       : out std_logic;
        sample_pt    : out std_logic;
        bit_start    : out std_logic;
        hard_sync    : out std_logic;
        resync       : out std_logic;
        sample_value : out std_logic
    );
end bit_timing;

architecture rtl of bit_timing is
    signal brp, prs, phs1, phs2, sjw : integer range 0 to 31;
    signal tq_cnt, phase, seg_cnt : integer range 0 to 31;
    signal sync_adj : integer range -4 to 4;
    signal sample_pos, bit_len : integer range 8 to 25;
    signal use_three_samples : boolean;
    signal sample_regs : std_logic_vector(2 downto 0);
    signal last_rx, edge_detected : std_logic;
    signal tq_pulse, sample_pulse, start_pulse : std_logic;
    signal sample_val_int : std_logic;
begin
    -- Decode timing parameters with strict validation
    process(can_bt1, can_bt2, can_bt3)
        variable brp_tmp, prs_tmp, phs1_tmp, phs2_tmp, sjw_tmp : integer;
        variable bit_len_tmp : integer;
        variable adj_done : boolean;
    begin
        brp_tmp := to_integer(unsigned(can_bt1(6 downto 1)));
        prs_tmp := to_integer(unsigned(can_bt2(3 downto 1))) + 1;  -- 1..8
        phs1_tmp := to_integer(unsigned(can_bt3(3 downto 1))) + 1; -- 1..8
        phs2_tmp := to_integer(unsigned(can_bt3(6 downto 4))) + 1; -- 1..8
        sjw_tmp := to_integer(unsigned(can_bt2(6 downto 5))) + 1;  -- 1..4

        -- BRP=0 special case: add 1 to PHS1, subtract 1 from PHS2
        if brp_tmp = 0 then
            if phs1_tmp < 8 then phs1_tmp := phs1_tmp + 1; end if;
            if phs2_tmp > 1 then phs2_tmp := phs2_tmp - 1; end if;
        end if;

        -- Apply constraints: PHS2 must be between 1 and PHS1, and at least 2 (IPT=2 TQ)
        if phs2_tmp > phs1_tmp then phs2_tmp := phs1_tmp; end if;
        if phs2_tmp < 2 then phs2_tmp := 2; end if; -- IPT = 2 TQ
        if phs1_tmp < 2 then phs1_tmp := 2; end if; -- ensure PHS1 at least IPT

        -- SJW cannot exceed 4 and cannot exceed PHS1
        if sjw_tmp > 4 then sjw_tmp := 4; end if;
        if sjw_tmp > phs1_tmp then sjw_tmp := phs1_tmp; end if;

        -- Total TQ must be 8..25; adjust if necessary, but keep segments within 1..8 and PHS2 >=2
        bit_len_tmp := 1 + prs_tmp + phs1_tmp + phs2_tmp;
        adj_done := false;
        if bit_len_tmp < 8 then
            -- Increase PRS first (up to 8)
            while bit_len_tmp < 8 and prs_tmp < 8 loop
                prs_tmp := prs_tmp + 1;
                bit_len_tmp := 1 + prs_tmp + phs1_tmp + phs2_tmp;
            end loop;
            if bit_len_tmp < 8 then
                -- Increase PHS1 (up to 8)
                while bit_len_tmp < 8 and phs1_tmp < 8 loop
                    phs1_tmp := phs1_tmp + 1;
                    bit_len_tmp := 1 + prs_tmp + phs1_tmp + phs2_tmp;
                end loop;
            end if;
            if bit_len_tmp < 8 then
                -- Increase PHS2 (but keep <= PHS1)
                while bit_len_tmp < 8 and phs2_tmp < phs1_tmp and phs2_tmp < 8 loop
                    phs2_tmp := phs2_tmp + 1;
                    bit_len_tmp := 1 + prs_tmp + phs1_tmp + phs2_tmp;
                end loop;
            end if;
        elsif bit_len_tmp > 25 then
            -- Decrease PHS2 first (down to 2)
            while bit_len_tmp > 25 and phs2_tmp > 2 loop
                phs2_tmp := phs2_tmp - 1;
                bit_len_tmp := 1 + prs_tmp + phs1_tmp + phs2_tmp;
            end loop;
            if bit_len_tmp > 25 then
                -- Decrease PHS1 (down to 2)
                while bit_len_tmp > 25 and phs1_tmp > 2 loop
                    phs1_tmp := phs1_tmp - 1;
                    bit_len_tmp := 1 + prs_tmp + phs1_tmp + phs2_tmp;
                end loop;
            end if;
            if bit_len_tmp > 25 then
                -- Decrease PRS (down to 1)
                while bit_len_tmp > 25 and prs_tmp > 1 loop
                    prs_tmp := prs_tmp - 1;
                    bit_len_tmp := 1 + prs_tmp + phs1_tmp + phs2_tmp;
                end loop;
            end if;
        end if;

        -- Final safety checks
        if prs_tmp < 1 then prs_tmp := 1; elsif prs_tmp > 8 then prs_tmp := 8; end if;
        if phs1_tmp < 1 then phs1_tmp := 1; elsif phs1_tmp > 8 then phs1_tmp := 8; end if;
        if phs2_tmp < 2 then phs2_tmp := 2; elsif phs2_tmp > phs1_tmp then phs2_tmp := phs1_tmp; end if;
        if phs2_tmp > 8 then phs2_tmp := 8; end if;

        brp <= brp_tmp;
        prs <= prs_tmp;
        phs1 <= phs1_tmp;
        phs2 <= phs2_tmp;
        sjw <= sjw_tmp;
        bit_len <= 1 + prs_tmp + phs1_tmp + phs2_tmp;
        sample_pos <= 1 + prs_tmp + phs1_tmp; -- position of sample point (0-based TQ index)
        use_three_samples <= (can_bt3(0) = '1' and brp_tmp /= 0);
    end process;

    -- Main timing FSM
    process(clk, reset_n)
        variable tq_cnt_v, phase_v, seg_cnt_v : integer range 0 to 31;
        variable sync_adj_v : integer range -4 to 4;
    begin
        if reset_n = '0' then
            tq_cnt_v := 0; phase_v := 0; seg_cnt_v := 0; sync_adj_v := 0;
            tq_pulse <= '0'; sample_pulse <= '0'; start_pulse <= '0';
            last_rx <= '1'; sample_regs <= "111"; sample_val_int <= '1';
        elsif rising_edge(clk) then
            tq_pulse <= '0'; sample_pulse <= '0'; start_pulse <= '0'; hard_sync <= '0'; resync <= '0';
            if enable = '1' then
                -- Edge detection
                edge_detected <= '0';
                if rx = '0' and last_rx = '1' then edge_detected <= '1'; end if;
                last_rx <= rx;

                if tq_cnt_v < brp then
                    tq_cnt_v := tq_cnt_v + 1;
                else
                    tq_cnt_v := 0;
                    tq_pulse <= '1';

                    -- Hard sync: edge at SYNC_SEG start
                    if edge_detected = '1' and phase_v = 0 and seg_cnt_v = 0 then
                        phase_v := 0; seg_cnt_v := 0; sync_adj_v := 0;
                        start_pulse <= '1'; hard_sync <= '1';
                    else
                        -- Soft resync
                        if edge_detected = '1' and not (phase_v = 0 and seg_cnt_v = 0) then
                            if phase_v = 2 then
                                sync_adj_v := sjw;
                                if sync_adj_v > phs1-1 then sync_adj_v := phs1-1; end if;
                            elsif phase_v = 3 then
                                sync_adj_v := -sjw;
                                if -sync_adj_v > phs2-2 then sync_adj_v := -(phs2-2); end if;
                            else
                                sync_adj_v := 0;
                            end if;
                            resync <= '1';
                        end if;

                        -- Advance phase
                        case phase_v is
                            when 0 => -- SYNC (1 TQ)
                                if seg_cnt_v = 0 then seg_cnt_v := 1;
                                else phase_v := 1; seg_cnt_v := 0; end if;
                            when 1 => -- PROP
                                if seg_cnt_v < prs-1 then seg_cnt_v := seg_cnt_v + 1;
                                else phase_v := 2; seg_cnt_v := 0; end if;
                            when 2 => -- PHS1 (lengthen)
                                if seg_cnt_v < phs1-1 + sync_adj_v then
                                    seg_cnt_v := seg_cnt_v + 1;
                                else
                                    sample_pulse <= '1';
                                    -- Sample
                                    if use_three_samples then
                                        sample_regs(2) <= sample_regs(1);
                                        sample_regs(1) <= sample_regs(0);
                                        sample_regs(0) <= rx;
                                        sample_val_int <= (sample_regs(0) and sample_regs(1)) or
                                                          (sample_regs(0) and sample_regs(2)) or
                                                          (sample_regs(1) and sample_regs(2));
                                    else
                                        sample_val_int <= rx;
                                    end if;
                                    phase_v := 3; seg_cnt_v := 0; sync_adj_v := 0;
                                end if;
                            when 3 => -- PHS2 (shorten)
                                if seg_cnt_v < phs2-1 + sync_adj_v then
                                    seg_cnt_v := seg_cnt_v + 1;
                                else
                                    phase_v := 0; seg_cnt_v := 0; sync_adj_v := 0;
                                    start_pulse <= '1';
                                end if;
                            when others => null;
                        end case;
                    end if;
                end if;
            else
                -- enable = 0: reset timing
                tq_cnt_v := 0; phase_v := 0; seg_cnt_v := 0; sync_adj_v := 0;
            end if;
        end if;
        tq_clk <= tq_pulse;
        sample_pt <= sample_pulse;
        bit_start <= start_pulse;
        sample_value <= sample_val_int;
    end process;
end rtl;

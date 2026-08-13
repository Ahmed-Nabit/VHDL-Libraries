-- ============================================================================
-- CAN Package - Constants, types, and helper functions
-- ============================================================================
-- Constraints respected:
--   * No Natural type
--   * No Real type
--   * No concurrent signal assignment inside modules
-- ============================================================================
-- Copyright © 2024-2026 Ahmed Nabit [Lazrdo@gmail.com](mailto:Lazrdo@gmail.com)
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at:
--     http://www.apache.org/licenses/LICENSE-2.0
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.
-- ============================================================================

library ieee;
use ieee.std_logic_1164.all;

package can_pkg is

    -- ========================================================================
    -- Basic CAN controller constants
    -- ========================================================================
    constant MOB_COUNT : integer := 6;

    subtype mob_idx is integer range 0 to MOB_COUNT - 1;

    -- ========================================================================
    -- MOb configuration modes - CANCDMOB.CONMOB[1:0]
    -- ========================================================================
    constant MOB_DISABLED : std_logic_vector(1 downto 0) := "00";
    constant MOB_TX       : std_logic_vector(1 downto 0) := "01";
    constant MOB_RX       : std_logic_vector(1 downto 0) := "10";
    constant MOB_FBUF_RX  : std_logic_vector(1 downto 0) := "11";

    -- ========================================================================
    -- Register addresses
    -- ========================================================================
    -- The PDF register organization lists the low MOb registers first:
    --   Enable MOb 2, Enable MOb 1
    --   Enable Interrupt MOb 2, Enable Interrupt MOb 1
    --   Status Interrupt MOb 2, Status Interrupt MOb 1
    -- Therefore the addresses below use CANEN2/CANIE2/CANSIT2 as the low
    -- registers.
    -- ========================================================================
    subtype reg_addr is std_logic_vector(5 downto 0);

    constant ADDR_CANGCON   : reg_addr := "000000";
    constant ADDR_CANGSTA   : reg_addr := "000001";
    constant ADDR_CANGIT    : reg_addr := "000010";
    constant ADDR_CANGIE    : reg_addr := "000011";

    constant ADDR_CANEN2    : reg_addr := "000100";
    constant ADDR_CANEN1    : reg_addr := "000101";

    constant ADDR_CANIE2    : reg_addr := "000110";
    constant ADDR_CANIE1    : reg_addr := "000111";

    constant ADDR_CANSIT2   : reg_addr := "001000";
    constant ADDR_CANSIT1   : reg_addr := "001001";

    constant ADDR_CANBT1    : reg_addr := "001010";
    constant ADDR_CANBT2    : reg_addr := "001011";
    constant ADDR_CANBT3    : reg_addr := "001100";

    constant ADDR_CANTCON   : reg_addr := "001101";
    constant ADDR_CANTIML   : reg_addr := "001110";
    constant ADDR_CANTIMH   : reg_addr := "001111";
    constant ADDR_CANTTCL   : reg_addr := "010000";
    constant ADDR_CANTTCH   : reg_addr := "010001";

    constant ADDR_CANTEC    : reg_addr := "010010";
    constant ADDR_CANREC    : reg_addr := "010011";

    constant ADDR_CANHPMOB  : reg_addr := "010100";
    constant ADDR_CANPAGE   : reg_addr := "010101";

    constant ADDR_MOB_BASE  : reg_addr := "100000";

    -- ========================================================================
    -- MOb register offsets within the selected MOb page
    -- ========================================================================
    constant MOB_OFFS_STMOB : integer := 0;
    constant MOB_OFFS_CDMOB : integer := 1;
    constant MOB_OFFS_IDT1  : integer := 2;
    constant MOB_OFFS_IDT2  : integer := 3;
    constant MOB_OFFS_IDT3  : integer := 4;
    constant MOB_OFFS_IDT4  : integer := 5;
    constant MOB_OFFS_IDM1  : integer := 6;
    constant MOB_OFFS_IDM2  : integer := 7;
    constant MOB_OFFS_IDM3  : integer := 8;
    constant MOB_OFFS_IDM4  : integer := 9;
    constant MOB_OFFS_STML  : integer := 10;
    constant MOB_OFFS_STMH  : integer := 11;
    constant MOB_OFFS_MSG   : integer := 12;

    -- ========================================================================
    -- MOb register record
    -- ========================================================================
    type mob_reg_t is record
        stmob : std_logic_vector(7 downto 0);
        cdmob : std_logic_vector(7 downto 0);
        idt1  : std_logic_vector(7 downto 0);
        idt2  : std_logic_vector(7 downto 0);
        idt3  : std_logic_vector(7 downto 0);
        idt4  : std_logic_vector(7 downto 0);
        idm1  : std_logic_vector(7 downto 0);
        idm2  : std_logic_vector(7 downto 0);
        idm3  : std_logic_vector(7 downto 0);
        idm4  : std_logic_vector(7 downto 0);
        stml  : std_logic_vector(7 downto 0);
        stmh  : std_logic_vector(7 downto 0);
    end record;

    type mob_array_t is array (0 to MOB_COUNT - 1) of mob_reg_t;
    type data_array_t is array (0 to MOB_COUNT - 1) of std_logic_vector(63 downto 0);

    -- ========================================================================
    -- Internal MOb state
    -- ========================================================================
    type mob_state_t is (MOB_IDLE, MOB_TX_PENDING, MOB_RX_MATCHED, MOB_AUTO_REPLY, MOB_FBUF_WAIT);
    type mob_state_vec_t is array (0 to MOB_COUNT - 1) of mob_state_t;

    -- ========================================================================
    -- Bit timing decoded configuration
    -- ========================================================================
    type bt_cfg_t is record
        brp          : integer range 0 to 63;
        prs          : integer range 0 to 31;
        phs1         : integer range 0 to 31;
        phs2         : integer range 0 to 31;
        sjw          : integer range 0 to 31;
        bit_len      : integer range 0 to 31;
        sample_pos   : integer range 0 to 31;
        three_sample : boolean;
    end record;

    -- ========================================================================
    -- Maximum unstuffed frame length in bits
    -- ========================================================================
    -- Extended frame maximum:
    --   SOF          : 1
    --   Base ID      : 11
    --   SRR          : 1
    --   IDE          : 1
    --   Extended ID  : 18
    --   RTR          : 1
    --   r1, r0       : 2
    --   DLC          : 4
    --   Data         : 64
    --   CRC          : 15
    --   CRC delimiter: 1
    --   ACK slot     : 1
    --   ACK delimiter: 1
    --   EOF          : 7
    --   Total        : 128
    -- ========================================================================
    constant MAX_FRAME_BITS : integer := 128;

    -- ========================================================================
    -- Helper function declarations
    -- ========================================================================
    function slv_to_int(
        v : std_logic_vector
    ) return integer;

    function int_to_slv3(
        v : integer
    ) return std_logic_vector(2 downto 0);

    function int_to_slv4(
        v : integer
    ) return std_logic_vector(3 downto 0);

    function int_to_slv8(
        v : integer
    ) return std_logic_vector(7 downto 0);

    function int_to_slv16(
        v : integer
    ) return std_logic_vector(15 downto 0);

    function majority3(
        a : std_logic;
        b : std_logic;
        c : std_logic
    ) return std_logic;

    function reverse_15(
        v : std_logic_vector(14 downto 0)
    ) return std_logic_vector(14 downto 0);

    -- ========================================================================
    -- CAN CRC bit update
    -- Polynomial:
    --   x^15 + x^14 + x^10 + x^8 + x^7 + x^4 + x^3 + 1
    -- ========================================================================
    function crc_bit(
        crc_in : std_logic_vector(14 downto 0);
        data   : std_logic
    ) return std_logic_vector(14 downto 0);

    -- ========================================================================
    -- Decode CANBT1/CANBT2/CANBT3 into a safe timing configuration
    -- ========================================================================
    function decode_can_bt(
        can_bt1 : std_logic_vector(7 downto 0);
        can_bt2 : std_logic_vector(7 downto 0);
        can_bt3 : std_logic_vector(7 downto 0)
    ) return bt_cfg_t;

end can_pkg;

package body can_pkg is

    -- ========================================================================
    -- Convert std_logic_vector to integer.
    -- The leftmost bit of the vector is treated as the most significant bit.
    -- ========================================================================
    function slv_to_int(
        v : std_logic_vector
    ) return integer is
        variable result : integer range 0 to 65535;
    begin
        result := 0;
        for i in v'range loop
            result := result * 2;
            if v(i) = '1' then
                result := result + 1;
            end if;
        end loop;
        return result;
    end function;

    -- ========================================================================
    -- Convert integer to 3-bit std_logic_vector, LSB at bit 0.
    -- ========================================================================
    function int_to_slv3(
        v : integer
    ) return std_logic_vector(2 downto 0) is
        variable result : std_logic_vector(2 downto 0);
        variable tmp    : integer range 0 to 7;
    begin
        if v < 0 then
            tmp := 0;
        elsif v > 7 then
            tmp := 7;
        else
            tmp := v;
        end if;

        for i in 0 to 2 loop
            if (tmp mod 2) = 1 then
                result(i) := '1';
            else
                result(i) := '0';
            end if;
            tmp := tmp / 2;
        end loop;

        return result;
    end function;

    -- ========================================================================
    -- Convert integer to 4-bit std_logic_vector, LSB at bit 0.
    -- ========================================================================
    function int_to_slv4(
        v : integer
    ) return std_logic_vector(3 downto 0) is
        variable result : std_logic_vector(3 downto 0);
        variable tmp    : integer range 0 to 15;
    begin
        if v < 0 then
            tmp := 0;
        elsif v > 15 then
            tmp := 15;
        else
            tmp := v;
        end if;

        for i in 0 to 3 loop
            if (tmp mod 2) = 1 then
                result(i) := '1';
            else
                result(i) := '0';
            end if;
            tmp := tmp / 2;
        end loop;

        return result;
    end function;

    -- ========================================================================
    -- Convert integer to 8-bit std_logic_vector, LSB at bit 0.
    -- ========================================================================
    function int_to_slv8(
        v : integer
    ) return std_logic_vector(7 downto 0) is
        variable result : std_logic_vector(7 downto 0);
        variable tmp    : integer range 0 to 255;
    begin
        if v < 0 then
            tmp := 0;
        elsif v > 255 then
            tmp := 255;
        else
            tmp := v;
        end if;

        for i in 0 to 7 loop
            if (tmp mod 2) = 1 then
                result(i) := '1';
            else
                result(i) := '0';
            end if;
            tmp := tmp / 2;
        end loop;

        return result;
    end function;

    -- ========================================================================
    -- Convert integer to 16-bit std_logic_vector, LSB at bit 0.
    -- ========================================================================
    function int_to_slv16(
        v : integer
    ) return std_logic_vector(15 downto 0) is
        variable result : std_logic_vector(15 downto 0);
        variable tmp    : integer range 0 to 65535;
    begin
        if v < 0 then
            tmp := 0;
        elsif v > 65535 then
            tmp := 65535;
        else
            tmp := v;
        end if;

        for i in 0 to 15 loop
            if (tmp mod 2) = 1 then
                result(i) := '1';
            else
                result(i) := '0';
            end if;
            tmp := tmp / 2;
        end loop;

        return result;
    end function;

    -- ========================================================================
    -- Majority vote for three samples.
    -- ========================================================================
    function majority3(
        a : std_logic;
        b : std_logic;
        c : std_logic
    ) return std_logic is
    begin
        if (a and b) = '1' then
            return '1';
        elsif (a and c) = '1' then
            return '1';
        elsif (b and c) = '1' then
            return '1';
        else
            return '0';
        end if;
    end function;

    -- ========================================================================
    -- Reverse a 15-bit vector.
    -- Useful when comparing received CRC bit order against computed CRC.
    -- ========================================================================
    function reverse_15(
        v : std_logic_vector(14 downto 0)
    ) return std_logic_vector(14 downto 0) is
        variable result : std_logic_vector(14 downto 0);
    begin
        for i in 0 to 14 loop
            result(i) := v(14 - i);
        end loop;
        return result;
    end function;

    -- ========================================================================
    -- CAN CRC bit update.
    -- ========================================================================
    function crc_bit(
        crc_in : std_logic_vector(14 downto 0);
        data   : std_logic
    ) return std_logic_vector(14 downto 0) is
        variable nc : std_logic_vector(14 downto 0);
        variable fb : std_logic;
    begin
        fb := crc_in(14) xor data;

        nc(0)  := fb;
        nc(1)  := crc_in(0);
        nc(2)  := crc_in(1);
        nc(3)  := crc_in(2) xor fb;
        nc(4)  := crc_in(3) xor fb;
        nc(5)  := crc_in(4);
        nc(6)  := crc_in(5);
        nc(7)  := crc_in(6) xor fb;
        nc(8)  := crc_in(7) xor fb;
        nc(9)  := crc_in(8);
        nc(10) := crc_in(9) xor fb;
        nc(11) := crc_in(10);
        nc(12) := crc_in(11);
        nc(13) := crc_in(12);
        nc(14) := crc_in(13) xor fb;

        return nc;
    end function;

    -- ========================================================================
    -- Decode CANBT registers.
    --
    -- PDF rules applied:
    --   SYNC = 1 TQ
    --   PRS  = 1..8 TQ
    --   PHS1 = 1..8 TQ
    --   PHS2 actual minimum is 2 TQ because IPT = 2 TQ
    --   PHS2 <= PHS1
    --   SJW  = 1..min(4, PHS1)
    --   Total bit time = 8..25 TQ
    --   BRP = 0 compensation:
    --     lengthen PHS1 by one TQ and shorten PHS2 by one TQ when possible.
    --   SMP = 1 is not compatible with BRP = 0.
    -- ========================================================================
    function decode_can_bt(
        can_bt1 : std_logic_vector(7 downto 0);
        can_bt2 : std_logic_vector(7 downto 0);
        can_bt3 : std_logic_vector(7 downto 0)
    ) return bt_cfg_t is
        variable cfg     : bt_cfg_t;
        variable brp_v   : integer range 0 to 63;
        variable prs_v   : integer range 0 to 31;
        variable phs1_v  : integer range 0 to 31;
        variable phs2_v  : integer range 0 to 31;
        variable sjw_v   : integer range 0 to 31;
        variable total_v : integer range 0 to 63;
    begin
        brp_v  := slv_to_int(can_bt1(6 downto 1));
        prs_v  := slv_to_int(can_bt2(3 downto 1)) + 1;
        phs1_v := slv_to_int(can_bt3(3 downto 1)) + 1;
        phs2_v := slv_to_int(can_bt3(6 downto 4)) + 1;
        sjw_v  := slv_to_int(can_bt2(6 downto 5)) + 1;

        -- ---------------------------------------------------------------
        -- BRP = 0 compensation.
        -- ---------------------------------------------------------------
        if brp_v = 0 then
            if phs1_v < 8 then
                phs1_v := phs1_v + 1;
            end if;

            if phs2_v > 2 then
                phs2_v := phs2_v - 1;
            end if;
        end if;

        -- ---------------------------------------------------------------
        -- Segment range clamping.
        -- ---------------------------------------------------------------
        if prs_v < 1 then
            prs_v := 1;
        elsif prs_v > 8 then
            prs_v := 8;
        end if;

        if phs1_v < 1 then
            phs1_v := 1;
        elsif phs1_v > 8 then
            phs1_v := 8;
        end if;

        if phs2_v < 2 then
            phs2_v := 2;
        elsif phs2_v > 8 then
            phs2_v := 8;
        end if;

        if phs2_v > phs1_v then
            phs2_v := phs1_v;
        end if;

        if sjw_v < 1 then
            sjw_v := 1;
        elsif sjw_v > 4 then
            sjw_v := 4;
        end if;

        if sjw_v > phs1_v then
            sjw_v := phs1_v;
        end if;

        -- ---------------------------------------------------------------
        -- Total TQ enforcement: 8..25.
        -- This keeps the bit timing FSM legal even if software programs
        -- an invalid combination.
        -- ---------------------------------------------------------------
        total_v := 1 + prs_v + phs1_v + phs2_v;

        for adj_loop in 0 to 31 loop
            total_v := 1 + prs_v + phs1_v + phs2_v;

            if total_v < 8 then
                if prs_v < 8 then
                    prs_v := prs_v + 1;
                elsif phs1_v < 8 then
                    phs1_v := phs1_v + 1;
                elsif phs2_v < phs1_v and phs2_v < 8 then
                    phs2_v := phs2_v + 1;
                end if;
            elsif total_v > 25 then
                if phs2_v > 2 then
                    phs2_v := phs2_v - 1;
                elsif phs1_v > 2 then
                    phs1_v := phs1_v - 1;
                elsif prs_v > 1 then
                    prs_v := prs_v - 1;
                end if;
            else
                exit;
            end if;
        end loop;

        -- ---------------------------------------------------------------
        -- Final legality.
        -- ---------------------------------------------------------------
        if phs1_v < 2 then
            phs1_v := 2;
        end if;

        if phs2_v < 2 then
            phs2_v := 2;
        end if;

        if phs2_v > phs1_v then
            phs2_v := phs1_v;
        end if;

        if phs1_v < phs2_v then
            phs1_v := phs2_v;
        end if;

        if sjw_v < 1 then
            sjw_v := 1;
        end if;

        if sjw_v > 4 then
            sjw_v := 4;
        end if;

        if sjw_v > phs1_v then
            sjw_v := phs1_v;
        end if;

        cfg.brp          := brp_v;
        cfg.prs          := prs_v;
        cfg.phs1         := phs1_v;
        cfg.phs2         := phs2_v;
        cfg.sjw          := sjw_v;
        cfg.bit_len      := 1 + prs_v + phs1_v + phs2_v;
        cfg.sample_pos   := 1 + prs_v + phs1_v;
        cfg.three_sample := (can_bt3(0) = '1') and (brp_v /= 0);

        return cfg;
    end function;

end can_pkg;

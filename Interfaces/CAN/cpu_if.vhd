-- ============================================================================
-- CPU Interface
-- ============================================================================
-- Fixed behavior:
--   * Correct register decode using the corrected CANEN2/CANIE2/CANSIT2
--     low-register convention.
--   * MOb register accesses are routed to the coherent MOb manager.
--   * CANSTMOB write-1-clear is performed in the MOb manager, not in a
--     separated CPU-side shadow copy.
--   * CANMSG reads/writes use the MOb manager data buffer and support
--     CANPAGE.AINC auto-increment with roll-over from index 7 to 0.
--   * CANGIT write-1-clear is provided as a one-cycle clear pulse.
--   * CANHPMOB low CGP bits are writable, high HPMOB bits are read-only.
--   * Reserved register bits are masked where the PDF requires them to be
--     written as zero.
--
-- Design constraints respected:
--   * No Natural type
--   * No Real type
--   * No concurrent signal assignment
--   * State is kept in process variables
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
use work.can_pkg.all;

entity cpu_if is
    port (
        clk      : in  std_logic;
        reset_n  : in  std_logic;

        --------------------------------------------------------------------
        -- Simple CPU bus
        --------------------------------------------------------------------
        cs       : in  std_logic;
        wr       : in  std_logic;
        rd       : in  std_logic;
        addr     : in  std_logic_vector(5 downto 0);
        data_in  : in  std_logic_vector(7 downto 0);
        data_out : out std_logic_vector(7 downto 0);

        --------------------------------------------------------------------
        -- General CAN registers
        --------------------------------------------------------------------
        can_gcon_o      : out std_logic_vector(7 downto 0);
        can_gsta_i      : in  std_logic_vector(7 downto 0);

        can_git_clear_o : out std_logic_vector(7 downto 0);
        can_git_i       : in  std_logic_vector(7 downto 0);

        can_gie_o       : out std_logic_vector(7 downto 0);

        -- CANIE1 is the high register, CANIE2 is the low MOb register.
        can_ie1_o       : out std_logic_vector(7 downto 0);
        can_ie2_o       : out std_logic_vector(7 downto 0);

        -- Read-only MOb enable/status interrupt inputs.
        can_en1_i       : in  std_logic_vector(7 downto 0);
        can_en2_i       : in  std_logic_vector(7 downto 0);
        can_sit1_i      : in  std_logic_vector(7 downto 0);
        can_sit2_i      : in  std_logic_vector(7 downto 0);

        --------------------------------------------------------------------
        -- Bit timing / timer registers
        --------------------------------------------------------------------
        can_bt1_o  : out std_logic_vector(7 downto 0);
        can_bt2_o  : out std_logic_vector(7 downto 0);
        can_bt3_o  : out std_logic_vector(7 downto 0);

        can_tcon_o : out std_logic_vector(7 downto 0);

        cantim_i   : in  std_logic_vector(15 downto 0);
        canttc_i   : in  std_logic_vector(15 downto 0);

        cantec_i   : in  std_logic_vector(7 downto 0);
        canrec_i   : in  std_logic_vector(7 downto 0);

        --------------------------------------------------------------------
        -- CANHPMOB
        --
        -- can_hpmob_i(7:4) is supplied by the interrupt/status logic.
        -- can_hpmob_o returns the merged read value:
        --   high nibble: hardware read-only
        --   low nibble : CGP bits, CPU writable
        --------------------------------------------------------------------
        can_hpmob_i : in  std_logic_vector(7 downto 0);
        can_hpmob_o : out std_logic_vector(7 downto 0);

        --------------------------------------------------------------------
        -- CANPAGE
        --------------------------------------------------------------------
        can_page_o : out std_logic_vector(7 downto 0);

        --------------------------------------------------------------------
        -- Software reset pulse
        --------------------------------------------------------------------
        sw_reset_o : out std_logic;

        --------------------------------------------------------------------
        -- Compatibility outputs. CANEN registers are read-only, therefore
        -- CPU-side write outputs are not used and are kept at zero.
        --------------------------------------------------------------------
        can_en1_o : out std_logic_vector(7 downto 0);
        can_en2_o : out std_logic_vector(7 downto 0);

        --------------------------------------------------------------------
        -- MOb manager CPU write interface
        --------------------------------------------------------------------
        mob_cpu_wr       : out std_logic;
        mob_cpu_addr     : out integer range 0 to 15;
        mob_cpu_mob      : out integer range 0 to 15;
        mob_cpu_data     : out std_logic_vector(7 downto 0);

        mob_cpu_msg_wr   : out std_logic;
        mob_cpu_msg_idx  : out integer range 0 to 7;
        mob_cpu_msg_data : out std_logic_vector(7 downto 0);

        --------------------------------------------------------------------
        -- MOb manager register/data readback
        --------------------------------------------------------------------
        mobs_i     : in mob_array_t;
        mob_data_i : in data_array_t
    );
end entity cpu_if;

architecture rtl of cpu_if is

    ------------------------------------------------------------------------
    -- Local conversion helpers, avoiding numeric_std and Natural.
    ------------------------------------------------------------------------
    function cpu_slv_to_int(
        v : std_logic_vector
    ) return integer is
        variable r : integer range 0 to 65535;
    begin
        r := 0;
        for i in v'range loop
            r := r * 2;
            if v(i) = '1' then
                r := r + 1;
            end if;
        end loop;
        return r;
    end function;

    function cpu_int_to_slv3(
        v : integer
    ) return std_logic_vector(2 downto 0) is
        variable r   : std_logic_vector(2 downto 0);
        variable tmp : integer range 0 to 7;
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
                r(i) := '1';
            else
                r(i) := '0';
            end if;
            tmp := tmp / 2;
        end loop;

        return r;
    end function;

begin

    cpu_proc : process (clk, reset_n)

        --------------------------------------------------------------------
        -- CPU register state
        --------------------------------------------------------------------
        variable can_gcon_v      : std_logic_vector(7 downto 0);
        variable can_gie_v       : std_logic_vector(7 downto 0);
        variable can_ie1_v       : std_logic_vector(7 downto 0);
        variable can_ie2_v       : std_logic_vector(7 downto 0);
        variable can_bt1_v       : std_logic_vector(7 downto 0);
        variable can_bt2_v       : std_logic_vector(7 downto 0);
        variable can_bt3_v       : std_logic_vector(7 downto 0);
        variable can_tcon_v      : std_logic_vector(7 downto 0);
        variable can_hpmob_cgp_v : std_logic_vector(3 downto 0);
        variable can_page_v      : std_logic_vector(7 downto 0);

        --------------------------------------------------------------------
        -- Read data
        --------------------------------------------------------------------
        variable read_mux_v : std_logic_vector(7 downto 0);

        --------------------------------------------------------------------
        -- Work variables
        --------------------------------------------------------------------
        variable addr_int   : integer range 0 to 63;
        variable idx_v      : integer range 0 to 15;
        variable offs_v     : integer range 0 to 31;
        variable data_idx_v : integer range 0 to 7;
        variable next_idx_v : integer range 0 to 7;

        --------------------------------------------------------------------
        -- MOb manager write pulse variables
        --------------------------------------------------------------------
        variable mob_wr_v       : boolean;
        variable mob_msg_wr_v   : boolean;
        variable mob_addr_v     : integer range 0 to 15;
        variable mob_mob_v      : integer range 0 to 15;
        variable mob_data_v     : std_logic_vector(7 downto 0);
        variable mob_msg_idx_v  : integer range 0 to 7;

        --------------------------------------------------------------------
        -- Other one-cycle pulse variables
        --------------------------------------------------------------------
        variable sw_reset_v    : boolean;
        variable git_clear_en_v : boolean;
        variable git_clear_v   : std_logic_vector(7 downto 0);

    begin
        if reset_n = '0' then
            can_gcon_v      := (others => '0');
            can_gie_v       := (others => '0');
            can_ie1_v       := (others => '0');
            can_ie2_v       := (others => '0');
            can_bt1_v       := (others => '0');
            can_bt2_v       := (others => '0');
            can_bt3_v       := (others => '0');
            can_tcon_v      := (others => '0');
            can_hpmob_cgp_v := (others => '0');
            can_page_v      := (others => '0');

            read_mux_v := (others => '0');

            addr_int   := 0;
            idx_v      := 0;
            offs_v     := 0;
            data_idx_v := 0;
            next_idx_v := 0;

            mob_wr_v      := false;
            mob_msg_wr_v  := false;
            mob_addr_v    := 0;
            mob_mob_v     := 0;
            mob_data_v    := (others => '0');
            mob_msg_idx_v := 0;

            sw_reset_v    := false;
            git_clear_en_v := false;
            git_clear_v   := (others => '0');

            data_out        <= (others => '0');
            can_gcon_o      <= (others => '0');
            can_git_clear_o <= (others => '0');
            can_gie_o       <= (others => '0');
            can_ie1_o       <= (others => '0');
            can_ie2_o       <= (others => '0');
            can_bt1_o       <= (others => '0');
            can_bt2_o       <= (others => '0');
            can_bt3_o       <= (others => '0');
            can_tcon_o      <= (others => '0');
            can_hpmob_o     <= (others => '0');
            can_page_o      <= (others => '0');
            sw_reset_o      <= '0';
            can_en1_o       <= (others => '0');
            can_en2_o       <= (others => '0');

            mob_cpu_wr       <= '0';
            mob_cpu_addr     <= 0;
            mob_cpu_mob      <= 0;
            mob_cpu_data     <= (others => '0');
            mob_cpu_msg_wr   <= '0';
            mob_cpu_msg_idx  <= 0;
            mob_cpu_msg_data <= (others => '0');

        elsif rising_edge(clk) then
            ------------------------------------------------------------------
            -- Default one-cycle pulse values.
            ------------------------------------------------------------------
            mob_wr_v      := false;
            mob_msg_wr_v  := false;
            sw_reset_v    := false;
            git_clear_en_v := false;

            mob_addr_v    := 0;
            mob_mob_v     := 0;
            mob_data_v    := (others => '0');
            mob_msg_idx_v := 0;
            git_clear_v   := (others => '0');

            ------------------------------------------------------------------
            -- Write cycle.
            ------------------------------------------------------------------
            if cs = '1' and wr = '1' then
                addr_int := cpu_slv_to_int(addr);

                case addr_int is
                    --------------------------------------------------------
                    -- CANGCON
                    --------------------------------------------------------
                    when 0 =>
                        can_gcon_v := data_in;

                        -- SWRES is auto-resettable.
                        can_gcon_v(0) := '0';

                        if data_in(0) = '1' then
                            sw_reset_v := true;
                        end if;

                    --------------------------------------------------------
                    -- CANGSTA is read-only.
                    --------------------------------------------------------
                    when 1 =>
                        null;

                    --------------------------------------------------------
                    -- CANGIT write-1-clear.
                    -- Bit 7 CANIT is read-only and is not cleared here.
                    ----------------------------------------------------------------
                    when 2 =>
                        git_clear_v    := data_in;
                        git_clear_v(7) := '0';
                        git_clear_en_v := true;

                    --------------------------------------------------------
                    -- CANGIE
                    --------------------------------------------------------
                    when 3 =>
                        can_gie_v := data_in;

                    --------------------------------------------------------
                    -- CANEN2 / CANEN1 are read-only.
                    --------------------------------------------------------
                    when 4 | 5 =>
                        null;

                    --------------------------------------------------------
                    -- CANIE2, low MOb interrupt enable register.
                    --------------------------------------------------------
                    when 6 =>
                        can_ie2_v := data_in;

                        -- Reserved upper bits of the 8-bit view are written zero.
                        can_ie2_v(7 downto 6) := "00";

                    --------------------------------------------------------
                    -- CANIE1, high MOb interrupt enable register.
                    --------------------------------------------------------
                    when 7 =>
                        can_ie1_v := data_in;

                        -- Reserved upper bits of the 8-bit view are written zero.
                        can_ie1_v(7 downto 6) := "00";

                    --------------------------------------------------------
                    -- CANSIT2 / CANSIT1 are read-only.
                    --------------------------------------------------------
                    when 8 | 9 =>
                        null;

                    --------------------------------------------------------
                    -- CANBT1
                    --------------------------------------------------------
                    when 10 =>
                        can_bt1_v := data_in;

                        -- Reserved bits.
                        can_bt1_v(7) := '0';
                        can_bt1_v(0) := '0';

                    --------------------------------------------------------
                    -- CANBT2
                    --------------------------------------------------------
                    when 11 =>
                        can_bt2_v := data_in;

                        -- Reserved bits.
                        can_bt2_v(7) := '0';
                        can_bt2_v(4) := '0';
                        can_bt2_v(0) := '0';

                    --------------------------------------------------------
                    -- CANBT3
                    --------------------------------------------------------
                    when 12 =>
                        can_bt3_v := data_in;

                        -- Reserved bit.
                        can_bt3_v(7) := '0';

                    --------------------------------------------------------
                    -- CANTCON
                    --------------------------------------------------------
                    when 13 =>
                        can_tcon_v := data_in;

                    --------------------------------------------------------
                    -- CANTIML/H, CANTTCL/H, CANTEC, CANREC are read-only.
                    --------------------------------------------------------
                    when 14 | 15 | 16 | 17 | 18 | 19 =>
                        null;

                    --------------------------------------------------------
                    -- CANHPMOB.
                    -- High nibble is read-only. Low nibble CGP is writable.
                    --------------------------------------------------------
                    when 20 =>
                        can_hpmob_cgp_v := data_in(3 downto 0);

                    --------------------------------------------------------
                    -- CANPAGE.
                    -- MOBNB3 must be written as zero for compatibility.
                    --------------------------------------------------------
                    when 21 =>
                        can_page_v    := data_in;
                        can_page_v(7) := '0';

                    --------------------------------------------------------
                    -- MOb page registers.
                    --------------------------------------------------------
                    when others =>
                        if addr_int >= 32 and addr_int < 64 then
                            idx_v  := cpu_slv_to_int(can_page_v(7 downto 4));
                            offs_v := addr_int - 32;

                            if idx_v < MOB_COUNT and offs_v <= 12 then
                                if offs_v = MOB_OFFS_MSG then
                                    ----------------------------------------
                                    -- CANMSG write through MOb manager.
                                    ----------------------------------------
                                    data_idx_v := cpu_slv_to_int(can_page_v(2 downto 0));

                                    mob_msg_wr_v   := true;
                                    mob_mob_v      := idx_v;
                                    mob_msg_idx_v  := data_idx_v;
                                    mob_data_v     := data_in;

                                    ----------------------------------------
                                    -- Auto-increment if AINC = 0.
                                    ----------------------------------------
                                    if can_page_v(3) = '0' then
                                        if data_idx_v = 7 then
                                            next_idx_v := 0;
                                        else
                                            next_idx_v := data_idx_v + 1;
                                        end if;

                                        can_page_v(2 downto 0) :=
                                            cpu_int_to_slv3(next_idx_v);
                                    end if;

                                else
                                    ----------------------------------------
                                    -- MOb register write through MOb manager.
                                    --
                                    -- This includes STMOB write-1-clear and
                                    -- CDMOB configuration writes.
                                    ----------------------------------------
                                    mob_wr_v   := true;
                                    mob_addr_v := offs_v;
                                    mob_mob_v  := idx_v;
                                    mob_data_v := data_in;
                                end if;
                            end if;
                        end if;
                end case;
            end if;

            ------------------------------------------------------------------
            -- Read cycle.
            ------------------------------------------------------------------
            if cs = '1' and rd = '1' then
                addr_int := cpu_slv_to_int(addr);

                case addr_int is
                    --------------------------------------------------------
                    -- General registers.
                    --------------------------------------------------------
                    when 0 =>
                        read_mux_v := can_gcon_v;

                    when 1 =>
                        read_mux_v := can_gsta_i;

                    when 2 =>
                        read_mux_v := can_git_i;

                    when 3 =>
                        read_mux_v := can_gie_v;

                    --------------------------------------------------------
                    -- CANEN2 low, CANEN1 high.
                    --------------------------------------------------------
                    when 4 =>
                        read_mux_v := can_en2_i;

                    when 5 =>
                        read_mux_v := can_en1_i;

                    --------------------------------------------------------
                    -- CANIE2 low, CANIE1 high.
                    --------------------------------------------------------
                    when 6 =>
                        read_mux_v := can_ie2_v;

                    when 7 =>
                        read_mux_v := can_ie1_v;

                    --------------------------------------------------------
                    -- CANSIT2 low, CANSIT1 high.
                    --------------------------------------------------------
                    when 8 =>
                        read_mux_v := can_sit2_i;

                    when 9 =>
                        read_mux_v := can_sit1_i;

                    --------------------------------------------------------
                    -- Bit timing registers.
                    --------------------------------------------------------
                    when 10 =>
                        read_mux_v := can_bt1_v;

                    when 11 =>
                        read_mux_v := can_bt2_v;

                    when 12 =>
                        read_mux_v := can_bt3_v;

                    --------------------------------------------------------
                    -- Timer control.
                    --------------------------------------------------------
                    when 13 =>
                        read_mux_v := can_tcon_v;

                    --------------------------------------------------------
                    -- Timer count registers.
                    --------------------------------------------------------
                    when 14 =>
                        read_mux_v := cantim_i(7 downto 0);

                    when 15 =>
                        read_mux_v := cantim_i(15 downto 8);

                    when 16 =>
                        read_mux_v := canttc_i(7 downto 0);

                    when 17 =>
                        read_mux_v := canttc_i(15 downto 8);

                    --------------------------------------------------------
                    -- Error counters.
                    --------------------------------------------------------
                    when 18 =>
                        read_mux_v := cantec_i;

                    when 19 =>
                        read_mux_v := canrec_i;

                    --------------------------------------------------------
                    -- CANHPMOB.
                    --------------------------------------------------------
                    when 20 =>
                        read_mux_v := can_hpmob_i(7 downto 4) & can_hpmob_cgp_v;

                    --------------------------------------------------------
                    -- CANPAGE.
                    --------------------------------------------------------
                    when 21 =>
                        read_mux_v := can_page_v;

                    --------------------------------------------------------
                    -- MOb page registers.
                    --------------------------------------------------------
                    when others =>
                        if addr_int >= 32 and addr_int < 64 then
                            idx_v  := cpu_slv_to_int(can_page_v(7 downto 4));
                            offs_v := addr_int - 32;

                            if idx_v < MOB_COUNT and offs_v <= 12 then
                                case offs_v is
                                    when MOB_OFFS_STMOB =>
                                        read_mux_v := mobs_i(idx_v).stmob;

                                    when MOB_OFFS_CDMOB =>
                                        read_mux_v := mobs_i(idx_v).cdmob;

                                    when MOB_OFFS_IDT1 =>
                                        read_mux_v := mobs_i(idx_v).idt1;

                                    when MOB_OFFS_IDT2 =>
                                        read_mux_v := mobs_i(idx_v).idt2;

                                    when MOB_OFFS_IDT3 =>
                                        read_mux_v := mobs_i(idx_v).idt3;

                                    when MOB_OFFS_IDT4 =>
                                        read_mux_v := mobs_i(idx_v).idt4;

                                    when MOB_OFFS_IDM1 =>
                                        read_mux_v := mobs_i(idx_v).idm1;

                                    when MOB_OFFS_IDM2 =>
                                        read_mux_v := mobs_i(idx_v).idm2;

                                    when MOB_OFFS_IDM3 =>
                                        read_mux_v := mobs_i(idx_v).idm3;

                                    when MOB_OFFS_IDM4 =>
                                        read_mux_v := mobs_i(idx_v).idm4;

                                    when MOB_OFFS_STML =>
                                        read_mux_v := mobs_i(idx_v).stml;

                                    when MOB_OFFS_STMH =>
                                        read_mux_v := mobs_i(idx_v).stmh;

                                    when MOB_OFFS_MSG =>
                                        ------------------------------------
                                        -- CANMSG read through MOb manager.
                                        ------------------------------------
                                        data_idx_v :=
                                            cpu_slv_to_int(can_page_v(2 downto 0));

                                        read_mux_v :=
                                            mob_data_i(idx_v)((data_idx_v * 8 + 7)
                                                              downto
                                                              (data_idx_v * 8));

                                        ------------------------------------
                                        -- Auto-increment if AINC = 0.
                                        ------------------------------------
                                        if can_page_v(3) = '0' then
                                            if data_idx_v = 7 then
                                                next_idx_v := 0;
                                            else
                                                next_idx_v := data_idx_v + 1;
                                            end if;

                                            can_page_v(2 downto 0) :=
                                                cpu_int_to_slv3(next_idx_v);
                                        end if;

                                    when others =>
                                        read_mux_v := (others => '0');
                                end case;
                            else
                                read_mux_v := (others => '0');
                            end if;
                        else
                            read_mux_v := (others => '0');
                        end if;
                end case;
            end if;

            ------------------------------------------------------------------
            -- Outputs.
            ------------------------------------------------------------------
            data_out    <= read_mux_v;

            can_gcon_o  <= can_gcon_v;
            can_gie_o   <= can_gie_v;
            can_ie1_o   <= can_ie1_v;
            can_ie2_o   <= can_ie2_v;

            can_bt1_o   <= can_bt1_v;
            can_bt2_o   <= can_bt2_v;
            can_bt3_o   <= can_bt3_v;
            can_tcon_o  <= can_tcon_v;

            can_hpmob_o <= can_hpmob_i(7 downto 4) & can_hpmob_cgp_v;
            can_page_o  <= can_page_v;

            if sw_reset_v then
                sw_reset_o <= '1';
            else
                sw_reset_o <= '0';
            end if;

            if git_clear_en_v then
                can_git_clear_o <= git_clear_v;
            else
                can_git_clear_o <= (others => '0');
            end if;

            -- CANEN registers are read-only.
            can_en1_o <= (others => '0');
            can_en2_o <= (others => '0');

            ------------------------------------------------------------------
            -- MOb manager write interface.
            ------------------------------------------------------------------
            if mob_wr_v then
                mob_cpu_wr <= '1';
            else
                mob_cpu_wr <= '0';
            end if;

            mob_cpu_addr <= mob_addr_v;
            mob_cpu_mob  <= mob_mob_v;
            mob_cpu_data <= mob_data_v;

            if mob_msg_wr_v then
                mob_cpu_msg_wr <= '1';
            else
                mob_cpu_msg_wr <= '0';
            end if;

            mob_cpu_msg_idx  <= mob_msg_idx_v;
            mob_cpu_msg_data <= mob_data_v;
        end if;
    end process cpu_proc;

end architecture rtl;

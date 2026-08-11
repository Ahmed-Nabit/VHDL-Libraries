-- ============================================================================
-- CPU Interface – Full register decode, MOB page access, auto‑increment,
-- write‑1‑clear for interrupt flags, and CGP bits preservation.
-- Now provides cdmob_written strobe for BXOK clear condition.
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

entity cpu_if is
    port (
        clk         : in  std_logic;
        reset_n     : in  std_logic;
        cs          : in  std_logic;
        wr          : in  std_logic;
        rd          : in  std_logic;
        addr        : in  std_logic_vector(5 downto 0);
        data_in     : in  std_logic_vector(7 downto 0);
        data_out    : out std_logic_vector(7 downto 0);
        can_gcon_o  : out std_logic_vector(7 downto 0);
        can_gsta_i  : in  std_logic_vector(7 downto 0);
        can_git_o   : out std_logic_vector(7 downto 0);
        can_git_i   : in  std_logic_vector(7 downto 0);
        can_gie_o   : out std_logic_vector(7 downto 0);
        can_en1_o   : out std_logic_vector(7 downto 0);
        can_en1_i   : in  std_logic_vector(7 downto 0);
        can_en2_o   : out std_logic_vector(7 downto 0);
        can_en2_i   : in  std_logic_vector(7 downto 0);
        can_ie1_o   : out std_logic_vector(7 downto 0);
        can_ie2_o   : out std_logic_vector(7 downto 0);
        can_sit1_i  : in  std_logic_vector(7 downto 0);
        can_sit2_i  : in  std_logic_vector(7 downto 0);
        can_bt1_o   : out std_logic_vector(7 downto 0);
        can_bt2_o   : out std_logic_vector(7 downto 0);
        can_bt3_o   : out std_logic_vector(7 downto 0);
        can_tcon_o  : out std_logic_vector(7 downto 0);
        cantim_i    : in  std_logic_vector(15 downto 0);
        canttc_i    : in  std_logic_vector(15 downto 0);
        cantec_i    : in  std_logic_vector(7 downto 0);
        canrec_i    : in  std_logic_vector(7 downto 0);
        can_hpmob_o : out std_logic_vector(7 downto 0);
        can_hpmob_i : in  std_logic_vector(7 downto 0);
        can_page_o  : out std_logic_vector(7 downto 0);
        can_page_i  : in  std_logic_vector(7 downto 0);
        mobs_o      : out mob_array_t;
        mobs_i      : in  mob_array_t;
        mob_data_o  : out data_array_t;
        mob_data_i  : in  data_array_t;
        bxok_clear  : out std_logic;
        bxok_clear_allowed : in  std_logic;
        sw_reset    : out std_logic;
        cdmob_written : out std_logic_vector(MOB_COUNT-1 downto 0)  -- strobe per MOB
    );
end cpu_if;

architecture rtl of cpu_if is
    signal can_gcon_r   : std_logic_vector(7 downto 0) := (others => '0');
    signal can_gie_r    : std_logic_vector(7 downto 0) := (others => '0');
    signal can_ie1_r    : std_logic_vector(7 downto 0) := (others => '0');
    signal can_ie2_r    : std_logic_vector(7 downto 0) := (others => '0');
    signal can_bt1_r    : std_logic_vector(7 downto 0) := (others => '0');
    signal can_bt2_r    : std_logic_vector(7 downto 0) := (others => '0');
    signal can_bt3_r    : std_logic_vector(7 downto 0) := (others => '0');
    signal can_tcon_r   : std_logic_vector(7 downto 0) := (others => '0');
    signal can_hpmob_r  : std_logic_vector(7 downto 0) := (others => '0');
    signal can_page_r   : std_logic_vector(7 downto 0) := (others => '0');
    signal can_git_clear : std_logic_vector(7 downto 0) := (others => '0');
    signal bxok_clear_int : std_logic := '0';
    signal sw_reset_int : std_logic := '0';
    signal sw_reset_clear : std_logic := '0';
    signal cdmob_written_int : std_logic_vector(MOB_COUNT-1 downto 0) := (others => '0');

    signal mobs_int     : mob_array_t;
    signal mob_data_int : data_array_t;
    signal read_mux     : std_logic_vector(7 downto 0);

    signal fbuf_mobs_re_enabled : std_logic_vector(MOB_COUNT-1 downto 0) := (others => '0');
    signal fbuf_mask_local : std_logic_vector(MOB_COUNT-1 downto 0);
begin
    can_git_o <= can_git_clear;
    bxok_clear <= bxok_clear_int;
    sw_reset <= sw_reset_int;
    cdmob_written <= cdmob_written_int;

    cpu_proc: process(clk, reset_n)
        variable addr_int : integer range 0 to 63;
        variable idx : integer range 0 to MOB_COUNT-1;
        variable offs: integer range 0 to 15;
        variable data_idx : integer range 0 to 7;
        variable all_re_enabled : boolean;
    begin
        if reset_n = '0' then
            can_gcon_r   <= (others => '0');
            can_gie_r    <= (others => '0');
            can_ie1_r    <= (others => '0');
            can_ie2_r    <= (others => '0');
            can_bt1_r    <= (others => '0');
            can_bt2_r    <= (others => '0');
            can_bt3_r    <= (others => '0');
            can_tcon_r   <= (others => '0');
            can_hpmob_r  <= (others => '0');
            can_page_r   <= (others => '0');
            can_git_clear <= (others => '0');
            bxok_clear_int <= '0';
            sw_reset_int <= '0';
            sw_reset_clear <= '0';
            cdmob_written_int <= (others => '0');
            for i in 0 to MOB_COUNT-1 loop
                mobs_int(i) <= (stmob=>(others=>'0'), cdmob=>MOB_DISABLED&"00"&"0000",
                                idt1=>(others=>'0'), idt2=>(others=>'0'),
                                idt3=>(others=>'0'), idt4=>(others=>'0'),
                                idm1=>(others=>'0'), idm2=>(others=>'0'),
                                idm3=>(others=>'0'), idm4=>(others=>'0'),
                                stml=>(others=>'0'), stmh=>(others=>'0'));
                mob_data_int(i) <= (others => '0');
            end loop;
            read_mux <= (others => '0');
            data_out <= (others => '0');
        elsif rising_edge(clk) then
            can_git_clear <= (others => '0');
            bxok_clear_int <= '0';
            cdmob_written_int <= (others => '0');  -- one-cycle strobe
            if sw_reset_int = '1' then
                sw_reset_int <= '0';
                sw_reset_clear <= '1';
                can_gcon_r(0) <= '0';
            end if;
            if sw_reset_clear = '1' then
                sw_reset_clear <= '0';
            end if;

            fbuf_mask_local <= (others => '0');
            for i in 0 to MOB_COUNT-1 loop
                if mobs_int(i).cdmob(7 downto 6) = MOB_FBUF_RX then
                    fbuf_mask_local(i) <= '1';
                end if;
            end loop;

            -- Write cycle
            if cs = '1' and wr = '1' then
                addr_int := to_integer(unsigned(addr));
                case addr_int is
                    when 0  => 
                        can_gcon_r <= data_in;
                        if data_in(0) = '1' then
                            sw_reset_int <= '1';
                        end if;
                    when 1  => null; -- CANGSTA read-only
                    when 2  => 
                        can_git_clear <= data_in;
                        if data_in(4) = '1' then
                            if bxok_clear_allowed = '1' then
                                bxok_clear_int <= '1';
                            else
                                can_git_clear(4) <= '0';
                            end if;
                        end if;
                    when 3  => can_gie_r <= data_in;
                    when 4|5 => null; -- CANEN read-only
                    when 6  => can_ie1_r <= data_in;
                    when 7  => can_ie2_r <= data_in;
                    when 8|9 => null; -- CANSIT read-only
                    when 10 => can_bt1_r <= data_in;
                    when 11 => can_bt2_r <= data_in;
                    when 12 => can_bt3_r <= data_in;
                    when 13 => can_tcon_r <= data_in;
                    when 14|15|16|17|18|19 => null;
                    when 20 => can_hpmob_r <= data_in;
                    when 21 => can_page_r <= data_in;
                    when others =>
                        if addr_int >= 32 and addr_int < 64 then
                            idx := to_integer(unsigned(can_page_r(7 downto 4)));
                            if idx < MOB_COUNT then
                                offs := addr_int - 32;
                                case offs is
                                    when MOB_OFFS_STMOB =>
                                        mobs_int(idx).stmob <= mobs_int(idx).stmob and not data_in;
                                    when MOB_OFFS_CDMOB =>
                                        mobs_int(idx).cdmob <= data_in;
                                        cdmob_written_int(idx) <= '1';  -- strobe
                                    when MOB_OFFS_IDT1 => mobs_int(idx).idt1 <= data_in;
                                    when MOB_OFFS_IDT2 => mobs_int(idx).idt2 <= data_in;
                                    when MOB_OFFS_IDT3 => mobs_int(idx).idt3 <= data_in;
                                    when MOB_OFFS_IDT4 => mobs_int(idx).idt4 <= data_in;
                                    when MOB_OFFS_IDM1 => mobs_int(idx).idm1 <= data_in;
                                    when MOB_OFFS_IDM2 => mobs_int(idx).idm2 <= data_in;
                                    when MOB_OFFS_IDM3 => mobs_int(idx).idm3 <= data_in;
                                    when MOB_OFFS_IDM4 => mobs_int(idx).idm4 <= data_in;
                                    when MOB_OFFS_STML => mobs_int(idx).stml <= data_in;
                                    when MOB_OFFS_STMH => mobs_int(idx).stmh <= data_in;
                                    when MOB_OFFS_MSG =>
                                        data_idx := to_integer(unsigned(can_page_r(2 downto 0)));
                                        mob_data_int(idx)((data_idx*8 + 7) downto (data_idx*8)) <= data_in;
                                        if can_page_r(3) = '0' then
                                            if data_idx = 7 then
                                                can_page_r(2 downto 0) <= "000";
                                            else
                                                can_page_r(2 downto 0) <= std_logic_vector(to_unsigned(data_idx+1, 3));
                                            end if;
                                        end if;
                                    when others => null;
                                end case;
                            end if;
                        end if;
                end case;
            end if;

            -- Read cycle
            if cs = '1' and rd = '1' then
                addr_int := to_integer(unsigned(addr));
                case addr_int is
                    when 0  => read_mux <= can_gcon_r;
                    when 1  => read_mux <= can_gsta_i;
                    when 2  => read_mux <= can_git_i;
                    when 3  => read_mux <= can_gie_r;
                    when 4  => read_mux <= can_en1_i;
                    when 5  => read_mux <= can_en2_i;
                    when 6  => read_mux <= can_ie1_r;
                    when 7  => read_mux <= can_ie2_r;
                    when 8  => read_mux <= can_sit1_i;
                    when 9  => read_mux <= can_sit2_i;
                    when 10 => read_mux <= can_bt1_r;
                    when 11 => read_mux <= can_bt2_r;
                    when 12 => read_mux <= can_bt3_r;
                    when 13 => read_mux <= can_tcon_r;
                    when 14 => read_mux <= cantim_i(7 downto 0);
                    when 15 => read_mux <= cantim_i(15 downto 8);
                    when 16 => read_mux <= canttc_i(7 downto 0);
                    when 17 => read_mux <= canttc_i(15 downto 8);
                    when 18 => read_mux <= cantec_i;
                    when 19 => read_mux <= canrec_i;
                    when 20 => read_mux <= can_hpmob_i;
                    when 21 => read_mux <= can_page_r;
                    when others =>
                        if addr_int >= 32 and addr_int < 64 then
                            idx := to_integer(unsigned(can_page_r(7 downto 4)));
                            if idx < MOB_COUNT then
                                offs := addr_int - 32;
                                case offs is
                                    when MOB_OFFS_STMOB => read_mux <= mobs_i(idx).stmob;
                                    when MOB_OFFS_CDMOB => read_mux <= mobs_i(idx).cdmob;
                                    when MOB_OFFS_IDT1  => read_mux <= mobs_i(idx).idt1;
                                    when MOB_OFFS_IDT2  => read_mux <= mobs_i(idx).idt2;
                                    when MOB_OFFS_IDT3  => read_mux <= mobs_i(idx).idt3;
                                    when MOB_OFFS_IDT4  => read_mux <= mobs_i(idx).idt4;
                                    when MOB_OFFS_IDM1  => read_mux <= mobs_i(idx).idm1;
                                    when MOB_OFFS_IDM2  => read_mux <= mobs_i(idx).idm2;
                                    when MOB_OFFS_IDM3  => read_mux <= mobs_i(idx).idm3;
                                    when MOB_OFFS_IDM4  => read_mux <= mobs_i(idx).idm4;
                                    when MOB_OFFS_STML  => read_mux <= mobs_i(idx).stml;
                                    when MOB_OFFS_STMH  => read_mux <= mobs_i(idx).stmh;
                                    when MOB_OFFS_MSG   =>
                                        data_idx := to_integer(unsigned(can_page_r(2 downto 0)));
                                        read_mux <= mob_data_i(idx)((data_idx*8 + 7) downto (data_idx*8));
                                        if can_page_r(3) = '0' then
                                            if data_idx = 7 then
                                                can_page_r(2 downto 0) <= "000";
                                            else
                                                can_page_r(2 downto 0) <= std_logic_vector(to_unsigned(data_idx+1, 3));
                                            end if;
                                        end if;
                                    when others => read_mux <= (others => '0');
                                end case;
                            end if;
                        end if;
                end case;
            end if;

            data_out <= read_mux;
        end if;
    end process;

    can_gcon_o  <= can_gcon_r;
    can_gie_o   <= can_gie_r;
    can_ie1_o   <= can_ie1_r;
    can_ie2_o   <= can_ie2_r;
    can_bt1_o   <= can_bt1_r;
    can_bt2_o   <= can_bt2_r;
    can_bt3_o   <= can_bt3_r;
    can_tcon_o  <= can_tcon_r;
    can_hpmob_o <= can_hpmob_r;
    can_page_o  <= can_page_r;
    mobs_o      <= mobs_int;
    mob_data_o  <= mob_data_int;
    can_en1_o   <= (others => '0');
    can_en2_o   <= (others => '0');
end rtl;

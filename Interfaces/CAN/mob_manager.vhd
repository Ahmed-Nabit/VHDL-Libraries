-- ============================================================================
-- MOB Manager – Full implementation with correct BXOK clear condition using
-- cdmob_written strobe from cpu_if.
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

entity mob_manager is
    port (
        clk         : in  std_logic;
        reset_n     : in  std_logic;
        mobs_cfg    : in  mob_array_t;
        mob_data_cfg: in  data_array_t;
        mobs_status : out mob_array_t;
        mob_data_status : out data_array_t;
        mob_state   : out mob_state_vec_t;
        mob_intr    : out std_logic_vector(MOB_COUNT-1 downto 0);
        -- Rx information from engine
        rx_id_11    : in  std_logic_vector(10 downto 0);
        rx_id_29    : in  std_logic_vector(28 downto 0);
        rx_ide      : in  std_logic;
        rx_rtr      : in  std_logic;
        rx_dlc      : in  integer range 0 to 8;
        rx_data     : in  std_logic_vector(63 downto 0);
        rx_match    : out std_logic;
        rx_mob_idx  : out integer range -1 to MOB_COUNT-1;
        rx_fbuf_done: out std_logic;
        -- Tx requests from engine
        tx_request  : out std_logic;
        tx_mob      : out integer range -1 to MOB_COUNT-1;
        tx_id_29    : out std_logic_vector(28 downto 0);
        tx_ide      : out std_logic;
        tx_rtr      : out std_logic;
        tx_dlc      : out integer range 0 to 8;
        tx_data     : out std_logic_vector(63 downto 0);
        tx_ok       : in  std_logic;
        tx_abort    : in  std_logic;
        -- Error signals from engine
        bit_err     : in  std_logic;
        stuff_err   : in  std_logic;
        crc_err     : in  std_logic;
        form_err    : in  std_logic;
        ack_err     : in  std_logic;
        -- Timer
        cantim      : in  std_logic_vector(15 downto 0);
        -- Enable/disable for CANEN registers
        can_en1     : out std_logic_vector(7 downto 0);
        can_en2     : out std_logic_vector(7 downto 0);
        -- BXOK control
        bxok_clear  : in  std_logic;
        bxok_clear_allowed : out std_logic;
        -- CPU write strobe for CDMOB (from cpu_if)
        cdmob_written : in  std_logic_vector(MOB_COUNT-1 downto 0)
    );
end mob_manager;

architecture rtl of mob_manager is
    signal mob_en : std_logic_vector(MOB_COUNT-1 downto 0) := (others => '0');
    signal mob_st : mob_state_vec_t := (others => IDLE);
    signal mobs_reg : mob_array_t;
    signal mob_data_reg : data_array_t;

    signal fbuf_mask : std_logic_vector(MOB_COUNT-1 downto 0);
    signal fbuf_received : std_logic_vector(MOB_COUNT-1 downto 0) := (others => '0');
    signal fbuf_re_enabled : std_logic_vector(MOB_COUNT-1 downto 0) := (others => '0');
    signal bxok_pending : std_logic := '0';

    signal rx_match_int, rx_fbuf_done_int : std_logic;
    signal rx_mob_idx_int : integer range -1 to MOB_COUNT-1;
    signal tx_request_int : std_logic;
    signal tx_mob_int : integer range -1 to MOB_COUNT-1;

    signal abort_pending : std_logic := '0';
    signal abort_tx_mob : integer range -1 to MOB_COUNT-1 := -1;

    function id_match_11(id_rx : std_logic_vector(10 downto 0);
                         id_tag : std_logic_vector(10 downto 0);
                         id_mask : std_logic_vector(10 downto 0)) return boolean is
        variable ok : boolean := true;
    begin
        for i in 0 to 10 loop
            if id_mask(i) = '1' and id_rx(i) /= id_tag(i) then ok := false; exit; end if;
        end loop;
        return ok;
    end function;

    function id_match_29(id_rx : std_logic_vector(28 downto 0);
                         id_tag : std_logic_vector(28 downto 0);
                         id_mask : std_logic_vector(28 downto 0)) return boolean is
        variable ok : boolean := true;
    begin
        for i in 0 to 28 loop
            if id_mask(i) = '1' and id_rx(i) /= id_tag(i) then ok := false; exit; end if;
        end loop;
        return ok;
    end function;

begin
    process(clk, reset_n)
        variable i : integer;
        variable match_idx : integer := -1;
        variable match_found : boolean;
        variable ext : boolean;
        variable id_tag_11, id_mask_11 : std_logic_vector(10 downto 0);
        variable id_tag_29, id_mask_29 : std_logic_vector(28 downto 0);
        variable ide_match, rtr_match, id_match_bool : boolean;
        variable dlc_int : integer;
        variable all_re_enabled : boolean;
        variable mob_cfg_mode : std_logic_vector(1 downto 0);
    begin
        if reset_n = '0' then
            mob_en <= (others => '0');
            mob_st <= (others => IDLE);
            for i in 0 to MOB_COUNT-1 loop
                mobs_reg(i) <= (stmob=>(others=>'0'),
                                cdmob=>MOB_DISABLED & "00" & "0000",
                                idt1=>(others=>'0'), idt2=>(others=>'0'),
                                idt3=>(others=>'0'), idt4=>(others=>'0'),
                                idm1=>(others=>'0'), idm2=>(others=>'0'),
                                idm3=>(others=>'0'), idm4=>(others=>'0'),
                                stml=>(others=>'0'), stmh=>(others=>'0'));
                mob_data_reg(i) <= (others => '0');
            end loop;
            fbuf_mask <= (others => '0');
            fbuf_received <= (others => '0');
            fbuf_re_enabled <= (others => '0');
            bxok_pending <= '0';
            rx_match_int <= '0';
            rx_mob_idx_int <= -1;
            rx_fbuf_done_int <= '0';
            tx_request_int <= '0';
            tx_mob_int <= -1;
            abort_pending <= '0';
            abort_tx_mob <= -1;
        elsif rising_edge(clk) then
            rx_match_int <= '0';
            rx_mob_idx_int <= -1;
            rx_fbuf_done_int <= '0';
            tx_request_int <= '0';
            tx_mob_int <= -1;
            bxok_clear_allowed <= '0';

            -- Update fbuf_mask from current configuration
            fbuf_mask <= (others => '0');
            for i in 0 to MOB_COUNT-1 loop
                if mobs_cfg(i).cdmob(7 downto 6) = MOB_FBUF_RX then
                    fbuf_mask(i) <= '1';
                end if;
            end loop;

            -- 1. Update MOB enable flags (with abort handling)
            if abort_pending = '1' then
                for i in 0 to MOB_COUNT-1 loop
                    if i /= abort_tx_mob then
                        mob_en(i) <= '0';
                    end if;
                end loop;
                if tx_ok = '1' and abort_tx_mob >= 0 then
                    mob_en <= (others => '0');
                    abort_pending <= '0';
                    abort_tx_mob <= -1;
                end if;
            else
                for i in 0 to MOB_COUNT-1 loop
                    mob_cfg_mode := mobs_cfg(i).cdmob(7 downto 6);
                    if mob_cfg_mode = MOB_DISABLED then
                        mob_en(i) <= '0';
                        if mob_st(i) = AUTO_REPLY then
                            mob_st(i) <= IDLE;
                        end if;
                    else
                        if mob_en(i) = '0' then
                            mob_en(i) <= '1';
                        end if;
                    end if;
                end loop;
            end if;

            if tx_abort = '1' and abort_pending = '0' then
                abort_pending <= '1';
                if tx_mob_int >= 0 then
                    abort_tx_mob <= tx_mob_int;
                else
                    abort_tx_mob <= -1;
                end if;
            end if;

            -- 2. RX Acceptance Filtering
            match_found := false;
            match_idx := -1;
            ext := (rx_ide = '1');
            for i in 0 to MOB_COUNT-1 loop
                if mob_en(i) = '1' then
                    mob_cfg_mode := mobs_cfg(i).cdmob(7 downto 6);
                    if (mob_cfg_mode = MOB_RX) or (mob_cfg_mode = MOB_FBUF_RX) then
                        ide_match := true;
                        if mobs_cfg(i).idm4(0) = '1' then
                            ide_match := (mobs_cfg(i).cdmob(4) = rx_ide);
                        end if;
                        rtr_match := true;
                        if mobs_cfg(i).idm4(2) = '1' then
                            rtr_match := (mobs_cfg(i).idt4(2) = rx_rtr);
                        end if;
                        if ext then
                            id_tag_29 := mobs_cfg(i).idt4(7 downto 3) &
                                         mobs_cfg(i).idt3 &
                                         mobs_cfg(i).idt2 &
                                         mobs_cfg(i).idt1(7 downto 3);
                            id_mask_29 := mobs_cfg(i).idm4(7 downto 3) &
                                          mobs_cfg(i).idm3 &
                                          mobs_cfg(i).idm2 &
                                          mobs_cfg(i).idm1(7 downto 3);
                            id_match_bool := id_match_29(rx_id_29, id_tag_29, id_mask_29);
                        else
                            id_tag_11 := mobs_cfg(i).idt2(7 downto 5) & mobs_cfg(i).idt1;
                            id_mask_11 := mobs_cfg(i).idm2(7 downto 5) & mobs_cfg(i).idm1;
                            id_match_bool := id_match_11(rx_id_11, id_tag_11, id_mask_11);
                        end if;
                        if ide_match and rtr_match and id_match_bool then
                            match_found := true;
                            match_idx := i;
                            exit;
                        end if;
                    end if;
                end if;
            end loop;

            if match_found then
                rx_match_int <= '1';
                rx_mob_idx_int <= match_idx;

                if ext then
                    mobs_reg(match_idx).idt4(7 downto 3) <= rx_id_29(28 downto 24);
                    mobs_reg(match_idx).idt3 <= rx_id_29(23 downto 16);
                    mobs_reg(match_idx).idt2 <= rx_id_29(15 downto 8);
                    mobs_reg(match_idx).idt1(7 downto 3) <= rx_id_29(7 downto 3);
                else
                    mobs_reg(match_idx).idt2(7 downto 5) <= rx_id_11(10 downto 8);
                    mobs_reg(match_idx).idt1 <= rx_id_11(7 downto 0);
                end if;
                mobs_reg(match_idx).cdmob(4) <= rx_ide;
                mobs_reg(match_idx).cdmob(3 downto 0) <= std_logic_vector(to_unsigned(rx_dlc, 4));
                mobs_reg(match_idx).stml <= cantim(7 downto 0);
                mobs_reg(match_idx).stmh <= cantim(15 downto 8);
                if rx_rtr = '0' then
                    mob_data_reg(match_idx) <= rx_data;
                end if;
                mobs_reg(match_idx).stmob(5) <= '1'; -- RXOK
                if mobs_cfg(match_idx).cdmob(3 downto 0) /= std_logic_vector(to_unsigned(rx_dlc, 4)) then
                    mobs_reg(match_idx).stmob(7) <= '1'; -- DLCW
                end if;

                mob_cfg_mode := mobs_cfg(match_idx).cdmob(7 downto 6);
                if mob_cfg_mode = MOB_FBUF_RX then
                    fbuf_received(match_idx) <= '1';
                    if fbuf_received = fbuf_mask then
                        bxok_pending <= '1';
                        rx_fbuf_done_int <= '1';
                    end if;
                else
                    if mobs_cfg(match_idx).cdmob(5) = '0' then
                        mob_en(match_idx) <= '0';
                    else
                        mobs_reg(match_idx).cdmob(7 downto 6) <= MOB_TX;
                        mobs_reg(match_idx).cdmob(5) <= '0';
                        mobs_reg(match_idx).idt4(2) <= '0';
                        mob_st(match_idx) <= AUTO_REPLY;
                    end if;
                end if;
            end if;

            -- 3. TX Arbitration
            if abort_pending = '0' then
                for i in 0 to MOB_COUNT-1 loop
                    if mob_en(i) = '1' and mobs_reg(i).cdmob(7 downto 6) = MOB_TX then
                        tx_request_int <= '1';
                        tx_mob_int <= i;
                        if mobs_reg(i).cdmob(4) = '0' then
                            tx_id_29 <= (others => '0');
                            tx_id_29(10 downto 0) <= mobs_reg(i).idt2(7 downto 5) & mobs_reg(i).idt1;
                        else
                            tx_id_29 <= mobs_reg(i).idt4(7 downto 3) &
                                        mobs_reg(i).idt3 &
                                        mobs_reg(i).idt2 &
                                        mobs_reg(i).idt1(7 downto 3);
                        end if;
                        tx_ide <= mobs_reg(i).cdmob(4);
                        tx_rtr <= mobs_reg(i).idt4(2);
                        dlc_int := to_integer(unsigned(mobs_reg(i).cdmob(3 downto 0)));
                        if dlc_int > 8 then dlc_int := 8; end if;
                        tx_dlc <= dlc_int;
                        tx_data <= mob_data_reg(i);
                        exit;
                    end if;
                end loop;
            end if;

            -- 4. TX completion
            if tx_ok = '1' and tx_mob_int >= 0 and abort_pending = '0' then
                mob_en(tx_mob_int) <= '0';
                mobs_reg(tx_mob_int).stmob(6) <= '1'; -- TXOK
                mobs_reg(tx_mob_int).stml <= cantim(7 downto 0);
                mobs_reg(tx_mob_int).stmh <= cantim(15 downto 8);
                if mob_st(tx_mob_int) = AUTO_REPLY then
                    mob_st(tx_mob_int) <= IDLE;
                end if;
            end if;

            -- 5. Error propagation (for matched receive or transmit)
            if rx_match_int = '1' and rx_mob_idx_int >= 0 then
                i := rx_mob_idx_int;
                if bit_err   = '1' then mobs_reg(i).stmob(4) <= '1'; end if;
                if stuff_err = '1' then mobs_reg(i).stmob(3) <= '1'; end if;
                if crc_err   = '1' then mobs_reg(i).stmob(2) <= '1'; end if;
                if form_err  = '1' then mobs_reg(i).stmob(1) <= '1'; end if;
                if ack_err   = '1' then mobs_reg(i).stmob(0) <= '1'; end if;
            end if;
            if tx_mob_int >= 0 and abort_pending = '0' then
                if bit_err   = '1' then mobs_reg(tx_mob_int).stmob(4) <= '1'; end if;
                if stuff_err = '1' then mobs_reg(tx_mob_int).stmob(3) <= '1'; end if;
                if crc_err   = '1' then mobs_reg(tx_mob_int).stmob(2) <= '1'; end if;
                if form_err  = '1' then mobs_reg(tx_mob_int).stmob(1) <= '1'; end if;
                if ack_err   = '1' then mobs_reg(tx_mob_int).stmob(0) <= '1'; end if;
            end if;

            -- 6. Frame buffer re‑enable tracking for BXOK clear
            -- When bxok_pending is active, we wait for each MOB in the buffer set to receive a
            -- cdmob_written strobe. Once all have been written, bxok_clear_allowed is asserted.
            if bxok_pending = '1' then
                for i in 0 to MOB_COUNT-1 loop
                    if fbuf_mask(i) = '1' then
                        if cdmob_written(i) = '1' then
                            fbuf_re_enabled(i) <= '1';
                        end if;
                    end if;
                end loop;
            else
                fbuf_re_enabled <= (others => '0');
            end if;

            all_re_enabled := true;
            for i in 0 to MOB_COUNT-1 loop
                if fbuf_mask(i) = '1' and fbuf_re_enabled(i) = '0' then
                    all_re_enabled := false;
                    exit;
                end if;
            end loop;
            bxok_clear_allowed <= all_re_enabled;

            if bxok_clear = '1' and all_re_enabled = '1' then
                fbuf_received <= (others => '0');
                fbuf_re_enabled <= (others => '0');
                bxok_pending <= '0';
                rx_fbuf_done_int <= '0';
            end if;

            -- 7. Interrupt generation
            mob_intr <= (others => '0');
            for i in 0 to MOB_COUNT-1 loop
                if mobs_reg(i).stmob /= X"00" then
                    mob_intr(i) <= '1';
                end if;
            end loop;

            -- 8. Outputs
            rx_match <= rx_match_int;
            rx_mob_idx <= rx_mob_idx_int;
            rx_fbuf_done <= rx_fbuf_done_int;
            tx_request <= tx_request_int;
            tx_mob <= tx_mob_int;
            mobs_status <= mobs_reg;
            mob_data_status <= mob_data_reg;
            mob_state <= mob_st;

            can_en1 <= (others => '0');
            can_en2 <= (others => '0');
            for i in 0 to MOB_COUNT-1 loop
                if mob_en(i) = '1' then
                    if i < 6 then
                        can_en2(i) <= '1';
                    else
                        can_en1(i-6) <= '1';
                    end if;
                end if;
            end loop;
        end if;
    end process;
end rtl;

-- ============================================================================
-- CAN Timer – 16-bit counter with prescaler, TTC capture, divide-by-8 included
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

entity can_timer is
    port (
        clk         : in  std_logic;
        reset_n     : in  std_logic;
        enable      : in  std_logic;   -- ENFG
        tcon        : in  std_logic_vector(7 downto 0);
        ttc_mode    : in  std_logic;   -- TTC bit
        ttc_sync    : in  std_logic;   -- SYNTTC (0=SOF,1=EOF)
        sof_pulse   : in  std_logic;   -- start of frame
        eof_pulse   : in  std_logic;   -- end of frame
        cantim_o    : out std_logic_vector(15 downto 0);
        canttc_o    : out std_logic_vector(15 downto 0);
        ovr_tim_int : out std_logic
    );
end can_timer;

architecture rtl of can_timer is
    signal prescaler_cnt : unsigned(15 downto 0); -- enough for divide-by-8 * (TCON+1)
    signal timer_cnt : unsigned(15 downto 0);
    signal ttc_cnt : unsigned(15 downto 0);
    signal ovr_int : std_logic;
    signal prescaler_max : integer range 0 to 2047; -- 8*(255+1) = 2048
    signal div8_cnt : unsigned(2 downto 0);
begin
    prescaler_max <= 8 * (to_integer(unsigned(tcon)) + 1);

    process(clk, reset_n)
    begin
        if reset_n = '0' then
            prescaler_cnt <= (others => '0');
            div8_cnt <= (others => '0');
            timer_cnt <= (others => '0');
            ttc_cnt <= (others => '0');
            ovr_int <= '0';
        elsif rising_edge(clk) then
            ovr_int <= '0';
            if enable = '1' then
                -- First divide by 8 (counts 0..7)
                if div8_cnt = 7 then
                    div8_cnt <= (others => '0');
                    -- Then divide by TCON+1
                    if prescaler_cnt >= prescaler_max-1 then
                        prescaler_cnt <= (others => '0');
                        if timer_cnt = X"FFFF" then
                            timer_cnt <= (others => '0');
                            ovr_int <= '1';
                        else
                            timer_cnt <= timer_cnt + 1;
                        end if;
                    else
                        prescaler_cnt <= prescaler_cnt + 1;
                    end if;
                else
                    div8_cnt <= div8_cnt + 1;
                end if;
            end if;

            -- TTC capture
            if ttc_mode = '1' then
                if (ttc_sync = '0' and sof_pulse = '1') or
                   (ttc_sync = '1' and eof_pulse = '1') then
                    ttc_cnt <= timer_cnt;
                end if;
            end if;

            cantim_o <= std_logic_vector(timer_cnt);
            canttc_o <= std_logic_vector(ttc_cnt);
            ovr_tim_int <= ovr_int;
        end if;
    end process;
end rtl;

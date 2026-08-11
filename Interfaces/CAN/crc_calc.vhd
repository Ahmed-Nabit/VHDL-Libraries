-- ============================================================================
-- CRC Calculator – 15-bit CAN CRC with serial input
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

entity crc_calc is
    port (
        clk         : in  std_logic;
        reset_n     : in  std_logic;
        enable      : in  std_logic;    -- start/reset
        data_in     : in  std_logic;
        crc_out     : out std_logic_vector(14 downto 0);
        crc_valid   : out std_logic     -- high when computation done
    );
end crc_calc;

architecture rtl of crc_calc is
    signal crc_reg : std_logic_vector(14 downto 0);
    signal active : std_logic;
    signal bit_count : integer range 0 to 127;
begin
    process(clk, reset_n)
    begin
        if reset_n = '0' then
            crc_reg <= (others => '0');
            active <= '0';
            bit_count <= 0;
        elsif rising_edge(clk) then
            if enable = '1' then
                crc_reg <= (others => '0');
                active <= '1';
                bit_count <= 0;
            elsif active = '1' then
                crc_reg <= crc_bit(crc_reg, data_in);
                bit_count <= bit_count + 1;
            end if;
            crc_out <= crc_reg;
            crc_valid <= active;
        end if;
    end process;
end rtl;

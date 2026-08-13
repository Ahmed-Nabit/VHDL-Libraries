-- ============================================================================
-- CAN CRC Calculator
-- ============================================================================
-- Fixed behavior:
--   * CRC updates only when data_valid is asserted.
--   * enable is a start/reset command. A rising edge on enable restarts CRC.
--   * If data_valid is asserted on the same cycle as the enable rising edge,
--     the first bit is processed immediately after CRC reset.
--   * crc_valid indicates that the CRC accumulator is active.
--
-- Design constraints respected:
--   * No Natural type
--   * No Real type
--   * No concurrent signal assignment
--   * All state kept in process variables
-- ============================================================================

library ieee;
use ieee.std_logic_1164.all;
use work.can_pkg.all;

entity crc_calc is
    port (
        clk        : in  std_logic;
        reset_n    : in  std_logic;

        -- Rising edge starts/restarts CRC calculation.
        enable     : in  std_logic;

        -- Qualifies each serial CRC input bit.
        data_valid : in  std_logic;
        data_in    : in  std_logic;

        -- Current 15-bit CRC value.
        crc_out    : out std_logic_vector(14 downto 0);

        -- High when CRC calculation is active.
        crc_valid  : out std_logic
    );
end entity crc_calc;

architecture rtl of crc_calc is
begin

    crc_proc : process (clk, reset_n)
        variable crc_v         : std_logic_vector(14 downto 0);
        variable active_v      : boolean;
        variable last_enable_v : std_logic;
    begin
        if reset_n = '0' then
            crc_v         := (others => '0');
            active_v      := false;
            last_enable_v := '0';

            crc_out   <= (others => '0');
            crc_valid <= '0';

        elsif rising_edge(clk) then
            ------------------------------------------------------------------
            -- Start/restart CRC on rising edge of enable.
            ------------------------------------------------------------------
            if enable = '1' and last_enable_v = '0' then
                crc_v    := (others => '0');
                active_v := true;

                ----------------------------------------------------------------
                -- If the first bit is already valid on the start cycle,
                -- process it immediately. This allows SOF to be included
                -- without requiring an extra cycle.
                ----------------------------------------------------------------
                if data_valid = '1' then
                    crc_v := crc_bit(crc_v, data_in);
                end if;

            ------------------------------------------------------------------
            -- Normal CRC update: only on valid destuffed CAN bits.
            ------------------------------------------------------------------
            elsif active_v and data_valid = '1' then
                crc_v := crc_bit(crc_v, data_in);
            end if;

            --------------------------------------------------------------------
            -- Remember enable level for rising-edge detection.
            --------------------------------------------------------------------
            last_enable_v := enable;

            --------------------------------------------------------------------
            -- Outputs.
            --------------------------------------------------------------------
            crc_out <= crc_v;

            if active_v then
                crc_valid <= '1';
            else
                crc_valid <= '0';
            end if;
        end if;
    end process crc_proc;

end architecture rtl;
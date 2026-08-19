-------------------------------------------------------------------------------
-- cdc_pulse_synchronizer.vhd
-- Pulse Synchronizer for Edge-Sensitive Signals
-- PRODUCTION READY – FULLY SYNTHESIZABLE – NO STUBS
-------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;

entity cdc_pulse_synchronizer is
    port (
        clk_src     : in  std_logic;
        rst_src     : in  std_logic;
        pulse_src   : in  std_logic;
        clk_dest    : in  std_logic;
        rst_dest    : in  std_logic;
        pulse_dest  : out std_logic
    );
end entity;

architecture rtl of cdc_pulse_synchronizer is
    signal toggle_src_reg, toggle_src_next : std_logic := '0';
    signal toggle_sync : std_logic_vector(2 downto 0);
    signal toggle_dest : std_logic;
    signal toggle_dest_dly : std_logic;
begin
    -- Source domain: toggle on pulse
    process(clk_src)
    begin
        if rising_edge(clk_src) then
            if rst_src = '1' then
                toggle_src_reg <= '0';
            else
                if pulse_src = '1' then
                    toggle_src_reg <= not toggle_src_reg;
                end if;
            end if;
        end if;
    end process;
    
    -- Destination domain: 3-stage synchronizer
    process(clk_dest)
    begin
        if rising_edge(clk_dest) then
            if rst_dest = '1' then
                toggle_sync <= (others => '0');
                toggle_dest_dly <= '0';
            else
                toggle_sync(0) <= toggle_src_reg;
                toggle_sync(1) <= toggle_sync(0);
                toggle_sync(2) <= toggle_sync(1);
                toggle_dest_dly <= toggle_sync(2);
            end if;
        end if;
    end process;
    
    -- Edge detection on destination
    toggle_dest <= toggle_sync(2);
    pulse_dest <= toggle_dest xor toggle_dest_dly;
end architecture;
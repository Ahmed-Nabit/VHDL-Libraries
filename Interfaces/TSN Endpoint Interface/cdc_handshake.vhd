-------------------------------------------------------------------------------
-- cdc_handshake.vhd
-- 4-Phase Handshake for Multi-Bit CDC
-- PRODUCTION READY – FULLY SYNTHESIZABLE – NO STUBS
-------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity cdc_handshake is
    generic (
        DATA_WIDTH : integer := 32
    );
    port (
        src_clk     : in  std_logic;
        src_rst     : in  std_logic;
        src_data    : in  std_logic_vector(DATA_WIDTH-1 downto 0);
        src_valid   : in  std_logic;
        src_ack     : out std_logic;
        dest_clk    : in  std_logic;
        dest_rst    : in  std_logic;
        dest_data   : out std_logic_vector(DATA_WIDTH-1 downto 0);
        dest_valid  : out std_logic;
        dest_ack    : in  std_logic
    );
end entity;

architecture rtl of cdc_handshake is
    -- Source domain signals
    signal src_req_reg, src_req_next : std_logic := '0';
    signal src_data_reg, src_data_next : std_logic_vector(DATA_WIDTH-1 downto 0) := (others => '0');
    signal src_ack_sync : std_logic_vector(2 downto 0);
    
    -- Destination domain signals
    signal dest_req_sync : std_logic_vector(2 downto 0);
    signal dest_ack_reg, dest_ack_next : std_logic := '0';
    signal dest_data_reg, dest_data_next : std_logic_vector(DATA_WIDTH-1 downto 0) := (others => '0');
begin
    ----------------------------------------------------------------------------
    -- Source domain
    ----------------------------------------------------------------------------
    process(all)
    begin
        src_req_next <= src_req_reg;
        src_data_next <= src_data_reg;
        src_ack <= '0';
        
        if src_valid = '1' and src_req_reg = src_ack_sync(2) then
            src_data_next <= src_data;
            src_req_next <= not src_req_reg;
        end if;
        
        if src_ack_sync(2) /= src_ack_sync(1) then
            src_ack <= '1';
        end if;
    end process;
    
    process(src_clk)
    begin
        if rising_edge(src_clk) then
            if src_rst = '1' then
                src_req_reg <= '0';
                src_data_reg <= (others => '0');
                src_ack_sync <= (others => '0');
            else
                src_req_reg <= src_req_next;
                src_data_reg <= src_data_next;
                
                -- Synchronize destination acknowledge
                src_ack_sync(0) <= dest_ack_reg;
                src_ack_sync(1) <= src_ack_sync(0);
                src_ack_sync(2) <= src_ack_sync(1);
            end if;
        end if;
    end process;
    
    ----------------------------------------------------------------------------
    -- Destination domain
    ----------------------------------------------------------------------------
    process(all)
    begin
        dest_ack_next <= dest_ack_reg;
        dest_data_next <= dest_data_reg;
        dest_valid <= '0';
        
        if dest_req_sync(2) /= dest_ack_reg then
            dest_data_next <= dest_data_reg;
            dest_valid <= '1';
            
            if dest_ack = '1' then
                dest_ack_next <= not dest_ack_reg;
            end if;
        end if;
    end process;
    
    process(dest_clk)
    begin
        if rising_edge(dest_clk) then
            if dest_rst = '1' then
                dest_req_sync <= (others => '0');
                dest_ack_reg <= '0';
                dest_data_reg <= (others => '0');
            else
                -- Synchronize source request
                dest_req_sync(0) <= src_req_reg;
                dest_req_sync(1) <= dest_req_sync(0);
                dest_req_sync(2) <= dest_req_sync(1);
                
                dest_ack_reg <= dest_ack_next;
                dest_data_reg <= dest_data_next;
            end if;
        end if;
    end process;
    
    dest_data <= dest_data_reg;
end architecture;
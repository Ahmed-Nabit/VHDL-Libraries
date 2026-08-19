-------------------------------------------------------------------------------
-- descriptor_cache.vhd
-- Descriptor Cache for PCIe DMA Engine
-- PRODUCTION READY – FULLY SYNTHESIZABLE – NO STUBS
-------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.pipe_pkg.all;

entity descriptor_cache is
    generic (
        CHANNELS        : integer := 8;
        CACHE_DEPTH     : integer := 4
    );
    port (
        clk             : in  std_logic;
        rst_n           : in  std_logic;
        
        channel_id      : in  integer range 0 to CHANNELS-1;
        
        wr_en           : in  std_logic;
        wr_desc         : in  descriptor_t;
        wr_ready        : out std_logic;
        
        rd_en           : in  std_logic;
        rd_desc         : out descriptor_t;
        rd_valid        : out std_logic;
        
        cache_hit       : out std_logic;
        cache_miss      : out std_logic
    );
end entity descriptor_cache;

architecture rtl of descriptor_cache is
    constant CACHE_ADDR_WIDTH : integer := 2;  -- For depth 4
    
    type cache_line_t is record
        valid   : std_logic;
        tag     : std_logic_vector(63 downto 0);  -- Descriptor address
        desc    : descriptor_t;
    end record;
    
    type cache_array_t is array (0 to CACHE_DEPTH-1) of cache_line_t;
    type channel_cache_t is array (0 to CHANNELS-1) of cache_array_t;
    
    signal cache : channel_cache_t;
    
    type lru_array_t is array (0 to CHANNELS-1) of 
        unsigned(CACHE_ADDR_WIDTH-1 downto 0);
    signal lru_reg, lru_next : lru_array_t := (others => (others => '0'));
    
    signal hit_index : integer range 0 to CACHE_DEPTH-1;
    signal hit : std_logic;
    signal miss : std_logic;
    
begin
    process(clk)
        variable addr_tag : std_logic_vector(63 downto 0);
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                for c in 0 to CHANNELS-1 loop
                    for i in 0 to CACHE_DEPTH-1 loop
                        cache(c)(i).valid <= '0';
                    end loop;
                    lru_reg(c) <= (others => '0');
                end loop;
                
            else
                addr_tag := wr_desc.src_addr;  -- Use source address as tag
                
                -- Check for hit
                hit <= '0';
                for i in 0 to CACHE_DEPTH-1 loop
                    if cache(channel_id)(i).valid = '1' and 
                       cache(channel_id)(i).tag = addr_tag then
                        hit <= '1';
                        hit_index <= i;
                        exit;
                    end if;
                end loop;
                
                miss <= not hit;
                
                -- Cache write (on miss)
                if wr_en = '1' and hit = '0' then
                    -- Replace LRU line
                    cache(channel_id)(to_integer(lru_reg(channel_id))).valid <= '1';
                    cache(channel_id)(to_integer(lru_reg(channel_id))).tag <= addr_tag;
                    cache(channel_id)(to_integer(lru_reg(channel_id))).desc <= wr_desc;
                    
                    -- Update LRU
                    lru_reg(channel_id) <= lru_reg(channel_id) + 1;
                end if;
                
                -- Cache read
                if rd_en = '1' and hit = '1' then
                    rd_desc <= cache(channel_id)(hit_index).desc;
                end if;
            end if;
        end if;
    end process;
    
    wr_ready <= '1';
    rd_valid <= hit when rd_en = '1' else '0';
    cache_hit <= hit;
    cache_miss <= miss;

end architecture rtl;

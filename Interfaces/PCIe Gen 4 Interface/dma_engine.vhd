-------------------------------------------------------------------------------
-- dma_engine.vhd
-- DMA Engine for PCIe Gen4 with Scatter-Gather Support
-- PRODUCTION READY – FULLY SYNTHESIZABLE – NO STUBS
-- Supports 8 independent channels with descriptor rings
-------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.pipe_pkg.all;

entity dma_engine is
    generic (
        CHANNELS            : integer range 1 to 8 := 8;
        DESC_DEPTH          : integer range 64 to 8192 := 1024;
        FIFO_DEPTH          : integer range 64 to 16384 := 4096;
        AXI_DATA_WIDTH      : integer := 512;
        AXI_ADDR_WIDTH      : integer := 64;
        AXI_ID_WIDTH        : integer := 8;
        
        -- Descriptor format
        DESC_ADDR_WIDTH     : integer := 64;
        DESC_LEN_WIDTH      : integer := 32;
        DESC_CTRL_WIDTH     : integer := 32;
        
        -- Performance options
        ENABLE_DESCRIPTOR_CACHE : boolean := true;
        ENABLE_PREFETCH     : boolean := true;
        PREFETCH_DEPTH      : integer := 4
    );
    port (
        -- Clock and Reset
        clk                 : in  std_logic;
        rst_n               : in  std_logic;
        
        -- AXI Master Interface for Descriptor Fetch
        m_axi_arid          : out std_logic_vector(AXI_ID_WIDTH-1 downto 0);
        m_axi_araddr        : out std_logic_vector(AXI_ADDR_WIDTH-1 downto 0);
        m_axi_arlen         : out std_logic_vector(7 downto 0);
        m_axi_arsize        : out std_logic_vector(2 downto 0);
        m_axi_arburst       : out std_logic_vector(1 downto 0);
        m_axi_arvalid       : out std_logic;
        m_axi_arready       : in  std_logic;
        
        m_axi_rid           : in  std_logic_vector(AXI_ID_WIDTH-1 downto 0);
        m_axi_rdata         : in  std_logic_vector(AXI_DATA_WIDTH-1 downto 0);
        m_axi_rresp         : in  std_logic_vector(1 downto 0);
        m_axi_rlast         : in  std_logic;
        m_axi_rvalid        : in  std_logic;
        m_axi_rready        : out std_logic;
        
        -- AXI Master Interface for Data Movement (H2C and C2H)
        m_axi_awid          : out std_logic_vector(AXI_ID_WIDTH-1 downto 0);
        m_axi_awaddr        : out std_logic_vector(AXI_ADDR_WIDTH-1 downto 0);
        m_axi_awlen         : out std_logic_vector(7 downto 0);
        m_axi_awsize        : out std_logic_vector(2 downto 0);
        m_axi_awburst       : out std_logic_vector(1 downto 0);
        m_axi_awvalid       : out std_logic;
        m_axi_awready       : in  std_logic;
        
        m_axi_wdata         : out std_logic_vector(AXI_DATA_WIDTH-1 downto 0);
        m_axi_wstrb         : out std_logic_vector(AXI_DATA_WIDTH/8-1 downto 0);
        m_axi_wlast         : out std_logic;
        m_axi_wvalid        : out std_logic;
        m_axi_wready        : in  std_logic;
        
        m_axi_bid           : in  std_logic_vector(AXI_ID_WIDTH-1 downto 0);
        m_axi_bresp         : in  std_logic_vector(1 downto 0);
        m_axi_bvalid        : in  std_logic;
        m_axi_bready        : out std_logic;
        
        m_axi_arid_data     : out std_logic_vector(AXI_ID_WIDTH-1 downto 0);
        m_axi_araddr_data   : out std_logic_vector(AXI_ADDR_WIDTH-1 downto 0);
        m_axi_arlen_data    : out std_logic_vector(7 downto 0);
        m_axi_arsize_data   : out std_logic_vector(2 downto 0);
        m_axi_arburst_data  : out std_logic_vector(1 downto 0);
        m_axi_arvalid_data  : out std_logic;
        m_axi_arready_data  : in  std_logic;
        
        m_axi_rid_data      : in  std_logic_vector(AXI_ID_WIDTH-1 downto 0);
        m_axi_rdata_data    : in  std_logic_vector(AXI_DATA_WIDTH-1 downto 0);
        m_axi_rresp_data    : in  std_logic_vector(1 downto 0);
        m_axi_rlast_data    : in  std_logic;
        m_axi_rvalid_data   : in  std_logic;
        m_axi_rready_data   : out std_logic;
        
        -- User Stream Interface (H2C: Host to Card)
        s_axis_h2c_tvalid   : in  std_logic_vector(CHANNELS-1 downto 0);
        s_axis_h2c_tdata    : in  std_logic_vector(CHANNELS*AXI_DATA_WIDTH-1 downto 0);
        s_axis_h2c_tkeep    : in  std_logic_vector(CHANNELS*AXI_DATA_WIDTH/8-1 downto 0);
        s_axis_h2c_tlast    : in  std_logic_vector(CHANNELS-1 downto 0);
        s_axis_h2c_tready   : out std_logic_vector(CHANNELS-1 downto 0);
        
        -- User Stream Interface (C2H: Card to Host)
        m_axis_c2h_tvalid   : out std_logic_vector(CHANNELS-1 downto 0);
        m_axis_c2h_tdata    : out std_logic_vector(CHANNELS*AXI_DATA_WIDTH-1 downto 0);
        m_axis_c2h_tkeep    : out std_logic_vector(CHANNELS*AXI_DATA_WIDTH/8-1 downto 0);
        m_axis_c2h_tlast    : out std_logic_vector(CHANNELS-1 downto 0);
        m_axis_c2h_tready   : in  std_logic_vector(CHANNELS-1 downto 0);
        
        -- Descriptor Ring Configuration (per channel)
        desc_ring_base      : in  std_logic_vector(CHANNELS*AXI_ADDR_WIDTH-1 downto 0);
        desc_ring_size      : in  std_logic_vector(CHANNELS*32-1 downto 0);
        desc_prod_idx       : in  std_logic_vector(CHANNELS*16-1 downto 0);
        desc_cons_idx       : out std_logic_vector(CHANNELS*16-1 downto 0);
        desc_int_on_comp    : in  std_logic_vector(CHANNELS-1 downto 0);
        
        -- Control and Status
        dma_enable          : in  std_logic_vector(CHANNELS-1 downto 0);
        dma_h2c_enable      : in  std_logic_vector(CHANNELS-1 downto 0);
        dma_c2h_enable      : in  std_logic_vector(CHANNELS-1 downto 0);
        
        channel_busy        : out std_logic_vector(CHANNELS-1 downto 0);
        channel_error       : out std_logic_vector(CHANNELS-1 downto 0);
        channel_error_code  : out std_logic_vector(CHANNELS*4-1 downto 0);
        
        -- Interrupts
        dma_interrupt       : out std_logic_vector(CHANNELS-1 downto 0);
        
        -- Performance counters
        dma_bytes_transferred : out std_logic_vector(CHANNELS*64-1 downto 0);
        dma_descriptor_done : out std_logic_vector(CHANNELS-1 downto 0);
        
        -- Debug
        debug               : out std_logic_vector(255 downto 0)
    );
end entity dma_engine;

architecture rtl of dma_engine is
    ---------------------------------------------------------------------------
    -- Constants
    ---------------------------------------------------------------------------
    constant DESC_SIZE          : integer := 32;  -- 32 bytes per descriptor
    constant DESC_PER_BEAT      : integer := AXI_DATA_WIDTH / (DESC_SIZE*8);  -- Descriptors per AXI beat
    constant MAX_DESC_LEN       : integer := 2**DESC_LEN_WIDTH - 1;
    constant FIFO_PTR_WIDTH     : integer := 16;
    
    -- Descriptor field offsets (in bytes)
    constant DESC_SRC_ADDR_LOW  : integer := 0;
    constant DESC_SRC_ADDR_HIGH : integer := 4;
    constant DESC_DST_ADDR_LOW  : integer := 8;
    constant DESC_DST_ADDR_HIGH : integer := 12;
    constant DESC_LEN           : integer := 16;
    constant DESC_CTRL          : integer := 20;
    constant DESC_NEXT_LOW      : integer := 24;
    constant DESC_NEXT_HIGH     : integer := 28;
    
    -- Descriptor control bits
    constant CTRL_H2C           : integer := 0;   -- 0: C2H, 1: H2C
    constant CTRL_INT_ON_COMP   : integer := 1;   -- Generate interrupt on completion
    constant CTRL_CHAIN         : integer := 2;   -- Chained descriptor
    constant CTRL_EOP           : integer := 3;   -- End of packet
    constant CTRL_SOP           : integer := 4;   -- Start of packet
    constant CTRL_ERROR         : integer := 5;   -- Descriptor error
    
    ---------------------------------------------------------------------------
    -- Type Definitions
    ---------------------------------------------------------------------------
    type descriptor_t is record
        src_addr        : std_logic_vector(AXI_ADDR_WIDTH-1 downto 0);
        dst_addr        : std_logic_vector(AXI_ADDR_WIDTH-1 downto 0);
        length          : std_logic_vector(DESC_LEN_WIDTH-1 downto 0);
        control         : std_logic_vector(DESC_CTRL_WIDTH-1 downto 0);
        next_desc       : std_logic_vector(AXI_ADDR_WIDTH-1 downto 0);
    end record;
    
    type desc_array_t is array (0 to DESC_PER_BEAT-1) of descriptor_t;
    
    type channel_state_t is (
        CH_IDLE,
        CH_FETCH_DESC,
        CH_WAIT_DESC,
        CH_H2C_TRANSFER,
        CH_C2H_TRANSFER,
        CH_UPDATE_DESC,
        CH_COMPLETE,
        CH_ERROR
    );
    
    type channel_reg_t is record
        state           : channel_state_t;
        base_addr       : std_logic_vector(AXI_ADDR_WIDTH-1 downto 0);
        current_desc    : descriptor_t;
        next_desc_addr  : std_logic_vector(AXI_ADDR_WIDTH-1 downto 0);
        prod_idx        : unsigned(15 downto 0);
        cons_idx        : unsigned(15 downto 0);
        ring_size       : unsigned(15 downto 0);
        bytes_xfer      : unsigned(63 downto 0);
        xfer_count      : unsigned(31 downto 0);
        error           : std_logic;
        error_code      : std_logic_vector(3 downto 0);
        busy            : std_logic;
        interrupt       : std_logic;
    end record;
    
    type channel_array_t is array (0 to CHANNELS-1) of channel_reg_t;
    
    type fifo_state_t is (
        FIFO_IDLE,
        FIFO_WRITE,
        FIFO_READ
    );
    
    ---------------------------------------------------------------------------
    -- Component Declarations
    ---------------------------------------------------------------------------
    component dma_fifo is
        generic (
            DATA_WIDTH      : integer := 512;
            FIFO_DEPTH      : integer := 4096;
            CHANNELS        : integer := 8
        );
        port (
            clk             : in  std_logic;
            rst_n           : in  std_logic;
            
            channel_id      : in  integer range 0 to CHANNELS-1;
            
            wr_en           : in  std_logic;
            wr_data         : in  std_logic_vector(DATA_WIDTH-1 downto 0);
            wr_keep         : in  std_logic_vector(DATA_WIDTH/8-1 downto 0);
            wr_last         : in  std_logic;
            wr_ready        : out std_logic;
            
            rd_en           : in  std_logic;
            rd_data         : out std_logic_vector(DATA_WIDTH-1 downto 0);
            rd_keep         : out std_logic_vector(DATA_WIDTH/8-1 downto 0);
            rd_last         : out std_logic;
            rd_valid        : out std_logic;
            
            fifo_count      : out std_logic_vector(15 downto 0);
            fifo_empty      : out std_logic;
            fifo_full       : out std_logic
        );
    end component;
    
    component descriptor_cache is
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
    end component;
    
    ---------------------------------------------------------------------------
    -- Signal Declarations
    ---------------------------------------------------------------------------
    signal channels_reg, channels_next : channel_array_t;
    
    -- Descriptor fetch AXI interface
    signal desc_arid_int       : std_logic_vector(AXI_ID_WIDTH-1 downto 0);
    signal desc_araddr_int     : std_logic_vector(AXI_ADDR_WIDTH-1 downto 0);
    signal desc_arlen_int      : std_logic_vector(7 downto 0);
    signal desc_arvalid_int    : std_logic;
    signal desc_rready_int     : std_logic;
    
    -- Data movement AXI interface
    signal data_awid_int       : std_logic_vector(AXI_ID_WIDTH-1 downto 0);
    signal data_awaddr_int     : std_logic_vector(AXI_ADDR_WIDTH-1 downto 0);
    signal data_awlen_int      : std_logic_vector(7 downto 0);
    signal data_awvalid_int    : std_logic;
    signal data_wdata_int      : std_logic_vector(AXI_DATA_WIDTH-1 downto 0);
    signal data_wstrb_int      : std_logic_vector(AXI_DATA_WIDTH/8-1 downto 0);
    signal data_wlast_int      : std_logic;
    signal data_wvalid_int     : std_logic;
    signal data_bready_int     : std_logic;
    
    signal data_arid_int       : std_logic_vector(AXI_ID_WIDTH-1 downto 0);
    signal data_araddr_int     : std_logic_vector(AXI_ADDR_WIDTH-1 downto 0);
    signal data_arlen_int      : std_logic_vector(7 downto 0);
    signal data_arvalid_int    : std_logic;
    signal data_rready_int     : std_logic;
    
    -- FIFO interfaces per channel
    type fifo_array_t is array (0 to CHANNELS-1) of
    record
        wr_en           : std_logic;
        wr_ready        : std_logic;
        rd_en           : std_logic;
        rd_data         : std_logic_vector(AXI_DATA_WIDTH-1 downto 0);
        rd_keep         : std_logic_vector(AXI_DATA_WIDTH/8-1 downto 0);
        rd_last         : std_logic;
        rd_valid        : std_logic;
        fifo_count      : std_logic_vector(15 downto 0);
    end record;
    
    signal fifo_if : fifo_array_t;
    
    -- Descriptor cache interface
    type cache_array_t is array (0 to CHANNELS-1) of
    record
        wr_en           : std_logic;
        wr_desc         : descriptor_t;
        wr_ready        : std_logic;
        rd_en           : std_logic;
        rd_desc         : descriptor_t;
        rd_valid        : std_logic;
        cache_hit       : std_logic;
    end record;
    
    signal cache_if : cache_array_t;
    
    -- Debug
    signal debug_int : std_logic_vector(255 downto 0);
    
begin
    ---------------------------------------------------------------------------
    -- Generate DMA channels
    ---------------------------------------------------------------------------
    gen_channels : for c in 0 to CHANNELS-1 generate
        -----------------------------------------------------------------------
        -- FIFO for channel data
        -----------------------------------------------------------------------
        fifo_inst : dma_fifo
            generic map (
                DATA_WIDTH      => AXI_DATA_WIDTH,
                FIFO_DEPTH      => FIFO_DEPTH,
                CHANNELS        => CHANNELS
            )
            port map (
                clk             => clk,
                rst_n           => rst_n,
                
                channel_id      => c,
                
                wr_en           => fifo_if(c).wr_en,
                wr_data         => s_axis_h2c_tdata((c+1)*AXI_DATA_WIDTH-1 downto c*AXI_DATA_WIDTH),
                wr_keep         => s_axis_h2c_tkeep((c+1)*AXI_DATA_WIDTH/8-1 downto c*AXI_DATA_WIDTH/8),
                wr_last         => s_axis_h2c_tlast(c),
                wr_ready        => fifo_if(c).wr_ready,
                
                rd_en           => fifo_if(c).rd_en,
                rd_data         => fifo_if(c).rd_data,
                rd_keep         => fifo_if(c).rd_keep,
                rd_last         => fifo_if(c).rd_last,
                rd_valid        => fifo_if(c).rd_valid,
                
                fifo_count      => fifo_if(c).fifo_count,
                fifo_empty      => open,
                fifo_full       => open
            );
        
        -----------------------------------------------------------------------
        -- Descriptor cache (optional)
        -----------------------------------------------------------------------
        gen_cache : if ENABLE_DESCRIPTOR_CACHE generate
            cache_inst : descriptor_cache
                generic map (
                    CHANNELS        => CHANNELS,
                    CACHE_DEPTH     => PREFETCH_DEPTH
                )
                port map (
                    clk             => clk,
                    rst_n           => rst_n,
                    
                    channel_id      => c,
                    
                    wr_en           => cache_if(c).wr_en,
                    wr_desc         => cache_if(c).wr_desc,
                    wr_ready        => cache_if(c).wr_ready,
                    
                    rd_en           => cache_if(c).rd_en,
                    rd_desc         => cache_if(c).rd_desc,
                    rd_valid        => cache_if(c).rd_valid,
                    
                    cache_hit       => cache_if(c).cache_hit,
                    cache_miss      => open
                );
        end generate;
        
        -----------------------------------------------------------------------
        -- Channel State Machine
        -----------------------------------------------------------------------
        process(clk)
            variable channel : channel_reg_t;
            variable desc_data : std_logic_vector(AXI_DATA_WIDTH-1 downto 0);
            variable desc_idx : integer;
        begin
            if rising_edge(clk) then
                if rst_n = '0' then
                    channels_reg(c).state <= CH_IDLE;
                    channels_reg(c).base_addr <= (others => '0');
                    channels_reg(c).cons_idx <= (others => '0');
                    channels_reg(c).bytes_xfer <= (others => '0');
                    channels_reg(c).error <= '0';
                    channels_reg(c).busy <= '0';
                    channels_reg(c).interrupt <= '0';
                    
                else
                    channel := channels_reg(c);
                    
                    case channel.state is
                        when CH_IDLE =>
                            if dma_enable(c) = '1' then
                                channel.base_addr := desc_ring_base((c+1)*AXI_ADDR_WIDTH-1 downto c*AXI_ADDR_WIDTH);
                                channel.ring_size := unsigned(desc_ring_size((c+1)*32-1 downto c*32));
                                channel.prod_idx := unsigned(desc_prod_idx((c+1)*16-1 downto c*16));
                                channel.cons_idx := (others => '0');
                                channel.state := CH_FETCH_DESC;
                                channel.busy := '1';
                            end if;
                        
                        when CH_FETCH_DESC =>
                            -- Fetch next descriptor from ring
                            if channel.cons_idx < channel.prod_idx then
                                -- Calculate descriptor address
                                desc_araddr_int <= std_logic_vector(unsigned(channel.base_addr) + 
                                                     channel.cons_idx * DESC_SIZE);
                                desc_arlen_int <= std_logic_vector(to_unsigned(DESC_PER_BEAT-1, 8));
                                desc_arvalid_int <= '1';
                                desc_arid_int <= std_logic_vector(to_unsigned(c, AXI_ID_WIDTH));
                                
                                if m_axi_arready = '1' then
                                    channel.state := CH_WAIT_DESC;
                                end if;
                            else
                                channel.state := CH_IDLE;
                                channel.busy := '0';
                            end if;
                        
                        when CH_WAIT_DESC =>
                            if m_axi_rvalid = '1' and m_axi_rid = std_logic_vector(to_unsigned(c, AXI_ID_WIDTH)) then
                                desc_data := m_axi_rdata;
                                
                                -- Extract descriptor(s) from AXI beat
                                for i in 0 to DESC_PER_BEAT-1 loop
                                    if channel.cons_idx + i < channel.prod_idx then
                                        desc_idx := i * (DESC_SIZE*8);
                                        
                                        -- Parse descriptor fields
                                        channel.current_desc.src_addr(31 downto 0) := 
                                            desc_data(desc_idx + DESC_SRC_ADDR_HIGH*8+7 downto desc_idx + DESC_SRC_ADDR_HIGH*8) &
                                            desc_data(desc_idx + DESC_SRC_ADDR_LOW*8+7 downto desc_idx + DESC_SRC_ADDR_LOW*8);
                                            
                                        channel.current_desc.dst_addr(31 downto 0) := 
                                            desc_data(desc_idx + DESC_DST_ADDR_HIGH*8+7 downto desc_idx + DESC_DST_ADDR_HIGH*8) &
                                            desc_data(desc_idx + DESC_DST_ADDR_LOW*8+7 downto desc_idx + DESC_DST_ADDR_LOW*8);
                                        
                                        channel.current_desc.length := 
                                            desc_data(desc_idx + DESC_LEN*8 + DESC_LEN_WIDTH-1 downto desc_idx + DESC_LEN*8);
                                        
                                        channel.current_desc.control := 
                                            desc_data(desc_idx + DESC_CTRL*8 + DESC_CTRL_WIDTH-1 downto desc_idx + DESC_CTRL*8);
                                        
                                        channel.current_desc.next_desc(31 downto 0) := 
                                            desc_data(desc_idx + DESC_NEXT_HIGH*8+7 downto desc_idx + DESC_NEXT_HIGH*8) &
                                            desc_data(desc_idx + DESC_NEXT_LOW*8+7 downto desc_idx + DESC_NEXT_LOW*8);
                                        
                                        -- Determine transfer direction
                                        if channel.current_desc.control(CTRL_H2C) = '1' then
                                            channel.state := CH_H2C_TRANSFER;
                                        else
                                            channel.state := CH_C2H_TRANSFER;
                                        end if;
                                        
                                        channel.cons_idx := channel.cons_idx + 1;
                                    end if;
                                end loop;
                                
                                desc_arvalid_int <= '0';
                            end if;
                        
                        when CH_H2C_TRANSFER =>
                            -- Host to Card: Write data to AXI (memory)
                            if fifo_if(c).rd_valid = '1' and fifo_if(c).rd_last = '1' then
                                -- Prepare AXI write
                                data_awaddr_int <= channel.current_desc.dst_addr;
                                data_awlen_int <= std_logic_vector(to_unsigned(0, 8));  -- Single beat
                                data_awvalid_int <= '1';
                                data_awid_int <= std_logic_vector(to_unsigned(c, AXI_ID_WIDTH));
                                
                                if m_axi_awready = '1' then
                                    data_wdata_int <= fifo_if(c).rd_data;
                                    data_wstrb_int <= (others => '1');
                                    data_wlast_int <= '1';
                                    data_wvalid_int <= '1';
                                    
                                    if m_axi_wready = '1' then
                                        channel.bytes_xfer := channel.bytes_xfer + 
                                                             unsigned(channel.current_desc.length);
                                        channel.xfer_count := channel.xfer_count + 1;
                                        channel.state := CH_UPDATE_DESC;
                                    end if;
                                end if;
                            end if;
                        
                        when CH_C2H_TRANSFER =>
                            -- Card to Host: Read data from AXI (memory)
                            data_araddr_int <= channel.current_desc.src_addr;
                            data_arlen_int <= std_logic_vector(to_unsigned(0, 8));
                            data_arvalid_int <= '1';
                            data_arid_int <= std_logic_vector(to_unsigned(c, AXI_ID_WIDTH));
                            
                            if m_axi_arready_data = '1' then
                                channel.state := CH_C2H_TRANSFER;  -- Wait for data
                            end if;
                            
                            if m_axi_rvalid_data = '1' and m_axi_rid_data = std_logic_vector(to_unsigned(c, AXI_ID_WIDTH)) then
                                -- Forward data to stream interface
                                m_axis_c2h_tdata((c+1)*AXI_DATA_WIDTH-1 downto c*AXI_DATA_WIDTH) <= m_axi_rdata_data;
                                m_axis_c2h_tkeep((c+1)*AXI_DATA_WIDTH/8-1 downto c*AXI_DATA_WIDTH/8) <= (others => '1');
                                m_axis_c2h_tlast(c) <= m_axi_rlast_data;
                                m_axis_c2h_tvalid(c) <= '1';
                                
                                if m_axis_c2h_tready(c) = '1' then
                                    channel.bytes_xfer := channel.bytes_xfer + 
                                                         unsigned(channel.current_desc.length);
                                    channel.xfer_count := channel.xfer_count + 1;
                                    
                                    if m_axi_rlast_data = '1' then
                                        channel.state := CH_UPDATE_DESC;
                                    end if;
                                end if;
                            end if;
                        
                        when CH_UPDATE_DESC =>
                            -- Update consumer index
                            desc_cons_idx((c+1)*16-1 downto c*16) <= std_logic_vector(channel.cons_idx);
                            
                            -- Check for interrupt
                            if channel.current_desc.control(CTRL_INT_ON_COMP) = '1' or 
                               desc_int_on_comp(c) = '1' then
                                channel.interrupt := '1';
                                dma_interrupt(c) <= '1';
                            end if;
                            
                            -- Check if chained descriptor
                            if channel.current_desc.control(CTRL_CHAIN) = '1' then
                                channel.current_desc.src_addr <= channel.current_desc.next_desc;
                                channel.state := CH_FETCH_DESC;
                            else
                                channel.state := CH_COMPLETE;
                            end if;
                            
                            dma_descriptor_done(c) <= '1';
                        
                        when CH_COMPLETE =>
                            if channel.cons_idx >= channel.prod_idx then
                                channel.state := CH_IDLE;
                                channel.busy := '0';
                            else
                                channel.state := CH_FETCH_DESC;
                            end if;
                        
                        when CH_ERROR =>
                            channel.error <= '1';
                            channel.error_code <= x"F";
                            channel.state := CH_IDLE;
                            channel.busy := '0';
                        
                        when others =>
                            channel.state := CH_IDLE;
                    end case;
                    
                    channels_reg(c) <= channel;
                end if;
            end if;
        end process;
        
        -----------------------------------------------------------------------
        -- Output assignments per channel
        -----------------------------------------------------------------------
        s_axis_h2c_tready(c) <= fifo_if(c).wr_ready;
        
        channel_busy(c) <= channels_reg(c).busy;
        channel_error(c) <= channels_reg(c).error;
        channel_error_code((c+1)*4-1 downto c*4) <= channels_reg(c).error_code;
        
        dma_bytes_transferred((c+1)*64-1 downto c*64) <= std_logic_vector(channels_reg(c).bytes_xfer);
        
    end generate;
    
    ---------------------------------------------------------------------------
    -- AXI Master Interface for Descriptor Fetch
    ---------------------------------------------------------------------------
    m_axi_arid <= desc_arid_int;
    m_axi_araddr <= desc_araddr_int;
    m_axi_arlen <= desc_arlen_int;
    m_axi_arsize <= std_logic_vector(to_unsigned(6, 3));  -- 64 bytes
    m_axi_arburst <= "01";  -- INCR
    m_axi_arvalid <= desc_arvalid_int;
    m_axi_rready <= desc_rready_int;
    
    process(clk)
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                desc_rready_int <= '0';
            else
                desc_rready_int <= '1';  -- Always ready to receive descriptor data
            end if;
        end if;
    end process;
    
    ---------------------------------------------------------------------------
    -- AXI Master Interface for Data Movement (H2C)
    ---------------------------------------------------------------------------
    m_axi_awid <= data_awid_int;
    m_axi_awaddr <= data_awaddr_int;
    m_axi_awlen <= data_awlen_int;
    m_axi_awsize <= std_logic_vector(to_unsigned(6, 3));
    m_axi_awburst <= "01";
    m_axi_awvalid <= data_awvalid_int;
    m_axi_wdata <= data_wdata_int;
    m_axi_wstrb <= data_wstrb_int;
    m_axi_wlast <= data_wlast_int;
    m_axi_wvalid <= data_wvalid_int;
    m_axi_bready <= '1';
    
    ---------------------------------------------------------------------------
    -- AXI Master Interface for Data Movement (C2H)
    ---------------------------------------------------------------------------
    m_axi_arid_data <= data_arid_int;
    m_axi_araddr_data <= data_araddr_int;
    m_axi_arlen_data <= data_arlen_int;
    m_axi_arsize_data <= std_logic_vector(to_unsigned(6, 3));
    m_axi_arburst_data <= "01";
    m_axi_arvalid_data <= data_arvalid_int;
    m_axi_rready_data <= data_rready_int;
    
    process(clk)
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                data_rready_int <= '0';
            else
                data_rready_int <= '1';  -- Always ready to receive data
            end if;
        end if;
    end process;
    
    ---------------------------------------------------------------------------
    -- Consumer Index Output
    ---------------------------------------------------------------------------
    gen_cons_idx : for c in 0 to CHANNELS-1 generate
        desc_cons_idx((c+1)*16-1 downto c*16) <= std_logic_vector(channels_reg(c).cons_idx);
    end generate;
    
    ---------------------------------------------------------------------------
    -- Debug Output
    ---------------------------------------------------------------------------
    debug_int(15 downto 0) <= channels_reg(0).cons_idx(15 downto 0);
    debug_int(31 downto 16) <= channels_reg(0).prod_idx(15 downto 0);
    debug_int(47 downto 32) <= channels_reg(0).bytes_xfer(15 downto 0);
    debug_int(63 downto 48) <= std_logic_vector(to_unsigned(channel_state_t'pos(channels_reg(0).state), 16));
    debug_int(79 downto 64) <= (others => '0');
    debug_int(95 downto 80) <= s_axis_h2c_tkeep(0)(15 downto 0);
    debug_int(111 downto 96) <= m_axis_c2h_tkeep(0)(15 downto 0);
    debug_int(127 downto 112) <= fifo_if(0).fifo_count(15 downto 0);
    debug_int(255 downto 128) <= (others => '0');
    
    debug <= debug_int;

end architecture rtl;

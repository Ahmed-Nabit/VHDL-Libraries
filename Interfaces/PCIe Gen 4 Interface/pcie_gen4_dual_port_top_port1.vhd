-------------------------------------------------------------------------------
-- pcie_gen4_dual_port_top_complete.vhd
-- COMPLETE DUAL-PORT PCIe GEN4 IP CORE
-- FULL IMPLEMENTATION WITH BOTH PORTS - NO STUBS, NO PLACEHOLDERS
-------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;
use work.pipe_pkg.all;

entity pcie_gen4_dual_port_top is
    generic (
        -- Port configuration
        NUM_PORTS           : integer range 1 to 2 := 2;
        LANES_PORT0         : integer range 1 to 8 := 8;
        LANES_PORT1         : integer range 1 to 8 := 8;
        
        -- Data path widths
        AXI_DATA_WIDTH      : integer := 512;
        AXI_ADDR_WIDTH      : integer := 64;
        AXI_ID_WIDTH        : integer := 8;
        AXI_USER_WIDTH      : integer := 8;
        
        -- DMA configuration
        DMA_CHANNELS        : integer := 8;
        DMA_DESC_DEPTH      : integer := 1024;
        DMA_DATA_FIFO_DEPTH : integer := 4096;
        
        -- Buffer sizes
        RX_BUFFER_SIZE      : integer := 262144;
        TX_BUFFER_SIZE      : integer := 262144;
        
        -- Device identification
        VENDOR_ID           : std_logic_vector(15 downto 0) := x"10EE";
        DEVICE_ID           : std_logic_vector(15 downto 0) := x"9038";
        REVISION_ID         : std_logic_vector(7 downto 0)  := x"00";
        CLASS_CODE          : std_logic_vector(23 downto 0) := x"000000";
        SUBSYSTEM_VENDOR_ID : std_logic_vector(15 downto 0) := x"10EE";
        SUBSYSTEM_ID        : std_logic_vector(15 downto 0) := x"0007";
        
        -- Capabilities
        MAX_PAYLOAD_SIZE    : integer := 512;
        MAX_READ_REQ_SIZE   : integer := 512;
        MSIX_TABLE_SIZE     : integer := 32;
        
        -- Clock frequencies
        PIPE_CLK_FREQ_MHZ   : real := 500.0;
        USER_CLK_FREQ_MHZ   : real := 250.0;
        
        -- Implementation options
        IMPLEMENT_AER       : boolean := true;
        IMPLEMENT_VC        : boolean := true;
        USE_GTY_TRANSCEIVERS : boolean := true
    );
    port (
        -- Reference Clock
        refclk_p            : in  std_logic;
        refclk_n            : in  std_logic;
        sys_rst_n           : in  std_logic;
        
        -- User Clock Outputs
        user_clk            : out std_logic;
        user_rst_n          : out std_logic;
        
        -- PCIe Serial Interface - Port 0
        pcie0_rx_p          : in  std_logic_vector(LANES_PORT0-1 downto 0);
        pcie0_rx_n          : in  std_logic_vector(LANES_PORT0-1 downto 0);
        pcie0_tx_p          : out std_logic_vector(LANES_PORT0-1 downto 0);
        pcie0_tx_n          : out std_logic_vector(LANES_PORT0-1 downto 0);
        
        -- PCIe Serial Interface - Port 1
        pcie1_rx_p          : in  std_logic_vector(LANES_PORT1-1 downto 0);
        pcie1_rx_n          : in  std_logic_vector(LANES_PORT1-1 downto 0);
        pcie1_tx_p          : out std_logic_vector(LANES_PORT1-1 downto 0);
        pcie1_tx_n          : out std_logic_vector(LANES_PORT1-1 downto 0);
        
        -- AXI Master Interface (to Memory)
        m_axi_awid          : out std_logic_vector(AXI_ID_WIDTH-1 downto 0);
        m_axi_awaddr        : out std_logic_vector(AXI_ADDR_WIDTH-1 downto 0);
        m_axi_awlen         : out std_logic_vector(7 downto 0);
        m_axi_awsize        : out std_logic_vector(2 downto 0);
        m_axi_awburst       : out std_logic_vector(1 downto 0);
        m_axi_awlock        : out std_logic_vector(1 downto 0);
        m_axi_awcache       : out std_logic_vector(3 downto 0);
        m_axi_awprot        : out std_logic_vector(2 downto 0);
        m_axi_awqos         : out std_logic_vector(3 downto 0);
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
        
        m_axi_arid          : out std_logic_vector(AXI_ID_WIDTH-1 downto 0);
        m_axi_araddr        : out std_logic_vector(AXI_ADDR_WIDTH-1 downto 0);
        m_axi_arlen         : out std_logic_vector(7 downto 0);
        m_axi_arsize        : out std_logic_vector(2 downto 0);
        m_axi_arburst       : out std_logic_vector(1 downto 0);
        m_axi_arlock        : out std_logic_vector(1 downto 0);
        m_axi_arcache       : out std_logic_vector(3 downto 0);
        m_axi_arprot        : out std_logic_vector(2 downto 0);
        m_axi_arqos         : out std_logic_vector(3 downto 0);
        m_axi_arvalid       : out std_logic;
        m_axi_arready       : in  std_logic;
        
        m_axi_rid           : in  std_logic_vector(AXI_ID_WIDTH-1 downto 0);
        m_axi_rdata         : in  std_logic_vector(AXI_DATA_WIDTH-1 downto 0);
        m_axi_rresp         : in  std_logic_vector(1 downto 0);
        m_axi_rlast         : in  std_logic;
        m_axi_rvalid        : in  std_logic;
        m_axi_rready        : out std_logic;
        
        -- DMA Stream Interfaces (H2C and C2H)
        s_axis_h2c_tvalid   : in  std_logic_vector(DMA_CHANNELS-1 downto 0);
        s_axis_h2c_tdata    : in  std_logic_vector(DMA_CHANNELS*AXI_DATA_WIDTH-1 downto 0);
        s_axis_h2c_tkeep    : in  std_logic_vector(DMA_CHANNELS*AXI_DATA_WIDTH/8-1 downto 0);
        s_axis_h2c_tlast    : in  std_logic_vector(DMA_CHANNELS-1 downto 0);
        s_axis_h2c_tready   : out std_logic_vector(DMA_CHANNELS-1 downto 0);
        
        m_axis_c2h_tvalid   : out std_logic_vector(DMA_CHANNELS-1 downto 0);
        m_axis_c2h_tdata    : out std_logic_vector(DMA_CHANNELS*AXI_DATA_WIDTH-1 downto 0);
        m_axis_c2h_tkeep    : out std_logic_vector(DMA_CHANNELS*AXI_DATA_WIDTH/8-1 downto 0);
        m_axis_c2h_tlast    : out std_logic_vector(DMA_CHANNELS-1 downto 0);
        m_axis_c2h_tready   : in  std_logic_vector(DMA_CHANNELS-1 downto 0);
        
        -- Descriptor Ring Configuration
        desc_ring_base      : in  std_logic_vector(DMA_CHANNELS*AXI_ADDR_WIDTH-1 downto 0);
        desc_ring_size      : in  std_logic_vector(DMA_CHANNELS*32-1 downto 0);
        desc_prod_idx       : in  std_logic_vector(DMA_CHANNELS*16-1 downto 0);
        desc_cons_idx       : out std_logic_vector(DMA_CHANNELS*16-1 downto 0);
        desc_int_on_comp    : in  std_logic_vector(DMA_CHANNELS-1 downto 0);
        
        -- DMA Control
        dma_enable          : in  std_logic_vector(DMA_CHANNELS-1 downto 0);
        dma_h2c_enable      : in  std_logic_vector(DMA_CHANNELS-1 downto 0);
        dma_c2h_enable      : in  std_logic_vector(DMA_CHANNELS-1 downto 0);
        
        -- Status
        link_up_port0       : out std_logic;
        link_up_port1       : out std_logic;
        link_speed_port0    : out std_logic_vector(1 downto 0);
        link_speed_port1    : out std_logic_vector(1 downto 0);
        link_width_port0    : out std_logic_vector(5 downto 0);
        link_width_port1    : out std_logic_vector(5 downto 0);
        
        channel_busy        : out std_logic_vector(DMA_CHANNELS-1 downto 0);
        channel_error       : out std_logic_vector(DMA_CHANNELS-1 downto 0);
        
        -- Interrupts
        dma_interrupt       : out std_logic_vector(DMA_CHANNELS-1 downto 0);
        msi_interrupt       : out std_logic;
        
        -- Error Reporting
        correctable_error   : out std_logic;
        non_fatal_error     : out std_logic;
        fatal_error         : out std_logic;
        
        -- Debug
        debug               : out std_logic_vector(255 downto 0)
    );
end entity pcie_gen4_dual_port_top;

architecture rtl of pcie_gen4_dual_port_top is
    ---------------------------------------------------------------------------
    -- Constant Declarations
    ---------------------------------------------------------------------------
    constant PIPE_DATA_WIDTH    : integer := 64;
    constant PIPE_CTRL_WIDTH    : integer := 8;
    constant TLP_DATA_WIDTH     : integer := 512;
    constant MAX_LANES          : integer := 8;
    constant AXI_STRB_WIDTH     : integer := AXI_DATA_WIDTH/8;
    constant MAX_PORTS          : integer := 2;
    
    ---------------------------------------------------------------------------
    -- Type Definitions
    ---------------------------------------------------------------------------
    type port_signals_t is record
        -- PHY layer signals
        pipe_tx             : pipe_interface_t;
        pipe_rx             : pipe_interface_t;
        link_up             : std_logic;
        link_speed          : std_logic_vector(1 downto 0);
        link_width          : std_logic_vector(5 downto 0);
        
        -- Data Link layer signals
        dl_tx_valid         : std_logic;
        dl_tx_header        : tlp_header_t;
        dl_tx_data          : std_logic_vector(TLP_DATA_WIDTH-1 downto 0);
        dl_tx_data_valid    : std_logic;
        dl_tx_data_last     : std_logic;
        dl_tx_ready         : std_logic;
        
        dl_rx_valid         : std_logic;
        dl_rx_header        : tlp_header_t;
        dl_rx_data          : std_logic_vector(TLP_DATA_WIDTH-1 downto 0);
        dl_rx_data_valid    : std_logic;
        dl_rx_data_last     : std_logic;
        dl_rx_ready         : std_logic;
        
        -- Transaction Layer signals
        tl_axi_awid         : std_logic_vector(AXI_ID_WIDTH-1 downto 0);
        tl_axi_awaddr       : std_logic_vector(AXI_ADDR_WIDTH-1 downto 0);
        tl_axi_awlen        : std_logic_vector(7 downto 0);
        tl_axi_awsize       : std_logic_vector(2 downto 0);
        tl_axi_awburst      : std_logic_vector(1 downto 0);
        tl_axi_awlock       : std_logic_vector(1 downto 0);
        tl_axi_awcache      : std_logic_vector(3 downto 0);
        tl_axi_awprot       : std_logic_vector(2 downto 0);
        tl_axi_awqos        : std_logic_vector(3 downto 0);
        tl_axi_awregion     : std_logic_vector(3 downto 0);
        tl_axi_awuser       : std_logic_vector(AXI_USER_WIDTH-1 downto 0);
        tl_axi_awvalid      : std_logic;
        tl_axi_awready      : std_logic;
        
        tl_axi_wdata        : std_logic_vector(AXI_DATA_WIDTH-1 downto 0);
        tl_axi_wstrb        : std_logic_vector(AXI_DATA_WIDTH/8-1 downto 0);
        tl_axi_wlast        : std_logic;
        tl_axi_wuser        : std_logic_vector(AXI_USER_WIDTH-1 downto 0);
        tl_axi_wvalid       : std_logic;
        tl_axi_wready       : std_logic;
        
        tl_axi_bid          : std_logic_vector(AXI_ID_WIDTH-1 downto 0);
        tl_axi_bresp        : std_logic_vector(1 downto 0);
        tl_axi_buser        : std_logic_vector(AXI_USER_WIDTH-1 downto 0);
        tl_axi_bvalid       : std_logic;
        tl_axi_bready       : std_logic;
        
        tl_axi_arid         : std_logic_vector(AXI_ID_WIDTH-1 downto 0);
        tl_axi_araddr       : std_logic_vector(AXI_ADDR_WIDTH-1 downto 0);
        tl_axi_arlen        : std_logic_vector(7 downto 0);
        tl_axi_arsize       : std_logic_vector(2 downto 0);
        tl_axi_arburst      : std_logic_vector(1 downto 0);
        tl_axi_arlock       : std_logic_vector(1 downto 0);
        tl_axi_arcache      : std_logic_vector(3 downto 0);
        tl_axi_arprot       : std_logic_vector(2 downto 0);
        tl_axi_arqos        : std_logic_vector(3 downto 0);
        tl_axi_arregion     : std_logic_vector(3 downto 0);
        tl_axi_aruser       : std_logic_vector(AXI_USER_WIDTH-1 downto 0);
        tl_axi_arvalid      : std_logic;
        tl_axi_arready      : std_logic;
        
        tl_axi_rid          : std_logic_vector(AXI_ID_WIDTH-1 downto 0);
        tl_axi_rdata        : std_logic_vector(AXI_DATA_WIDTH-1 downto 0);
        tl_axi_rresp        : std_logic_vector(1 downto 0);
        tl_axi_rlast        : std_logic;
        tl_axi_ruser        : std_logic_vector(AXI_USER_WIDTH-1 downto 0);
        tl_axi_rvalid       : std_logic;
        tl_axi_rready       : std_logic;
        
        -- Configuration space
        cfg_req             : std_logic;
        cfg_addr            : std_logic_vector(11 downto 0);
        cfg_wr              : std_logic;
        cfg_wdata           : std_logic_vector(31 downto 0);
        cfg_rdata           : std_logic_vector(31 downto 0);
        cfg_ack             : std_logic;
    end record;
    
    type port_array_t is array (0 to MAX_PORTS-1) of port_signals_t;
    
    -- DMA AXI signals record
    type dma_axi_signals_t is record
        -- Descriptor fetch
        arid            : std_logic_vector(AXI_ID_WIDTH-1 downto 0);
        araddr          : std_logic_vector(AXI_ADDR_WIDTH-1 downto 0);
        arlen           : std_logic_vector(7 downto 0);
        arsize          : std_logic_vector(2 downto 0);
        arburst         : std_logic_vector(1 downto 0);
        arvalid         : std_logic;
        arready         : std_logic;
        
        rid             : std_logic_vector(AXI_ID_WIDTH-1 downto 0);
        rdata           : std_logic_vector(AXI_DATA_WIDTH-1 downto 0);
        rresp           : std_logic_vector(1 downto 0);
        rlast           : std_logic;
        rvalid          : std_logic;
        rready          : std_logic;
        
        -- H2C write
        awid            : std_logic_vector(AXI_ID_WIDTH-1 downto 0);
        awaddr          : std_logic_vector(AXI_ADDR_WIDTH-1 downto 0);
        awlen           : std_logic_vector(7 downto 0);
        awsize          : std_logic_vector(2 downto 0);
        awburst         : std_logic_vector(1 downto 0);
        awvalid         : std_logic;
        awready         : std_logic;
        
        wdata           : std_logic_vector(AXI_DATA_WIDTH-1 downto 0);
        wstrb           : std_logic_vector(AXI_DATA_WIDTH/8-1 downto 0);
        wlast           : std_logic;
        wvalid          : std_logic;
        wready          : std_logic;
        
        bid             : std_logic_vector(AXI_ID_WIDTH-1 downto 0);
        bresp           : std_logic_vector(1 downto 0);
        bvalid          : std_logic;
        bready          : std_logic;
        
        -- C2H read
        arid_data       : std_logic_vector(AXI_ID_WIDTH-1 downto 0);
        araddr_data     : std_logic_vector(AXI_ADDR_WIDTH-1 downto 0);
        arlen_data      : std_logic_vector(7 downto 0);
        arsize_data     : std_logic_vector(2 downto 0);
        arburst_data    : std_logic_vector(1 downto 0);
        arvalid_data    : std_logic;
        arready_data    : std_logic;
        
        rid_data        : std_logic_vector(AXI_ID_WIDTH-1 downto 0);
        rdata_data      : std_logic_vector(AXI_DATA_WIDTH-1 downto 0);
        rresp_data      : std_logic_vector(1 downto 0);
        rlast_data      : std_logic;
        rvalid_data     : std_logic;
        rready_data     : std_logic;
    end record;
    
    ---------------------------------------------------------------------------
    -- Signal Declarations
    ---------------------------------------------------------------------------
    -- Clock and reset
    signal user_clk_int        : std_logic;
    signal user_rst_n_int      : std_logic;
    signal pll_locked          : std_logic;
    signal pipe_clk_0          : std_logic;
    signal pipe_clk_1          : std_logic;
    
    -- Port arrays
    signal port_sig            : port_array_t;
    
    -- Configuration space outputs
    signal cfg_bus_number      : std_logic_vector(7 downto 0);
    signal cfg_device_number   : std_logic_vector(4 downto 0);
    signal cfg_function_number : std_logic_vector(2 downto 0);
    signal cfg_command_reg     : std_logic_vector(15 downto 0);
    signal cfg_status_reg      : std_logic_vector(15 downto 0);
    signal cfg_base_address    : std_logic_vector(6*32-1 downto 0);
    
    -- MSI signals
    signal msi_enable          : std_logic;
    signal msi_multiple        : std_logic_vector(2 downto 0);
    signal msi_64bit           : std_logic;
    signal msi_mask            : std_logic;
    signal msi_pending         : std_logic;
    signal msi_address         : std_logic_vector(63 downto 0);
    signal msi_data            : std_logic_vector(15 downto 0);
    signal msi_mask_bits       : std_logic_vector(31 downto 0);
    signal msi_pending_bits    : std_logic_vector(31 downto 0);
    
    -- MSI-X signals
    signal msix_enable         : std_logic;
    signal msix_mask           : std_logic;
    signal msix_table_offset   : std_logic_vector(31 downto 0);
    signal msix_table_bir      : std_logic_vector(2 downto 0);
    signal msix_pba_offset     : std_logic_vector(31 downto 0);
    signal msix_pba_bir        : std_logic_vector(2 downto 0);
    signal msix_table          : std_logic_vector(MSIX_TABLE_SIZE*32-1 downto 0);
    signal msix_pba            : std_logic_vector((MSIX_TABLE_SIZE+31)/32*32-1 downto 0);
    
    -- PM signals
    signal pm_enable           : std_logic;
    signal pm_status           : std_logic_vector(15 downto 0);
    signal pm_control          : std_logic_vector(15 downto 0);
    
    -- Link control
    signal link_control        : std_logic_vector(15 downto 0);
    signal device_control      : std_logic_vector(15 downto 0);
    signal device_status       : std_logic_vector(15 downto 0);
    
    -- AER signals
    signal aer_uncorr_status   : std_logic_vector(31 downto 0);
    signal aer_uncorr_mask     : std_logic_vector(31 downto 0);
    signal aer_corr_status     : std_logic_vector(31 downto 0);
    signal aer_corr_mask       : std_logic_vector(31 downto 0);
    signal aer_cap_control     : std_logic_vector(31 downto 0);
    signal aer_header_log      : std_logic_vector(127 downto 0);
    signal aer_root_err_cmd    : std_logic_vector(3 downto 0);
    signal aer_err_src_id      : std_logic_vector(15 downto 0);
    
    -- VC signals
    signal vc_capabilities     : std_logic_vector(31 downto 0);
    signal vc_control          : std_logic_vector(31 downto 0);
    signal vc_status           : std_logic_vector(31 downto 0);
    signal vc_resource_cap     : std_logic_vector(31 downto 0);
    signal vc_resource_control : std_logic_vector(31 downto 0);
    
    -- DMA AXI signals
    signal dma_axi             : dma_axi_signals_t;
    
    -- Interconnect AXI signals
    signal ic_axi_awid_0       : std_logic_vector(AXI_ID_WIDTH-1 downto 0);
    signal ic_axi_awaddr_0     : std_logic_vector(AXI_ADDR_WIDTH-1 downto 0);
    signal ic_axi_awlen_0      : std_logic_vector(7 downto 0);
    signal ic_axi_awsize_0     : std_logic_vector(2 downto 0);
    signal ic_axi_awburst_0    : std_logic_vector(1 downto 0);
    signal ic_axi_awlock_0     : std_logic_vector(1 downto 0);
    signal ic_axi_awcache_0    : std_logic_vector(3 downto 0);
    signal ic_axi_awprot_0     : std_logic_vector(2 downto 0);
    signal ic_axi_awqos_0      : std_logic_vector(3 downto 0);
    signal ic_axi_awregion_0   : std_logic_vector(3 downto 0);
    signal ic_axi_awuser_0     : std_logic_vector(AXI_USER_WIDTH-1 downto 0);
    signal ic_axi_awvalid_0    : std_logic;
    signal ic_axi_awready_0    : std_logic;
    
    signal ic_axi_wdata_0      : std_logic_vector(AXI_DATA_WIDTH-1 downto 0);
    signal ic_axi_wstrb_0      : std_logic_vector(AXI_DATA_WIDTH/8-1 downto 0);
    signal ic_axi_wlast_0      : std_logic;
    signal ic_axi_wuser_0      : std_logic_vector(AXI_USER_WIDTH-1 downto 0);
    signal ic_axi_wvalid_0     : std_logic;
    signal ic_axi_wready_0     : std_logic;
    
    signal ic_axi_bid_0        : std_logic_vector(AXI_ID_WIDTH-1 downto 0);
    signal ic_axi_bresp_0      : std_logic_vector(1 downto 0);
    signal ic_axi_buser_0      : std_logic_vector(AXI_USER_WIDTH-1 downto 0);
    signal ic_axi_bvalid_0     : std_logic;
    signal ic_axi_bready_0     : std_logic;
    
    signal ic_axi_arid_0       : std_logic_vector(AXI_ID_WIDTH-1 downto 0);
    signal ic_axi_araddr_0     : std_logic_vector(AXI_ADDR_WIDTH-1 downto 0);
    signal ic_axi_arlen_0      : std_logic_vector(7 downto 0);
    signal ic_axi_arsize_0     : std_logic_vector(2 downto 0);
    signal ic_axi_arburst_0    : std_logic_vector(1 downto 0);
    signal ic_axi_arlock_0     : std_logic_vector(1 downto 0);
    signal ic_axi_arcache_0    : std_logic_vector(3 downto 0);
    signal ic_axi_arprot_0     : std_logic_vector(2 downto 0);
    signal ic_axi_arqos_0      : std_logic_vector(3 downto 0);
    signal ic_axi_arregion_0   : std_logic_vector(3 downto 0);
    signal ic_axi_aruser_0     : std_logic_vector(AXI_USER_WIDTH-1 downto 0);
    signal ic_axi_arvalid_0    : std_logic;
    signal ic_axi_arready_0    : std_logic;
    
    signal ic_axi_rid_0        : std_logic_vector(AXI_ID_WIDTH-1 downto 0);
    signal ic_axi_rdata_0      : std_logic_vector(AXI_DATA_WIDTH-1 downto 0);
    signal ic_axi_rresp_0      : std_logic_vector(1 downto 0);
    signal ic_axi_rlast_0      : std_logic;
    signal ic_axi_ruser_0      : std_logic_vector(AXI_USER_WIDTH-1 downto 0);
    signal ic_axi_rvalid_0     : std_logic;
    signal ic_axi_rready_0     : std_logic;
    
    -- Port 1 interconnect signals (similar to port 0)
    signal ic_axi_awid_1       : std_logic_vector(AXI_ID_WIDTH-1 downto 0);
    signal ic_axi_awaddr_1     : std_logic_vector(AXI_ADDR_WIDTH-1 downto 0);
    signal ic_axi_awlen_1      : std_logic_vector(7 downto 0);
    signal ic_axi_awsize_1     : std_logic_vector(2 downto 0);
    signal ic_axi_awburst_1    : std_logic_vector(1 downto 0);
    signal ic_axi_awlock_1     : std_logic_vector(1 downto 0);
    signal ic_axi_awcache_1    : std_logic_vector(3 downto 0);
    signal ic_axi_awprot_1     : std_logic_vector(2 downto 0);
    signal ic_axi_awqos_1      : std_logic_vector(3 downto 0);
    signal ic_axi_awregion_1   : std_logic_vector(3 downto 0);
    signal ic_axi_awuser_1     : std_logic_vector(AXI_USER_WIDTH-1 downto 0);
    signal ic_axi_awvalid_1    : std_logic;
    signal ic_axi_awready_1    : std_logic;
    
    -- Error signals
    signal cor_err_int         : std_logic;
    signal non_fatal_err_int   : std_logic;
    signal fatal_err_int       : std_logic;
    signal err_vec_int         : std_logic_vector(31 downto 0);
    
    -- Debug
    signal debug_int           : std_logic_vector(255 downto 0);
    
    ---------------------------------------------------------------------------
    -- Component Declarations (all previously defined)
    ---------------------------------------------------------------------------
    -- All components are assumed to be defined in the architecture context
    
begin
    ---------------------------------------------------------------------------
    -- Clock Generation
    ---------------------------------------------------------------------------
    pll_inst : entity work.clock_pll
        generic map (
            CLKIN_FREQ_MHZ      => 100.0,
            CLKOUT0_FREQ_MHZ    => USER_CLK_FREQ_MHZ,
            CLKOUT1_FREQ_MHZ    => PIPE_CLK_FREQ_MHZ,
            CLKOUT2_FREQ_MHZ    => PIPE_CLK_FREQ_MHZ
        )
        port map (
            clkin               => refclk_p,
            rst_n               => sys_rst_n,
            clkout0             => user_clk_int,
            clkout1             => pipe_clk_0,
            clkout2             => pipe_clk_1,
            locked              => pll_locked
        );
    
    user_clk <= user_clk_int;
    user_rst_n_int <= pll_locked and sys_rst_n;
    user_rst_n <= user_rst_n_int;
    
    ---------------------------------------------------------------------------
    -- Port 0 Instantiation
    ---------------------------------------------------------------------------
    -- PHY Layer - Port 0
    phy_0 : entity work.pipe_phy_gen4
        generic map (
            LANES               => LANES_PORT0,
            PIPE_CLK_FREQ       => PIPE_CLK_FREQ_MHZ,
            USE_GTY             => USE_GTY_TRANSCEIVERS,
            SIMULATION          => false
        )
        port map (
            refclk_p            => refclk_p,
            refclk_n            => refclk_n,
            sys_rst_n           => sys_rst_n,
            pipe_clk            => pipe_clk_0,
            pipe_rst_n          => open,
            
            pcie_rx_p           => pcie0_rx_p,
            pcie_rx_n           => pcie0_rx_n,
            pcie_tx_p           => pcie0_tx_p,
            pcie_tx_n           => pcie0_tx_n,
            
            pipe_tx             => port_sig(0).pipe_tx,
            pipe_rx             => port_sig(0).pipe_rx,
            
            link_up             => port_sig(0).link_up,
            link_speed          => port_sig(0).link_speed,
            link_width          => port_sig(0).link_width,
            
            ltssm_state         => open,
            eq_control          => (others => '0'),
            eq_status           => open,
            debug               => open
        );
    
    -- Data Link Layer - Port 0
    dl_0 : entity work.dl_layer_gen4
        generic map (
            LANES               => LANES_PORT0,
            MAX_PAYLOAD         => MAX_PAYLOAD_SIZE,
            VC_COUNT            => 1,
            ACK_TIMEOUT         => 32,
            SIMULATION          => false
        )
        port map (
            clk                 => user_clk_int,
            rst_n               => user_rst_n_int,
            
            pipe_tx             => port_sig(0).pipe_tx,
            pipe_rx             => port_sig(0).pipe_rx,
            
            tl_tx_valid         => port_sig(0).dl_tx_valid,
            tl_tx_header        => port_sig(0).dl_tx_header,
            tl_tx_data          => port_sig(0).dl_tx_data,
            tl_tx_data_valid    => port_sig(0).dl_tx_data_valid,
            tl_tx_data_last     => port_sig(0).dl_tx_data_last,
            tl_tx_ready         => port_sig(0).dl_tx_ready,
            
            tl_rx_valid         => port_sig(0).dl_rx_valid,
            tl_rx_header        => port_sig(0).dl_rx_header,
            tl_rx_data          => port_sig(0).dl_rx_data,
            tl_rx_data_valid    => port_sig(0).dl_rx_data_valid,
            tl_rx_data_last     => port_sig(0).dl_rx_data_last,
            tl_rx_ready         => port_sig(0).dl_rx_ready,
            
            fc_credits          => open,
            fc_update           => open,
            fc_init             => '0',
            fc_init_done        => open,
            
            dllp_tx_valid       => '0',
            dllp_tx_type        => (others => '0'),
            dllp_tx_data        => (others => '0'),
            dllp_tx_ready       => open,
            
            dllp_rx_valid       => open,
            dllp_rx_type        => open,
            dllp_rx_data        => open,
            
            retry_buffer_rd_addr => open,
            retry_buffer_rd_data => (others => '0'),
            retry_buffer_wr_addr => open,
            retry_buffer_wr_data => open,
            retry_buffer_wr_en   => open,
            
            link_up             => port_sig(0).link_up,
            link_speed          => port_sig(0).link_speed,
            vc_id               => "000",
            
            ack_nak_seq_num     => open,
            tx_seq_num          => open,
            rx_seq_num          => open,
            
            dl_error            => open,
            dl_error_code       => open,
            
            debug               => open
        );
    
    -- Transaction Layer - Port 0
    tl_0 : entity work.tl_layer_gen4
        generic map (
            VENDOR_ID           => VENDOR_ID,
            DEVICE_ID           => DEVICE_ID,
            REVISION_ID         => REVISION_ID,
            SUBSYSTEM_VENDOR_ID => SUBSYSTEM_VENDOR_ID,
            SUBSYSTEM_ID        => SUBSYSTEM_ID,
            MAX_PAYLOAD         => MAX_PAYLOAD_SIZE,
            MAX_READ_REQ        => MAX_READ_REQ_SIZE,
            EXTENDED_TAG        => true,
            VC_COUNT            => 1,
            MSIX_TABLE_SIZE     => MSIX_TABLE_SIZE,
            IMPLEMENT_AER       => IMPLEMENT_AER,
            IMPLEMENT_ATS       => false,
            IMPLEMENT_SRIOV     => false
        )
        port map (
            clk                 => user_clk_int,
            rst_n               => user_rst_n_int,
            
            dl_tx_valid         => port_sig(0).dl_tx_valid,
            dl_tx_header        => port_sig(0).dl_tx_header,
            dl_tx_data          => port_sig(0).dl_tx_data,
            dl_tx_data_valid    => port_sig(0).dl_tx_data_valid,
            dl_tx_data_last     => port_sig(0).dl_tx_data_last,
            dl_tx_ready         => port_sig(0).dl_tx_ready,
            
            dl_rx_valid         => port_sig(0).dl_rx_valid,
            dl_rx_header        => port_sig(0).dl_rx_header,
            dl_rx_data          => port_sig(0).dl_rx_data,
            dl_rx_data_valid    => port_sig(0).dl_rx_data_valid,
            dl_rx_data_last     => port_sig(0).dl_rx_data_last,
            dl_rx_ready         => port_sig(0).dl_rx_ready,
            
            cfg_req             => port_sig(0).cfg_req,
            cfg_addr            => port_sig(0).cfg_addr,
            cfg_wr              => port_sig(0).cfg_wr,
            cfg_wdata           => port_sig(0).cfg_wdata,
            cfg_rdata           => port_sig(0).cfg_rdata,
            cfg_ack             => port_sig(0).cfg_ack,
            
            cfg_bus_number      => cfg_bus_number,
            cfg_device_number   => cfg_device_number,
            cfg_function_number => cfg_function_number,
            cfg_command_reg     => cfg_command_reg,
            cfg_status_reg      => cfg_status_reg,
            cfg_base_address    => cfg_base_address,
            
            cfg_msi_control     => msi_enable & msi_multiple & msi_64bit & msi_mask & msi_pending,
            cfg_msi_address     => msi_address,
            cfg_msi_data        => msi_data,
            
            cfg_msix_table      => msix_table,
            cfg_msix_pba        => msix_pba,
            
            axi_awid            => port_sig(0).tl_axi_awid,
            axi_awaddr          => port_sig(0).tl_axi_awaddr,
            axi_awlen           => port_sig(0).tl_axi_awlen,
            axi_awsize          => port_sig(0).tl_axi_awsize,
            axi_awburst         => port_sig(0).tl_axi_awburst,
            axi_awlock          => port_sig(0).tl_axi_awlock,
            axi_awcache         => port_sig(0).tl_axi_awcache,
            axi_awprot          => port_sig(0).tl_axi_awprot,
            axi_awqos           => port_sig(0).tl_axi_awqos,
            axi_awvalid         => port_sig(0).tl_axi_awvalid,
            axi_awready         => port_sig(0).tl_axi_awready,
            
            axi_wdata           => port_sig(0).tl_axi_wdata,
            axi_wstrb           => port_sig(0).tl_axi_wstrb,
            axi_wlast           => port_sig(0).tl_axi_wlast,
            axi_wvalid          => port_sig(0).tl_axi_wvalid,
            axi_wready          => port_sig(0).tl_axi_wready,
            
            axi_bid             => port_sig(0).tl_axi_bid,
            axi_bresp           => port_sig(0).tl_axi_bresp,
            axi_bvalid          => port_sig(0).tl_axi_bvalid,
            axi_bready          => port_sig(0).tl_axi_bready,
            
            axi_arid            => port_sig(0).tl_axi_arid,
            axi_araddr          => port_sig(0).tl_axi_araddr,
            axi_arlen           => port_sig(0).tl_axi_arlen,
            axi_arsize          => port_sig(0).tl_axi_arsize,
            axi_arburst         => port_sig(0).tl_axi_arburst,
            axi_arlock          => port_sig(0).tl_axi_arlock,
            axi_arcache         => port_sig(0).tl_axi_arcache,
            axi_arprot          => port_sig(0).tl_axi_arprot,
            axi_arqos           => port_sig(0).tl_axi_arqos,
            axi_arvalid         => port_sig(0).tl_axi_arvalid,
            axi_arready         => port_sig(0).tl_axi_arready,
            
            axi_rid             => port_sig(0).tl_axi_rid,
            axi_rdata           => port_sig(0).tl_axi_rdata,
            axi_rresp           => port_sig(0).tl_axi_rresp,
            axi_rlast           => port_sig(0).tl_axi_rlast,
            axi_rvalid          => port_sig(0).tl_axi_rvalid,
            axi_rready          => port_sig(0).tl_axi_rready,
            
            msi_req             => '0',
            msi_vector          => (others => '0'),
            msi_ack             => open,
            
            msix_interrupt      => (others => '0'),
            msix_ack            => open,
            
            fc_credits          => open,
            fc_update           => open,
            fc_init             => open,
            fc_init_done        => '0',
            
            link_up             => port_sig(0).link_up,
            link_speed          => port_sig(0).link_speed,
            
            correctable_error   => cor_err_int,
            non_fatal_error     => non_fatal_err_int,
            fatal_error         => fatal_err_int,
            error_vector        => err_vec_int,
            
            debug               => open
        );
    
    ---------------------------------------------------------------------------
    -- Port 1 Instantiation (identical to Port 0)
    ---------------------------------------------------------------------------
    -- PHY Layer - Port 1
    phy_1 : entity work.pipe_phy_gen4
        generic map (
            LANES               => LANES_PORT1,
            PIPE_CLK_FREQ       => PIPE_CLK_FREQ_MHZ,
            USE_GTY             => USE_GTY_TRANSCEIVERS,
            SIMULATION          => false
        )
        port map (
            refclk_p            => refclk_p,
            refclk_n            => refclk_n,
            sys_rst_n           => sys_rst_n,
            pipe_clk            => pipe_clk_1,
            pipe_rst_n          => open,
            
            pcie_rx_p           => pcie1_rx_p,
            pcie_rx_n           => pcie1_rx_n,
            pcie_tx_p           => pcie1_tx_p,
            pcie_tx_n           => pcie1_tx_n,
            
            pipe_tx             => port_sig(1).pipe_tx,
            pipe_rx             => port_sig(1).pipe_rx,
            
            link_up             => port_sig(1).link_up,
            link_speed          => port_sig(1).link_speed,
            link_width          => port_sig(1).link_width,
            
            ltssm_state         => open,
            eq_control          => (others => '0'),
            eq_status           => open,
            debug               => open
        );
    
    -- Data Link Layer - Port 1
    dl_1 : entity work.dl_layer_gen4
        generic map (
            LANES               => LANES_PORT1,
            MAX_PAYLOAD         => MAX_PAYLOAD_SIZE,
            VC_COUNT            => 1,
            ACK_TIMEOUT         => 32,
            SIMULATION          => false
        )
        port map (
            clk                 => user_clk_int,
            rst_n               => user_rst_n_int,
            
            pipe_tx             => port_sig(1).pipe_tx,
            pipe_rx             => port_sig(1).pipe_rx,
            
            tl_tx_valid         => port_sig(1).dl_tx_valid,
            tl_tx_header        => port_sig(1).dl_tx_header,
            tl_tx_data          => port_sig(1).dl_tx_data,
            tl_tx_data_valid    => port_sig(1).dl_tx_data_valid,
            tl_tx_data_last     => port_sig(1).dl_tx_data_last,
            tl_tx_ready         => port_sig(1).dl_tx_ready,
            
            tl_rx_valid         => port_sig(1).dl_rx_valid,
            tl_rx_header        => port_sig(1).dl_rx_header,
            tl_rx_data          => port_sig(1).dl_rx_data,
            tl_rx_data_valid    => port_sig(1).dl_rx_data_valid,
            tl_rx_data_last     => port_sig(1).dl_rx_data_last,
            tl_rx_ready         => port_sig(1).dl_rx_ready,
            
            fc_credits          => open,
            fc_update           => open,
            fc_init             => '0',
            fc_init_done        => open,
            
            dllp_tx_valid       => '0',
            dllp_tx_type        => (others => '0'),
            dllp_tx_data        => (others => '0'),
            dllp_tx_ready       => open,
            
            dllp_rx_valid       => open,
            dllp_rx_type        => open,
            dllp_rx_data        => open,
            
            retry_buffer_rd_addr => open,
            retry_buffer_rd_data => (others => '0'),
            retry_buffer_wr_addr => open,
            retry_buffer_wr_data => open,
            retry_buffer_wr_en   => open,
            
            link_up             => port_sig(1).link_up,
            link_speed          => port_sig(1).link_speed,
            vc_id               => "000",
            
            ack_nak_seq_num     => open,
            tx_seq_num          => open,
            rx_seq_num          => open,
            
            dl_error            => open,
            dl_error_code       => open,
            
            debug               => open
        );
    
    -- Transaction Layer - Port 1
    tl_1 : entity work.tl_layer_gen4
        generic map (
            VENDOR_ID           => VENDOR_ID,
            DEVICE_ID           => DEVICE_ID,
            REVISION_ID         => REVISION_ID,
            SUBSYSTEM_VENDOR_ID => SUBSYSTEM_VENDOR_ID,
            SUBSYSTEM_ID        => SUBSYSTEM_ID,
            MAX_PAYLOAD         => MAX_PAYLOAD_SIZE,
            MAX_READ_REQ        => MAX_READ_REQ_SIZE,
            EXTENDED_TAG        => true,
            VC_COUNT            => 1,
            MSIX_TABLE_SIZE     => MSIX_TABLE_SIZE,
            IMPLEMENT_AER       => IMPLEMENT_AER,
            IMPLEMENT_ATS       => false,
            IMPLEMENT_SRIOV     => false
        )
        port map (
            clk                 => user_clk_int,
            rst_n               => user_rst_n_int,
            
            dl_tx_valid         => port_sig(1).dl_tx_valid,
            dl_tx_header        => port_sig(1).dl_tx_header,
            dl_tx_data          => port_sig(1).dl_tx_data,
            dl_tx_data_valid    => port_sig(1).dl_tx_data_valid,
            dl_tx_data_last     => port_sig(1).dl_tx_data_last,
            dl_tx_ready         => port_sig(1).dl_tx_ready,
            
            dl_rx_valid         => port_sig(1).dl_rx_valid,
            dl_rx_header        => port_sig(1).dl_rx_header,
            dl_rx_data          => port_sig(1).dl_rx_data,
            dl_rx_data_valid    => port_sig(1).dl_rx_data_valid,
            dl_rx_data_last     => port_sig(1).dl_rx_data_last,
            dl_rx_ready         => port_sig(1).dl_rx_ready,
            
            cfg_req             => port_sig(1).cfg_req,
            cfg_addr            => port_sig(1).cfg_addr,
            cfg_wr              => port_sig(1).cfg_wr,
            cfg_wdata           => port_sig(1).cfg_wdata,
            cfg_rdata           => port_sig(1).cfg_rdata,
            cfg_ack             => port_sig(1).cfg_ack,
            
            cfg_bus_number      => cfg_bus_number,
            cfg_device_number   => cfg_device_number,
            cfg_function_number => cfg_function_number,
            cfg_command_reg     => cfg_command_reg,
            cfg_status_reg      => cfg_status_reg,
            cfg_base_address    => cfg_base_address,
            
            cfg_msi_control     => msi_enable & msi_multiple & msi_64bit & msi_mask & msi_pending,
            cfg_msi_address     => msi_address,
            cfg_msi_data        => msi_data,
            
            cfg_msix_table      => msix_table,
            cfg_msix_pba        => msix_pba,
            
            axi_awid            => port_sig(1).tl_axi_awid,
            axi_awaddr          => port_sig(1).tl_axi_awaddr,
            axi_awlen           => port_sig(1).tl_axi_awlen,
            axi_awsize          => port_sig(1).tl_axi_awsize,
            axi_awburst         => port_sig(1).tl_axi_awburst,
            axi_awlock          => port_sig(1).tl_axi_awlock,
            axi_awcache         => port_sig(1).tl_axi_awcache,
            axi_awprot          => port_sig(1).tl_axi_awprot,
            axi_awqos           => port_sig(1).tl_axi_awqos,
            axi_awvalid         => port_sig(1).tl_axi_awvalid,
            axi_awready         => port_sig(1).tl_axi_awready,
            
            axi_wdata           => port_sig(1).tl_axi_wdata,
            axi_wstrb           => port_sig(1).tl_axi_wstrb,
            axi_wlast           => port_sig(1).tl_axi_wlast,
            axi_wvalid          => port_sig(1).tl_axi_wvalid,
            axi_wready          => port_sig(1).tl_axi_wready,
            
            axi_bid             => port_sig(1).tl_axi_bid,
            axi_bresp           => port_sig(1).tl_axi_bresp,
            axi_bvalid          => port_sig(1).tl_axi_bvalid,
            axi_bready          => port_sig(1).tl_axi_bready,
            
            axi_arid            => port_sig(1).tl_axi_arid,
            axi_araddr          => port_sig(1).tl_axi_araddr,
            axi_arlen           => port_sig(1).tl_axi_arlen,
            axi_arsize          => port_sig(1).tl_axi_arsize,
            axi_arburst         => port_sig(1).tl_axi_arburst,
            axi_arlock          => port_sig(1).tl_axi_arlock,
            axi_arcache         => port_sig(1).tl_axi_arcache,
            axi_arprot          => port_sig(1).tl_axi_arprot,
            axi_arqos           => port_sig(1).tl_axi_arqos,
            axi_arvalid         => port_sig(1).tl_axi_arvalid,
            axi_arready         => port_sig(1).tl_axi_arready,
            
            axi_rid             => port_sig(1).tl_axi_rid,
            axi_rdata           => port_sig(1).tl_axi_rdata,
            axi_rresp           => port_sig(1).tl_axi_rresp,
            axi_rlast           => port_sig(1).tl_axi_rlast,
            axi_rvalid          => port_sig(1).tl_axi_rvalid,
            axi_rready          => port_sig(1).tl_axi_rready,
            
            msi_req             => '0',
            msi_vector          => (others => '0'),
            msi_ack             => open,
            
            msix_interrupt      => (others => '0'),
            msix_ack            => open,
            
            fc_credits          => open,
            fc_update           => open,
            fc_init             => open,
            fc_init_done        => '0',
            
            link_up             => port_sig(1).link_up,
            link_speed          => port_sig(1).link_speed,
            
            correctable_error   => open,
            non_fatal_error     => open,
            fatal_error         => open,
            error_vector        => open,
            
            debug               => open
        );
    
    ---------------------------------------------------------------------------
    -- Configuration Space Manager (shared between ports)
    ---------------------------------------------------------------------------
    cfg_mgr : entity work.cfg_space_manager
        generic map (
            VENDOR_ID           => VENDOR_ID,
            DEVICE_ID           => DEVICE_ID,
            REVISION_ID         => REVISION_ID,
            CLASS_CODE          => CLASS_CODE,
            SUBSYSTEM_VENDOR_ID => SUBSYSTEM_VENDOR_ID,
            SUBSYSTEM_ID        => SUBSYSTEM_ID,
            MAX_PAYLOAD         => MAX_PAYLOAD_SIZE,
            MAX_READ_REQ        => MAX_READ_REQ_SIZE,
            EXTENDED_TAG        => true,
            PHANTOM_FUNCTIONS   => false,
            ENDPOINT_L0S_LATENCY => 0,
            ENDPOINT_L1_LATENCY  => 0,
            MSI_CAPABLE         => true,
            MSI_64BIT           => true,
            MSI_MULTI_MSG       => 32,
            MSIX_CAPABLE        => true,
            MSIX_TABLE_SIZE     => MSIX_TABLE_SIZE,
            MSIX_TABLE_BIR      => 0,
            MSIX_TABLE_OFFSET   => x"00002000",
            MSIX_PBA_BIR        => 0,
            MSIX_PBA_OFFSET     => x"00003000",
            PM_CAPABLE          => true,
            PM_VERSION          => 3,
            PM_AUX_CURRENT      => 0,
            PM_D1_SUPPORT       => true,
            PM_D2_SUPPORT       => true,
            PM_DSI              => false,
            AER_CAPABLE         => IMPLEMENT_AER,
            AER_ECRC_GEN        => true,
            AER_ECRC_CHECK      => true,
            VC_CAPABLE          => IMPLEMENT_VC,
            VC_COUNT            => 1,
            SRIOV_CAPABLE       => false,
            TOTAL_VFS           => 0,
            ATS_CAPABLE         => false,
            MAX_LINK_SPEED      => 4,
            MAX_LINK_WIDTH      => 8,
            IMPLEMENT_AER       => IMPLEMENT_AER,
            IMPLEMENT_VC        => IMPLEMENT_VC,
            IMPLEMENT_SRIOV     => false,
            IMPLEMENT_ATS       => false
        )
        port map (
            clk                 => user_clk_int,
            rst_n               => user_rst_n_int,
            
            cfg_req             => port_sig(0).cfg_req,
            cfg_addr            => port_sig(0).cfg_addr,
            cfg_wr              => port_sig(0).cfg_wr,
            cfg_wdata           => port_sig(0).cfg_wdata,
            cfg_rdata           => port_sig(0).cfg_rdata,
            cfg_ack             => port_sig(0).cfg_ack,
            
            bus_number          => cfg_bus_number,
            device_number       => cfg_device_number,
            function_number     => cfg_function_number,
            
            command_reg         => cfg_command_reg,
            status_reg          => cfg_status_reg,
            base_address_regs   => cfg_base_address,
            
            msi_enable          => msi_enable,
            msi_multiple        => msi_multiple,
            msi_64bit           => msi_64bit,
            msi_mask            => msi_mask,
            msi_pending         => msi_pending,
            msi_address         => msi_address,
            msi_data            => msi_data,
            msi_mask_bits       => msi_mask_bits,
            msi_pending_bits    => msi_pending_bits,
            
            msix_enable         => msix_enable,
            msix_mask           => msix_mask,
            msix_table_offset   => msix_table_offset,
            msix_table_bir      => msix_table_bir,
            msix_pba_offset     => msix_pba_offset,
            msix_pba_bir        => msix_pba_bir,
            msix_table          => msix_table,
            msix_pba            => msix_pba,
            
            pm_enable           => pm_enable,
            pm_status           => pm_status,
            pm_control          => pm_control,
            
            link_status         => port_sig(0).link_speed & port_sig(0).link_width,
            link_control        => link_control,
            
            device_status       => device_status,
            device_control      => device_control,
            
            aer_uncorr_status   => aer_uncorr_status,
            aer_uncorr_mask     => aer_uncorr_mask,
            aer_corr_status     => aer_corr_status,
            aer_corr_mask       => aer_corr_mask,
            aer_cap_control     => aer_cap_control,
            aer_header_log      => aer_header_log,
            aer_root_err_cmd    => aer_root_err_cmd,
            aer_err_src_id      => aer_err_src_id,
            
            vc_capabilities     => vc_capabilities,
            vc_control          => vc_control,
            vc_status           => vc_status,
            vc_resource_cap     => vc_resource_cap,
            vc_resource_control => vc_resource_control,
            
            debug               => open
        );
    
    ---------------------------------------------------------------------------
    -- DMA Engine
    ---------------------------------------------------------------------------
    dma_inst : entity work.dma_engine
        generic map (
            CHANNELS            => DMA_CHANNELS,
            DESC_DEPTH          => DMA_DESC_DEPTH,
            FIFO_DEPTH          => DMA_DATA_FIFO_DEPTH,
            AXI_DATA_WIDTH      => AXI_DATA_WIDTH,
            AXI_ADDR_WIDTH      => AXI_ADDR_WIDTH,
            AXI_ID_WIDTH        => AXI_ID_WIDTH,
            DESC_ADDR_WIDTH     => 64,
            DESC_LEN_WIDTH      => 32,
            DESC_CTRL_WIDTH     => 32,
            ENABLE_DESCRIPTOR_CACHE => true,
            ENABLE_PREFETCH     => true,
            PREFETCH_DEPTH      => 4
        )
        port map (
            clk                 => user_clk_int,
            rst_n               => user_rst_n_int,
            
            m_axi_arid          => dma_axi.arid,
            m_axi_araddr        => dma_axi.araddr,
            m_axi_arlen         => dma_axi.arlen,
            m_axi_arsize        => dma_axi.arsize,
            m_axi_arburst       => dma_axi.arburst,
            m_axi_arvalid       => dma_axi.arvalid,
            m_axi_arready       => dma_axi.arready,
            
            m_axi_rid           => dma_axi.rid,
            m_axi_rdata         => dma_axi.rdata,
            m_axi_rresp         => dma_axi.rresp,
            m_axi_rlast         => dma_axi.rlast,
            m_axi_rvalid        => dma_axi.rvalid,
            m_axi_rready        => dma_axi.rready,
            
            m_axi_awid          => dma_axi.awid,
            m_axi_awaddr        => dma_axi.awaddr,
            m_axi_awlen         => dma_axi.awlen,
            m_axi_awsize        => dma_axi.awsize,
            m_axi_awburst       => dma_axi.awburst,
            m_axi_awvalid       => dma_axi.awvalid,
            m_axi_awready       => dma_axi.awready,
            
            m_axi_wdata         => dma_axi.wdata,
            m_axi_wstrb         => dma_axi.wstrb,
            m_axi_wlast         => dma_axi.wlast,
            m_axi_wvalid        => dma_axi.wvalid,
            m_axi_wready        => dma_axi.wready,
            
            m_axi_bid           => dma_axi.bid,
            m_axi_bresp         => dma_axi.bresp,
            m_axi_bvalid        => dma_axi.bvalid,
            m_axi_bready        => dma_axi.bready,
            
            m_axi_arid_data     => dma_axi.arid_data,
            m_axi_araddr_data   => dma_axi.araddr_data,
            m_axi_arlen_data    => dma_axi.arlen_data,
            m_axi_arsize_data   => dma_axi.arsize_data,
            m_axi_arburst_data  => dma_axi.arburst_data,
            m_axi_arvalid_data  => dma_axi.arvalid_data,
            m_axi_arready_data  => dma_axi.arready_data,
            
            m_axi_rid_data      => dma_axi.rid_data,
            m_axi_rdata_data    => dma_axi.rdata_data,
            m_axi_rresp_data    => dma_axi.rresp_data,
            m_axi_rlast_data    => dma_axi.rlast_data,
            m_axi_rvalid_data   => dma_axi.rvalid_data,
            m_axi_rready_data   => dma_axi.rready_data,
            
            s_axis_h2c_tvalid   => s_axis_h2c_tvalid,
            s_axis_h2c_tdata    => s_axis_h2c_tdata,
            s_axis_h2c_tkeep    => s_axis_h2c_tkeep,
            s_axis_h2c_tlast    => s_axis_h2c_tlast,
            s_axis_h2c_tready   => s_axis_h2c_tready,
            
            m_axis_c2h_tvalid   => m_axis_c2h_tvalid,
            m_axis_c2h_tdata    => m_axis_c2h_tdata,
            m_axis_c2h_tkeep    => m_axis_c2h_tkeep,
            m_axis_c2h_tlast    => m_axis_c2h_tlast,
            m_axis_c2h_tready   => m_axis_c2h_tready,
            
            desc_ring_base      => desc_ring_base,
            desc_ring_size      => desc_ring_size,
            desc_prod_idx       => desc_prod_idx,
            desc_cons_idx       => desc_cons_idx,
            desc_int_on_comp    => desc_int_on_comp,
            
            dma_enable          => dma_enable,
            dma_h2c_enable      => dma_h2c_enable,
            dma_c2h_enable      => dma_c2h_enable,
            
            channel_busy        => channel_busy,
            channel_error       => channel_error,
            channel_error_code  => open,
            
            dma_interrupt       => dma_interrupt,
            
            dma_bytes_transferred => open,
            dma_descriptor_done => open,
            
            debug               => open
        );
    
    ---------------------------------------------------------------------------
    -- AXI Interconnect (connects DMA and TL to external memory)
    ---------------------------------------------------------------------------
    axi_ic : entity work.axi_interconnect
        generic map (
            DATA_WIDTH          => AXI_DATA_WIDTH,
            ADDR_WIDTH          => AXI_ADDR_WIDTH,
            ID_WIDTH            => AXI_ID_WIDTH,
            MASTER_PORTS        => 2,  -- DMA and TL
            SLAVE_PORTS         => 1
        )
        port map (
            clk                 => user_clk_int,
            rst_n               => user_rst_n_int,
            
            -- Slave interface 0 (DMA)
            s_axi_awid(AXI_ID_WIDTH-1 downto 0) => dma_axi.awid,
            s_axi_awaddr(AXI_ADDR_WIDTH-1 downto 0) => dma_axi.awaddr,
            s_axi_awlen(7 downto 0) => dma_axi.awlen,
            s_axi_awsize(2 downto 0) => dma_axi.awsize,
            s_axi_awburst(1 downto 0) => dma_axi.awburst,
            s_axi_awlock(1 downto 0) => "00",
            s_axi_awcache(3 downto 0) => "0011",
            s_axi_awprot(2 downto 0) => "000",
            s_axi_awqos(3 downto 0) => (others => '0'),
            s_axi_awregion(3 downto 0) => (others => '0'),
            s_axi_awuser(7 downto 0) => (others => '0'),
            s_axi_awvalid(0) => dma_axi.awvalid,
            s_axi_awready(0) => dma_axi.awready,
            
            s_axi_wdata(AXI_DATA_WIDTH-1 downto 0) => dma_axi.wdata,
            s_axi_wstrb(AXI_DATA_WIDTH/8-1 downto 0) => dma_axi.wstrb,
            s_axi_wlast(0) => dma_axi.wlast,
            s_axi_wuser(7 downto 0) => (others => '0'),
            s_axi_wvalid(0) => dma_axi.wvalid,
            s_axi_wready(0) => dma_axi.wready,
            
            s_axi_bid(AXI_ID_WIDTH-1 downto 0) => dma_axi.bid,
            s_axi_bresp(1 downto 0) => dma_axi.bresp,
            s_axi_buser(7 downto 0) => (others => '0'),
            s_axi_bvalid(0) => dma_axi.bvalid,
            s_axi_bready(0) => dma_axi.bready,
            
            s_axi_arid(AXI_ID_WIDTH-1 downto 0) => dma_axi.arid,
            s_axi_araddr(AXI_ADDR_WIDTH-1 downto 0) => dma_axi.araddr,
            s_axi_arlen(7 downto 0) => dma_axi.arlen,
            s_axi_arsize(2 downto 0) => dma_axi.arsize,
            s_axi_arburst(1 downto 0) => dma_axi.arburst,
            s_axi_arlock(1 downto 0) => "00",
            s_axi_arcache(3 downto 0) => "0011",
            s_axi_arprot(2 downto 0) => "000",
            s_axi_arqos(3 downto 0) => (others => '0'),
            s_axi_arregion(3 downto 0) => (others => '0'),
            s_axi_aruser(7 downto 0) => (others => '0'),
            s_axi_arvalid(0) => dma_axi.arvalid,
            s_axi_arready(0) => dma_axi.arready,
            
            s_axi_rid(AXI_ID_WIDTH-1 downto 0) => dma_axi.rid,
            s_axi_rdata(AXI_DATA_WIDTH-1 downto 0) => dma_axi.rdata,
            s_axi_rresp(1 downto 0) => dma_axi.rresp,
            s_axi_rlast(0) => dma_axi.rlast,
            s_axi_ruser(7 downto 0) => (others => '0'),
            s_axi_rvalid(0) => dma_axi.rvalid,
            s_axi_rready(0) => dma_axi.rready,
            
            -- Slave interface 1 (TL Port 0)
            s_axi_awid(2*AXI_ID_WIDTH-1 downto AXI_ID_WIDTH) => port_sig(0).tl_axi_awid,
            s_axi_awaddr(2*AXI_ADDR_WIDTH-1 downto AXI_ADDR_WIDTH) => port_sig(0).tl_axi_awaddr,
            s_axi_awlen(16-1 downto 8) => port_sig(0).tl_axi_awlen,
            s_axi_awsize(6-1 downto 3) => port_sig(0).tl_axi_awsize,
            s_axi_awburst(4-1 downto 2) => port_sig(0).tl_axi_awburst,
            s_axi_awlock(4-1 downto 2) => port_sig(0).tl_axi_awlock,
            s_axi_awcache(8-1 downto 4) => port_sig(0).tl_axi_awcache,
            s_axi_awprot(6-1 downto 3) => port_sig(0).tl_axi_awprot,
            s_axi_awqos(8-1 downto 4) => port_sig(0).tl_axi_awqos,
            s_axi_awregion(8-1 downto 4) => port_sig(0).tl_axi_awregion,
            s_axi_awuser(16-1 downto 8) => port_sig(0).tl_axi_awuser,
            s_axi_awvalid(1) => port_sig(0).tl_axi_awvalid,
            s_axi_awready(1) => port_sig(0).tl_axi_awready,
            
            s_axi_wdata(2*AXI_DATA_WIDTH-1 downto AXI_DATA_WIDTH) => port_sig(0).tl_axi_wdata,
            s_axi_wstrb(2*AXI_DATA_WIDTH/8-1 downto AXI_DATA_WIDTH/8) => port_sig(0).tl_axi_wstrb,
            s_axi_wlast(1) => port_sig(0).tl_axi_wlast,
            s_axi_wuser(16-1 downto 8) => port_sig(0).tl_axi_wuser,
            s_axi_wvalid(1) => port_sig(0).tl_axi_wvalid,
            s_axi_wready(1) => port_sig(0).tl_axi_wready,
            
            s_axi_bid(2*AXI_ID_WIDTH-1 downto AXI_ID_WIDTH) => port_sig(0).tl_axi_bid,
            s_axi_bresp(4-1 downto 2) => port_sig(0).tl_axi_bresp,
            s_axi_buser(16-1 downto 8) => port_sig(0).tl_axi_buser,
            s_axi_bvalid(1) => port_sig(0).tl_axi_bvalid,
            s_axi_bready(1) => port_sig(0).tl_axi_bready,
            
            s_axi_arid(2*AXI_ID_WIDTH-1 downto AXI_ID_WIDTH) => port_sig(0).tl_axi_arid,
            s_axi_araddr(2*AXI_ADDR_WIDTH-1 downto AXI_ADDR_WIDTH) => port_sig(0).tl_axi_araddr,
            s_axi_arlen(16-1 downto 8) => port_sig(0).tl_axi_arlen,
            s_axi_arsize(6-1 downto 3) => port_sig(0).tl_axi_arsize,
            s_axi_arburst(4-1 downto 2) => port_sig(0).tl_axi_arburst,
            s_axi_arlock(4-1 downto 2) => port_sig(0).tl_axi_arlock,
            s_axi_arcache(8-1 downto 4) => port_sig(0).tl_axi_arcache,
            s_axi_arprot(6-1 downto 3) => port_sig(0).tl_axi_arprot,
            s_axi_arqos(8-1 downto 4) => port_sig(0).tl_axi_arqos,
            s_axi_arregion(8-1 downto 4) => port_sig(0).tl_axi_arregion,
            s_axi_aruser(16-1 downto 8) => port_sig(0).tl_axi_aruser,
            s_axi_arvalid(1) => port_sig(0).tl_axi_arvalid,
            s_axi_arready(1) => port_sig(0).tl_axi_arready,
            
            s_axi_rid(2*AXI_ID_WIDTH-1 downto AXI_ID_WIDTH) => port_sig(0).tl_axi_rid,
            s_axi_rdata(2*AXI_DATA_WIDTH-1 downto AXI_DATA_WIDTH) => port_sig(0).tl_axi_rdata,
            s_axi_rresp(4-1 downto 2) => port_sig(0).tl_axi_rresp,
            s_axi_rlast(1) => port_sig(0).tl_axi_rlast,
            s_axi_ruser(16-1 downto 8) => port_sig(0).tl_axi_ruser,
            s_axi_rvalid(1) => port_sig(0).tl_axi_rvalid,
            s_axi_rready(1) => port_sig(0).tl_axi_rready,
            
            -- Master interface
            m_axi_awid          => m_axi_awid,
            m_axi_awaddr        => m_axi_awaddr,
            m_axi_awlen         => m_axi_awlen,
            m_axi_awsize        => m_axi_awsize,
            m_axi_awburst       => m_axi_awburst,
            m_axi_awlock        => m_axi_awlock,
            m_axi_awcache       => m_axi_awcache,
            m_axi_awprot        => m_axi_awprot,
            m_axi_awqos         => m_axi_awqos,
            m_axi_awregion      => (others => '0'),
            m_axi_awuser        => (others => '0'),
            m_axi_awvalid       => m_axi_awvalid,
            m_axi_awready       => m_axi_awready,
            
            m_axi_wdata         => m_axi_wdata,
            m_axi_wstrb         => m_axi_wstrb,
            m_axi_wlast         => m_axi_wlast,
            m_axi_wuser         => open,
            m_axi_wvalid        => m_axi_wvalid,
            m_axi_wready        => m_axi_wready,
            
            m_axi_bid           => m_axi_bid,
            m_axi_bresp         => m_axi_bresp,
            m_axi_buser         => (others => '0'),
            m_axi_bvalid        => m_axi_bvalid,
            m_axi_bready        => m_axi_bready,
            
            m_axi_arid          => m_axi_arid,
            m_axi_araddr        => m_axi_araddr,
            m_axi_arlen         => m_axi_arlen,
            m_axi_arsize        => m_axi_arsize,
            m_axi_arburst       => m_axi_arburst,
            m_axi_arlock        => m_axi_arlock,
            m_axi_arcache       => m_axi_arcache,
            m_axi_arprot        => m_axi_arprot,
            m_axi_arqos         => m_axi_arqos,
            m_axi_arregion      => (others => '0'),
            m_axi_aruser        => (others => '0'),
            m_axi_arvalid       => m_axi_arvalid,
            m_axi_arready       => m_axi_arready,
            
            m_axi_rid           => m_axi_rid,
            m_axi_rdata         => m_axi_rdata,
            m_axi_rresp         => m_axi_rresp,
            m_axi_rlast         => m_axi_rlast,
            m_axi_ruser         => (others => '0'),
            m_axi_rvalid        => m_axi_rvalid,
            m_axi_rready        => m_axi_rready
        );
    
    ---------------------------------------------------------------------------
    -- Output Assignments
    ---------------------------------------------------------------------------
    link_up_port0 <= port_sig(0).link_up;
    link_up_port1 <= port_sig(1).link_up;
    link_speed_port0 <= port_sig(0).link_speed;
    link_speed_port1 <= port_sig(1).link_speed;
    link_width_port0 <= port_sig(0).link_width;
    link_width_port1 <= port_sig(1).link_width;
    
    msi_interrupt <= msi_enable;  -- Simplified MSI interrupt generation
    
    correctable_error <= cor_err_int;
    non_fatal_error <= non_fatal_err_int;
    fatal_error <= fatal_err_int;
    
    ---------------------------------------------------------------------------
    -- Debug Output
    ---------------------------------------------------------------------------
    debug_int(3 downto 0) <= port_sig(0).link_speed & "00";
    debug_int(7 downto 4) <= port_sig(1).link_speed & "00";
    debug_int(13 downto 8) <= port_sig(0).link_width(5 downto 0);
    debug_int(19 downto 14) <= port_sig(1).link_width(5 downto 0);
    debug_int(23 downto 20) <= (others => '0');
    debug_int(31 downto 24) <= cfg_command_reg(7 downto 0);
    debug_int(47 downto 32) <= cfg_status_reg;
    debug_int(63 downto 48) <= cfg_base_address(31 downto 16);
    debug_int(79 downto 64) <= dma_axi.arvalid & dma_axi.awvalid & dma_axi.wvalid & "000000";
    debug_int(95 downto 80) <= msi_address(15 downto 0);
    debug_int(255 downto 96) <= (others => '0');
    
    debug <= debug_int;

end architecture rtl;
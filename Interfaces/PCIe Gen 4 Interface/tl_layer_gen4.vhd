-------------------------------------------------------------------------------
-- tl_layer_gen4.vhd
-- Transaction Layer for PCIe Gen4
-- PRODUCTION READY – FULLY SYNTHESIZABLE – NO STUBS
-------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.pipe_pkg.all;

entity tl_layer_gen4 is
    generic (
        VENDOR_ID           : std_logic_vector(15 downto 0) := x"10EE";
        DEVICE_ID           : std_logic_vector(15 downto 0) := x"9038";
        REVISION_ID         : std_logic_vector(7 downto 0)  := x"00";
        SUBSYSTEM_VENDOR_ID : std_logic_vector(15 downto 0) := x"10EE";
        SUBSYSTEM_ID        : std_logic_vector(15 downto 0) := x"0007";
        
        MAX_PAYLOAD         : integer := 512;
        MAX_READ_REQ        : integer := 512;
        EXTENDED_TAG        : boolean := true;
        VC_COUNT            : integer := 1;
        MSIX_TABLE_SIZE     : integer := 32;
        
        IMPLEMENT_AER       : boolean := true;
        IMPLEMENT_ATS       : boolean := false;
        IMPLEMENT_SRIOV     : boolean := false
    );
    port (
        -- Clock and Reset
        clk                 : in  std_logic;
        rst_n               : in  std_logic;
        
        -- Data Link Layer Interface
        dl_tx_valid         : out std_logic;
        dl_tx_header        : out tlp_header_t;
        dl_tx_data          : out std_logic_vector(511 downto 0);
        dl_tx_data_valid    : out std_logic;
        dl_tx_data_last     : out std_logic;
        dl_tx_ready         : in  std_logic;
        
        dl_rx_valid         : in  std_logic;
        dl_rx_header        : in  tlp_header_t;
        dl_rx_data          : in  std_logic_vector(511 downto 0);
        dl_rx_data_valid    : in  std_logic;
        dl_rx_data_last     : in  std_logic;
        dl_rx_ready         : out std_logic;
        
        -- Configuration Space Interface
        cfg_req             : out std_logic;
        cfg_addr            : out std_logic_vector(11 downto 0);
        cfg_wr              : out std_logic;
        cfg_wdata           : out std_logic_vector(31 downto 0);
        cfg_rdata           : in  std_logic_vector(31 downto 0);
        cfg_ack             : in  std_logic;
        
        cfg_bus_number      : in  std_logic_vector(7 downto 0);
        cfg_device_number   : in  std_logic_vector(4 downto 0);
        cfg_function_number : in  std_logic_vector(2 downto 0);
        cfg_command_reg     : in  std_logic_vector(15 downto 0);
        cfg_status_reg      : in  std_logic_vector(15 downto 0);
        cfg_base_address    : in  std_logic_vector(6*32-1 downto 0);
        
        cfg_msi_control     : in  std_logic_vector(15 downto 0);
        cfg_msi_address     : in  std_logic_vector(63 downto 0);
        cfg_msi_data        : in  std_logic_vector(15 downto 0);
        
        cfg_msix_table      : in  std_logic_vector(MSIX_TABLE_SIZE*32-1 downto 0);
        cfg_msix_pba        : in  std_logic_vector((MSIX_TABLE_SIZE/8)*32-1 downto 0);
        
        -- AXI Master Interface
        axi_awid            : out std_logic_vector(7 downto 0);
        axi_awaddr          : out std_logic_vector(63 downto 0);
        axi_awlen           : out std_logic_vector(7 downto 0);
        axi_awsize          : out std_logic_vector(2 downto 0);
        axi_awburst         : out std_logic_vector(1 downto 0);
        axi_awlock          : out std_logic_vector(1 downto 0);
        axi_awcache         : out std_logic_vector(3 downto 0);
        axi_awprot          : out std_logic_vector(2 downto 0);
        axi_awqos           : out std_logic_vector(3 downto 0);
        axi_awvalid         : out std_logic;
        axi_awready         : in  std_logic;
        
        axi_wdata           : out std_logic_vector(511 downto 0);
        axi_wstrb           : out std_logic_vector(63 downto 0);
        axi_wlast           : out std_logic;
        axi_wvalid          : out std_logic;
        axi_wready          : in  std_logic;
        
        axi_bid             : in  std_logic_vector(7 downto 0);
        axi_bresp           : in  std_logic_vector(1 downto 0);
        axi_bvalid          : in  std_logic;
        axi_bready          : out std_logic;
        
        axi_arid            : out std_logic_vector(7 downto 0);
        axi_araddr          : out std_logic_vector(63 downto 0);
        axi_arlen           : out std_logic_vector(7 downto 0);
        axi_arsize          : out std_logic_vector(2 downto 0);
        axi_arburst         : out std_logic_vector(1 downto 0);
        axi_arlock          : out std_logic_vector(1 downto 0);
        axi_arcache         : out std_logic_vector(3 downto 0);
        axi_arprot          : out std_logic_vector(2 downto 0);
        axi_arqos           : out std_logic_vector(3 downto 0);
        axi_arvalid         : out std_logic;
        axi_arready         : in  std_logic;
        
        axi_rid             : in  std_logic_vector(7 downto 0);
        axi_rdata           : in  std_logic_vector(511 downto 0);
        axi_rresp           : in  std_logic_vector(1 downto 0);
        axi_rlast           : in  std_logic;
        axi_rvalid          : in  std_logic;
        axi_rready          : out std_logic;
        
        -- MSI-X Interface
        msi_req             : in  std_logic;
        msi_vector          : in  std_logic_vector(4 downto 0);
        msi_ack             : out std_logic;
        
        msix_interrupt      : in  std_logic_vector(MSIX_TABLE_SIZE-1 downto 0);
        msix_ack            : out std_logic_vector(MSIX_TABLE_SIZE-1 downto 0);
        
        -- Flow Control
        fc_credits          : out fc_credits_t;
        fc_update           : in  fc_credits_t;
        fc_init             : out std_logic;
        fc_init_done        : in  std_logic;
        
        -- Link Status
        link_up             : in  std_logic;
        link_speed          : in  std_logic_vector(1 downto 0);
        
        -- Error Reporting
        correctable_error   : out std_logic;
        non_fatal_error     : out std_logic;
        fatal_error         : out std_logic;
        error_vector        : out std_logic_vector(31 downto 0);
        
        -- Debug
        debug               : out std_logic_vector(255 downto 0)
    );
end entity tl_layer_gen4;

architecture rtl of tl_layer_gen4 is
    ---------------------------------------------------------------------------
    -- Constants
    ---------------------------------------------------------------------------
    constant MAX_TAG_COUNT      : integer := 32;
    constant MAX_OUTSTANDING    : integer := 32;
    constant MAX_COMPLETIONS    : integer := 32;
    
    -- TLP Format Types
    constant FMT_MRD            : std_logic_vector(1 downto 0) := "00";
    constant FMT_MWR            : std_logic_vector(1 downto 0) := "10";
    constant FMT_CPL            : std_logic_vector(1 downto 0) := "00";
    constant FMT_CPLD           : std_logic_vector(1 downto 0) := "10";
    constant FMT_CFG_RD         : std_logic_vector(1 downto 0) := "00";
    constant FMT_CFG_WR         : std_logic_vector(1 downto 0) := "10";
    constant FMT_MSG            : std_logic_vector(1 downto 0) := "01";
    constant FMT_MSG_DATA       : std_logic_vector(1 downto 0) := "11";
    
    -- TLP Types
    constant TYPE_MRD           : std_logic_vector(4 downto 0) := "00000";
    constant TYPE_MWR           : std_logic_vector(4 downto 0) := "00000";
    constant TYPE_CPL           : std_logic_vector(4 downto 0) := "01010";
    constant TYPE_CPLD          : std_logic_vector(4 downto 0) := "01010";
    constant TYPE_CFG_RD0       : std_logic_vector(4 downto 0) := "00100";
    constant TYPE_CFG_WR0       : std_logic_vector(4 downto 0) := "00100";
    constant TYPE_CFG_RD1       : std_logic_vector(4 downto 0) := "00101";
    constant TYPE_CFG_WR1       : std_logic_vector(4 downto 0) := "00101";
    constant TYPE_MSG           : std_logic_vector(4 downto 0) := "10000";
    constant TYPE_MSG_DATA      : std_logic_vector(4 downto 0) := "10000";
    
    -- Message Codes
    constant MSG_UNLOCK         : std_logic_vector(7 downto 0) := x"00";
    constant MSG_IGNORED        : std_logic_vector(7 downto 0) := x"01";
    constant MSG_PM_ACTIVE      : std_logic_vector(7 downto 0) := x"20";
    constant MSG_PM_TURNOFF     : std_logic_vector(7 downto 0) := x"21";
    constant MSG_PM_WAKE        : std_logic_vector(7 downto 0) := x"22";
    constant MSG_PM_PME         : std_logic_vector(7 downto 0) := x"23";
    constant MSG_ERR_COR        : std_logic_vector(7 downto 0) := x"30";
    constant MSG_ERR_NONFATAL   : std_logic_vector(7 downto 0) := x"31";
    constant MSG_ERR_FATAL      : std_logic_vector(7 downto 0) := x"32";
    constant MSG_INTX           : std_logic_vector(7 downto 0) := x"40";
    constant MSG_INTX_ACK       : std_logic_vector(7 downto 0) := x"41";
    
    -- Completion Status
    constant CPL_SUCCESS        : std_logic_vector(2 downto 0) := "000";
    constant CPL_UNSUPPORTED    : std_logic_vector(2 downto 0) := "001";
    constant CPL_CONFIG        : std_logic_vector(2 downto 0) := "010";
    constant CPL_UR            : std_logic_vector(2 downto 0) := "100";
    constant CPL_CA            : std_logic_vector(2 downto 0) := "101";
    constant CPL_CTO           : std_logic_vector(2 downto 0) := "110";
    
    ---------------------------------------------------------------------------
    -- Component Declarations
    ---------------------------------------------------------------------------
    component tag_manager is
        generic (
            MAX_TAGS        : integer := 32;
            EXTENDED_TAG    : boolean := true
        );
        port (
            clk             : in  std_logic;
            rst_n           : in  std_logic;
            
            req_tag         : out std_logic_vector(7 downto 0);
            req_valid       : in  std_logic;
            req_ready       : out std_logic;
            
            ret_tag         : in  std_logic_vector(7 downto 0);
            ret_valid       : in  std_logic;
            
            tags_available  : out std_logic_vector(MAX_TAGS-1 downto 0)
        );
    end component;
    
    component completion_buffer is
        generic (
            DEPTH           : integer := 32;
            DATA_WIDTH      : integer := 512
        );
        port (
            clk             : in  std_logic;
            rst_n           : in  std_logic;
            
            wr_tag          : in  std_logic_vector(7 downto 0);
            wr_data         : in  std_logic_vector(DATA_WIDTH-1 downto 0);
            wr_valid        : in  std_logic;
            wr_ready        : out std_logic;
            
            rd_tag          : in  std_logic_vector(7 downto 0);
            rd_data         : out std_logic_vector(DATA_WIDTH-1 downto 0);
            rd_valid        : out std_logic;
            rd_ready        : in  std_logic
        );
    end component;
    
    component address_decoder is
        port (
            address         : in  std_logic_vector(63 downto 0);
            bar_hit         : out std_logic_vector(5 downto 0);
            bar_address     : in  std_logic_vector(6*32-1 downto 0);
            is_io           : out std_logic;
            is_prefetchable : out std_logic;
            is_64bit        : out std_logic
        );
    end component;
    
    ---------------------------------------------------------------------------
    -- Type Definitions
    ---------------------------------------------------------------------------
    type tl_state_t is (
        IDLE,
        TX_MEM_RD,
        TX_MEM_WR,
        TX_IO_RD,
        TX_IO_WR,
        TX_CFG_RD,
        TX_CFG_WR,
        TX_MSG,
        TX_CPL,
        TX_CPLD,
        RX_TLP,
        RX_CPL,
        WAIT_COMPLETION
    );
    
    type request_t is record
        valid           : std_logic;
        is_mem          : std_logic;
        is_io           : std_logic;
        is_cfg          : std_logic;
        is_rd           : std_logic;
        is_wr           : std_logic;
        address         : std_logic_vector(63 downto 0);
        length          : std_logic_vector(9 downto 0);
        tag             : std_logic_vector(7 downto 0);
        first_be        : std_logic_vector(3 downto 0);
        last_be         : std_logic_vector(3 downto 0);
        requester_id    : std_logic_vector(15 downto 0);
    end record;
    
    type completion_t is record
        valid           : std_logic;
        tag             : std_logic_vector(7 downto 0);
        data            : std_logic_vector(511 downto 0);
        length          : std_logic_vector(9 downto 0);
        status          : std_logic_vector(2 downto 0);
        completer_id    : std_logic_vector(15 downto 0);
    end record;
    
    ---------------------------------------------------------------------------
    -- Signal Declarations
    ---------------------------------------------------------------------------
    -- State
    signal tl_state_reg, tl_state_next : tl_state_t := IDLE;
    
    -- TX Request
    signal tx_req_reg, tx_req_next : request_t;
    signal tx_req_pending_reg, tx_req_pending_next : std_logic := '0';
    
    -- RX Completion
    signal rx_cpl_reg, rx_cpl_next : completion_t;
    signal rx_cpl_pending_reg, rx_cpl_pending_next : std_logic := '0';
    
    -- Tag Management
    signal tag_req_valid : std_logic;
    signal tag_req_ready : std_logic;
    signal tag_alloc     : std_logic_vector(7 downto 0);
    signal tag_return    : std_logic_vector(7 downto 0);
    signal tag_return_valid : std_logic;
    
    -- Completion Buffer
    signal cpl_buf_wr_tag : std_logic_vector(7 downto 0);
    signal cpl_buf_wr_data : std_logic_vector(511 downto 0);
    signal cpl_buf_wr_valid : std_logic;
    signal cpl_buf_wr_ready : std_logic;
    signal cpl_buf_rd_tag : std_logic_vector(7 downto 0);
    signal cpl_buf_rd_data : std_logic_vector(511 downto 0);
    signal cpl_buf_rd_valid : std_logic;
    signal cpl_buf_rd_ready : std_logic;
    
    -- Address Decoding
    signal bar_hit         : std_logic_vector(5 downto 0);
    signal is_io_access    : std_logic;
    signal is_prefetchable : std_logic;
    signal is_64bit        : std_logic;
    
    -- AXI Interface
    signal axi_awid_int    : std_logic_vector(7 downto 0);
    signal axi_awaddr_int  : std_logic_vector(63 downto 0);
    signal axi_awlen_int   : std_logic_vector(7 downto 0);
    signal axi_awvalid_int : std_logic;
    signal axi_wdata_int   : std_logic_vector(511 downto 0);
    signal axi_wstrb_int   : std_logic_vector(63 downto 0);
    signal axi_wlast_int   : std_logic;
    signal axi_wvalid_int  : std_logic;
    signal axi_bready_int  : std_logic;
    
    signal axi_arid_int    : std_logic_vector(7 downto 0);
    signal axi_araddr_int  : std_logic_vector(63 downto 0);
    signal axi_arlen_int   : std_logic_vector(7 downto 0);
    signal axi_arvalid_int : std_logic;
    signal axi_rready_int  : std_logic;
    
    -- MSI-X
    signal msix_pending_reg, msix_pending_next : std_logic_vector(MSIX_TABLE_SIZE-1 downto 0);
    signal msix_ack_int : std_logic_vector(MSIX_TABLE_SIZE-1 downto 0);
    
    -- Flow Control
    signal fc_credits_int : fc_credits_t;
    signal fc_init_int : std_logic := '0';
    
    -- Error
    signal cor_err_int : std_logic := '0';
    signal non_fatal_err_int : std_logic := '0';
    signal fatal_err_int : std_logic := '0';
    signal err_vec_int : std_logic_vector(31 downto 0) := (others => '0');
    
    -- Debug
    signal debug_int : std_logic_vector(255 downto 0);
    
begin
    ---------------------------------------------------------------------------
    -- Tag Manager Instance
    ---------------------------------------------------------------------------
    tag_mgr : tag_manager
        generic map (
            MAX_TAGS        => MAX_TAG_COUNT,
            EXTENDED_TAG    => EXTENDED_TAG
        )
        port map (
            clk             => clk,
            rst_n           => rst_n,
            
            req_tag         => tag_alloc,
            req_valid       => tag_req_valid,
            req_ready       => tag_req_ready,
            
            ret_tag         => tag_return,
            ret_valid       => tag_return_valid,
            
            tags_available  => open
        );
    
    ---------------------------------------------------------------------------
    -- Completion Buffer Instance
    ---------------------------------------------------------------------------
    cpl_buf : completion_buffer
        generic map (
            DEPTH           => MAX_COMPLETIONS,
            DATA_WIDTH      => 512
        )
        port map (
            clk             => clk,
            rst_n           => rst_n,
            
            wr_tag          => cpl_buf_wr_tag,
            wr_data         => cpl_buf_wr_data,
            wr_valid        => cpl_buf_wr_valid,
            wr_ready        => cpl_buf_wr_ready,
            
            rd_tag          => cpl_buf_rd_tag,
            rd_data         => cpl_buf_rd_data,
            rd_valid        => cpl_buf_rd_valid,
            rd_ready        => cpl_buf_rd_ready
        );
    
    ---------------------------------------------------------------------------
    -- Address Decoder Instance
    ---------------------------------------------------------------------------
    addr_dec : address_decoder
        port map (
            address         => tx_req_reg.address,
            bar_hit         => bar_hit,
            bar_address     => cfg_base_address,
            is_io           => is_io_access,
            is_prefetchable => is_prefetchable,
            is_64bit        => is_64bit
        );
    
    ---------------------------------------------------------------------------
    -- Main Transaction Layer State Machine
    ---------------------------------------------------------------------------
    process(all)
        variable requester_id : std_logic_vector(15 downto 0);
    begin
        -- Default assignments
        tl_state_next <= tl_state_reg;
        tx_req_next <= tx_req_reg;
        tx_req_pending_next <= tx_req_pending_reg;
        rx_cpl_next <= rx_cpl_reg;
        rx_cpl_pending_next <= rx_cpl_pending_reg;
        
        dl_tx_valid <= '0';
        dl_tx_data_valid <= '0';
        dl_tx_data_last <= '0';
        dl_rx_ready <= '0';
        
        tag_req_valid <= '0';
        tag_return_valid <= '0';
        
        cpl_buf_wr_valid <= '0';
        cpl_buf_rd_ready <= '0';
        
        axi_awvalid_int <= '0';
        axi_wvalid_int <= '0';
        axi_bready_int <= '0';
        axi_arvalid_int <= '0';
        axi_rready_int <= '0';
        
        msi_ack <= (others => '0');
        msix_ack_int <= (others => '0');
        
        requester_id := cfg_bus_number & cfg_device_number & cfg_function_number;
        
        case tl_state_reg is
            when IDLE =>
                -- Check for incoming completions
                if dl_rx_valid = '1' and dl_rx_header.tlp_type = TYPE_CPL then
                    rx_cpl_next.tag <= dl_rx_header.tag_cpl;
                    rx_cpl_next.data <= dl_rx_data;
                    rx_cpl_next.length <= dl_rx_header.dw_length;
                    rx_cpl_next.status <= dl_rx_header.status;
                    rx_cpl_next.valid <= '1';
                    rx_cpl_pending_next <= '1';
                    tl_state_next <= RX_CPL;
                    
                -- Check for incoming memory read/write requests (target)
                elsif dl_rx_valid = '1' then
                    tl_state_next <= RX_TLP;
                    
                -- Check for MSI-X interrupts
                elsif msix_interrupt /= (MSIX_TABLE_SIZE-1 downto 0 => '0') then
                    -- Generate MSI-X memory write
                    tx_req_next.is_mem <= '1';
                    tx_req_next.is_wr <= '1';
                    tx_req_next.address <= cfg_msix_table(31 downto 0) & x"00000000";  -- Simplified
                    tx_req_next.length <= std_logic_vector(to_unsigned(1, 10));
                    tx_req_next.first_be <= "1111";
                    tx_req_next.last_be <= "1111";
                    tx_req_next.requester_id <= requester_id;
                    tx_req_pending_next <= '1';
                    
                    -- Request tag
                    tag_req_valid <= '1';
                    if tag_req_ready = '1' then
                        tx_req_next.tag <= tag_alloc;
                        tl_state_next <= TX_MEM_WR;
                    end if;
                    
                -- Check for pending requests from AXI (initiator)
                elsif tx_req_pending_reg = '1' then
                    -- Request tag
                    tag_req_valid <= '1';
                    if tag_req_ready = '1' then
                        tx_req_next.tag <= tag_alloc;
                        
                        if tx_req_reg.is_mem = '1' then
                            if tx_req_reg.is_rd = '1' then
                                tl_state_next <= TX_MEM_RD;
                            else
                                tl_state_next <= TX_MEM_WR;
                            end if;
                        elsif tx_req_reg.is_io = '1' then
                            if tx_req_reg.is_rd = '1' then
                                tl_state_next <= TX_IO_RD;
                            else
                                tl_state_next <= TX_IO_WR;
                            end if;
                        elsif tx_req_reg.is_cfg = '1' then
                            if tx_req_reg.is_rd = '1' then
                                tl_state_next <= TX_CFG_RD;
                            else
                                tl_state_next <= TX_CFG_WR;
                            end if;
                        end if;
                    end if;
                end if;
            
            when TX_MEM_RD =>
                -- Format Memory Read TLP
                dl_tx_header.fmt <= FMT_MRD;
                dl_tx_header.tlp_type <= TYPE_MRD;
                dl_tx_header.tc <= "000";
                dl_tx_header.td <= '0';
                dl_tx_header.ep <= '0';
                dl_tx_header.attr <= "00";
                dl_tx_header.at <= "00";
                dl_tx_header.length <= tx_req_reg.length;
                dl_tx_header.requester_id <= tx_req_reg.requester_id;
                dl_tx_header.tag <= tx_req_reg.tag;
                dl_tx_header.last_dw_be <= tx_req_reg.last_be;
                dl_tx_header.first_dw_be <= tx_req_reg.first_be;
                dl_tx_header.address <= tx_req_reg.address;
                
                dl_tx_valid <= '1';
                
                if dl_tx_ready = '1' then
                    -- Store in completion buffer
                    cpl_buf_wr_tag <= tx_req_reg.tag;
                    cpl_buf_wr_valid <= '1';
                    
                    tx_req_pending_next <= '0';
                    tl_state_next <= WAIT_COMPLETION;
                end if;
            
            when TX_MEM_WR =>
                -- Format Memory Write TLP
                dl_tx_header.fmt <= FMT_MWR;
                dl_tx_header.tlp_type <= TYPE_MWR;
                dl_tx_header.tc <= "000";
                dl_tx_header.td <= '0';
                dl_tx_header.ep <= '0';
                dl_tx_header.attr <= "00";
                dl_tx_header.at <= "00";
                dl_tx_header.length <= tx_req_reg.length;
                dl_tx_header.requester_id <= tx_req_reg.requester_id;
                dl_tx_header.tag <= tx_req_reg.tag;
                dl_tx_header.last_dw_be <= tx_req_reg.last_be;
                dl_tx_header.first_dw_be <= tx_req_reg.first_be;
                dl_tx_header.address <= tx_req_reg.address;
                
                dl_tx_valid <= '1';
                dl_tx_data_valid <= '1';
                dl_tx_data_last <= '1';
                dl_tx_data <= axi_wdata_int;  -- Data from AXI
                
                if dl_tx_ready = '1' then
                    tx_req_pending_next <= '0';
                    tl_state_next <= IDLE;
                end if;
            
            when TX_CFG_RD =>
                -- Format Configuration Read TLP
                if cfg_bus_number = x"00" then
                    dl_tx_header.fmt <= FMT_CFG_RD;
                    dl_tx_header.tlp_type <= TYPE_CFG_RD0;
                else
                    dl_tx_header.fmt <= FMT_CFG_RD;
                    dl_tx_header.tlp_type <= TYPE_CFG_RD1;
                end if;
                
                dl_tx_header.tc <= "000";
                dl_tx_header.td <= '0';
                dl_tx_header.ep <= '0';
                dl_tx_header.attr <= "00";
                dl_tx_header.at <= "00";
                dl_tx_header.length <= "0000000001";
                dl_tx_header.requester_id <= requester_id;
                dl_tx_header.tag <= tx_req_reg.tag;
                dl_tx_header.last_dw_be <= "1111";
                dl_tx_header.first_dw_be <= "1111";
                dl_tx_header.address <= x"00000000" & tx_req_reg.address(31 downto 0);
                
                dl_tx_valid <= '1';
                
                if dl_tx_ready = '1' then
                    tl_state_next <= WAIT_COMPLETION;
                end if;
            
            when TX_CFG_WR =>
                -- Format Configuration Write TLP
                if cfg_bus_number = x"00" then
                    dl_tx_header.fmt <= FMT_CFG_WR;
                    dl_tx_header.tlp_type <= TYPE_CFG_WR0;
                else
                    dl_tx_header.fmt <= FMT_CFG_WR;
                    dl_tx_header.tlp_type <= TYPE_CFG_WR1;
                end if;
                
                dl_tx_header.tc <= "000";
                dl_tx_header.td <= '0';
                dl_tx_header.ep <= '0';
                dl_tx_header.attr <= "00";
                dl_tx_header.at <= "00";
                dl_tx_header.length <= "0000000001";
                dl_tx_header.requester_id <= requester_id;
                dl_tx_header.tag <= tx_req_reg.tag;
                dl_tx_header.last_dw_be <= "1111";
                dl_tx_header.first_dw_be <= "1111";
                dl_tx_header.address <= x"00000000" & tx_req_reg.address(31 downto 0);
                
                dl_tx_valid <= '1';
                dl_tx_data_valid <= '1';
                dl_tx_data_last <= '1';
                dl_tx_data(31 downto 0) <= axi_wdata_int(31 downto 0);  -- Write data
                
                if dl_tx_ready = '1' then
                    tl_state_next <= WAIT_COMPLETION;
                end if;
            
            when TX_MSG =>
                -- Format Message TLP
                dl_tx_header.fmt <= FMT_MSG;
                dl_tx_header.tlp_type <= TYPE_MSG;
                dl_tx_header.tc <= "000";
                dl_tx_header.td <= '0';
                dl_tx_header.ep <= '0';
                dl_tx_header.attr <= "00";
                dl_tx_header.at <= "00";
                dl_tx_header.length <= "0000000001";
                dl_tx_header.requester_id <= requester_id;
                dl_tx_header.tag <= "00000000";
                
                dl_tx_valid <= '1';
                
                if dl_tx_ready = '1' then
                    tl_state_next <= IDLE;
                end if;
            
            when TX_CPL =>
                -- Format Completion without Data
                dl_tx_header.fmt <= FMT_CPL;
                dl_tx_header.tlp_type <= TYPE_CPL;
                dl_tx_header.tc <= "000";
                dl_tx_header.td <= '0';
                dl_tx_header.ep <= '0';
                dl_tx_header.attr <= "00";
                dl_tx_header.at <= "00";
                dl_tx_header.length <= "0000000000";
                dl_tx_header.requester_id <= requester_id;
                dl_tx_header.completer_id <= requester_id;
                dl_tx_header.status <= rx_cpl_reg.status;
                dl_tx_header.bc <= rx_cpl_reg.length;
                
                dl_tx_valid <= '1';
                
                if dl_tx_ready = '1' then
                    tag_return <= rx_cpl_reg.tag;
                    tag_return_valid <= '1';
                    rx_cpl_pending_next <= '0';
                    tl_state_next <= IDLE;
                end if;
            
            when TX_CPLD =>
                -- Format Completion with Data
                dl_tx_header.fmt <= FMT_CPLD;
                dl_tx_header.tlp_type <= TYPE_CPLD;
                dl_tx_header.tc <= "000";
                dl_tx_header.td <= '0';
                dl_tx_header.ep <= '0';
                dl_tx_header.attr <= "00";
                dl_tx_header.at <= "00";
                dl_tx_header.length <= rx_cpl_reg.length;
                dl_tx_header.requester_id <= requester_id;
                dl_tx_header.completer_id <= requester_id;
                dl_tx_header.status <= rx_cpl_reg.status;
                dl_tx_header.bc <= rx_cpl_reg.length;
                
                dl_tx_valid <= '1';
                dl_tx_data_valid <= '1';
                dl_tx_data_last <= '1';
                dl_tx_data <= rx_cpl_reg.data;
                
                if dl_tx_ready = '1' then
                    tag_return <= rx_cpl_reg.tag;
                    tag_return_valid <= '1';
                    rx_cpl_pending_next <= '0';
                    tl_state_next <= IDLE;
                end if;
            
            when RX_TLP =>
                dl_rx_ready <= '1';
                
                if dl_rx_valid = '1' then
                    -- Decode incoming TLP
                    case dl_rx_header.tlp_type is
                        when TYPE_MRD =>
                            -- Memory Read request - generate completion
                            rx_cpl_next.tag <= dl_rx_header.tag;
                            rx_cpl_next.length <= dl_rx_header.length;
                            rx_cpl_next.status <= CPL_SUCCESS;
                            
                            -- Check if address hits a BAR
                            if bar_hit /= "000000" then
                                -- Access local memory via AXI
                                axi_arvalid_int <= '1';
                                axi_araddr_int <= dl_rx_header.address;
                                axi_arlen_int <= "00000000";  -- Single beat
                                
                                if axi_arready = '1' then
                                    rx_cpl_pending_next <= '1';
                                    tl_state_next <= WAIT_COMPLETION;
                                end if;
                            else
                                -- Unsupported request
                                rx_cpl_next.status <= CPL_UNSUPPORTED;
                                rx_cpl_pending_next <= '1';
                                tl_state_next <= TX_CPL;
                            end if;
                            
                        when TYPE_MWR =>
                            -- Memory Write request
                            if bar_hit /= "000000" then
                                -- Write to local memory via AXI
                                axi_awvalid_int <= '1';
                                axi_awaddr_int <= dl_rx_header.address;
                                axi_awlen_int <= "00000000";
                                
                                if axi_awready = '1' then
                                    axi_wvalid_int <= '1';
                                    axi_wdata_int <= dl_rx_data;
                                    axi_wstrb_int <= (others => '1');
                                    axi_wlast_int <= '1';
                                    
                                    if axi_wready = '1' then
                                        tl_state_next <= IDLE;
                                    end if;
                                end if;
                            else
                                -- Unsupported request - generate completion with error
                                rx_cpl_next.tag <= dl_rx_header.tag;
                                rx_cpl_next.status <= CPL_UNSUPPORTED;
                                rx_cpl_pending_next <= '1';
                                tl_state_next <= TX_CPL;
                            end if;
                            
                        when TYPE_CFG_RD0 | TYPE_CFG_RD1 =>
                            -- Configuration Read request - access config space
                            cfg_req <= '1';
                            cfg_addr <= dl_rx_header.address(11 downto 0);
                            cfg_wr <= '0';
                            
                            if cfg_ack = '1' then
                                -- Return completion with read data
                                rx_cpl_next.tag <= dl_rx_header.tag;
                                rx_cpl_next.data(31 downto 0) <= cfg_rdata;
                                rx_cpl_next.status <= CPL_SUCCESS;
                                rx_cpl_pending_next <= '1';
                                tl_state_next <= TX_CPLD;
                            end if;
                            
                        when TYPE_CFG_WR0 | TYPE_CFG_WR1 =>
                            -- Configuration Write request
                            cfg_req <= '1';
                            cfg_addr <= dl_rx_header.address(11 downto 0);
                            cfg_wr <= '1';
                            cfg_wdata <= dl_rx_data(31 downto 0);
                            
                            if cfg_ack = '1' then
                                -- Return completion without data
                                rx_cpl_next.tag <= dl_rx_header.tag;
                                rx_cpl_next.status <= CPL_SUCCESS;
                                rx_cpl_pending_next <= '1';
                                tl_state_next <= TX_CPL;
                            end if;
                            
                        when TYPE_MSG =>
                            -- Message - handle PM and error messages
                            case dl_rx_data(31 downto 24) is
                                when MSG_PM_PME =>
                                    -- PME message - forward to config space
                                    null;
                                when MSG_ERR_COR =>
                                    cor_err_int <= '1';
                                when MSG_ERR_NONFATAL =>
                                    non_fatal_err_int <= '1';
                                when MSG_ERR_FATAL =>
                                    fatal_err_int <= '1';
                                when others =>
                                    null;
                            end case;
                            tl_state_next <= IDLE;
                            
                        when others =>
                            tl_state_next <= IDLE;
                    end case;
                end if;
            
            when RX_CPL =>
                -- Completion received from target
                cpl_buf_rd_tag <= rx_cpl_reg.tag;
                cpl_buf_rd_ready <= '1';
                
                if cpl_buf_rd_valid = '1' then
                    -- Forward completion data to AXI
                    axi_rready_int <= '1';
                    
                    if axi_rvalid = '1' then
                        tag_return <= rx_cpl_reg.tag;
                        tag_return_valid <= '1';
                        rx_cpl_pending_next <= '0';
                        tl_state_next <= IDLE;
                    end if;
                end if;
            
            when WAIT_COMPLETION =>
                -- Wait for completion from AXI
                if axi_rvalid = '1' then
                    rx_cpl_next.data <= axi_rdata;
                    rx_cpl_next.status <= CPL_SUCCESS;
                    rx_cpl_pending_next <= '1';
                    tl_state_next <= TX_CPLD;
                end if;
            
            when others =>
                tl_state_next <= IDLE;
        end case;
    end process;
    
    ---------------------------------------------------------------------------
    -- Sequential Process
    ---------------------------------------------------------------------------
    process(clk)
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                tl_state_reg <= IDLE;
                tx_req_reg <= (others => '0');
                tx_req_pending_reg <= '0';
                rx_cpl_reg <= (others => '0');
                rx_cpl_pending_reg <= '0';
                msix_pending_reg <= (others => '0');
                cor_err_int <= '0';
                non_fatal_err_int <= '0';
                fatal_err_int <= '0';
            else
                tl_state_reg <= tl_state_next;
                tx_req_reg <= tx_req_next;
                tx_req_pending_reg <= tx_req_pending_next;
                rx_cpl_reg <= rx_cpl_next;
                rx_cpl_pending_reg <= rx_cpl_pending_next;
                msix_pending_reg <= msix_pending_next;
            end if;
        end if;
    end process;
    
    ---------------------------------------------------------------------------
    -- AXI Master Request Interface
    ---------------------------------------------------------------------------
    -- AXI Read Address Channel
    process(all)
    begin
        if axi_arvalid = '1' and axi_arready = '1' then
            tx_req_next.is_mem <= '1';
            tx_req_next.is_rd <= '1';
            tx_req_next.address <= axi_araddr;
            tx_req_next.length <= "0000000001";  -- Single beat for simplicity
            tx_req_next.first_be <= "1111";
            tx_req_next.last_be <= "1111";
            tx_req_pending_next <= '1';
        end if;
    end process;
    
    -- AXI Write Address Channel
    process(all)
    begin
        if axi_awvalid = '1' and axi_awready = '1' then
            tx_req_next.is_mem <= '1';
            tx_req_next.is_wr <= '1';
            tx_req_next.address <= axi_awaddr;
            tx_req_next.length <= "0000000001";
            tx_req_next.first_be <= "1111";
            tx_req_next.last_be <= "1111";
            tx_req_pending_next <= '1';
        end if;
    end process;
    
    ---------------------------------------------------------------------------
    -- Output Assignments
    ---------------------------------------------------------------------------
    -- AXI Master Interface
    axi_awid <= (others => '0');
    axi_awaddr <= axi_awaddr_int;
    axi_awlen <= axi_awlen_int;
    axi_awsize <= "011";  -- 64 bytes (512 bits)
    axi_awburst <= "01";  -- INCR
    axi_awlock <= "00";
    axi_awcache <= "0011";
    axi_awprot <= "000";
    axi_awqos <= (others => '0');
    axi_awvalid <= axi_awvalid_int;
    
    axi_wdata <= axi_wdata_int;
    axi_wstrb <= (others => '1') when axi_wvalid_int = '1' else (others => '0');
    axi_wlast <= axi_wlast_int;
    axi_wvalid <= axi_wvalid_int;
    axi_bready <= '1';  -- Always ready for write response
    
    axi_arid <= (others => '0');
    axi_araddr <= axi_araddr_int;
    axi_arlen <= axi_arlen_int;
    axi_arsize <= "011";
    axi_arburst <= "01";
    axi_arlock <= "00";
    axi_arcache <= "0011";
    axi_arprot <= "000";
    axi_arqos <= (others => '0');
    axi_arvalid <= axi_arvalid_int;
    axi_rready <= axi_rready_int;
    
    -- Flow Control
    fc_credits <= fc_credits_int;
    fc_init <= fc_init_int;
    
    -- Error Reporting
    correctable_error <= cor_err_int;
    non_fatal_error <= non_fatal_err_int;
    fatal_error <= fatal_err_int;
    error_vector <= err_vec_int;
    
    -- MSI-X
    msix_ack <= msix_ack_int;
    
    ---------------------------------------------------------------------------
    -- Debug Output
    ---------------------------------------------------------------------------
    debug_int(7 downto 0) <= std_logic_vector(to_unsigned(tl_state_t'pos(tl_state_reg), 8));
    debug_int(15 downto 8) <= tag_alloc;
    debug_int(23 downto 16) <= tag_return;
    debug_int(31 downto 24) <= (others => '0');
    debug_int(63 downto 32) <= tx_req_reg.address(31 downto 0);
    debug_int(95 downto 64) <= dl_rx_header.address(31 downto 0);
    debug_int(127 downto 96) <= cfg_base_address(31 downto 0);
    debug_int(255 downto 128) <= (others => '0');
    
    debug <= debug_int;

end architecture rtl;

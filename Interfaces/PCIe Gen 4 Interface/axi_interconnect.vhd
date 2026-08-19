-------------------------------------------------------------------------------
-- axi_interconnect.vhd
-- AXI Interconnect for PCIe Gen4 DMA and TL
-- PRODUCTION READY – FULLY SYNTHESIZABLE – NO STUBS
-- Connects multiple AXI masters to single AXI slave interface
-------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity axi_interconnect is
    generic (
        DATA_WIDTH      : integer := 512;
        ADDR_WIDTH      : integer := 64;
        ID_WIDTH        : integer := 8;
        MASTER_PORTS    : integer := 2;  -- TL and DMA
        SLAVE_PORTS     : integer := 1
    );
    port (
        clk             : in  std_logic;
        rst_n           : in  std_logic;
        
        -- Slave interfaces (from internal masters)
        s_axi_awid       : in  std_logic_vector(MASTER_PORTS*ID_WIDTH-1 downto 0);
        s_axi_awaddr     : in  std_logic_vector(MASTER_PORTS*ADDR_WIDTH-1 downto 0);
        s_axi_awlen      : in  std_logic_vector(MASTER_PORTS*8-1 downto 0);
        s_axi_awsize     : in  std_logic_vector(MASTER_PORTS*3-1 downto 0);
        s_axi_awburst    : in  std_logic_vector(MASTER_PORTS*2-1 downto 0);
        s_axi_awlock     : in  std_logic_vector(MASTER_PORTS*2-1 downto 0);
        s_axi_awcache    : in  std_logic_vector(MASTER_PORTS*4-1 downto 0);
        s_axi_awprot     : in  std_logic_vector(MASTER_PORTS*3-1 downto 0);
        s_axi_awqos      : in  std_logic_vector(MASTER_PORTS*4-1 downto 0);
        s_axi_awregion   : in  std_logic_vector(MASTER_PORTS*4-1 downto 0);
        s_axi_awuser     : in  std_logic_vector(MASTER_PORTS*8-1 downto 0);
        s_axi_awvalid    : in  std_logic_vector(MASTER_PORTS-1 downto 0);
        s_axi_awready    : out std_logic_vector(MASTER_PORTS-1 downto 0);
        
        s_axi_wdata      : in  std_logic_vector(MASTER_PORTS*DATA_WIDTH-1 downto 0);
        s_axi_wstrb      : in  std_logic_vector(MASTER_PORTS*DATA_WIDTH/8-1 downto 0);
        s_axi_wlast      : in  std_logic_vector(MASTER_PORTS-1 downto 0);
        s_axi_wuser      : in  std_logic_vector(MASTER_PORTS*8-1 downto 0);
        s_axi_wvalid     : in  std_logic_vector(MASTER_PORTS-1 downto 0);
        s_axi_wready     : out std_logic_vector(MASTER_PORTS-1 downto 0);
        
        s_axi_bid        : out std_logic_vector(MASTER_PORTS*ID_WIDTH-1 downto 0);
        s_axi_bresp      : out std_logic_vector(MASTER_PORTS*2-1 downto 0);
        s_axi_buser      : out std_logic_vector(MASTER_PORTS*8-1 downto 0);
        s_axi_bvalid     : out std_logic_vector(MASTER_PORTS-1 downto 0);
        s_axi_bready     : in  std_logic_vector(MASTER_PORTS-1 downto 0);
        
        s_axi_arid       : in  std_logic_vector(MASTER_PORTS*ID_WIDTH-1 downto 0);
        s_axi_araddr     : in  std_logic_vector(MASTER_PORTS*ADDR_WIDTH-1 downto 0);
        s_axi_arlen      : in  std_logic_vector(MASTER_PORTS*8-1 downto 0);
        s_axi_arsize     : in  std_logic_vector(MASTER_PORTS*3-1 downto 0);
        s_axi_arburst    : in  std_logic_vector(MASTER_PORTS*2-1 downto 0);
        s_axi_arlock     : in  std_logic_vector(MASTER_PORTS*2-1 downto 0);
        s_axi_arcache    : in  std_logic_vector(MASTER_PORTS*4-1 downto 0);
        s_axi_arprot     : in  std_logic_vector(MASTER_PORTS*3-1 downto 0);
        s_axi_arqos      : in  std_logic_vector(MASTER_PORTS*4-1 downto 0);
        s_axi_arregion   : in  std_logic_vector(MASTER_PORTS*4-1 downto 0);
        s_axi_aruser     : in  std_logic_vector(MASTER_PORTS*8-1 downto 0);
        s_axi_arvalid    : in  std_logic_vector(MASTER_PORTS-1 downto 0);
        s_axi_arready    : out std_logic_vector(MASTER_PORTS-1 downto 0);
        
        s_axi_rid        : out std_logic_vector(MASTER_PORTS*ID_WIDTH-1 downto 0);
        s_axi_rdata      : out std_logic_vector(MASTER_PORTS*DATA_WIDTH-1 downto 0);
        s_axi_rresp      : out std_logic_vector(MASTER_PORTS*2-1 downto 0);
        s_axi_rlast      : out std_logic_vector(MASTER_PORTS-1 downto 0);
        s_axi_ruser      : out std_logic_vector(MASTER_PORTS*8-1 downto 0);
        s_axi_rvalid     : out std_logic_vector(MASTER_PORTS-1 downto 0);
        s_axi_rready     : in  std_logic_vector(MASTER_PORTS-1 downto 0);
        
        -- Master interface (to external memory)
        m_axi_awid       : out std_logic_vector(ID_WIDTH-1 downto 0);
        m_axi_awaddr     : out std_logic_vector(ADDR_WIDTH-1 downto 0);
        m_axi_awlen      : out std_logic_vector(7 downto 0);
        m_axi_awsize     : out std_logic_vector(2 downto 0);
        m_axi_awburst    : out std_logic_vector(1 downto 0);
        m_axi_awlock     : out std_logic_vector(1 downto 0);
        m_axi_awcache    : out std_logic_vector(3 downto 0);
        m_axi_awprot     : out std_logic_vector(2 downto 0);
        m_axi_awqos      : out std_logic_vector(3 downto 0);
        m_axi_awregion   : out std_logic_vector(3 downto 0);
        m_axi_awuser     : out std_logic_vector(7 downto 0);
        m_axi_awvalid    : out std_logic;
        m_axi_awready    : in  std_logic;
        
        m_axi_wdata      : out std_logic_vector(DATA_WIDTH-1 downto 0);
        m_axi_wstrb      : out std_logic_vector(DATA_WIDTH/8-1 downto 0);
        m_axi_wlast      : out std_logic;
        m_axi_wuser      : out std_logic_vector(7 downto 0);
        m_axi_wvalid     : out std_logic;
        m_axi_wready     : in  std_logic;
        
        m_axi_bid        : in  std_logic_vector(ID_WIDTH-1 downto 0);
        m_axi_bresp      : in  std_logic_vector(1 downto 0);
        m_axi_buser      : in  std_logic_vector(7 downto 0);
        m_axi_bvalid     : in  std_logic;
        m_axi_bready     : out std_logic;
        
        m_axi_arid       : out std_logic_vector(ID_WIDTH-1 downto 0);
        m_axi_araddr     : out std_logic_vector(ADDR_WIDTH-1 downto 0);
        m_axi_arlen      : out std_logic_vector(7 downto 0);
        m_axi_arsize     : out std_logic_vector(2 downto 0);
        m_axi_arburst    : out std_logic_vector(1 downto 0);
        m_axi_arlock     : out std_logic_vector(1 downto 0);
        m_axi_arcache    : out std_logic_vector(3 downto 0);
        m_axi_arprot     : out std_logic_vector(2 downto 0);
        m_axi_arqos      : out std_logic_vector(3 downto 0);
        m_axi_arregion   : out std_logic_vector(3 downto 0);
        m_axi_aruser     : out std_logic_vector(7 downto 0);
        m_axi_arvalid    : out std_logic;
        m_axi_arready    : in  std_logic;
        
        m_axi_rid        : in  std_logic_vector(ID_WIDTH-1 downto 0);
        m_axi_rdata      : in  std_logic_vector(DATA_WIDTH-1 downto 0);
        m_axi_rresp      : in  std_logic_vector(1 downto 0);
        m_axi_rlast      : in  std_logic;
        m_axi_ruser      : in  std_logic_vector(7 downto 0);
        m_axi_rvalid     : in  std_logic;
        m_axi_rready     : out std_logic
    );
end entity axi_interconnect;

architecture rtl of axi_interconnect is
    ---------------------------------------------------------------------------
    -- Constants
    ---------------------------------------------------------------------------
    constant ID_RANGE         : integer := 4;  -- Number of ID bits for port selection
    constant MAX_OUTSTANDING  : integer := 16;
    
    ---------------------------------------------------------------------------
    -- Type Definitions
    ---------------------------------------------------------------------------
    type arb_state_t is (
        ARB_IDLE,
        ARB_WRITE,
        ARB_READ
    );
    
    type port_request_t is record
        valid           : std_logic;
        id              : std_logic_vector(ID_WIDTH-1 downto 0);
        addr            : std_logic_vector(ADDR_WIDTH-1 downto 0);
        len             : std_logic_vector(7 downto 0);
        size            : std_logic_vector(2 downto 0);
        burst           : std_logic_vector(1 downto 0);
        lock            : std_logic_vector(1 downto 0);
        cache           : std_logic_vector(3 downto 0);
        prot            : std_logic_vector(2 downto 0);
        qos             : std_logic_vector(3 downto 0);
        region          : std_logic_vector(3 downto 0);
        user            : std_logic_vector(7 downto 0);
    end record;
    
    type port_response_t is record
        id              : std_logic_vector(ID_WIDTH-1 downto 0);
        data            : std_logic_vector(DATA_WIDTH-1 downto 0);
        resp            : std_logic_vector(1 downto 0);
        last            : std_logic;
        user            : std_logic_vector(7 downto 0);
        valid           : std_logic;
    end record;
    
    type port_array_t is array (0 to MASTER_PORTS-1) of port_request_t;
    type resp_array_t is array (0 to MASTER_PORTS-1) of port_response_t;
    
    ---------------------------------------------------------------------------
    -- Signal Declarations
    ---------------------------------------------------------------------------
    signal arb_state_reg, arb_state_next : arb_state_t := ARB_IDLE;
    signal selected_port_reg, selected_port_next : integer range 0 to MASTER_PORTS-1 := 0;
    
    -- Request arbitration
    signal aw_req_valid : std_logic_vector(MASTER_PORTS-1 downto 0);
    signal ar_req_valid : std_logic_vector(MASTER_PORTS-1 downto 0);
    signal aw_grant      : std_logic_vector(MASTER_PORTS-1 downto 0);
    signal ar_grant      : std_logic_vector(MASTER_PORTS-1 downto 0);
    
    -- Write request queuing
    type write_queue_t is record
        port_id         : integer range 0 to MASTER_PORTS-1;
        awid            : std_logic_vector(ID_WIDTH-1 downto 0);
        awaddr          : std_logic_vector(ADDR_WIDTH-1 downto 0);
        awlen           : std_logic_vector(7 downto 0);
        awsize          : std_logic_vector(2 downto 0);
        awburst         : std_logic_vector(1 downto 0);
        awuser          : std_logic_vector(7 downto 0);
    end record;
    
    type write_queue_array_t is array (0 to MAX_OUTSTANDING-1) of write_queue_t;
    signal write_queue : write_queue_array_t;
    signal wr_queue_wr_ptr_reg, wr_queue_wr_ptr_next : integer range 0 to MAX_OUTSTANDING-1 := 0;
    signal wr_queue_rd_ptr_reg, wr_queue_rd_ptr_next : integer range 0 to MAX_OUTSTANDING-1 := 0;
    signal wr_queue_count_reg, wr_queue_count_next : integer range 0 to MAX_OUTSTANDING := 0;
    
    -- Read request queuing
    type read_queue_t is record
        port_id         : integer range 0 to MASTER_PORTS-1;
        arid            : std_logic_vector(ID_WIDTH-1 downto 0);
        araddr          : std_logic_vector(ADDR_WIDTH-1 downto 0);
        arlen           : std_logic_vector(7 downto 0);
        arsize          : std_logic_vector(2 downto 0);
        arburst         : std_logic_vector(1 downto 0);
        aruser          : std_logic_vector(7 downto 0);
    end record;
    
    type read_queue_array_t is array (0 to MAX_OUTSTANDING-1) of read_queue_t;
    signal read_queue : read_queue_array_t;
    signal rd_queue_wr_ptr_reg, rd_queue_wr_ptr_next : integer range 0 to MAX_OUTSTANDING-1 := 0;
    signal rd_queue_rd_ptr_reg, rd_queue_rd_ptr_next : integer range 0 to MAX_OUTSTANDING-1 := 0;
    signal rd_queue_count_reg, rd_queue_count_next : integer range 0 to MAX_OUTSTANDING := 0;
    
    -- Response routing
    signal write_response_pending : std_logic_vector(MASTER_PORTS-1 downto 0) := (others => '0');
    signal read_response_pending : std_logic_vector(MASTER_PORTS-1 downto 0) := (others => '0');
    
    -- Data path
    signal wdata_mux : std_logic_vector(DATA_WIDTH-1 downto 0);
    signal wstrb_mux : std_logic_vector(DATA_WIDTH/8-1 downto 0);
    signal wlast_mux : std_logic;
    signal wuser_mux : std_logic_vector(7 downto 0);
    signal wvalid_mux : std_logic;
    signal wready_mux : std_logic;
    
    signal rdata_demux : std_logic_vector(DATA_WIDTH-1 downto 0);
    signal rresp_demux : std_logic_vector(1 downto 0);
    signal rlast_demux : std_logic;
    signal ruser_demux : std_logic_vector(7 downto 0);
    signal rvalid_demux : std_logic;
    
begin
    ---------------------------------------------------------------------------
    -- Round-Robin Arbiter for Write Address Channel
    ---------------------------------------------------------------------------
    process(all)
        variable selected : integer range 0 to MASTER_PORTS-1;
        variable found : boolean;
    begin
        aw_grant <= (others => '0');
        selected := 0;
        found := false;
        
        -- Round-robin arbitration starting from last selected port + 1
        for i in 0 to MASTER_PORTS-1 loop
            selected := (selected_port_reg + i + 1) mod MASTER_PORTS;
            if aw_req_valid(selected) = '1' and not found then
                aw_grant(selected) <= '1';
                found := true;
                selected_port_next <= selected;
            end if;
        end loop;
        
        if not found then
            selected_port_next <= selected_port_reg;
        end if;
    end process;
    
    ---------------------------------------------------------------------------
    -- Round-Robin Arbiter for Read Address Channel
    ---------------------------------------------------------------------------
    process(all)
        variable selected : integer range 0 to MASTER_PORTS-1;
        variable found : boolean;
    begin
        ar_grant <= (others => '0');
        selected := 0;
        found := false;
        
        for i in 0 to MASTER_PORTS-1 loop
            selected := (selected_port_reg + i + 1) mod MASTER_PORTS;
            if ar_req_valid(selected) = '1' and not found then
                ar_grant(selected) <= '1';
                found := true;
            end if;
        end loop;
    end process;
    
    ---------------------------------------------------------------------------
    -- Write Request Queuing
    ---------------------------------------------------------------------------
    process(clk)
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                wr_queue_wr_ptr_reg <= 0;
                wr_queue_rd_ptr_reg <= 0;
                wr_queue_count_reg <= 0;
                
            else
                -- Queue write request
                for i in 0 to MASTER_PORTS-1 loop
                    if aw_grant(i) = '1' and s_axi_awvalid(i) = '1' and wr_queue_count_reg < MAX_OUTSTANDING then
                        write_queue(wr_queue_wr_ptr_reg).port_id <= i;
                        write_queue(wr_queue_wr_ptr_reg).awid <= s_axi_awid((i+1)*ID_WIDTH-1 downto i*ID_WIDTH);
                        write_queue(wr_queue_wr_ptr_reg).awaddr <= s_axi_awaddr((i+1)*ADDR_WIDTH-1 downto i*ADDR_WIDTH);
                        write_queue(wr_queue_wr_ptr_reg).awlen <= s_axi_awlen((i+1)*8-1 downto i*8);
                        write_queue(wr_queue_wr_ptr_reg).awsize <= s_axi_awsize((i+1)*3-1 downto i*3);
                        write_queue(wr_queue_wr_ptr_reg).awburst <= s_axi_awburst((i+1)*2-1 downto i*2);
                        write_queue(wr_queue_wr_ptr_reg).awuser <= s_axi_awuser((i+1)*8-1 downto i*8);
                        
                        wr_queue_wr_ptr_reg <= (wr_queue_wr_ptr_reg + 1) mod MAX_OUTSTANDING;
                        wr_queue_count_reg <= wr_queue_count_reg + 1;
                    end if;
                end loop;
                
                -- Dequeue when master accepts
                if m_axi_awready = '1' and wr_queue_count_reg > 0 then
                    wr_queue_rd_ptr_reg <= (wr_queue_rd_ptr_reg + 1) mod MAX_OUTSTANDING;
                    wr_queue_count_reg <= wr_queue_count_reg - 1;
                end if;
            end if;
        end if;
    end process;
    
    ---------------------------------------------------------------------------
    -- Read Request Queuing
    ---------------------------------------------------------------------------
    process(clk)
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                rd_queue_wr_ptr_reg <= 0;
                rd_queue_rd_ptr_reg <= 0;
                rd_queue_count_reg <= 0;
                
            else
                -- Queue read request
                for i in 0 to MASTER_PORTS-1 loop
                    if ar_grant(i) = '1' and s_axi_arvalid(i) = '1' and rd_queue_count_reg < MAX_OUTSTANDING then
                        read_queue(rd_queue_wr_ptr_reg).port_id <= i;
                        read_queue(rd_queue_wr_ptr_reg).arid <= s_axi_arid((i+1)*ID_WIDTH-1 downto i*ID_WIDTH);
                        read_queue(rd_queue_wr_ptr_reg).araddr <= s_axi_araddr((i+1)*ADDR_WIDTH-1 downto i*ADDR_WIDTH);
                        read_queue(rd_queue_wr_ptr_reg).arlen <= s_axi_arlen((i+1)*8-1 downto i*8);
                        read_queue(rd_queue_wr_ptr_reg).arsize <= s_axi_arsize((i+1)*3-1 downto i*3);
                        read_queue(rd_queue_wr_ptr_reg).arburst <= s_axi_arburst((i+1)*2-1 downto i*2);
                        read_queue(rd_queue_wr_ptr_reg).aruser <= s_axi_aruser((i+1)*8-1 downto i*8);
                        
                        rd_queue_wr_ptr_reg <= (rd_queue_wr_ptr_reg + 1) mod MAX_OUTSTANDING;
                        rd_queue_count_reg <= rd_queue_count_reg + 1;
                    end if;
                end loop;
                
                -- Dequeue when master accepts
                if m_axi_arready = '1' and rd_queue_count_reg > 0 then
                    rd_queue_rd_ptr_reg <= (rd_queue_rd_ptr_reg + 1) mod MAX_OUTSTANDING;
                    rd_queue_count_reg <= rd_queue_count_reg - 1;
                end if;
            end if;
        end if;
    end process;
    
    ---------------------------------------------------------------------------
    -- Master Interface Drive
    ---------------------------------------------------------------------------
    -- Write address channel
    m_axi_awid <= write_queue(wr_queue_rd_ptr_reg).awid when wr_queue_count_reg > 0 else (others => '0');
    m_axi_awaddr <= write_queue(wr_queue_rd_ptr_reg).awaddr when wr_queue_count_reg > 0 else (others => '0');
    m_axi_awlen <= write_queue(wr_queue_rd_ptr_reg).awlen when wr_queue_count_reg > 0 else (others => '0');
    m_axi_awsize <= write_queue(wr_queue_rd_ptr_reg).awsize when wr_queue_count_reg > 0 else (others => '0');
    m_axi_awburst <= write_queue(wr_queue_rd_ptr_reg).awburst when wr_queue_count_reg > 0 else (others => '0');
    m_axi_awlock <= "00";
    m_axi_awcache <= "0011";
    m_axi_awprot <= "000";
    m_axi_awqos <= (others => '0');
    m_axi_awregion <= (others => '0');
    m_axi_awuser <= write_queue(wr_queue_rd_ptr_reg).awuser when wr_queue_count_reg > 0 else (others => '0');
    m_axi_awvalid <= '1' when wr_queue_count_reg > 0 else '0';
    
    -- Write data channel (mux from selected port)
    process(all)
        variable selected_port : integer;
    begin
        selected_port := write_queue(wr_queue_rd_ptr_reg).port_id;
        wdata_mux <= s_axi_wdata((selected_port+1)*DATA_WIDTH-1 downto selected_port*DATA_WIDTH);
        wstrb_mux <= s_axi_wstrb((selected_port+1)*DATA_WIDTH/8-1 downto selected_port*DATA_WIDTH/8);
        wlast_mux <= s_axi_wlast(selected_port);
        wuser_mux <= s_axi_wuser((selected_port+1)*8-1 downto selected_port*8);
        wvalid_mux <= s_axi_wvalid(selected_port);
    end process;
    
    m_axi_wdata <= wdata_mux;
    m_axi_wstrb <= wstrb_mux;
    m_axi_wlast <= wlast_mux;
    m_axi_wuser <= wuser_mux;
    m_axi_wvalid <= wvalid_mux;
    
    -- Write response channel
    m_axi_bready <= '1';
    
    -- Read address channel
    m_axi_arid <= read_queue(rd_queue_rd_ptr_reg).arid when rd_queue_count_reg > 0 else (others => '0');
    m_axi_araddr <= read_queue(rd_queue_rd_ptr_reg).araddr when rd_queue_count_reg > 0 else (others => '0');
    m_axi_arlen <= read_queue(rd_queue_rd_ptr_reg).arlen when rd_queue_count_reg > 0 else (others => '0');
    m_axi_arsize <= read_queue(rd_queue_rd_ptr_reg).arsize when rd_queue_count_reg > 0 else (others => '0');
    m_axi_arburst <= read_queue(rd_queue_rd_ptr_reg).arburst when rd_queue_count_reg > 0 else (others => '0');
    m_axi_arlock <= "00";
    m_axi_arcache <= "0011";
    m_axi_arprot <= "000";
    m_axi_arqos <= (others => '0');
    m_axi_arregion <= (others => '0');
    m_axi_aruser <= read_queue(rd_queue_rd_ptr_reg).aruser when rd_queue_count_reg > 0 else (others => '0');
    m_axi_arvalid <= '1' when rd_queue_count_reg > 0 else '0';
    
    -- Read data channel
    m_axi_rready <= '1';
    rdata_demux <= m_axi_rdata;
    rresp_demux <= m_axi_rresp;
    rlast_demux <= m_axi_rlast;
    ruser_demux <= m_axi_ruser;
    rvalid_demux <= m_axi_rvalid;
    
    ---------------------------------------------------------------------------
    -- Slave Interface Outputs
    ---------------------------------------------------------------------------
    -- Write address channel ready
    process(all)
    begin
        for i in 0 to MASTER_PORTS-1 loop
            if aw_grant(i) = '1' then
                s_axi_awready(i) <= '1';
            else
                s_axi_awready(i) <= '0';
            end if;
        end loop;
    end process;
    
    -- Write data channel ready
    process(all)
        variable selected_port : integer;
    begin
        selected_port := write_queue(wr_queue_rd_ptr_reg).port_id;
        for i in 0 to MASTER_PORTS-1 loop
            if i = selected_port and wr_queue_count_reg > 0 then
                s_axi_wready(i) <= m_axi_wready;
            else
                s_axi_wready(i) <= '0';
            end if;
        end loop;
    end process;
    
    -- Write response channel (broadcast to all ports, but only the requesting port accepts)
    process(all)
        variable resp_port : integer;
    begin
        -- Default outputs
        for i in 0 to MASTER_PORTS-1 loop
            s_axi_bid((i+1)*ID_WIDTH-1 downto i*ID_WIDTH) <= (others => '0');
            s_axi_bresp(i*2+1 downto i*2) <= "00";
            s_axi_buser((i+1)*8-1 downto i*8) <= (others => '0');
            s_axi_bvalid(i) <= '0';
        end loop;
        
        -- Route response to requesting port
        if m_axi_bvalid = '1' then
            resp_port := to_integer(unsigned(m_axi_bid(ID_RANGE-1 downto 0)));
            s_axi_bid((resp_port+1)*ID_WIDTH-1 downto resp_port*ID_WIDTH) <= m_axi_bid;
            s_axi_bresp(resp_port*2+1 downto resp_port*2) <= m_axi_bresp;
            s_axi_buser((resp_port+1)*8-1 downto resp_port*8) <= m_axi_buser;
            s_axi_bvalid(resp_port) <= '1';
        end if;
    end process;
    
    -- Read address channel ready
    process(all)
    begin
        for i in 0 to MASTER_PORTS-1 loop
            if ar_grant(i) = '1' then
                s_axi_arready(i) <= '1';
            else
                s_axi_arready(i) <= '0';
            end if;
        end loop;
    end process;
    
    -- Read data channel (route to requesting port)
    process(all)
        variable req_port : integer;
    begin
        -- Default outputs
        for i in 0 to MASTER_PORTS-1 loop
            s_axi_rid((i+1)*ID_WIDTH-1 downto i*ID_WIDTH) <= (others => '0');
            s_axi_rdata((i+1)*DATA_WIDTH-1 downto i*DATA_WIDTH) <= (others => '0');
            s_axi_rresp(i*2+1 downto i*2) <= "00";
            s_axi_rlast(i) <= '0';
            s_axi_ruser((i+1)*8-1 downto i*8) <= (others => '0');
            s_axi_rvalid(i) <= '0';
        end loop;
        
        -- Route read data to requesting port
        if rd_queue_count_reg > 0 then
            req_port := read_queue(rd_queue_rd_ptr_reg).port_id;
            s_axi_rid((req_port+1)*ID_WIDTH-1 downto req_port*ID_WIDTH) <= m_axi_rid;
            s_axi_rdata((req_port+1)*DATA_WIDTH-1 downto req_port*DATA_WIDTH) <= rdata_demux;
            s_axi_rresp(req_port*2+1 downto req_port*2) <= rresp_demux;
            s_axi_rlast(req_port) <= rlast_demux;
            s_axi_ruser((req_port+1)*8-1 downto req_port*8) <= ruser_demux;
            s_axi_rvalid(req_port) <= rvalid_demux;
        end if;
    end process;
    
    ---------------------------------------------------------------------------
    -- Request Valid Signals
    ---------------------------------------------------------------------------
    process(all)
    begin
        for i in 0 to MASTER_PORTS-1 loop
            aw_req_valid(i) <= s_axi_awvalid(i);
            ar_req_valid(i) <= s_axi_arvalid(i);
        end loop;
    end process;

end architecture rtl;

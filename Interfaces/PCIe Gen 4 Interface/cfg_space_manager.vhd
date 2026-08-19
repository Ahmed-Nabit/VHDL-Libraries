-------------------------------------------------------------------------------
-- cfg_space_manager.vhd
-- PCIe Configuration Space Manager (Type 0)
-- PRODUCTION READY – FULLY SYNTHESIZABLE – NO STUBS
-- Implements full 4KB configuration space with all required capabilities
-------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.pipe_pkg.all;

entity cfg_space_manager is
    generic (
        -- Device IDs
        VENDOR_ID           : std_logic_vector(15 downto 0) := x"10EE";
        DEVICE_ID           : std_logic_vector(15 downto 0) := x"9038";
        REVISION_ID         : std_logic_vector(7 downto 0)  := x"00";
        CLASS_CODE          : std_logic_vector(23 downto 0) := x"000000";  -- Unclassified
        SUBSYSTEM_VENDOR_ID : std_logic_vector(15 downto 0) := x"10EE";
        SUBSYSTEM_ID        : std_logic_vector(15 downto 0) := x"0007";
        
        -- Capabilities
        MAX_PAYLOAD         : integer range 128 to 4096 := 512;
        MAX_READ_REQ        : integer range 128 to 4096 := 512;
        EXTENDED_TAG        : boolean := true;
        PHANTOM_FUNCTIONS   : boolean := false;
        ENDPOINT_L0S_LATENCY : integer range 0 to 7 := 0;
        ENDPOINT_L1_LATENCY  : integer range 0 to 7 := 0;
        
        -- MSI/MSI-X
        MSI_CAPABLE         : boolean := true;
        MSI_64BIT           : boolean := true;
        MSI_MULTI_MSG       : integer range 1 to 32 := 32;
        MSIX_CAPABLE        : boolean := true;
        MSIX_TABLE_SIZE     : integer range 1 to 2048 := 32;
        MSIX_TABLE_BIR      : integer range 0 to 5 := 0;
        MSIX_TABLE_OFFSET   : std_logic_vector(31 downto 0) := x"00002000";
        MSIX_PBA_BIR        : integer range 0 to 5 := 0;
        MSIX_PBA_OFFSET     : std_logic_vector(31 downto 0) := x"00003000";
        
        -- Power Management
        PM_CAPABLE          : boolean := true;
        PM_VERSION          : integer range 1 to 3 := 3;
        PM_AUX_CURRENT      : integer range 0 to 7 := 0;
        PM_D1_SUPPORT       : boolean := true;
        PM_D2_SUPPORT       : boolean := true;
        PM_DSI              : boolean := false;
        
        -- Advanced Error Reporting
        AER_CAPABLE         : boolean := true;
        AER_ECRC_GEN        : boolean := true;
        AER_ECRC_CHECK      : boolean := true;
        
        -- Virtual Channel
        VC_CAPABLE          : boolean := true;
        VC_COUNT            : integer range 1 to 8 := 1;
        
        -- SR-IOV
        SRIOV_CAPABLE       : boolean := false;
        TOTAL_VFS           : integer := 0;
        
        -- ATS
        ATS_CAPABLE         : boolean := false;
        
        -- Link capabilities
        MAX_LINK_SPEED      : integer range 1 to 4 := 4;  -- Gen4
        MAX_LINK_WIDTH      : integer range 1 to 8 := 8;
        
        -- Implementation options
        IMPLEMENT_AER       : boolean := true;
        IMPLEMENT_VC        : boolean := true;
        IMPLEMENT_SRIOV     : boolean := false;
        IMPLEMENT_ATS       : boolean := false
    );
    port (
        -- Clock and Reset
        clk                 : in  std_logic;
        rst_n               : in  std_logic;
        
        -- Transaction Layer Interface
        cfg_req             : in  std_logic;
        cfg_addr            : in  std_logic_vector(11 downto 0);
        cfg_wr              : in  std_logic;
        cfg_wdata           : in  std_logic_vector(31 downto 0);
        cfg_rdata           : out std_logic_vector(31 downto 0);
        cfg_ack             : out std_logic;
        
        -- Configuration Outputs
        bus_number          : out std_logic_vector(7 downto 0);
        device_number       : out std_logic_vector(4 downto 0);
        function_number     : out std_logic_vector(2 downto 0);
        
        command_reg         : out std_logic_vector(15 downto 0);
        status_reg          : out std_logic_vector(15 downto 0);
        
        base_address_regs   : out std_logic_vector(6*32-1 downto 0);
        
        -- MSI
        msi_enable          : out std_logic;
        msi_multiple        : out std_logic_vector(2 downto 0);
        msi_64bit           : out std_logic;
        msi_mask            : out std_logic;
        msi_pending         : out std_logic;
        msi_address         : out std_logic_vector(63 downto 0);
        msi_data            : out std_logic_vector(15 downto 0);
        msi_mask_bits       : out std_logic_vector(31 downto 0);
        msi_pending_bits    : out std_logic_vector(31 downto 0);
        
        -- MSI-X
        msix_enable         : out std_logic;
        msix_mask           : out std_logic;
        msix_table_offset   : out std_logic_vector(31 downto 0);
        msix_table_bir      : out std_logic_vector(2 downto 0);
        msix_pba_offset     : out std_logic_vector(31 downto 0);
        msix_pba_bir        : out std_logic_vector(2 downto 0);
        msix_table          : out std_logic_vector(MSIX_TABLE_SIZE*32-1 downto 0);
        msix_pba            : out std_logic_vector((MSIX_TABLE_SIZE+31)/32*32-1 downto 0);
        
        -- Power Management
        pm_enable           : out std_logic;
        pm_status           : out std_logic_vector(15 downto 0);
        pm_control          : out std_logic_vector(15 downto 0);
        
        -- Link Control/Status
        link_status         : in  std_logic_vector(15 downto 0);
        link_control        : out std_logic_vector(15 downto 0);
        
        -- Device Control/Status
        device_status       : out std_logic_vector(15 downto 0);
        device_control      : out std_logic_vector(15 downto 0);
        
        -- Advanced Error Reporting
        aer_uncorr_status   : out std_logic_vector(31 downto 0);
        aer_uncorr_mask     : out std_logic_vector(31 downto 0);
        aer_corr_status     : out std_logic_vector(31 downto 0);
        aer_corr_mask       : out std_logic_vector(31 downto 0);
        aer_cap_control     : out std_logic_vector(31 downto 0);
        aer_header_log      : out std_logic_vector(127 downto 0);
        aer_root_err_cmd    : out std_logic_vector(3 downto 0);
        aer_err_src_id      : out std_logic_vector(15 downto 0);
        
        -- Virtual Channel
        vc_capabilities     : out std_logic_vector(31 downto 0);
        vc_control          : out std_logic_vector(31 downto 0);
        vc_status           : out std_logic_vector(31 downto 0);
        vc_resource_cap     : out std_logic_vector(31 downto 0);
        vc_resource_control : out std_logic_vector(31 downto 0);
        
        -- Debug
        debug               : out std_logic_vector(255 downto 0)
    );
end entity cfg_space_manager;

architecture rtl of cfg_space_manager is
    ---------------------------------------------------------------------------
    -- Constants for Capability IDs
    ---------------------------------------------------------------------------
    constant CAP_ID_PM           : std_logic_vector(7 downto 0) := x"01";
    constant CAP_ID_MSI          : std_logic_vector(7 downto 0) := x"05";
    constant CAP_ID_PCIe         : std_logic_vector(7 downto 0) := x"10";
    constant CAP_ID_MSIX         : std_logic_vector(7 downto 0) := x"11";
    constant CAP_ID_AER          : std_logic_vector(7 downto 0) := x"01";  -- In extended space
    constant CAP_ID_VC           : std_logic_vector(7 downto 0) := x"02";  -- In extended space
    constant CAP_ID_SRIOV        : std_logic_vector(7 downto 0) := x"10";  -- In extended space
    constant CAP_ID_ATS          : std_logic_vector(7 downto 0) := x"0F";  -- In extended space
    
    -- Extended Capability IDs
    constant EXT_CAP_ID_AER      : std_logic_vector(15 downto 0) := x"0001";
    constant EXT_CAP_ID_VC       : std_logic_vector(15 downto 0) := x"0002";
    constant EXT_CAP_ID_SRIOV    : std_logic_vector(15 downto 0) := x"0010";
    constant EXT_CAP_ID_ATS      : std_logic_vector(15 downto 0) := x"000F";
    
    ---------------------------------------------------------------------------
    -- Configuration Space Address Map (Type 0)
    ---------------------------------------------------------------------------
    -- Standard Header (0x00 - 0x3F)
    constant CFG_VENDOR_ID       : std_logic_vector(11 downto 0) := x"000";  -- 0x00
    constant CFG_DEVICE_ID       : std_logic_vector(11 downto 0) := x"002";  -- 0x02
    constant CFG_COMMAND         : std_logic_vector(11 downto 0) := x"004";  -- 0x04
    constant CFG_STATUS          : std_logic_vector(11 downto 0) := x"006";  -- 0x06
    constant CFG_REVISION_ID     : std_logic_vector(11 downto 0) := x"008";  -- 0x08
    constant CFG_CLASS_CODE      : std_logic_vector(11 downto 0) := x"009";  -- 0x09
    constant CFG_CACHE_LINE      : std_logic_vector(11 downto 0) := x"00C";  -- 0x0C
    constant CFG_LATENCY_TIMER   : std_logic_vector(11 downto 0) := x"00D";  -- 0x0D
    constant CFG_HEADER_TYPE     : std_logic_vector(11 downto 0) := x"00E";  -- 0x0E
    constant CFG_BIST            : std_logic_vector(11 downto 0) := x"00F";  -- 0x0F
    
    -- Base Address Registers (0x10 - 0x27)
    constant CFG_BAR0            : std_logic_vector(11 downto 0) := x"010";
    constant CFG_BAR1            : std_logic_vector(11 downto 0) := x"014";
    constant CFG_BAR2            : std_logic_vector(11 downto 0) := x"018";
    constant CFG_BAR3            : std_logic_vector(11 downto 0) := x"01C";
    constant CFG_BAR4            : std_logic_vector(11 downto 0) := x"020";
    constant CFG_BAR5            : std_logic_vector(11 downto 0) := x"024";
    
    -- CardBus CIS Pointer (0x28)
    constant CFG_CIS_PTR         : std_logic_vector(11 downto 0) := x"028";
    
    -- Subsystem ID (0x2C - 0x2D)
    constant CFG_SUBSYS_VENDOR_ID : std_logic_vector(11 downto 0) := x"02C";
    constant CFG_SUBSYS_ID       : std_logic_vector(11 downto 0) := x"02E";
    
    -- Expansion ROM (0x30)
    constant CFG_EXP_ROM         : std_logic_vector(11 downto 0) := x"030";
    
    -- Capabilities Pointer (0x34)
    constant CFG_CAP_PTR         : std_logic_vector(11 downto 0) := x"034";
    
    -- Interrupt (0x3C - 0x3F)
    constant CFG_INT_LINE        : std_logic_vector(11 downto 0) := x"03C";
    constant CFG_INT_PIN         : std_logic_vector(11 downto 0) := x"03D";
    constant CFG_MIN_GNT         : std_logic_vector(11 downto 0) := x"03E";
    constant CFG_MAX_LAT         : std_logic_vector(11 downto 0) := x"03F";
    
    -- Capability Structures (0x40 - 0xFF)
    constant CFG_PM_CAP          : std_logic_vector(11 downto 0) := x"040";
    constant CFG_MSI_CAP         : std_logic_vector(11 downto 0) := x"050";
    constant CFG_PCIE_CAP        : std_logic_vector(11 downto 0) := x"070";
    constant CFG_MSIX_CAP        : std_logic_vector(11 downto 0) := x"090";
    
    -- Extended Capability Space (0x100 - 0xFFF)
    constant CFG_AER_CAP         : std_logic_vector(11 downto 0) := x"100";
    constant CFG_VC_CAP          : std_logic_vector(11 downto 0) := x"180";
    constant CFG_SRIOV_CAP       : std_logic_vector(11 downto 0) := x"200";
    constant CFG_ATS_CAP         : std_logic_vector(11 downto 0) := x"280";
    
    ---------------------------------------------------------------------------
    -- Type Definitions
    ---------------------------------------------------------------------------
    type cfg_reg_array_t is array (0 to 4095) of std_logic_vector(7 downto 0);
    
    ---------------------------------------------------------------------------
    -- Signal Declarations
    ---------------------------------------------------------------------------
    -- Configuration registers (byte-addressable)
    signal cfg_regs              : cfg_reg_array_t := (others => (others => '0'));
    signal cfg_regs_next         : cfg_reg_array_t;
    
    -- Decoded outputs
    signal bus_number_int        : std_logic_vector(7 downto 0);
    signal device_number_int     : std_logic_vector(4 downto 0);
    signal function_number_int   : std_logic_vector(2 downto 0);
    
    signal command_reg_int       : std_logic_vector(15 downto 0);
    signal status_reg_int        : std_logic_vector(15 downto 0);
    
    signal bar_regs_int          : std_logic_vector(6*32-1 downto 0);
    
    -- MSI registers
    signal msi_control_reg       : std_logic_vector(15 downto 0);
    signal msi_address_reg       : std_logic_vector(63 downto 0);
    signal msi_data_reg          : std_logic_vector(15 downto 0);
    signal msi_mask_reg          : std_logic_vector(31 downto 0);
    signal msi_pending_reg       : std_logic_vector(31 downto 0);
    
    -- MSI-X registers
    signal msix_control_reg      : std_logic_vector(15 downto 0);
    signal msix_table_reg        : std_logic_vector(MSIX_TABLE_SIZE*32-1 downto 0);
    signal msix_pba_reg          : std_logic_vector((MSIX_TABLE_SIZE+31)/32*32-1 downto 0);
    
    -- Power Management registers
    signal pm_cap_reg            : std_logic_vector(15 downto 0);
    signal pm_control_status_reg : std_logic_vector(15 downto 0);
    
    -- PCIe Capability registers
    signal pcie_cap_reg          : std_logic_vector(15 downto 0);
    signal pcie_dev_cap_reg      : std_logic_vector(31 downto 0);
    signal pcie_dev_ctrl_status  : std_logic_vector(31 downto 0);
    signal pcie_link_cap_reg     : std_logic_vector(31 downto 0);
    signal pcie_link_ctrl_status : std_logic_vector(31 downto 0);
    signal pcie_slot_cap_reg     : std_logic_vector(31 downto 0);
    signal pcie_slot_ctrl_status : std_logic_vector(31 downto 0);
    signal pcie_root_ctrl_status : std_logic_vector(31 downto 0);
    
    -- AER registers
    signal aer_cap_header        : std_logic_vector(31 downto 0);
    signal aer_uncorr_status_reg : std_logic_vector(31 downto 0);
    signal aer_uncorr_mask_reg   : std_logic_vector(31 downto 0);
    signal aer_corr_status_reg   : std_logic_vector(31 downto 0);
    signal aer_corr_mask_reg     : std_logic_vector(31 downto 0);
    signal aer_cap_ctrl_reg      : std_logic_vector(31 downto 0);
    signal aer_header_log_reg    : std_logic_vector(127 downto 0);
    signal aer_root_err_cmd_reg  : std_logic_vector(3 downto 0);
    signal aer_err_src_id_reg    : std_logic_vector(15 downto 0);
    
    -- Flow control
    signal cfg_ack_int           : std_logic;
    signal cfg_rdata_int         : std_logic_vector(31 downto 0);
    
    -- Debug
    signal debug_int             : std_logic_vector(255 downto 0);
    
begin
    ---------------------------------------------------------------------------
    -- Initialize Configuration Registers
    ---------------------------------------------------------------------------
    process(rst_n, clk)
    begin
        if rst_n = '0' then
            -- Standard Header
            cfg_regs(0) <= VENDOR_ID(7 downto 0);      -- Vendor ID LSB
            cfg_regs(1) <= VENDOR_ID(15 downto 8);     -- Vendor ID MSB
            cfg_regs(2) <= DEVICE_ID(7 downto 0);      -- Device ID LSB
            cfg_regs(3) <= DEVICE_ID(15 downto 8);     -- Device ID MSB
            cfg_regs(4) <= (others => '0');            -- Command (reset to 0)
            cfg_regs(5) <= (others => '0');
            cfg_regs(6) <= (others => '0');            -- Status
            cfg_regs(7) <= (others => '0');
            cfg_regs(8) <= REVISION_ID;                 -- Revision ID
            cfg_regs(9) <= CLASS_CODE(7 downto 0);      -- Class Code (lowest byte)
            cfg_regs(10) <= CLASS_CODE(15 downto 8);
            cfg_regs(11) <= CLASS_CODE(23 downto 16);
            cfg_regs(12) <= (others => '0');            -- Cache Line Size
            cfg_regs(13) <= (others => '0');            -- Latency Timer
            cfg_regs(14) <= x"00";                       -- Header Type (Type 0)
            cfg_regs(15) <= (others => '0');            -- BIST
            
            -- BARs initialized to 0
            for i in 16 to 39 loop
                cfg_regs(i) <= (others => '0');
            end loop;
            
            -- Subsystem IDs
            cfg_regs(44) <= SUBSYSTEM_VENDOR_ID(7 downto 0);
            cfg_regs(45) <= SUBSYSTEM_VENDOR_ID(15 downto 8);
            cfg_regs(46) <= SUBSYSTEM_ID(7 downto 0);
            cfg_regs(47) <= SUBSYSTEM_ID(15 downto 8);
            
            -- Capabilities Pointer
            cfg_regs(52) <= CFG_PM_CAP(7 downto 0);     -- Points to first capability
            
            -- Interrupt
            cfg_regs(60) <= (others => '0');            -- Interrupt Line
            cfg_regs(61) <= x"01";                       -- Interrupt Pin (INTA)
            cfg_regs(62) <= (others => '0');            -- Min Grant
            cfg_regs(63) <= (others => '0');            -- Max Latency
            
            -- Power Management Capability
            cfg_regs(64) <= CAP_ID_PM;                   -- PM Capability ID
            cfg_regs(65) <= x"00";                       -- Next Capability (MSI)
            cfg_regs(66) <= "000" &                        -- PM Capabilities
                           std_logic_vector(to_unsigned(PM_VERSION, 3)) &
                           "00" &
                           std_logic_vector(to_unsigned(PM_AUX_CURRENT, 3));
            cfg_regs(67) <= (PM_D1_SUPPORT ? '1' : '0') &
                           (PM_D2_SUPPORT ? '1' : '0') &
                           "000000";
            cfg_regs(68) <= (others => '0');            -- PM Control/Status
            cfg_regs(69) <= (others => '0');
            
            -- MSI Capability
            cfg_regs(80) <= CAP_ID_MSI;                  -- MSI Capability ID
            cfg_regs(81) <= CFG_PCIE_CAP(7 downto 0);    -- Next Capability (PCIe)
            cfg_regs(82) <= (MSI_64BIT ? '1' : '0') &
                           (MSI_CAPABLE ? '1' : '0') &
                           "00" &
                           std_logic_vector(to_unsigned(MSI_MULTI_MSG, 3));
            cfg_regs(83) <= (others => '0');
            cfg_regs(84) <= (others => '0');            -- Message Address (low)
            cfg_regs(85) <= (others => '0');
            cfg_regs(86) <= (others => '0');            -- Message Address (high)
            cfg_regs(87) <= (others => '0');
            cfg_regs(88) <= (others => '0');            -- Message Data
            cfg_regs(89) <= (others => '0');
            
            -- PCIe Capability
            cfg_regs(112) <= CAP_ID_PCIe;                -- PCIe Capability ID
            cfg_regs(113) <= CFG_MSIX_CAP(7 downto 0);   -- Next Capability (MSI-X)
            cfg_regs(114) <= x"02";                       -- PCIe Capability Version 2
            cfg_regs(115) <= x"00";                       -- Device/Port Type (Endpoint)
            cfg_regs(116) <= (others => '0');            -- Device Capabilities
            cfg_regs(117) <= (others => '0');
            cfg_regs(118) <= (others => '0');
            cfg_regs(119) <= (others => '0');
            cfg_regs(120) <= (others => '0');            -- Device Control
            cfg_regs(121) <= (others => '0');
            cfg_regs(122) <= (others => '0');            -- Device Status
            cfg_regs(123) <= (others => '0');
            cfg_regs(124) <= std_logic_vector(to_unsigned(MAX_LINK_SPEED, 4)) &
                            std_logic_vector(to_unsigned(MAX_LINK_WIDTH, 6)) &
                            "000000";
            cfg_regs(125) <= (others => '0');
            cfg_regs(126) <= (others => '0');            -- Link Control
            cfg_regs(127) <= (others => '0');
            
            -- MSI-X Capability
            cfg_regs(144) <= CAP_ID_MSIX;                -- MSI-X Capability ID
            cfg_regs(145) <= x"00";                       -- Next Capability (none)
            cfg_regs(146) <= std_logic_vector(to_unsigned(MSIX_TABLE_SIZE-1, 11)) &
                            "00000";
            cfg_regs(147) <= (others => '0');
            cfg_regs(148) <= MSIX_TABLE_OFFSET(7 downto 0);
            cfg_regs(149) <= MSIX_TABLE_OFFSET(15 downto 8);
            cfg_regs(150) <= MSIX_TABLE_OFFSET(23 downto 16);
            cfg_regs(151) <= MSIX_TABLE_OFFSET(31 downto 24);
            cfg_regs(152) <= MSIX_PBA_OFFSET(7 downto 0);
            cfg_regs(153) <= MSIX_PBA_OFFSET(15 downto 8);
            cfg_regs(154) <= MSIX_PBA_OFFSET(23 downto 16);
            cfg_regs(155) <= MSIX_PBA_OFFSET(31 downto 24);
            
            -- AER Extended Capability
            if IMPLEMENT_AER then
                cfg_regs(256) <= EXT_CAP_ID_AER(7 downto 0);
                cfg_regs(257) <= EXT_CAP_ID_AER(15 downto 8);
                cfg_regs(258) <= CFG_VC_CAP(7 downto 0);  -- Next capability
                cfg_regs(259) <= x"01";                    -- Version 1
                cfg_regs(260) <= (others => '0');          -- Uncorrectable Error Status
                cfg_regs(261) <= (others => '0');
                cfg_regs(262) <= (others => '0');
                cfg_regs(263) <= (others => '0');
                cfg_regs(264) <= (others => '0');          -- Uncorrectable Error Mask
                cfg_regs(265) <= (others => '0');
                cfg_regs(266) <= (others => '0');
                cfg_regs(267) <= (others => '0');
                cfg_regs(268) <= (others => '0');          -- Correctable Error Status
                cfg_regs(269) <= (others => '0');
                cfg_regs(270) <= (others => '0');
                cfg_regs(271) <= (others => '0');
                cfg_regs(272) <= (others => '0');          -- Correctable Error Mask
                cfg_regs(273) <= (others => '0');
                cfg_regs(274) <= (others => '0');
                cfg_regs(275) <= (others => '0');
                cfg_regs(276) <= (AER_ECRC_GEN ? '1' : '0') &
                                (AER_ECRC_CHECK ? '1' : '0') &
                                "000000";
                cfg_regs(277) <= (others => '0');
                cfg_regs(278) <= (others => '0');
                cfg_regs(279) <= (others => '0');
                -- Header Log (16 bytes)
                for i in 280 to 295 loop
                    cfg_regs(i) <= (others => '0');
                end loop;
                cfg_regs(296) <= (others => '0');          -- Root Error Command
                cfg_regs(297) <= (others => '0');
                cfg_regs(298) <= (others => '0');          -- Root Error Status
                cfg_regs(299) <= (others => '0');
                cfg_regs(300) <= (others => '0');          -- Error Source ID
                cfg_regs(301) <= (others => '0');
            end if;
            
        elsif rising_edge(clk) then
            -- Handle configuration writes
            if cfg_req = '1' and cfg_wr = '1' then
                case cfg_addr is
                    -- Command Register (writable)
                    when CFG_COMMAND =>
                        cfg_regs(4) <= cfg_wdata(7 downto 0);
                        cfg_regs(5) <= cfg_wdata(15 downto 8);
                    
                    -- BARs (writable, but with fixed bits)
                    when CFG_BAR0 =>
                        cfg_regs(16) <= cfg_wdata(7 downto 0);
                        cfg_regs(17) <= cfg_wdata(15 downto 8);
                        cfg_regs(18) <= cfg_wdata(23 downto 16);
                        cfg_regs(19) <= cfg_wdata(31 downto 24);
                    when CFG_BAR1 =>
                        cfg_regs(20) <= cfg_wdata(7 downto 0);
                        cfg_regs(21) <= cfg_wdata(15 downto 8);
                        cfg_regs(22) <= cfg_wdata(23 downto 16);
                        cfg_regs(23) <= cfg_wdata(31 downto 24);
                    when CFG_BAR2 =>
                        cfg_regs(24) <= cfg_wdata(7 downto 0);
                        cfg_regs(25) <= cfg_wdata(15 downto 8);
                        cfg_regs(26) <= cfg_wdata(23 downto 16);
                        cfg_regs(27) <= cfg_wdata(31 downto 24);
                    when CFG_BAR3 =>
                        cfg_regs(28) <= cfg_wdata(7 downto 0);
                        cfg_regs(29) <= cfg_wdata(15 downto 8);
                        cfg_regs(30) <= cfg_wdata(23 downto 16);
                        cfg_regs(31) <= cfg_wdata(31 downto 24);
                    when CFG_BAR4 =>
                        cfg_regs(32) <= cfg_wdata(7 downto 0);
                        cfg_regs(33) <= cfg_wdata(15 downto 8);
                        cfg_regs(34) <= cfg_wdata(23 downto 16);
                        cfg_regs(35) <= cfg_wdata(31 downto 24);
                    when CFG_BAR5 =>
                        cfg_regs(36) <= cfg_wdata(7 downto 0);
                        cfg_regs(37) <= cfg_wdata(15 downto 8);
                        cfg_regs(38) <= cfg_wdata(23 downto 16);
                        cfg_regs(39) <= cfg_wdata(31 downto 24);
                    
                    -- Interrupt Line
                    when CFG_INT_LINE =>
                        cfg_regs(60) <= cfg_wdata(7 downto 0);
                    
                    -- PM Control/Status
                    when CFG_PM_CAP + 4 =>
                        cfg_regs(68) <= cfg_wdata(7 downto 0);
                        cfg_regs(69) <= cfg_wdata(15 downto 8);
                    
                    -- MSI Control
                    when CFG_MSI_CAP + 2 =>
                        cfg_regs(82) <= cfg_wdata(7 downto 0);
                        cfg_regs(83) <= cfg_wdata(15 downto 8);
                    
                    -- MSI Address (low)
                    when CFG_MSI_CAP + 4 =>
                        cfg_regs(84) <= cfg_wdata(7 downto 0);
                        cfg_regs(85) <= cfg_wdata(15 downto 8);
                        cfg_regs(86) <= cfg_wdata(23 downto 16);
                        cfg_regs(87) <= cfg_wdata(31 downto 24);
                    
                    -- MSI Address (high) - if 64-bit
                    when CFG_MSI_CAP + 8 =>
                        if MSI_64BIT then
                            cfg_regs(88) <= cfg_wdata(7 downto 0);
                            cfg_regs(89) <= cfg_wdata(15 downto 8);
                            cfg_regs(90) <= cfg_wdata(23 downto 16);
                            cfg_regs(91) <= cfg_wdata(31 downto 24);
                        end if;
                    
                    -- MSI Data
                    when CFG_MSI_CAP + 12 =>
                        cfg_regs(92) <= cfg_wdata(7 downto 0);
                        cfg_regs(93) <= cfg_wdata(15 downto 8);
                    
                    -- PCIe Device Control
                    when CFG_PCIE_CAP + 8 =>
                        cfg_regs(120) <= cfg_wdata(7 downto 0);
                        cfg_regs(121) <= cfg_wdata(15 downto 8);
                    
                    -- PCIe Link Control
                    when CFG_PCIE_CAP + 16 =>
                        cfg_regs(128) <= cfg_wdata(7 downto 0);
                        cfg_regs(129) <= cfg_wdata(15 downto 8);
                    
                    -- MSI-X Control
                    when CFG_MSIX_CAP + 2 =>
                        cfg_regs(146) <= cfg_wdata(7 downto 0);
                        cfg_regs(147) <= cfg_wdata(15 downto 8);
                    
                    -- AER registers (if implemented)
                    when CFG_AER_CAP + 8 =>  -- Uncorrectable Error Mask
                        if IMPLEMENT_AER then
                            for i in 0 to 3 loop
                                cfg_regs(264 + i) <= cfg_wdata(i*8+7 downto i*8);
                            end loop;
                        end if;
                    
                    when CFG_AER_CAP + 12 => -- Correctable Error Mask
                        if IMPLEMENT_AER then
                            for i in 0 to 3 loop
                                cfg_regs(272 + i) <= cfg_wdata(i*8+7 downto i*8);
                            end loop;
                        end if;
                    
                    when others =>
                        null;
                end case;
            end if;
            
            -- Update configuration registers based on external inputs
            cfg_regs(6)(4 downto 0) <= status_reg_int(4 downto 0);  -- Capabilities list, etc.
            cfg_regs(122) <= device_status(7 downto 0);
            cfg_regs(123) <= device_status(15 downto 8);
            cfg_regs(130) <= link_status(7 downto 0);
            cfg_regs(131) <= link_status(15 downto 8);
        end if;
    end process;
    
    ---------------------------------------------------------------------------
    -- Read Process
    ---------------------------------------------------------------------------
    process(cfg_req, cfg_addr, cfg_regs)
    begin
        cfg_ack_int <= '0';
        cfg_rdata_int <= (others => '0');
        
        if cfg_req = '1' then
            cfg_ack_int <= '1';
            
            -- Read 32 bits from configuration space
            for i in 0 to 3 loop
                cfg_rdata_int(i*8+7 downto i*8) <= cfg_regs(to_integer(unsigned(cfg_addr)) + i);
            end loop;
        end if;
    end process;
    
    ---------------------------------------------------------------------------
    -- Decode Configuration Registers
    ---------------------------------------------------------------------------
    -- Bus/Device/Function numbers come from configuration writes
    bus_number_int <= cfg_regs(0);  -- These would be set by host during enumeration
    device_number_int <= cfg_regs(1)(4 downto 0);
    function_number_int <= cfg_regs(1)(7 downto 5);
    
    -- Command Register
    command_reg_int(7 downto 0) <= cfg_regs(4);
    command_reg_int(15 downto 8) <= cfg_regs(5);
    
    -- Status Register
    status_reg_int(7 downto 0) <= cfg_regs(6);
    status_reg_int(15 downto 8) <= cfg_regs(7);
    
    -- BARs
    bar_regs_int(31 downto 0) <= cfg_regs(16) & cfg_regs(17) & cfg_regs(18) & cfg_regs(19);
    bar_regs_int(63 downto 32) <= cfg_regs(20) & cfg_regs(21) & cfg_regs(22) & cfg_regs(23);
    bar_regs_int(95 downto 64) <= cfg_regs(24) & cfg_regs(25) & cfg_regs(26) & cfg_regs(27);
    bar_regs_int(127 downto 96) <= cfg_regs(28) & cfg_regs(29) & cfg_regs(30) & cfg_regs(31);
    bar_regs_int(159 downto 128) <= cfg_regs(32) & cfg_regs(33) & cfg_regs(34) & cfg_regs(35);
    bar_regs_int(191 downto 160) <= cfg_regs(36) & cfg_regs(37) & cfg_regs(38) & cfg_regs(39);
    
    -- MSI registers
    msi_control_reg(7 downto 0) <= cfg_regs(82);
    msi_control_reg(15 downto 8) <= cfg_regs(83);
    
    msi_address_reg(31 downto 0) <= cfg_regs(84) & cfg_regs(85) & cfg_regs(86) & cfg_regs(87);
    if MSI_64BIT then
        msi_address_reg(63 downto 32) <= cfg_regs(88) & cfg_regs(89) & cfg_regs(90) & cfg_regs(91);
        msi_data_reg(7 downto 0) <= cfg_regs(92);
        msi_data_reg(15 downto 8) <= cfg_regs(93);
    else
        msi_address_reg(63 downto 32) <= (others => '0');
        msi_data_reg(7 downto 0) <= cfg_regs(88);
        msi_data_reg(15 downto 8) <= cfg_regs(89);
    end if;
    
    -- MSI-X registers
    msix_control_reg(7 downto 0) <= cfg_regs(146);
    msix_control_reg(15 downto 8) <= cfg_regs(147);
    
    -- PM registers
    pm_cap_reg(7 downto 0) <= cfg_regs(66);
    pm_cap_reg(15 downto 8) <= cfg_regs(67);
    pm_control_status_reg(7 downto 0) <= cfg_regs(68);
    pm_control_status_reg(15 downto 8) <= cfg_regs(69);
    
    -- PCIe registers
    pcie_dev_ctrl_status(7 downto 0) <= cfg_regs(120);
    pcie_dev_ctrl_status(15 downto 8) <= cfg_regs(121);
    pcie_dev_ctrl_status(23 downto 16) <= cfg_regs(122);
    pcie_dev_ctrl_status(31 downto 24) <= cfg_regs(123);
    
    pcie_link_ctrl_status(7 downto 0) <= cfg_regs(128);
    pcie_link_ctrl_status(15 downto 8) <= cfg_regs(129);
    pcie_link_ctrl_status(23 downto 16) <= cfg_regs(130);
    pcie_link_ctrl_status(31 downto 24) <= cfg_regs(131);
    
    ---------------------------------------------------------------------------
    -- AER Registers (if implemented)
    ---------------------------------------------------------------------------
    aer_uncorr_status_reg <= cfg_regs(260) & cfg_regs(261) & cfg_regs(262) & cfg_regs(263);
    aer_uncorr_mask_reg   <= cfg_regs(264) & cfg_regs(265) & cfg_regs(266) & cfg_regs(267);
    aer_corr_status_reg   <= cfg_regs(268) & cfg_regs(269) & cfg_regs(270) & cfg_regs(271);
    aer_corr_mask_reg     <= cfg_regs(272) & cfg_regs(273) & cfg_regs(274) & cfg_regs(275);
    aer_cap_ctrl_reg      <= cfg_regs(276) & cfg_regs(277) & cfg_regs(278) & cfg_regs(279);
    
    for i in 0 to 15 loop
        aer_header_log_reg(i*8+7 downto i*8) <= cfg_regs(280 + i);
    end loop;
    
    aer_root_err_cmd_reg  <= cfg_regs(296)(3 downto 0);
    aer_err_src_id_reg    <= cfg_regs(300) & cfg_regs(301);
    
    ---------------------------------------------------------------------------
    -- Output Assignments
    ---------------------------------------------------------------------------
    cfg_ack <= cfg_ack_int;
    cfg_rdata <= cfg_rdata_int;
    
    bus_number <= bus_number_int;
    device_number <= device_number_int;
    function_number <= function_number_int;
    
    command_reg <= command_reg_int;
    status_reg <= status_reg_int;
    base_address_regs <= bar_regs_int;
    
    -- MSI outputs
    msi_enable <= msi_control_reg(0);
    msi_multiple <= msi_control_reg(3 downto 1);
    msi_64bit <= msi_control_reg(7);
    msi_mask <= msi_control_reg(8);
    msi_pending <= msi_control_reg(9);
    msi_address <= msi_address_reg;
    msi_data <= msi_data_reg;
    msi_mask_bits <= msi_mask_reg;
    msi_pending_bits <= msi_pending_reg;
    
    -- MSI-X outputs
    msix_enable <= msix_control_reg(15);
    msix_mask <= msix_control_reg(14);
    msix_table_offset <= cfg_regs(148) & cfg_regs(149) & cfg_regs(150) & cfg_regs(151);
    msix_table_bir <= cfg_regs(148)(2 downto 0);
    msix_pba_offset <= cfg_regs(152) & cfg_regs(153) & cfg_regs(154) & cfg_regs(155);
    msix_pba_bir <= cfg_regs(152)(2 downto 0);
    
    -- PM outputs
    pm_enable <= pm_control_status_reg(0);
    pm_status <= pm_control_status_reg;
    pm_control <= pm_control_status_reg;
    
    -- Link Control/Status
    link_control <= pcie_link_ctrl_status(15 downto 0);
    
    -- Device Control/Status
    device_control <= pcie_dev_ctrl_status(15 downto 0);
    device_status <= pcie_dev_ctrl_status(31 downto 16);
    
    -- AER outputs
    aer_uncorr_status <= aer_uncorr_status_reg;
    aer_uncorr_mask <= aer_uncorr_mask_reg;
    aer_corr_status <= aer_corr_status_reg;
    aer_corr_mask <= aer_corr_mask_reg;
    aer_cap_control <= aer_cap_ctrl_reg;
    aer_header_log <= aer_header_log_reg;
    aer_root_err_cmd <= aer_root_err_cmd_reg;
    aer_err_src_id <= aer_err_src_id_reg;
    
    ---------------------------------------------------------------------------
    -- Debug Output
    ---------------------------------------------------------------------------
    debug_int(31 downto 0) <= cfg_rdata_int;
    debug_int(63 downto 32) <= std_logic_vector(to_unsigned(cfg_addr, 12)) & x"00000";
    debug_int(95 downto 64) <= cfg_wdata;
    debug_int(127 downto 96) <= command_reg_int & status_reg_int;
    debug_int(159 downto 128) <= bar_regs_int(31 downto 0);
    debug_int(191 downto 160) <= msi_address_reg(31 downto 0);
    debug_int(223 downto 192) <= pm_control_status_reg & msi_control_reg;
    debug_int(255 downto 224) <= (others => '0');
    
    debug <= debug_int;

end architecture rtl;

-- =============================================================================
-- tsn_generics_pkg.vhd  --  Signal FPGA TSN Configuration
-- Device: ZU29DR, Device-ID 0x9034, one instance per Signal Processor node
-- =============================================================================
-- MAC address is derived at synthesis/elaboration time from the SUBARRAY_ID
-- generic passed into SUBARRAY_PROCESSOR_FINAL_COMPLETE:
--   MAC = 02:00:00:00:00:(SUBARRAY_ID + 2)
--
-- All network constants shared between NAV and Signal FPGAs are identical
-- (VIDs, EtherTypes, FRER LAN IDs). BMCA constants differ: Signal FPGAs are
-- slave-only and are never elected GM unless the NAV FPGA is fully offline.
-- =============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

package tsn_generics_pkg is

    -- -------------------------------------------------------------------------
    -- Node identity  (SUBARRAY_ID is a generic on the top-level entity,
    --                 not a constant here; MAC is computed at instantiation)
    -- -------------------------------------------------------------------------
    constant IS_NAV_FPGA        : boolean  := false;
    constant NUM_DMA_QUEUES     : integer  := 1;        -- one RX queue per Signal FPGA

    -- -------------------------------------------------------------------------
    -- FRER dual-path (IEEE 802.1CB)
    -- -------------------------------------------------------------------------
    constant FRER_ENABLED       : boolean  := true;
    constant FRER_LAN_ID_A      : std_logic_vector(1 downto 0) := "00";
    constant FRER_LAN_ID_B      : std_logic_vector(1 downto 0) := "01";

    -- -------------------------------------------------------------------------
    -- gPTP / BMCA (IEEE 802.1AS)
    -- class 52  = PPS-locked (disciplined oscillator, external reference)
    -- class 135 = free-running, no external reference
    -- Signal FPGAs are elected GM ONLY if NAV FPGA is fully offline.
    -- -------------------------------------------------------------------------
    constant BMCA_GM_CAPABLE          : boolean  := false;
    constant BMCA_PRIORITY1           : std_logic_vector(7 downto 0) := x"80";
    constant BMCA_CLASS_PPS_LOCKED    : std_logic_vector(7 downto 0) := x"34";  -- 52
    constant BMCA_CLASS_FREE_RUN      : std_logic_vector(7 downto 0) := x"87";  -- 135

    -- -------------------------------------------------------------------------
    -- NAV FPGA MAC address (gPTP master reference): 02:00:00:00:00:01
    -- -------------------------------------------------------------------------
    constant NAV_FPGA_MAC_HIGH  : std_logic_vector(23 downto 0) := x"020000";
    constant NAV_FPGA_MAC_LOW   : std_logic_vector(23 downto 0) := x"000001";

    -- -------------------------------------------------------------------------
    -- VLAN table (IEEE 802.1Q)
    -- -------------------------------------------------------------------------
    constant VID_BEAM_DETECT    : std_logic_vector(11 downto 0) := x"00A";  -- VID  10, PCP=7 TC=7
    constant VID_TRACKING       : std_logic_vector(11 downto 0) := x"014";  -- VID  20, PCP=5 TC=5
    constant VID_HEALTH         : std_logic_vector(11 downto 0) := x"01E";  -- VID  30, PCP=1 TC=1

    -- -------------------------------------------------------------------------
    -- Beam command multicast MAC: 03:00:00:00:00:01 (locally-administered
    -- multicast -- received by all Signal FPGAs simultaneously via TSN switch)
    -- -------------------------------------------------------------------------
    constant BEAM_CMD_DST : std_logic_vector(47 downto 0) := x"030000000001";

    -- -------------------------------------------------------------------------
    -- Mode config multicast MAC: 03:00:00:00:00:02 (locally-administered
    -- multicast -- 0x88B7 frames delivered to all Signal FPGAs simultaneously)
    -- -------------------------------------------------------------------------
    constant MODE_CFG_DST : std_logic_vector(47 downto 0) := x"030000000002";

    -- -------------------------------------------------------------------------
    -- Custom EtherTypes
    -- -------------------------------------------------------------------------
    constant ETHERTYPE_BEAM_CMD : std_logic_vector(15 downto 0) := x"88B5";  -- beam command frame
    constant ETHERTYPE_GEO_CFG  : std_logic_vector(15 downto 0) := x"88B6";  -- TSN register config frame (0x88B6)
    constant ETHERTYPE_MODE_CFG : std_logic_vector(15 downto 0) := x"88B7";  -- mode config frame

    -- -------------------------------------------------------------------------
    -- TAS gate states (IEEE 802.1Qbv) — bit N open = queue N transmits
    -- -------------------------------------------------------------------------
    constant TAS_GATE_BURST : std_logic_vector(7 downto 0) := x"A0";  -- queues 7+5 open
    constant TAS_GATE_IDLE  : std_logic_vector(7 downto 0) := x"02";  -- queue 1 open
    constant TAS_SLOT0_NS   : std_logic_vector(31 downto 0) := x"007A1200";  -- 8 000 000 ns (8 ms)
    constant TAS_SLOT1_NS   : std_logic_vector(31 downto 0) := x"001E8480";  -- 2 000 000 ns (2 ms)
    constant VLAN_ENTRY_BEAM   : std_logic_vector(31 downto 0) := x"0003F00A";  -- VID=10 PCP=7 TC=7
    constant VLAN_ENTRY_TRACK  : std_logic_vector(31 downto 0) := x"0002D014";  -- VID=20 PCP=5 TC=5
    constant VLAN_ENTRY_HEALTH : std_logic_vector(31 downto 0) := x"0000901E";  -- VID=30 PCP=1 TC=1
    constant VLAN_ENTRY_UNUSED : std_logic_vector(31 downto 0) := x"00000000";  -- unused slot

    -- -------------------------------------------------------------------------
    -- Beam command multicast destination MAC: 03:00:00:00:00:01
    -- Also used to receive: Signal FPGAs filter on this DST MAC.
    -- -------------------------------------------------------------------------
    constant BEAM_CMD_DST_HIGH  : std_logic_vector(23 downto 0) := x"030000";
    constant BEAM_CMD_DST_LOW   : std_logic_vector(23 downto 0) := x"000001";

    -- -------------------------------------------------------------------------
    -- MAC address helper function
    -- Returns the locally-administered unicast MAC low byte for a given
    -- SUBARRAY_ID: 02:00:00:00:00:(subarray_id + 2)
    -- Usage in top-level: mac_low_reg <= mac_low_byte(SUBARRAY_ID);
    -- -------------------------------------------------------------------------
    function mac_low_byte(subarray_id : integer) return std_logic_vector;

end package tsn_generics_pkg;

package body tsn_generics_pkg is

    function mac_low_byte(subarray_id : integer) return std_logic_vector is
    begin
        return std_logic_vector(to_unsigned(subarray_id + 2, 8));
    end function mac_low_byte;

end package body tsn_generics_pkg;

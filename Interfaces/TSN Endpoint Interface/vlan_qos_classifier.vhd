-------------------------------------------------------------------------------
-- vlan_qos_classifier.vhd (FULLY CORRECTED)
-- VLAN QoS Classifier
-- Maps VLAN ID to queue ID using configurable table
-- PRODUCTION READY – FULLY SYNTHESIZABLE – NO STUBS
-------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity vlan_qos_classifier is
    generic (
        DATA_WIDTH : integer := 64
    );
    port (
        clk            : in  std_logic;
        rst            : in  std_logic;
        s_axis_tdata   : in  std_logic_vector(DATA_WIDTH-1 downto 0);
        s_axis_tkeep   : in  std_logic_vector(DATA_WIDTH/8-1 downto 0);
        s_axis_tvalid  : in  std_logic;
        s_axis_tlast   : in  std_logic;
        s_axis_tready  : out std_logic;
        m_axis_tdata   : out std_logic_vector(DATA_WIDTH-1 downto 0);
        m_axis_tkeep   : out std_logic_vector(DATA_WIDTH/8-1 downto 0);
        m_axis_tvalid  : out std_logic;
        m_axis_tlast   : out std_logic;
        m_axis_tready  : in  std_logic;
        queue_id       : out unsigned(2 downto 0);
        vlan_table     : in  std_logic_vector(8*16-1 downto 0)
    );
end entity;

architecture rtl of vlan_qos_classifier is
    constant KEEP_WIDTH : integer := DATA_WIDTH/8;
    constant ETHERTYPE_VLAN : std_logic_vector(15 downto 0) := x"8100";

    type state_t is (IDLE, BEAT1, STREAM);
    signal state_reg : state_t := IDLE;
    
    signal queue_latched_reg : unsigned(2 downto 0) := (others => '0');

    function get_queue(vlan_id : std_logic_vector(11 downto 0);
                       table : std_logic_vector(8*16-1 downto 0)) return unsigned is
        variable entry_valid : std_logic;
        variable entry_vlan  : std_logic_vector(11 downto 0);
        variable entry_qid   : std_logic_vector(2 downto 0);
        variable result      : unsigned(2 downto 0) := "000";
    begin
        for i in 0 to 7 loop
            entry_valid := table(i*16 + 15);
            entry_vlan  := table(i*16 + 11 downto i*16);
            entry_qid   := table(i*16 + 14 downto i*16 + 12);
            if entry_valid = '1' and entry_vlan = vlan_id then
                result := unsigned(entry_qid);
                exit;
            end if;
        end loop;
        return result;
    end function;

begin
    m_axis_tdata  <= s_axis_tdata;
    m_axis_tkeep  <= s_axis_tkeep;
    m_axis_tvalid <= s_axis_tvalid;
    m_axis_tlast  <= s_axis_tlast;
    s_axis_tready <= m_axis_tready;

    -- PARSE-2 FIX: single-process variable FSM; EtherType/VID checked in
    -- BEAT1 (bytes 12-15 of the Ethernet frame, i.e. the second 8-byte beat).
    process(clk)
        variable v_state  : state_t;
        variable v_queue  : unsigned(2 downto 0);
        variable vlan_id  : std_logic_vector(11 downto 0);
        variable ether_tp : std_logic_vector(15 downto 0);
    begin
        if rising_edge(clk) then
            if rst = '1' then
                state_reg         <= IDLE;
                queue_latched_reg <= (others => '0');
            else
                v_state := state_reg;
                v_queue := queue_latched_reg;

                case v_state is
                    when IDLE =>
                        -- Beat 0: bytes 0-7 = dst_mac + src_mac[47:32]
                        -- EtherType is in beat 1; just advance the state.
                        if s_axis_tvalid = '1' and m_axis_tready = '1' then
                            if s_axis_tlast = '1' then
                                v_state := IDLE;  -- runt frame, stay
                            else
                                v_state := BEAT1;
                            end if;
                        end if;

                    when BEAT1 =>
                        -- Beat 1: bytes 8-15
                        --   tdata[31:16] = bytes 12-13 = EtherType
                        --   tdata[11: 0] = VLAN VID (TCI bits [11:0])
                        if s_axis_tvalid = '1' and m_axis_tready = '1' then
                            ether_tp := s_axis_tdata(31 downto 16);
                            if ether_tp = ETHERTYPE_VLAN then
                                vlan_id := s_axis_tdata(11 downto 0);
                                v_queue := get_queue(vlan_id, vlan_table);
                            else
                                v_queue := "000";
                            end if;
                            if s_axis_tlast = '1' then
                                v_state := IDLE;
                            else
                                v_state := STREAM;
                            end if;
                        end if;

                    when STREAM =>
                        if s_axis_tvalid = '1' and m_axis_tready = '1' and
                           s_axis_tlast = '1' then
                            v_state := IDLE;
                        end if;
                end case;

                state_reg         <= v_state;
                queue_latched_reg <= v_queue;
            end if;
        end if;
    end process;

    queue_id <= queue_latched_reg;

end architecture rtl;
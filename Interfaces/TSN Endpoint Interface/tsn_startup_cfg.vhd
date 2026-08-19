-------------------------------------------------------------------------------
-- register_config_decoder.vhd
-- Signal FPGA  --  Decodes TSN register-config frames (EtherType 0x88B6)
-- received on app_rx_tdata and issues AXI-Lite writes to tsn_endpoint.
--
-- Frame format (IEEE 802.1Q VLAN-tagged, MSB-first 64-bit AXI-S):
--
--   Beat 0  (bytes  0- 7): dst_mac[47:0]  + src_mac[47:32]
--   Beat 1  (bytes  8-15): src_mac[31:0]  + TPID(0x8100) + TCI
--   Beat 2  (bytes 16-23): EtherType(0x88B6) + subarray_id +
--                          reg_offset[31:0]    + value[31:24]
--   Beat 3  (bytes 24-31): value[23:0] + write_strobe(0x0F) + padding
--
-- Only frames whose subarray_id matches SUBARRAY_ID generic (or 0xFF = bcast)
-- result in an AXI-Lite write.  All other frames are silently discarded.
--
-- AXI-Lite write is issued in Beat 3 once all fields are captured.
-- The write channel is held until bready (bvalid accepted).
-- Back-to-back frames are supported; a new frame can start while the previous
-- AXI write-response is pending (cfg_pending gate prevents overwrite).
--
-- Bus convention: MSB-first 64-bit AXI-S (byte N at tdata[63-N*8 : 63-N*8-7])
-------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.tsn_generics_pkg.all;

-- RETIRED April 14, 2026 -- Absorbed into tsn_frame_decoder.vhd (unified EtherType dispatcher).
-- This file is kept for reference only. Do not instantiate.
entity register_config_decoder is
    generic (
        SUBARRAY_ID : std_logic_vector(7 downto 0) := x"00"
    );
    port (
        clk         : in  std_logic;
        rst         : in  std_logic;

        -- AXI-S input (from tsn_endpoint app_rx, 64-bit, MSB-first)
        rx_tvalid   : in  std_logic;
        rx_tdata    : in  std_logic_vector(63 downto 0);
        rx_tkeep    : in  std_logic_vector(7 downto 0);
        rx_tlast    : in  std_logic;
        rx_tready   : out std_logic;

        -- AXI-Lite master (to tsn_endpoint cfg_* slave)
        cfg_awvalid : out std_logic;
        cfg_awaddr  : out std_logic_vector(31 downto 0);
        cfg_awready : in  std_logic;
        cfg_wvalid  : out std_logic;
        cfg_wdata   : out std_logic_vector(31 downto 0);
        cfg_wstrb   : out std_logic_vector(3 downto 0);
        cfg_wready  : in  std_logic;
        cfg_bvalid  : in  std_logic;
        cfg_bresp   : in  std_logic_vector(1 downto 0);
        cfg_bready  : out std_logic;

        -- Status
        write_count : out std_logic_vector(15 downto 0);
        bad_count   : out std_logic_vector(15 downto 0)
    );
end entity register_config_decoder;

architecture rtl of register_config_decoder is

    ---------------------------------------------------------------------------
    -- Beat state machine
    ---------------------------------------------------------------------------
    type beat_state_t is (
        S_BEAT0,    -- wait for first beat; check dst / src MACs
        S_BEAT1,    -- TPID + TCI
        S_BEAT2,    -- EtherType + subarray_id + reg_offset + value[31:24]
        S_BEAT3,    -- value[23:0] + strobe; issues AXI write
        S_DRAIN,    -- absorb remaining beats after valid payload (or error)
        S_AXI_WAIT  -- wait for AXI BRESP to come back
    );
    signal beat_state  : beat_state_t := S_BEAT0;

    -- Capture registers
    signal reg_offset_r : std_logic_vector(31 downto 0) := (others => '0');
    signal value_hi_r   : std_logic_vector(7 downto 0)  := (others => '0');
    signal value_r      : std_logic_vector(31 downto 0) := (others => '0');
    signal do_write_r   : std_logic := '0';   -- write is valid
    signal frame_ok_r   : std_logic := '0';   -- frame header checks passed

    -- AXI-Lite driver
    signal axi_awvalid  : std_logic := '0';
    signal axi_awaddr   : std_logic_vector(31 downto 0) := (others => '0');
    signal axi_wvalid   : std_logic := '0';
    signal axi_wdata    : std_logic_vector(31 downto 0) := (others => '0');
    signal axi_wstrb    : std_logic_vector(3 downto 0)  := x"F";
    signal axi_pending  : std_logic := '0';   -- write not yet acknowledged

    -- Status counters
    signal write_cnt    : unsigned(15 downto 0) := (others => '0');
    signal bad_cnt      : unsigned(15 downto 0) := (others => '0');

    ---------------------------------------------------------------------------
    -- MSB-first 64-bit byte extractor helpers
    -- byte N within a 64-bit beat => bits [63-N*8 : 63-N*8-7]
    ---------------------------------------------------------------------------
    -- b8 : single byte (relative to start of beat, 0-indexed)
    function b8(d : std_logic_vector(63 downto 0); n : integer)
        return std_logic_vector is
    begin
        return d(63 - n*8 downto 63 - n*8 - 7);
    end function;

    -- b16 : 16-bit big-endian word starting at beat-relative byte n
    function b16(d : std_logic_vector(63 downto 0); n : integer)
        return std_logic_vector is
    begin
        return d(63 - n*8 downto 63 - n*8 - 15);
    end function;

    -- b32 : 32-bit big-endian word starting at beat-relative byte n
    function b32(d : std_logic_vector(63 downto 0); n : integer)
        return std_logic_vector is
    begin
        return d(63 - n*8 downto 63 - n*8 - 31);
    end function;

begin

    rx_tready   <= '1';    -- always consume (no backpressure)
    cfg_bready  <= '1';    -- always accept write responses

    cfg_awvalid <= axi_awvalid;
    cfg_awaddr  <= axi_awaddr;
    cfg_wvalid  <= axi_wvalid;
    cfg_wdata   <= axi_wdata;
    cfg_wstrb   <= axi_wstrb;

    write_count <= std_logic_vector(write_cnt);
    bad_count   <= std_logic_vector(bad_cnt);

    ---------------------------------------------------------------------------
    -- Frame decoder
    ---------------------------------------------------------------------------
    decode_proc : process(clk)
    begin
        if rising_edge(clk) then

            -- Clear AXI strobes after slave acknowledges
            if cfg_awready = '1' then axi_awvalid <= '0'; end if;
            if cfg_wready  = '1' then axi_wvalid  <= '0'; end if;
            if cfg_bvalid  = '1' then axi_pending  <= '0'; end if;

            if rst = '1' then
                beat_state  <= S_BEAT0;
                axi_awvalid <= '0';
                axi_wvalid  <= '0';
                axi_pending <= '0';
                do_write_r  <= '0';
                frame_ok_r  <= '0';
                write_cnt   <= (others => '0');
                bad_cnt     <= (others => '0');

            else
                case beat_state is

                    -----------------------------------------------------------
                    -- Beat 0: bytes 0-7 -- dst_mac + src_mac[47:32]
                    -- Accept all dst MACs (unicast to this FPGA handled by
                    -- tsn_endpoint's receive filter before we see the frame).
                    -----------------------------------------------------------
                    when S_BEAT0 =>
                        frame_ok_r <= '0';
                        do_write_r <= '0';
                        if rx_tvalid = '1' then
                            if rx_tlast = '1' then
                                beat_state <= S_BEAT0;  -- runt; stay
                            else
                                beat_state <= S_BEAT1;
                            end if;
                        end if;

                    -----------------------------------------------------------
                    -- Beat 1: bytes 8-15 -- src_mac[31:0] + TPID + TCI
                    -- Check TPID=0x8100 (802.1Q)
                    -----------------------------------------------------------
                    when S_BEAT1 =>
                        if rx_tvalid = '1' then
                            -- TPID at beat-relative bytes 4-5 = tdata[31:16]
                            if b16(rx_tdata, 4) = x"8100" then
                                frame_ok_r <= '1';
                            else
                                frame_ok_r <= '0';  -- not VLAN-tagged; discard
                            end if;
                            if rx_tlast = '1' then
                                beat_state <= S_BEAT0;
                            else
                                beat_state <= S_BEAT2;
                            end if;
                        end if;

                    -----------------------------------------------------------
                    -- Beat 2: bytes 16-23
                    --   [63:48] EtherType (check = 0x88B6)
                    --   [47:40] subarray_id
                    --   [39:8]  reg_offset[31:0]
                    --   [7:0]   value[31:24]      <- latched for beat3
                    -----------------------------------------------------------
                    when S_BEAT2 =>
                        if rx_tvalid = '1' then
                            if b16(rx_tdata, 0) = ETHERTYPE_GEO_CFG and
                               frame_ok_r = '1' and
                               (b8(rx_tdata, 2) = SUBARRAY_ID or
                                b8(rx_tdata, 2) = x"FF") then
                                -- Frame is for us
                                reg_offset_r <= b32(rx_tdata, 3);  -- bytes 19-22
                                value_hi_r   <= b8(rx_tdata, 7);   -- byte 23
                                frame_ok_r   <= '1';
                            else
                                frame_ok_r <= '0';
                            end if;
                            if rx_tlast = '1' then
                                if frame_ok_r = '0' then
                                    bad_cnt <= bad_cnt + 1;
                                end if;
                                beat_state <= S_BEAT0;
                            else
                                beat_state <= S_BEAT3;
                            end if;
                        end if;

                    -----------------------------------------------------------
                    -- Beat 3: bytes 24-31
                    --   [63:40] value[23:0]       completes value
                    --   [39:32] write_strobe        (ignored; use x"F")
                    --   [31:0]  padding
                    -----------------------------------------------------------
                    when S_BEAT3 =>
                        if rx_tvalid = '1' then
                            if frame_ok_r = '1' then
                                -- Assemble full 32-bit value
                                value_r <= value_hi_r &
                                           b8(rx_tdata, 0) &
                                           b8(rx_tdata, 1) &
                                           b8(rx_tdata, 2);
                                do_write_r <= '1';
                                write_cnt  <= write_cnt + 1;
                            else
                                bad_cnt <= bad_cnt + 1;
                            end if;

                            -- Issue AXI write if previous write is done
                            if frame_ok_r = '1' and axi_pending = '0' then
                                axi_awvalid <= '1';
                                axi_awaddr  <= reg_offset_r;
                                axi_wvalid  <= '1';
                                -- value completes next cycle; latch here
                                axi_wdata   <= value_hi_r &
                                               b8(rx_tdata, 0) &
                                               b8(rx_tdata, 1) &
                                               b8(rx_tdata, 2);
                                axi_wstrb   <= x"F";
                                axi_pending <= '1';
                            end if;

                            if rx_tlast = '1' then
                                beat_state <= S_BEAT0;
                            else
                                beat_state <= S_DRAIN;
                            end if;
                        end if;

                    -----------------------------------------------------------
                    -- Drain remaining beats (padding) until tlast
                    -----------------------------------------------------------
                    when S_DRAIN =>
                        if rx_tvalid = '1' and rx_tlast = '1' then
                            beat_state <= S_BEAT0;
                        end if;

                    -----------------------------------------------------------
                    -- Legacy catch-all; should not be reached
                    -----------------------------------------------------------
                    when S_AXI_WAIT =>
                        if axi_pending = '0' then
                            beat_state <= S_BEAT0;
                        end if;

                    when others =>
                        beat_state <= S_BEAT0;

                end case;
            end if;
        end if;
    end process;

end architecture rtl;
                                                                                                                                                                                                                                                                                                                                                                                                                                               

                    -- 0x0100: QBV base time [31:0]
                    when S_W_BASE_TIME_LO =>
                        axi_awvalid <= '1'; axi_awaddr <= x"00000100";
                        axi_wvalid  <= '1';
                        axi_wdata   <= ptp_latch(31 downto 0);
                        axi_wstrb   <= x"F";
                        next_state  <= S_W_BASE_TIME_HI; state <= S_WAIT_BRESP;

                    -- 0x0104: QBV base time [63:32]
                    when S_W_BASE_TIME_HI =>
                        axi_awvalid <= '1'; axi_awaddr <= x"00000104";
                        axi_wvalid  <= '1';
                        axi_wdata   <= ptp_latch(63 downto 32);
                        axi_wstrb   <= x"F";
                        next_state  <= S_DONE; state <= S_WAIT_BRESP;

                    when S_DONE =>
                        class_prev <= bmca_class_i;
                        state      <= S_MONITOR_BMCA;

                    -- Idle: detect BMCA class changes and re-write 0x0028
                    when S_MONITOR_BMCA =>
                        if bmca_class_i /= class_prev then
                            state <= S_REWRITE_CLASS;
                        end if;

                    when S_REWRITE_CLASS =>
                        class_prev  <= bmca_class_i;
                        axi_awvalid <= '1'; axi_awaddr <= x"00000028";
                        axi_wvalid  <= '1';
                        axi_wdata   <= x"000000" & bmca_class_i;
                        axi_wstrb   <= x"1";
                        next_state  <= S_MONITOR_BMCA;
                        state       <= S_WAIT_BRESP;

                    -- Shared BRESP gate
                    when S_WAIT_BRESP =>
                        if cfg_bvalid = '1' then
                            state <= next_state;
                        end if;

                    when others =>
                        state <= S_RESET;

                end case;
            end if;
        end if;
    end process;

end architecture rtl;

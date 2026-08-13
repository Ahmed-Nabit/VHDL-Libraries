-- ============================================================================
-- CAN Engine - MAC/PLS protocol engine
-- ============================================================================
-- This module implements the bit-sampled CAN frame engine:
--   * sample-driven frame FSM
--   * correct standard and extended frame parsing/generation
--   * correct remote frame handling
--   * bit stuffing/de-stuffing
--   * CRC feed and CRC comparison
--   * transmitter monitoring, arbitration loss, ACK error
--   * error frames, error counters, error passive/bus off behavior
--   * overload frame request handling
--   * listening mode with internal loopback and recessive TXCAN pin
--
-- This module is written to connect to the fixed mob_manager interface.
--
-- Design constraints respected:
--   * No Natural type
--   * No Real type
--   * No concurrent signal assignment
--   * State is kept in process variables
-- ============================================================================

library ieee;
use ieee.std_logic_1164.all;
use work.can_pkg.all;

entity can_engine is
    port (
        clk       : in  std_logic;
        reset_n   : in  std_logic;
        sw_reset  : in  std_logic;

        rx        : in  std_logic;
        tx        : out std_logic;

        --------------------------------------------------------------------
        -- General control/status
        --------------------------------------------------------------------
        can_gcon_i : in  std_logic_vector(7 downto 0);
        can_gsta_o : out std_logic_vector(7 downto 0);

        -- General interrupt flags.
        -- Bit 5 (OVRTIM) is kept zero here and may be ORed by top level.
        can_git_clear_i : in  std_logic_vector(7 downto 0);
        can_git_o       : out std_logic_vector(7 downto 0);

        --------------------------------------------------------------------
        -- Bit timing
        --------------------------------------------------------------------
        can_bt1_i : in std_logic_vector(7 downto 0);
        can_bt2_i : in std_logic_vector(7 downto 0);
        can_bt3_i : in std_logic_vector(7 downto 0);

        --------------------------------------------------------------------
        -- Error counters
        --------------------------------------------------------------------
        tec_o : out std_logic_vector(7 downto 0);
        rec_o : out std_logic_vector(7 downto 0);

        --------------------------------------------------------------------
        -- TTC pulses
        --------------------------------------------------------------------
        sof_pulse_o : out std_logic;
        eof_pulse_o : out std_logic;

        --------------------------------------------------------------------
        -- MOb manager TX interface
        --------------------------------------------------------------------
        mob_tx_ready_o  : out std_logic;

        mob_tx_start_i  : in  std_logic;
        mob_tx_id_29_i  : in  std_logic_vector(28 downto 0);
        mob_tx_ide_i    : in  std_logic;
        mob_tx_rtr_i    : in  std_logic;
        mob_tx_dlc_i    : in  integer range 0 to 8;
        mob_tx_data_i   : in  std_logic_vector(63 downto 0);

        mob_tx_done_o         : out std_logic;
        mob_tx_error_strobe_o : out std_logic;
        mob_tx_error_bits_o   : out std_logic_vector(4 downto 0);

        --------------------------------------------------------------------
        -- MOb manager RX interface
        --------------------------------------------------------------------
        mob_rx_filter_strobe_o : out std_logic;
        mob_rx_id_11_o         : out std_logic_vector(10 downto 0);
        mob_rx_id_29_o         : out std_logic_vector(28 downto 0);
        mob_rx_ide_o           : out std_logic;
        mob_rx_rtr_o           : out std_logic;

        mob_rx_match_active_i  : in  std_logic;

        mob_rx_dlc_strobe_o    : out std_logic;
        mob_rx_dlc_o           : out integer range 0 to 15;

        mob_rx_complete_strobe_o : out std_logic;
        mob_rx_success_o         : out std_logic;
        mob_rx_data_o            : out std_logic_vector(63 downto 0);
        mob_rx_error_bits_o      : out std_logic_vector(4 downto 0);

        --------------------------------------------------------------------
        -- Frame buffer BXOK interface for CANGIT.BXOK
        --------------------------------------------------------------------
        mob_bxok_flag_i           : in  std_logic;
        mob_bxok_clear_req_o      : out std_logic;
        mob_bxok_clear_allowed_i  : in  std_logic;

        --------------------------------------------------------------------
        -- Raw MOb interrupt status for CANIT polling image
        --------------------------------------------------------------------
        mob_intr_raw_i : in std_logic_vector(MOB_COUNT - 1 downto 0);

        --------------------------------------------------------------------
        -- Manager side commands
        --------------------------------------------------------------------
        mob_abort_req_o     : out std_logic;
        mob_standby_strobe_o : out std_logic
    );
end entity can_engine;

architecture rtl of can_engine is

    ------------------------------------------------------------------------
    -- Engine state
    ------------------------------------------------------------------------
    type engine_state_t is (
        ST_IDLE,
        ST_SOF,
        ST_ARBITRATION,
        ST_CONTROL,
        ST_DATA,
        ST_CRC_SEQ,
        ST_CRC_DELIM,
        ST_ACK_SLOT,
        ST_ACK_DELIM,
        ST_EOF,
        ST_INTERMISSION,
        ST_ERROR_FRAME,
        ST_OVERLOAD,
        ST_BUS_OFF
    );

    ------------------------------------------------------------------------
    -- Local conversion helpers, avoiding numeric_std and Natural.
    ------------------------------------------------------------------------
    function int_to_slv4(
        v : integer
    ) return std_logic_vector(3 downto 0) is
        variable r   : std_logic_vector(3 downto 0);
        variable tmp : integer range 0 to 15;
    begin
        if v < 0 then
            tmp := 0;
        elsif v > 15 then
            tmp := 15;
        else
            tmp := v;
        end if;

        for i in 0 to 3 loop
            if (tmp mod 2) = 1 then
                r(i) := '1';
            else
                r(i) := '0';
            end if;
            tmp := tmp / 2;
        end loop;

        return r;
    end function;

    function int_to_slv8(
        v : integer
    ) return std_logic_vector(7 downto 0) is
        variable r   : std_logic_vector(7 downto 0);
        variable tmp : integer range 0 to 255;
    begin
        if v < 0 then
            tmp := 0;
        elsif v > 255 then
            tmp := 255;
        else
            tmp := v;
        end if;

        for i in 0 to 7 loop
            if (tmp mod 2) = 1 then
                r(i) := '1';
            else
                r(i) := '0';
            end if;
            tmp := tmp / 2;
        end loop;

        return r;
    end function;

    ------------------------------------------------------------------------
    -- Frame construction helper.
    -- update_crc is true for SOF through data/control fields.
    -- The CRC field itself is inserted without updating the CRC generator.
    ------------------------------------------------------------------------
    procedure put_bit(
        frame      : inout std_logic_vector(MAX_FRAME_BITS - 1 downto 0);
        pos        : inout integer;
        crc        : inout std_logic_vector(14 downto 0);
        b          : in    std_logic;
        update_crc : in    boolean
    ) is
    begin
        if pos >= 0 and pos < MAX_FRAME_BITS then
            frame(pos) := b;
            pos := pos + 1;
        end if;

        if update_crc then
            crc := crc_bit(crc, b);
        end if;
    end procedure;

    ------------------------------------------------------------------------
    -- Internal interconnect signals
    ------------------------------------------------------------------------
    signal internal_reset : std_logic;

    signal bt_enable         : std_logic;
    signal bt_rx             : std_logic;
    signal bt_hard_sync_req  : std_logic;
    signal bt_sample_pt      : std_logic;
    signal bt_bit_start      : std_logic;
    signal bt_sample_value   : std_logic;

    signal crc_enable      : std_logic;
    signal crc_data_valid  : std_logic;
    signal crc_data_in     : std_logic;
    signal crc_out         : std_logic_vector(14 downto 0);

begin

    ------------------------------------------------------------------------
    -- Internal reset generation.
    ------------------------------------------------------------------------
    reset_proc : process (reset_n, sw_reset)
    begin
        if reset_n = '0' or sw_reset = '1' then
            internal_reset <= '1';
        else
            internal_reset <= '0';
        end if;
    end process reset_proc;

    ------------------------------------------------------------------------
    -- Bit timing instance.
    ------------------------------------------------------------------------
    bit_timing_inst : entity work.bit_timing
        port map (
            clk           => clk,
            reset_n       => internal_reset,
            enable        => bt_enable,
            rx            => bt_rx,
            hard_sync_req => bt_hard_sync_req,
            can_bt1       => can_bt1_i,
            can_bt2       => can_bt2_i,
            can_bt3       => can_bt3_i,
            tq_clk        => open,
            sample_pt     => bt_sample_pt,
            bit_start     => bt_bit_start,
            hard_sync     => open,
            resync        => open,
            sample_value  => bt_sample_value
        );

    ------------------------------------------------------------------------
    -- CRC instance.
    ------------------------------------------------------------------------
    crc_inst : entity work.crc_calc
        port map (
            clk        => clk,
            reset_n    => internal_reset,
            enable     => crc_enable,
            data_valid => crc_data_valid,
            data_in    => crc_data_in,
            crc_out    => crc_out,
            crc_valid  => open
        );

    ------------------------------------------------------------------------
    -- Main engine process.
    ------------------------------------------------------------------------
    engine_proc : process (clk, internal_reset)

        --------------------------------------------------------------------
        -- Frame state
        --------------------------------------------------------------------
        variable state_v          : engine_state_t;
        variable frame_active_v   : boolean;
        variable tx_active_v      : boolean;
        variable last_tx_v        : boolean;
        variable rx_after_arb_lost_v : boolean;

        --------------------------------------------------------------------
        -- General mode/status variables
        --------------------------------------------------------------------
        variable listen_v    : boolean;
        variable ttc_v       : boolean;
        variable synttc_v    : boolean;
        variable ovrq_v      : boolean;
        variable ovrq_last_v : std_logic;
        variable ovrq_pending_v : boolean;
        variable ovrq_send_v    : boolean;

        variable enfg_v           : std_logic;
        variable enable_cnt_v     : integer range 0 to 15;
        variable hard_start_v     : boolean;
        variable last_ena_v       : std_logic;

        variable ovrg_v      : std_logic;
        variable txbsy_v     : std_logic;
        variable rxbsy_v     : std_logic;

        --------------------------------------------------------------------
        -- Error confinement variables
        --------------------------------------------------------------------
        variable tec_v  : integer range 0 to 512;
        variable rec_v  : integer range 0 to 512;
        variable errp_v : std_logic;
        variable boff_v : std_logic;

        variable bus_off_bit_v : integer range 0 to 15;
        variable bus_off_seq_v : integer range 0 to 130;

        --------------------------------------------------------------------
        -- General interrupt register
        --------------------------------------------------------------------
        variable git_v        : std_logic_vector(7 downto 0);
        variable last_bxok_v  : std_logic;
        variable canit_v      : std_logic;
        variable git_out_v    : std_logic_vector(7 downto 0);
        variable gsta_v       : std_logic_vector(7 downto 0);

        --------------------------------------------------------------------
        -- RX synchronizer / edge detection
        --------------------------------------------------------------------
        variable rx_meta_v   : std_logic;
        variable rx_sync_v   : std_logic;
        variable last_rx_v   : std_logic;
        variable rx_edge_v   : boolean;
        variable rx_source_v : std_logic;

        --------------------------------------------------------------------
        -- Idle bit counter
        --------------------------------------------------------------------
        variable idle_bits_v : integer range 0 to 31;

        --------------------------------------------------------------------
        -- TX frame variables
        --------------------------------------------------------------------
        variable tx_frame_v     : std_logic_vector(MAX_FRAME_BITS - 1 downto 0);
        variable tx_len_v       : integer range 0 to MAX_FRAME_BITS;
        variable tx_ptr_v       : integer range 0 to MAX_FRAME_BITS;
        variable tx_stuff_cnt_v : integer range 0 to 8;
        variable tx_last_v      : std_logic;
        variable tx_bit_v       : std_logic;
        variable stuff_end_v    : integer range 0 to MAX_FRAME_BITS;
        variable tx_dlc_v       : integer range 0 to 8;

        --------------------------------------------------------------------
        -- RX frame variables
        --------------------------------------------------------------------
        variable rx_buffer_v          : std_logic_vector(MAX_FRAME_BITS - 1 downto 0);
        variable rx_ptr_v             : integer range 0 to MAX_FRAME_BITS;
        variable rx_destuffed_count_v : integer range 0 to MAX_FRAME_BITS;

        variable rx_stuff_cnt_v : integer range 0 to 8;
        variable rx_last_bit_v  : std_logic;
        variable store_bit_v    : boolean;

        variable ext_frame_v     : boolean;
        variable remote_frame_v  : boolean;
        variable rx_dlc_raw_v    : integer range 0 to 15;
        variable rx_dlc_eff_v    : integer range 0 to 8;
        variable data_start_v    : integer range 0 to MAX_FRAME_BITS;
        variable data_end_v      : integer range 0 to MAX_FRAME_BITS;

        variable rx_crc_rx_v    : std_logic_vector(14 downto 0);
        variable crc_rx_count_v : integer range 0 to 15;
        variable crc_ok_v       : boolean;

        variable rx_data_v      : std_logic_vector(63 downto 0);
        variable rx_completed_v : boolean;

        variable frame_error_v   : boolean;
        variable error_detected_v : boolean;
        variable err_bits_v      : std_logic_vector(4 downto 0);

        --------------------------------------------------------------------
        -- Frame field work variables
        --------------------------------------------------------------------
        variable count_v      : integer range 0 to MAX_FRAME_BITS;
        variable dlc_start_v  : integer range 0 to MAX_FRAME_BITS;
        variable id11_v       : std_logic_vector(10 downto 0);
        variable id29_v       : std_logic_vector(28 downto 0);
        variable dlc_bits_v   : std_logic_vector(3 downto 0);
        variable sample_v     : std_logic;
        variable ack_allowed_v : boolean;

        --------------------------------------------------------------------
        -- EOF / intermission / error frame / overload counters
        --------------------------------------------------------------------
        variable eof_cnt_v      : integer range 0 to 8;
        variable interm_cnt_v   : integer range 0 to 4;
        variable err_flag_cnt_v : integer range 0 to 8;
        variable err_delim_cnt_v : integer range 0 to 8;
        variable overload_cnt_v : integer range 0 to 16;

        --------------------------------------------------------------------
        -- Internal TX driver
        --------------------------------------------------------------------
        variable tx_internal_v : std_logic;

    begin
        if internal_reset = '1' then
            state_v        := ST_IDLE;
            frame_active_v := false;
            tx_active_v    := false;
            last_tx_v      := false;
            rx_after_arb_lost_v := false;

            listen_v    := false;
            ttc_v       := false;
            synttc_v    := false;
            ovrq_v      := false;
            ovrq_last_v := '0';
            ovrq_pending_v := false;
            ovrq_send_v    := false;

            enfg_v       := '0';
            enable_cnt_v := 0;
            hard_start_v := false;
            last_ena_v   := '0';

            ovrg_v  := '0';
            txbsy_v := '0';
            rxbsy_v := '0';

            tec_v  := 0;
            rec_v  := 0;
            errp_v := '0';
            boff_v := '0';

            bus_off_bit_v := 0;
            bus_off_seq_v := 0;

            git_v       := (others => '0');
            last_bxok_v := '0';

            rx_meta_v := '1';
            rx_sync_v := '1';
            last_rx_v := '1';
            rx_edge_v := false;

            idle_bits_v := 0;

            tx_frame_v     := (others => '0');
            tx_len_v       := 0;
            tx_ptr_v       := 0;
            tx_stuff_cnt_v := 0;
            tx_last_v      := '1';
            tx_bit_v       := '1';
            stuff_end_v    := 0;
            tx_dlc_v       := 0;

            rx_buffer_v          := (others => '0');
            rx_ptr_v             := 0;
            rx_destuffed_count_v := 0;

            rx_stuff_cnt_v := 0;
            rx_last_bit_v  := '1';
            store_bit_v    := false;

            ext_frame_v    := false;
            remote_frame_v := false;
            rx_dlc_raw_v   := 0;
            rx_dlc_eff_v   := 0;
            data_start_v   := 0;
            data_end_v     := 0;

            rx_crc_rx_v    := (others => '0');
            crc_rx_count_v := 0;
            crc_ok_v       := false;

            rx_data_v      := (others => '0');
            rx_completed_v := false;

            frame_error_v   := false;
            error_detected_v := false;
            err_bits_v      := (others => '0');

            count_v     := 0;
            dlc_start_v := 0;
            id11_v      := (others => '0');
            id29_v      := (others => '0');
            dlc_bits_v  := (others => '0');
            sample_v    := '1';
            ack_allowed_v := false;

            eof_cnt_v       := 0;
            interm_cnt_v    := 0;
            err_flag_cnt_v  := 0;
            err_delim_cnt_v := 0;
            overload_cnt_v  := 0;

            tx_internal_v := '1';

            bt_enable        <= '0';
            bt_rx            <= '1';
            bt_hard_sync_req <= '0';

            crc_enable     <= '0';
            crc_data_valid <= '0';
            crc_data_in    <= '1';

            tx <= '1';

            can_gsta_o <= (others => '0');
            can_git_o  <= (others => '0');

            tec_o <= (others => '0');
            rec_o <= (others => '0');

            sof_pulse_o <= '0';
            eof_pulse_o <= '0';

            mob_tx_ready_o        <= '0';
            mob_tx_done_o         <= '0';
            mob_tx_error_strobe_o <= '0';
            mob_tx_error_bits_o   <= (others => '0');

            mob_rx_filter_strobe_o <= '0';
            mob_rx_id_11_o         <= (others => '0');
            mob_rx_id_29_o         <= (others => '0');
            mob_rx_ide_o           <= '0';
            mob_rx_rtr_o           <= '0';

            mob_rx_dlc_strobe_o <= '0';
            mob_rx_dlc_o        <= 0;

            mob_rx_complete_strobe_o <= '0';
            mob_rx_success_o         <= '0';
            mob_rx_data_o            <= (others => '0');
            mob_rx_error_bits_o      <= (others => '0');

            mob_bxok_clear_req_o <= '0';

            mob_abort_req_o      <= '0';
            mob_standby_strobe_o <= '0';

        elsif rising_edge(clk) then
            ------------------------------------------------------------------
            -- Default one-cycle pulses.
            ------------------------------------------------------------------
            bt_hard_sync_req <= '0';
            crc_enable       <= '0';
            crc_data_valid   <= '0';

            sof_pulse_o <= '0';
            eof_pulse_o <= '0';

            mob_tx_done_o         <= '0';
            mob_tx_error_strobe_o <= '0';

            mob_rx_filter_strobe_o <= '0';
            mob_rx_dlc_strobe_o    <= '0';
            mob_rx_complete_strobe_o <= '0';

            mob_bxok_clear_req_o <= '0';
            mob_standby_strobe_o <= '0';

            ------------------------------------------------------------------
            -- Decode general control bits.
            ------------------------------------------------------------------
            listen_v := (can_gcon_i(3) = '1');
            ttc_v    := (can_gcon_i(5) = '1');
            synttc_v := (can_gcon_i(4) = '1');
            ovrq_v   := (can_gcon_i(6) = '1');

            ------------------------------------------------------------------
            -- Overload request edge detection.
            ------------------------------------------------------------------
            if ovrq_v and ovrq_last_v = '0' then
                ovrq_pending_v := true;
            end if;
            ovrq_last_v := can_gcon_i(6);

            ------------------------------------------------------------------
            -- Abort request to MOb manager.
            ------------------------------------------------------------------
            mob_abort_req_o <= can_gcon_i(7);

            ------------------------------------------------------------------
            -- Write-1-clear handling for CANGIT.
            -- Bit 7 is read-only. Bit 5 is not set by this engine.
            ------------------------------------------------------------------
            for i in 0 to 7 loop
                if can_git_clear_i(i) = '1' then
                    if i = 4 then
                        if mob_bxok_clear_allowed_i = '1' then
                            git_v(4) := '0';
                            mob_bxok_clear_req_o <= '1';
                        end if;
                    elsif i /= 7 then
                        git_v(i) := '0';
                    end if;
                end if;
            end loop;

            ------------------------------------------------------------------
            -- BXOK flag capture from MOb manager.
            ------------------------------------------------------------------
            if mob_bxok_flag_i = '1' and last_bxok_v = '0' then
                git_v(4) := '1';
            end if;
            last_bxok_v := mob_bxok_flag_i;

            ------------------------------------------------------------------
            -- RX input selection:
            --   normal mode : external RX pin
            --   listen mode : internal TxCAN looped to internal RxCAN
            ------------------------------------------------------------------
            if listen_v then
                rx_source_v := tx_internal_v;
            else
                rx_source_v := rx;
            end if;

            rx_sync_v := rx_meta_v;
            rx_meta_v := rx_source_v;

            rx_edge_v := (rx_sync_v = '0' and last_rx_v = '1');
            last_rx_v := rx_sync_v;

            ------------------------------------------------------------------
            -- Bit timing enable follows ENA/STB command, not ENFG status.
            ------------------------------------------------------------------
            bt_enable <= can_gcon_i(1);

            ------------------------------------------------------------------
            -- RX/TX pin routing.
            -- In listen mode the external TXCAN pin is forced recessive.
            ------------------------------------------------------------------
            if listen_v then
                bt_rx <= tx_internal_v;
                tx    <= '1';
            else
                bt_rx <= rx;
                tx    <= tx_internal_v;
            end if;

            ------------------------------------------------------------------
            -- Enable/standby handling.
            ------------------------------------------------------------------
            if can_gcon_i(1) = '1' then
                if boff_v = '0' and enfg_v = '0' then
                    ----------------------------------------------------------
                    -- Enter enable mode after 11 recessive sampled bits.
                    -- Start bit timing once if needed.
                    ----------------------------------------------------------
                    if not hard_start_v and rx_sync_v = '1' then
                        bt_hard_sync_req <= '1';
                        hard_start_v := true;
                    end if;

                    if bt_sample_pt = '1' then
                        if bt_sample_value = '1' then
                            if enable_cnt_v < 11 then
                                enable_cnt_v := enable_cnt_v + 1;
                            else
                                enfg_v := '1';
                            end if;
                        else
                            enable_cnt_v := 0;
                        end if;
                    end if;
                end if;
            else
                --------------------------------------------------------------
                -- Standby command.
                --------------------------------------------------------------
                if state_v = ST_IDLE or state_v = ST_INTERMISSION then
                    if enfg_v = '1' then
                        mob_standby_strobe_o <= '1';
                    end if;

                    enfg_v       := '0';
                    enable_cnt_v := 0;
                    hard_start_v := false;

                    frame_active_v := false;
                    tx_active_v    := false;
                    last_tx_v      := false;
                    tx_internal_v  := '1';
                end if;
            end if;

            ------------------------------------------------------------------
            -- Abort request handling in engine.
            ------------------------------------------------------------------
            if can_gcon_i(7) = '1' then
                if frame_active_v or tx_active_v then
                    frame_active_v := false;
                    tx_active_v    := false;
                    last_tx_v      := false;
                    tx_internal_v  := '1';
                    state_v        := ST_IDLE;
                end if;
            end if;

            ------------------------------------------------------------------
            -- MOb TX start.
            ------------------------------------------------------------------
            if mob_tx_start_i = '1' and
               state_v = ST_IDLE and
               not frame_active_v and
               enfg_v = '1' and
               boff_v = '0' then

                ------------------------------------------------------------
                -- Build the complete unstuffed TX frame.
                ------------------------------------------------------------
                tx_dlc_v := mob_tx_dlc_i;
                if tx_dlc_v > 8 then
                    tx_dlc_v := 8;
                end if;

                declare
                    variable pos_v     : integer;
                    variable crc_v     : std_logic_vector(14 downto 0);
                    variable crc_tx_v  : std_logic_vector(14 downto 0);
                begin
                    pos_v := 0;
                    crc_v := (others => '0');

                    ------------------------------------------------------------
                    -- SOF
                    ------------------------------------------------------------
                    put_bit(tx_frame_v, pos_v, crc_v, '0', true);

                    if mob_tx_ide_i = '0' then
                        --------------------------------------------------------
                        -- CAN standard frame:
                        -- SOF, ID10..0, RTR, IDE, r0, DLC
                        --------------------------------------------------------
                        for i in 10 downto 0 loop
                            put_bit(tx_frame_v, pos_v, crc_v, mob_tx_id_29_i(i), true);
                        end loop;

                        put_bit(tx_frame_v, pos_v, crc_v, mob_tx_rtr_i, true);
                        put_bit(tx_frame_v, pos_v, crc_v, '0', true); -- IDE dominant
                        put_bit(tx_frame_v, pos_v, crc_v, '0', true); -- r0

                        dlc_bits_v := int_to_slv4(tx_dlc_v);
                        for i in 3 downto 0 loop
                            put_bit(tx_frame_v, pos_v, crc_v, dlc_bits_v(i), true);
                        end loop;
                    else
                        --------------------------------------------------------
                        -- CAN extended frame:
                        -- SOF, ID28..18, SRR, IDE, ID17..0, RTR, r1, r0, DLC
                        --------------------------------------------------------
                        for i in 28 downto 18 loop
                            put_bit(tx_frame_v, pos_v, crc_v, mob_tx_id_29_i(i), true);
                        end loop;

                        put_bit(tx_frame_v, pos_v, crc_v, '1', true); -- SRR
                        put_bit(tx_frame_v, pos_v, crc_v, '1', true); -- IDE recessive

                        for i in 17 downto 0 loop
                            put_bit(tx_frame_v, pos_v, crc_v, mob_tx_id_29_i(i), true);
                        end loop;

                        put_bit(tx_frame_v, pos_v, crc_v, mob_tx_rtr_i, true);
                        put_bit(tx_frame_v, pos_v, crc_v, '0', true); -- r1
                        put_bit(tx_frame_v, pos_v, crc_v, '0', true); -- r0

                        dlc_bits_v := int_to_slv4(tx_dlc_v);
                        for i in 3 downto 0 loop
                            put_bit(tx_frame_v, pos_v, crc_v, dlc_bits_v(i), true);
                        end loop;
                    end if;

                    ------------------------------------------------------------
                    -- Data field only for data frames.
                    ------------------------------------------------------------
                    if mob_tx_rtr_i = '0' then
                        for byte_i in 0 to tx_dlc_v - 1 loop
                            for bit_i in 7 downto 0 loop
                                put_bit(tx_frame_v,
                                        pos_v,
                                        crc_v,
                                        mob_tx_data_i(byte_i * 8 + bit_i),
                                        true);
                            end loop;
                        end loop;
                    end if;

                    ------------------------------------------------------------
                    -- CRC field.
                    -- The saved CRC is transmitted MSB first.
                    -- The CRC generator must not be updated with CRC bits.
                    ------------------------------------------------------------
                    crc_tx_v := crc_v;

                    for i in 14 downto 0 loop
                        put_bit(tx_frame_v, pos_v, crc_v, crc_tx_v(i), false);
                    end loop;

                    ------------------------------------------------------------
                    -- CRC delimiter, ACK slot, ACK delimiter, EOF.
                    ------------------------------------------------------------
                    put_bit(tx_frame_v, pos_v, crc_v, '1', false); -- CRC delimiter
                    put_bit(tx_frame_v, pos_v, crc_v, '1', false); -- ACK slot
                    put_bit(tx_frame_v, pos_v, crc_v, '1', false); -- ACK delimiter

                    for i in 1 to 7 loop
                        put_bit(tx_frame_v, pos_v, crc_v, '1', false);
                    end loop;

                    tx_len_v := pos_v;
                end;

                stuff_end_v := tx_len_v - 11;

                ------------------------------------------------------------
                -- Start transmission state.
                ------------------------------------------------------------
                state_v        := ST_SOF;
                frame_active_v := true;
                tx_active_v    := true;
                last_tx_v      := true;
                rx_after_arb_lost_v := false;

                tx_ptr_v       := 1;      -- SOF is already being driven
                tx_stuff_cnt_v := 1;
                tx_last_v      := '0';
                tx_bit_v       := '0';
                tx_internal_v  := '0';

                ext_frame_v    := (mob_tx_ide_i = '1');
                remote_frame_v := (mob_tx_rtr_i = '1');

                ------------------------------------------------------------
                -- Reset RX capture path for self-monitoring.
                ------------------------------------------------------------
                rx_buffer_v          := (others => '0');
                rx_ptr_v             := 0;
                rx_destuffed_count_v := 0;
                rx_stuff_cnt_v       := 0;
                rx_last_bit_v        := '1';
                rx_crc_rx_v          := (others => '0');
                crc_rx_count_v       := 0;
                crc_ok_v             := false;
                rx_data_v            := (others => '0');
                rx_completed_v       := false;
                frame_error_v        := false;
                rx_dlc_raw_v         := tx_dlc_v;
                rx_dlc_eff_v         := tx_dlc_v;
                data_start_v         := 0;
                data_end_v           := 0;

                ------------------------------------------------------------
                -- Start CRC and hard synchronize bit timing.
                ------------------------------------------------------------
                crc_enable       <= '1';
                bt_hard_sync_req <= '1';
                sof_pulse_o      <= '1';
            end if;

            ------------------------------------------------------------------
            -- Start reception on falling edge when idle.
            ------------------------------------------------------------------
            if state_v = ST_IDLE and
               not frame_active_v and
               enfg_v = '1' and
               boff_v = '0' and
               rx_edge_v and
               idle_bits_v > 0 then

                state_v        := ST_SOF;
                frame_active_v := true;
                tx_active_v    := false;
                last_tx_v      := false;
                rx_after_arb_lost_v := false;

                tx_internal_v := '1';

                ext_frame_v    := false;
                remote_frame_v := false;
                rx_dlc_raw_v   := 0;
                rx_dlc_eff_v   := 0;
                data_start_v   := 0;
                data_end_v     := 0;

                rx_buffer_v          := (others => '0');
                rx_ptr_v             := 0;
                rx_destuffed_count_v := 0;
                rx_stuff_cnt_v       := 0;
                rx_last_bit_v        := '1';
                rx_crc_rx_v          := (others => '0');
                crc_rx_count_v       := 0;
                crc_ok_v             := false;
                rx_data_v            := (others => '0');
                rx_completed_v       := false;
                frame_error_v        := false;

                crc_enable       <= '1';
                bt_hard_sync_req <= '1';
                sof_pulse_o      <= '1';
            end if;

            ------------------------------------------------------------------
            -- Bit start processing: drive the next transmitted bit.
            ------------------------------------------------------------------
            if bt_bit_start = '1' then
                case state_v is
                    when ST_ERROR_FRAME =>
                        --------------------------------------------------------
                        -- Error frame:
                        --   error active : 6 dominant bits
                        --   error passive: 6 recessive bits
                        -- followed by 8 recessive delimiter bits.
                        --------------------------------------------------------
                        if err_flag_cnt_v < 6 then
                            if errp_v = '0' then
                                tx_internal_v := '0';
                            else
                                tx_internal_v := '1';
                            end if;

                            err_flag_cnt_v := err_flag_cnt_v + 1;
                        else
                            tx_internal_v := '1';

                            if err_delim_cnt_v < 8 then
                                err_delim_cnt_v := err_delim_cnt_v + 1;
                            else
                                state_v        := ST_IDLE;
                                frame_active_v := false;
                                tx_internal_v  := '1';
                                err_flag_cnt_v  := 0;
                                err_delim_cnt_v := 0;
                            end if;
                        end if;

                    when ST_OVERLOAD =>
                        --------------------------------------------------------
                        -- Overload frame:
                        --   6 dominant bits + 8 recessive delimiter bits.
                        --------------------------------------------------------
                        if overload_cnt_v < 6 then
                            tx_internal_v := '0';
                            overload_cnt_v := overload_cnt_v + 1;
                        elsif overload_cnt_v < 14 then
                            tx_internal_v := '1';
                            overload_cnt_v := overload_cnt_v + 1;
                        else
                            state_v        := ST_IDLE;
                            frame_active_v := false;
                            ovrg_v         := '0';
                            tx_internal_v  := '1';
                            overload_cnt_v := 0;
                        end if;

                    when ST_BUS_OFF =>
                        tx_internal_v := '1';

                    when others =>
                        if tx_active_v and
                           (state_v = ST_ARBITRATION or
                            state_v = ST_CONTROL or
                            state_v = ST_DATA or
                            state_v = ST_CRC_SEQ or
                            state_v = ST_CRC_DELIM or
                            state_v = ST_ACK_SLOT or
                            state_v = ST_ACK_DELIM or
                            state_v = ST_EOF) then

                            ----------------------------------------------------
                            -- TX serialization with stuffing.
                            ----------------------------------------------------
                            if tx_ptr_v < tx_len_v then
                                if tx_ptr_v <= stuff_end_v then
                                    if tx_stuff_cnt_v = 5 then
                                        tx_bit_v       := not tx_last_v;
                                        tx_internal_v  := tx_bit_v;
                                        tx_last_v      := tx_bit_v;
                                        tx_stuff_cnt_v := 0;
                                    else
                                        tx_bit_v := tx_frame_v(tx_ptr_v);

                                        if tx_bit_v = tx_last_v then
                                            tx_stuff_cnt_v := tx_stuff_cnt_v + 1;
                                        else
                                            tx_stuff_cnt_v := 1;
                                            tx_last_v      := tx_bit_v;
                                        end if;

                                        tx_internal_v := tx_bit_v;
                                        tx_ptr_v      := tx_ptr_v + 1;
                                    end if;
                                else
                                    tx_bit_v      := tx_frame_v(tx_ptr_v);
                                    tx_internal_v := tx_bit_v;
                                    tx_last_v     := tx_bit_v;
                                    tx_ptr_v      := tx_ptr_v + 1;
                                end if;
                            else
                                tx_internal_v := '1';
                            end if;

                        elsif state_v = ST_ACK_SLOT and not tx_active_v then
                            ----------------------------------------------------
                            -- Receiver ACK.
                            -- A receiver that received a correct frame sends
                            -- dominant in the ACK slot, regardless of the
                            -- acceptance test result.
                            ----------------------------------------------------
                            ack_allowed_v := (not frame_error_v) and
                                             crc_ok_v and
                                             not listen_v;

                            if ack_allowed_v then
                                tx_internal_v := '0';
                            else
                                tx_internal_v := '1';
                            end if;

                        else
                            tx_internal_v := '1';
                        end if;
                end case;
            end if;

            ------------------------------------------------------------------
            -- Sample point processing.
            ------------------------------------------------------------------
            if bt_sample_pt = '1' then
                sample_v        := bt_sample_value;
                error_detected_v := false;
                err_bits_v      := (others => '0');

                case state_v is
                    when ST_IDLE =>
                        --------------------------------------------------------
                        -- Idle bit counting for enable and TX readiness.
                        --------------------------------------------------------
                        if enfg_v = '0' then
                            if sample_v = '1' then
                                if enable_cnt_v < 11 then
                                    enable_cnt_v := enable_cnt_v + 1;
                                else
                                    enfg_v := '1';
                                end if;
                            else
                                enable_cnt_v := 0;
                            end if;
                        else
                            if sample_v = '1' then
                                if idle_bits_v < 31 then
                                    idle_bits_v := idle_bits_v + 1;
                                end if;
                            else
                                idle_bits_v := 0;
                            end if;
                        end if;

                    when ST_SOF =>
                        --------------------------------------------------------
                        -- SOF must be dominant.
                        --------------------------------------------------------
                        if tx_active_v and tx_bit_v /= sample_v then
                            err_bits_v(4) := '1'; -- BERR
                            error_detected_v := true;
                        end if;

                        if not error_detected_v then
                            if sample_v = '0' then
                                rx_buffer_v(0) := '0';
                                rx_ptr_v := 1;
                                rx_destuffed_count_v := 1;

                                rx_stuff_cnt_v := 1;
                                rx_last_bit_v  := '0';

                                crc_data_valid <= '1';
                                crc_data_in    <= '0';

                                state_v := ST_ARBITRATION;
                            else
                                err_bits_v(1) := '1'; -- FERR
                                error_detected_v := true;
                            end if;
                        end if;

                    when ST_ARBITRATION | ST_CONTROL | ST_DATA | ST_CRC_SEQ =>
                        --------------------------------------------------------
                        -- Transmitter monitoring and arbitration loss.
                        --------------------------------------------------------
                        if tx_active_v then
                            if state_v = ST_ARBITRATION and
                               tx_bit_v = '1' and
                               sample_v = '0' then
                                ------------------------------------------------
                                -- Arbitration lost: recessive sent, dominant
                                -- observed. This is not a bit error.
                                ------------------------------------------------
                                tx_active_v    := false;
                                rx_after_arb_lost_v := true;
                                last_tx_v      := false;
                                tx_internal_v  := '1';

                                mob_tx_error_strobe_o <= '1';
                                mob_tx_error_bits_o   <= (others => '0');

                            elsif tx_bit_v /= sample_v then
                                err_bits_v(4) := '1'; -- BERR
                                error_detected_v := true;
                            end if;
                        end if;

                        --------------------------------------------------------
                        -- RX de-stuff and storage.
                        --------------------------------------------------------
                        if not error_detected_v then
                            store_bit_v := true;

                            if rx_stuff_cnt_v = 5 then
                                if sample_v /= rx_last_bit_v then
                                    --------------------------------------------
                                    -- Valid stuff bit: discard it.
                                    --------------------------------------------
                                    rx_last_bit_v  := sample_v;
                                    rx_stuff_cnt_v := 0;
                                    store_bit_v    := false;
                                else
                                    --------------------------------------------
                                    -- Stuff error: sixth identical bit.
                                    --------------------------------------------
                                    err_bits_v(3) := '1'; -- SERR
                                    error_detected_v := true;
                                    store_bit_v := false;
                                end if;
                            else
                                if sample_v = rx_last_bit_v then
                                    rx_stuff_cnt_v := rx_stuff_cnt_v + 1;
                                else
                                    rx_stuff_cnt_v := 1;
                                    rx_last_bit_v  := sample_v;
                                end if;
                            end if;

                            if store_bit_v then
                                if rx_ptr_v < MAX_FRAME_BITS then
                                    rx_buffer_v(rx_ptr_v) := sample_v;
                                    rx_ptr_v := rx_ptr_v + 1;
                                end if;

                                rx_destuffed_count_v := rx_destuffed_count_v + 1;

                                if state_v /= ST_CRC_SEQ then
                                    crc_data_valid <= '1';
                                    crc_data_in    <= sample_v;
                                else
                                    rx_crc_rx_v := rx_crc_rx_v(13 downto 0) & sample_v;
                                    crc_rx_count_v := crc_rx_count_v + 1;
                                end if;

                                count_v := rx_destuffed_count_v;

                                ------------------------------------------------
                                -- Field parsing / state transitions.
                                ------------------------------------------------
                                if state_v = ST_ARBITRATION then
                                    if count_v = 14 then
                                        ext_frame_v := (rx_buffer_v(13) = '1');

                                        if not ext_frame_v then
                                            ------------------------------------
                                            -- Standard frame:
                                            -- ID and RTR/IDE are now available.
                                            ------------------------------------
                                            if not tx_active_v or rx_after_arb_lost_v then
                                                for k in 0 to 10 loop
                                                    id11_v(10 - k) := rx_buffer_v(1 + k);
                                                end loop;

                                                mob_rx_filter_strobe_o <= '1';
                                                mob_rx_id_11_o         <= id11_v;

                                                id29_v := (others => '0');
                                                id29_v(10 downto 0) := id11_v;
                                                mob_rx_id_29_o <= id29_v;

                                                mob_rx_ide_o <= '0';
                                                mob_rx_rtr_o <= rx_buffer_v(12);

                                                remote_frame_v := (rx_buffer_v(12) = '1');
                                            end if;

                                            state_v := ST_CONTROL;
                                        end if;

                                    elsif ext_frame_v and count_v = 33 then
                                        ----------------------------------------
                                        -- Extended frame:
                                        -- full ID and RTR are now available.
                                        ----------------------------------------
                                        if not tx_active_v or rx_after_arb_lost_v then
                                            for k in 0 to 10 loop
                                                id29_v(28 - k) := rx_buffer_v(1 + k);
                                            end loop;

                                            for k in 0 to 17 loop
                                                id29_v(17 - k) := rx_buffer_v(14 + k);
                                            end loop;

                                            mob_rx_filter_strobe_o <= '1';
                                            mob_rx_id_11_o         <= (others => '0');
                                            mob_rx_id_29_o         <= id29_v;
                                            mob_rx_ide_o           <= '1';
                                            mob_rx_rtr_o           <= rx_buffer_v(32);

                                            remote_frame_v := (rx_buffer_v(32) = '1');
                                        end if;

                                        state_v := ST_CONTROL;
                                    end if;

                                elsif state_v = ST_CONTROL then
                                    if (not ext_frame_v and count_v = 19) or
                                       (ext_frame_v and count_v = 39) then

                                        ----------------------------------------
                                        -- DLC extraction.
                                        ----------------------------------------
                                        if ext_frame_v then
                                            dlc_start_v := 35;
                                            data_start_v := 39;
                                        else
                                            dlc_start_v := 15;
                                            data_start_v := 19;
                                        end if;

                                        if not tx_active_v or rx_after_arb_lost_v then
                                            rx_dlc_raw_v := 0;

                                            for k in 0 to 3 loop
                                                rx_dlc_raw_v := rx_dlc_raw_v * 2;
                                                if rx_buffer_v(dlc_start_v + k) = '1' then
                                                    rx_dlc_raw_v := rx_dlc_raw_v + 1;
                                                end if;
                                            end loop;

                                            mob_rx_dlc_o        <= rx_dlc_raw_v;
                                            mob_rx_dlc_strobe_o <= '1';
                                        else
                                            rx_dlc_raw_v := tx_dlc_v;
                                        end if;

                                        if rx_dlc_raw_v > 8 then
                                            rx_dlc_eff_v := 8;
                                        else
                                            rx_dlc_eff_v := rx_dlc_raw_v;
                                        end if;

                                        if remote_frame_v or rx_dlc_eff_v = 0 then
                                            state_v := ST_CRC_SEQ;
                                            crc_rx_count_v := 0;
                                        else
                                            state_v := ST_DATA;
                                            data_end_v := data_start_v + rx_dlc_eff_v * 8;
                                        end if;
                                    end if;

                                elsif state_v = ST_DATA then
                                    if count_v >= data_end_v then
                                        state_v := ST_CRC_SEQ;
                                        crc_rx_count_v := 0;
                                    end if;

                                elsif state_v = ST_CRC_SEQ then
                                    if crc_rx_count_v = 15 then
                                        ----------------------------------------
                                        -- CRC comparison.
                                        -- CERR is receive-only.
                                        ----------------------------------------
                                        if crc_out = rx_crc_rx_v then
                                            crc_ok_v := true;
                                        else
                                            crc_ok_v := false;

                                            if not tx_active_v or rx_after_arb_lost_v then
                                                err_bits_v(2) := '1'; -- CERR
                                                error_detected_v := true;
                                            end if;
                                        end if;

                                        if not error_detected_v then
                                            state_v := ST_CRC_DELIM;
                                        end if;
                                    end if;
                                end if;
                            end if;
                        end if;

                    when ST_CRC_DELIM =>
                        --------------------------------------------------------
                        -- CRC delimiter must be recessive.
                        --------------------------------------------------------
                        if tx_active_v and tx_bit_v /= sample_v then
                            err_bits_v(4) := '1'; -- BERR
                            error_detected_v := true;
                        end if;

                        if not error_detected_v then
                            if sample_v = '0' then
                                err_bits_v(1) := '1'; -- FERR
                                error_detected_v := true;
                            else
                                state_v := ST_ACK_SLOT;
                            end if;
                        end if;

                    when ST_ACK_SLOT =>
                        --------------------------------------------------------
                        -- Transmitter ACK error:
                        -- no dominant received in ACK slot.
                        --------------------------------------------------------
                        if tx_active_v then
                            if sample_v = '1' then
                                err_bits_v(0) := '1'; -- AERR
                                error_detected_v := true;
                            end if;
                        end if;

                        if not error_detected_v then
                            state_v := ST_ACK_DELIM;
                        end if;

                    when ST_ACK_DELIM =>
                        --------------------------------------------------------
                        -- ACK delimiter must be recessive.
                        --------------------------------------------------------
                        if tx_active_v and tx_bit_v /= sample_v then
                            err_bits_v(4) := '1'; -- BERR
                            error_detected_v := true;
                        end if;

                        if not error_detected_v then
                            if sample_v = '0' then
                                err_bits_v(1) := '1'; -- FERR
                                error_detected_v := true;
                            else
                                state_v   := ST_EOF;
                                eof_cnt_v := 0;
                            end if;
                        end if;

                    when ST_EOF =>
                        --------------------------------------------------------
                        -- EOF is 7 recessive bits.
                        -- RXOK is captured at the end of the 6th EOF bit.
                        --------------------------------------------------------
                        if tx_active_v and tx_bit_v /= sample_v then
                            err_bits_v(4) := '1'; -- BERR
                            error_detected_v := true;
                        end if;

                        if not error_detected_v then
                            if sample_v = '0' then
                                err_bits_v(1) := '1'; -- FERR
                                error_detected_v := true;
                            else
                                eof_cnt_v := eof_cnt_v + 1;

                                --------------------------------------------
                                -- RX completion.
                                --------------------------------------------
                                if eof_cnt_v = 6 and
                                   not tx_active_v and
                                   not rx_completed_v and
                                   not frame_error_v and
                                   crc_ok_v then

                                    rx_completed_v := true;

                                    ----------------------------------------
                                    -- Extract data bytes.
                                    ----------------------------------------
                                    rx_data_v := (others => '0');

                                    if not remote_frame_v then
                                        for byte_i in 0 to rx_dlc_eff_v - 1 loop
                                            for bit_i in 7 downto 0 loop
                                                rx_data_v(byte_i * 8 + bit_i) :=
                                                    rx_buffer_v(data_start_v +
                                                                byte_i * 8 +
                                                                (7 - bit_i));
                                            end loop;
                                        end loop;
                                    end if;

                                    mob_rx_complete_strobe_o <= '1';
                                    mob_rx_success_o         <= '1';
                                    mob_rx_data_o            <= rx_data_v;
                                    mob_rx_error_bits_o      <= (others => '0');

                                    ----------------------------------------
                                    -- Successful reception decrements REC.
                                    ----------------------------------------
                                    if not listen_v then
                                        if rec_v > 0 then
                                            rec_v := rec_v - 1;
                                        end if;
                                    end if;
                                end if;

                                --------------------------------------------
                                -- End of EOF field.
                                --------------------------------------------
                                if eof_cnt_v >= 7 then
                                    eof_pulse_o <= '1';

                                    if tx_active_v and not frame_error_v then
                                        mob_tx_done_o <= '1';
                                        tx_active_v   := false;

                                        ------------------------------------
                                        -- Successful transmission decrements
                                        -- TEC.
                                        ------------------------------------
                                        if not listen_v then
                                            if tec_v > 0 then
                                                tec_v := tec_v - 1;
                                            end if;
                                        end if;
                                    end if;

                                    ----------------------------------------
                                    -- Overload request after next reception.
                                    ----------------------------------------
                                    if not last_tx_v and ovrq_pending_v and not listen_v then
                                        ovrq_send_v    := true;
                                        ovrq_pending_v := false;
                                    end if;

                                    state_v       := ST_INTERMISSION;
                                    interm_cnt_v  := 0;
                                end if;
                            end if;
                        end if;

                    when ST_INTERMISSION =>
                        --------------------------------------------------------
                        -- Intermission: 3 recessive bits.
                        --------------------------------------------------------
                        if sample_v = '0' and interm_cnt_v >= 3 then
                            ----------------------------------------------------
                            -- Next frame starts immediately after IFS.
                            ----------------------------------------------------
                            state_v        := ST_SOF;
                            frame_active_v := true;
                            tx_active_v    := false;
                            last_tx_v      := false;
                            rx_after_arb_lost_v := false;

                            tx_internal_v := '1';

                            ext_frame_v    := false;
                            remote_frame_v := false;
                            rx_dlc_raw_v   := 0;
                            rx_dlc_eff_v   := 0;
                            data_start_v   := 0;
                            data_end_v     := 0;

                            rx_buffer_v          := (others => '0');
                            rx_ptr_v             := 0;
                            rx_destuffed_count_v := 0;
                            rx_stuff_cnt_v       := 0;
                            rx_last_bit_v        := '1';
                            rx_crc_rx_v          := (others => '0');
                            crc_rx_count_v       := 0;
                            crc_ok_v             := false;
                            rx_data_v            := (others => '0');
                            rx_completed_v       := false;
                            frame_error_v        := false;

                            crc_enable       <= '1';
                            bt_hard_sync_req <= '1';
                            sof_pulse_o      <= '1';

                        elsif sample_v = '1' then
                            if interm_cnt_v < 3 then
                                interm_cnt_v := interm_cnt_v + 1;
                            else
                                state_v        := ST_IDLE;
                                frame_active_v := false;
                                last_tx_v      := false;
                                idle_bits_v    := 0;

                                if ovrq_send_v and not listen_v then
                                    state_v        := ST_OVERLOAD;
                                    overload_cnt_v := 0;
                                    ovrg_v         := '1';
                                    ovrq_send_v    := false;
                                    frame_active_v := true;
                                end if;
                            end if;
                        else
                            interm_cnt_v := 0;
                        end if;

                    when ST_BUS_OFF =>
                        --------------------------------------------------------
                        -- Bus off recovery:
                        -- 128 occurrences of 11 consecutive recessive bits.
                        --------------------------------------------------------
                        if sample_v = '1' then
                            if bus_off_bit_v < 11 then
                                bus_off_bit_v := bus_off_bit_v + 1;
                            else
                                bus_off_bit_v := 0;

                                if bus_off_seq_v < 128 then
                                    bus_off_seq_v := bus_off_seq_v + 1;
                                else
                                    boff_v := '0';
                                    errp_v := '0';
                                    tec_v  := 0;
                                    rec_v  := 0;

                                    bus_off_seq_v := 0;
                                    bus_off_bit_v := 0;

                                    state_v        := ST_IDLE;
                                    frame_active_v := false;
                                    tx_internal_v  := '1';
                                end if;
                            end if;
                        else
                            bus_off_bit_v := 0;
                        end if;

                    when others =>
                        null;
                end case;

                ----------------------------------------------------------------
                -- Error handling and fault confinement.
                ----------------------------------------------------------------
                if error_detected_v and not frame_error_v then
                    frame_error_v := true;

                    if listen_v then
                        --------------------------------------------------------
                        -- Listening mode freezes counters and must not
                        -- influence the bus.
                        --------------------------------------------------------
                        state_v        := ST_IDLE;
                        frame_active_v := false;
                        tx_active_v    := false;
                        last_tx_v      := false;
                        tx_internal_v  := '1';

                    else
                        if tx_active_v then
                            ----------------------------------------------------
                            -- Transmission error: MOb level, TEC increase.
                            ----------------------------------------------------
                            tec_v := tec_v + 8;

                            mob_tx_error_strobe_o <= '1';
                            mob_tx_error_bits_o   <= err_bits_v;

                            tx_active_v := false;
                        else
                            ----------------------------------------------------
                            -- Reception error: REC increase.
                            ----------------------------------------------------
                            if rec_v < 511 then
                                rec_v := rec_v + 1;
                            end if;

                            if mob_rx_match_active_i = '1' then
                                mob_rx_complete_strobe_o <= '1';
                                mob_rx_success_o         <= '0';
                                mob_rx_error_bits_o      <= err_bits_v;
                            else
                                ------------------------------------------------
                                -- General error flags.
                                ------------------------------------------------
                                if err_bits_v(3) = '1' then
                                    git_v(3) := '1'; -- SERG
                                end if;

                                if err_bits_v(2) = '1' then
                                    git_v(2) := '1'; -- CERG
                                end if;

                                if err_bits_v(1) = '1' then
                                    git_v(1) := '1'; -- FERG
                                end if;

                                if err_bits_v(0) = '1' then
                                    git_v(0) := '1'; -- AERG
                                end if;
                            end if;
                        end if;

                        --------------------------------------------------------
                        -- Fault confinement state update.
                        --------------------------------------------------------
                        if tec_v > 255 then
                            boff_v := '1';
                            errp_v := '0';

                            git_v(6) := '1'; -- BOFFIT

                            state_v        := ST_BUS_OFF;
                            frame_active_v := false;
                            tx_active_v    := false;
                            last_tx_v      := false;
                            tx_internal_v  := '1';

                            bus_off_bit_v := 0;
                            bus_off_seq_v := 0;

                            bt_hard_sync_req <= '1';

                        elsif tec_v > 127 or rec_v > 127 then
                            errp_v := '1';

                            state_v         := ST_ERROR_FRAME;
                            frame_active_v  := true;
                            tx_active_v     := false;
                            last_tx_v       := false;
                            err_flag_cnt_v  := 0;
                            err_delim_cnt_v := 0;

                        else
                            errp_v := '0';

                            state_v         := ST_ERROR_FRAME;
                            frame_active_v  := true;
                            tx_active_v     := false;
                            last_tx_v       := false;
                            err_flag_cnt_v  := 0;
                            err_delim_cnt_v := 0;
                        end if;
                    end if;
                end if;

                ----------------------------------------------------------------
                -- Error passive / error active recovery check after successful
                -- counter decrements.
                ----------------------------------------------------------------
                if not listen_v and boff_v = '0' then
                    if tec_v <= 127 and rec_v <= 127 then
                        errp_v := '0';
                    end if;
                end if;
            end if;

            ------------------------------------------------------------------
            -- TX readiness for MOb manager.
            ------------------------------------------------------------------
            if enfg_v = '1' and
               state_v = ST_IDLE and
               not frame_active_v and
               boff_v = '0' and
               can_gcon_i(7) = '0' then

                if errp_v = '1' then
                    if idle_bits_v >= 8 then
                        mob_tx_ready_o <= '1';
                    end if;
                else
                    if idle_bits_v >= 3 then
                        mob_tx_ready_o <= '1';
                    end if;
                end if;
            else
                mob_tx_ready_o <= '0';
            end if;

            ------------------------------------------------------------------
            -- Status outputs.
            ------------------------------------------------------------------
            txbsy_v := '0';
            rxbsy_v := '0';

            if frame_active_v or
               state_v = ST_ERROR_FRAME or
               state_v = ST_OVERLOAD then
                rxbsy_v := '1';
            end if;

            if tx_active_v or
               state_v = ST_ERROR_FRAME or
               state_v = ST_OVERLOAD or
               (state_v = ST_INTERMISSION and last_tx_v) then
                txbsy_v := '1';
            end if;

            gsta_v := (others => '0');
            gsta_v(6) := ovrg_v;
            gsta_v(4) := txbsy_v;
            gsta_v(3) := rxbsy_v;
            gsta_v(2) := enfg_v;
            gsta_v(1) := boff_v;
            gsta_v(0) := errp_v;

            can_gsta_o <= gsta_v;

            ------------------------------------------------------------------
            -- Error counters.
            ------------------------------------------------------------------
            if tec_v > 255 then
                tec_o <= int_to_slv8(255);
            else
                tec_o <= int_to_slv8(tec_v);
            end if;

            if rec_v > 255 then
                rec_o <= int_to_slv8(255);
            else
                rec_o <= int_to_slv8(rec_v);
            end if;

            ------------------------------------------------------------------
            -- CANGIT.CANIT polling image.
            ------------------------------------------------------------------
            canit_v := '0';

            for i in 0 to 6 loop
                if git_v(i) = '1' then
                    canit_v := '1';
                end if;
            end loop;

            for i in 0 to MOB_COUNT - 1 loop
                if mob_intr_raw_i(i) = '1' then
                    canit_v := '1';
                end if;
            end loop;

            git_out_v    := git_v;
            git_out_v(7) := canit_v;
            git_out_v(5) := '0'; -- OVRTIM is combined at top level if needed.

            can_git_o <= git_out_v;
        end if;
    end process engine_proc;

end architecture rtl;
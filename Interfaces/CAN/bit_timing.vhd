-- ============================================================================
-- CAN Bit Timing
-- ============================================================================
-- PDF-compliant behavior implemented:
--   * TQ = (BRP + 1) clkIO
--   * SYNC = 1 TQ
--   * PROP/PHS1/PHS2 segment state machine
--   * Sample point at end of PHASE_SEG1
--   * Optional three-sample majority voting
--   * Hard synchronization request input
--   * Resynchronization on recessive-to-dominant edges
--   * SJW-based PHASE_SEG1 lengthening / PHASE_SEG2 shortening
--
-- Design constraints respected:
--   * No Natural type
--   * No Real type
--   * No concurrent signal assignment
--   * All state kept in process variables
-- ============================================================================

library ieee;
use ieee.std_logic_1164.all;
use work.can_pkg.all;

entity bit_timing is
    port (
        clk           : in  std_logic;
        reset_n       : in  std_logic;
        enable        : in  std_logic;
        rx            : in  std_logic;
        hard_sync_req : in  std_logic;
        can_bt1       : in  std_logic_vector(7 downto 0);
        can_bt2       : in  std_logic_vector(7 downto 0);
        can_bt3       : in  std_logic_vector(7 downto 0);
        tq_clk        : out std_logic;
        sample_pt     : out std_logic;
        bit_start     : out std_logic;
        hard_sync     : out std_logic;
        resync        : out std_logic;
        sample_value  : out std_logic
    );
end entity bit_timing;

architecture rtl of bit_timing is
begin

    timing_proc : process (clk, reset_n)
        -- Decoded timing configuration
        variable bt_now_v : bt_cfg_t;
        variable bt_v     : bt_cfg_t;

        -- RX synchronizer variables
        variable rx_meta_v  : std_logic;
        variable rx_sync_v  : std_logic;
        variable last_rx_v  : std_logic;
        variable edge_now_v : boolean;

        -- Three-sample history
        variable samp0_v : std_logic;
        variable samp1_v : std_logic;
        variable samp2_v : std_logic;

        -- Bit timing state
        variable synced_v       : boolean;
        variable tq_cnt_v       : integer range 0 to 63;
        variable phase_v        : integer range 0 to 3;
        variable seg_cnt_v      : integer range 0 to 31;
        variable phs1_len_v     : integer range 0 to 31;
        variable phs2_len_v     : integer range 0 to 31;
        variable resync_done_v  : boolean;

        -- Sample output
        variable sample_val_v : std_logic;

        -- One-cycle event flags
        variable tq_now_v        : boolean;
        variable sample_now_v    : boolean;
        variable bit_start_now_v : boolean;
        variable hard_sync_now_v : boolean;
        variable resync_now_v    : boolean;
    begin
        if reset_n = '0' then
            -- Safe default timing:
            -- 1 SYNC + 3 PROP + 2 PHS1 + 2 PHS2 = 8 TQ
            bt_v.brp          := 0;
            bt_v.prs          := 3;
            bt_v.phs1         := 2;
            bt_v.phs2         := 2;
            bt_v.sjw          := 1;
            bt_v.bit_len      := 8;
            bt_v.sample_pos   := 6;
            bt_v.three_sample := false;

            rx_meta_v  := '1';
            rx_sync_v  := '1';
            last_rx_v  := '1';

            samp0_v := '1';
            samp1_v := '1';
            samp2_v := '1';

            synced_v      := false;
            tq_cnt_v      := 0;
            phase_v       := 0;
            seg_cnt_v     := 0;
            phs1_len_v    := 2;
            phs2_len_v    := 2;
            resync_done_v := false;

            sample_val_v := '1';

            tq_clk       <= '0';
            sample_pt    <= '0';
            bit_start    <= '0';
            hard_sync    <= '0';
            resync       <= '0';
            sample_value <= '1';

        elsif rising_edge(clk) then
            -- Default one-cycle pulses
            tq_clk    <= '0';
            sample_pt <= '0';
            bit_start <= '0';
            hard_sync <= '0';
            resync    <= '0';

            tq_now_v        := false;
            sample_now_v    := false;
            bit_start_now_v := false;
            hard_sync_now_v := false;
            resync_now_v    := false;

            ----------------------------------------------------------------
            -- Decode current CANBT registers.
            -- The decoded configuration is latched when unsynchronized or
            -- when a hard synchronization occurs, so timing changes do not
            -- disturb an ongoing bit/frame.
            ----------------------------------------------------------------
            bt_now_v := decode_can_bt(can_bt1, can_bt2, can_bt3);

            ----------------------------------------------------------------
            -- RX double-flop synchronizer.
            -- rx_meta_v is the first flop, rx_sync_v is the second flop.
            ----------------------------------------------------------------
            rx_sync_v := rx_meta_v;
            rx_meta_v := rx;

            ----------------------------------------------------------------
            -- Keep a three-clock sample history for SMP=1 majority voting.
            ----------------------------------------------------------------
            samp2_v := samp1_v;
            samp1_v := samp0_v;
            samp0_v := rx_sync_v;

            ----------------------------------------------------------------
            -- Falling-edge detection on synchronized RX.
            -- CAN synchronization edges are recessive-to-dominant, i.e. '1' -> '0'.
            ----------------------------------------------------------------
            edge_now_v := (rx_sync_v = '0' and last_rx_v = '1');
            last_rx_v  := rx_sync_v;

            if enable = '1' then
                -- While waiting for first synchronization, allow timing
                -- configuration to track the CPU-programmed values.
                if not synced_v then
                    bt_v := bt_now_v;
                end if;

                ----------------------------------------------------------------
                -- Hard synchronization:
                --   * CPU/engine request, or
                --   * first falling edge after enable
                ----------------------------------------------------------------
                if hard_sync_req = '1' or (not synced_v and edge_now_v) then
                    bt_v := bt_now_v;

                    synced_v      := true;
                    hard_sync_now_v := true;
                    bit_start_now_v := true;

                    tq_cnt_v      := 0;
                    phase_v       := 0;
                    seg_cnt_v     := 0;
                    phs1_len_v    := bt_v.phs1;
                    phs2_len_v    := bt_v.phs2;
                    resync_done_v := false;

                elsif synced_v then
                    ------------------------------------------------------------
                    -- Resynchronization.
                    --
                    -- One resynchronization is allowed per bit.
                    --
                    -- Edge before sample point:
                    --   lengthen PHASE_SEG1 by SJW.
                    --
                    -- Edge after sample point:
                    --   shorten PHASE_SEG2 by SJW, but keep minimum IPT = 2 TQ.
                    ------------------------------------------------------------
                    if edge_now_v and not resync_done_v then
                        if phase_v = 1 or phase_v = 2 then
                            phs1_len_v := bt_v.phs1 + bt_v.sjw;
                            if phs1_len_v > 31 then
                                phs1_len_v := 31;
                            end if;

                            resync_done_v := true;
                            resync_now_v  := true;

                        elsif phase_v = 3 then
                            phs2_len_v := bt_v.phs2 - bt_v.sjw;
                            if phs2_len_v < 2 then
                                phs2_len_v := 2;
                            end if;

                            resync_done_v := true;
                            resync_now_v  := true;
                        end if;
                    end if;

                    ------------------------------------------------------------
                    -- Time quantum generator.
                    -- TQ period is BRP + 1 system clocks.
                    ------------------------------------------------------------
                    if tq_cnt_v < bt_v.brp then
                        tq_cnt_v := tq_cnt_v + 1;
                    else
                        tq_cnt_v := 0;
                        tq_now_v := true;

                        --------------------------------------------------------
                        -- Segment FSM:
                        --   0 = SYNC_SEG, fixed 1 TQ
                        --   1 = PROP_SEG
                        --   2 = PHASE_SEG1
                        --   3 = PHASE_SEG2
                        --------------------------------------------------------
                        case phase_v is
                            when 0 =>
                                -- End of SYNC_SEG
                                phase_v   := 1;
                                seg_cnt_v := 0;

                            when 1 =>
                                -- PROP_SEG
                                if seg_cnt_v < bt_v.prs - 1 then
                                    seg_cnt_v := seg_cnt_v + 1;
                                else
                                    phase_v   := 2;
                                    seg_cnt_v := 0;
                                end if;

                            when 2 =>
                                -- PHASE_SEG1.
                                -- Sample point is at the end of PHASE_SEG1.
                                if seg_cnt_v < phs1_len_v - 1 then
                                    seg_cnt_v := seg_cnt_v + 1;
                                else
                                    sample_now_v := true;
                                    phase_v      := 3;
                                    seg_cnt_v    := 0;
                                end if;

                            when 3 =>
                                -- PHASE_SEG2.
                                -- End of PHASE_SEG2 is the start of next bit.
                                if seg_cnt_v < phs2_len_v - 1 then
                                    seg_cnt_v := seg_cnt_v + 1;
                                else
                                    bit_start_now_v := true;

                                    phase_v       := 0;
                                    seg_cnt_v     := 0;
                                    phs1_len_v    := bt_v.phs1;
                                    phs2_len_v    := bt_v.phs2;
                                    resync_done_v := false;
                                end if;

                            when others =>
                                phase_v   := 0;
                                seg_cnt_v := 0;
                        end case;
                    end if;
                end if;
            else
                ----------------------------------------------------------------
                -- enable = '0'
                -- Reset bit timing state, but keep RX synchronizer running.
                ----------------------------------------------------------------
                bt_v := bt_now_v;

                synced_v      := false;
                tq_cnt_v      := 0;
                phase_v       := 0;
                seg_cnt_v     := 0;
                phs1_len_v    := bt_v.phs1;
                phs2_len_v    := bt_v.phs2;
                resync_done_v := false;
            end if;

            --------------------------------------------------------------------
            -- Sample value selection.
            --
            -- SMP = 0:
            --   single sample at sample point.
            --
            -- SMP = 1:
            --   majority vote of current sample and previous two clkIO samples.
            --------------------------------------------------------------------
            if sample_now_v then
                if bt_v.three_sample then
                    sample_val_v := majority3(samp0_v, samp1_v, samp2_v);
                else
                    sample_val_v := rx_sync_v;
                end if;
            end if;

            --------------------------------------------------------------------
            -- Outputs
            --------------------------------------------------------------------
            if tq_now_v then
                tq_clk <= '1';
            end if;

            if sample_now_v then
                sample_pt <= '1';
            end if;

            if bit_start_now_v then
                bit_start <= '1';
            end if;

            if hard_sync_now_v then
                hard_sync <= '1';
            end if;

            if resync_now_v then
                resync <= '1';
            end if;

            sample_value <= sample_val_v;
        end if;
    end process timing_proc;

end architecture rtl;
-- ============================================================================
-- CAN MOb Manager
-- ============================================================================
-- This module is the single source of truth for MOb configuration, status,
-- data buffers, CANEN state, acceptance filtering, TX arbitration, RX
-- completion, automatic reply, frame buffer BXOK management, and MOb
-- interrupt generation.
--
-- PDF-compliant behavior implemented:
--   * Lower MOb index has priority for RX matching and TX arbitration.
--   * CONMOB bits are not automatically cleared after communication.
--   * CANEN is set by CONMOB configuration and cleared by TXOK/RXOK,
--     disabled mode, abort, or standby.
--   * RXOK is set only at successful frame completion, not at early match.
--   * Data is stored only for successful data frames.
--   * Timestamp is captured on TXOK/RXOK.
--   * Automatic reply:
--       - remote frame only,
--       - RPLV=1,
--       - no RXOK/interrupt for the incoming remote frame,
--       - RTRTAG and RPLV are reset,
--       - MOb becomes ready to transmit the reply.
--   * Frame buffer receive:
--       - RXOK is set per received frame,
--       - BXOK is set only when all MObs in the frame-buffer set have
--         received their frames,
--       - BXOK clear is allowed only after all CONMOB fields in the set
--         have been re-written.
--   * DLC warning is set when expected DLC differs from received DLC.
--   * DLC values greater than 8 are treated as effective DLC = 8.
--   * MOb interrupts are generated from CANSTMOB bits 6:0. DLCW bit 7 is not
--     an interrupt source.
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

entity mob_manager is
    port (
        clk   : in std_logic;
        reset_n : in std_logic;

        --------------------------------------------------------------------
        -- CPU register access
        --
        -- cpu_wr / cpu_addr / cpu_mob / cpu_data are used for MOb page
        -- registers STMOB, CDMOB, IDT, IDM, STM.
        --
        -- Message buffer writes use cpu_msg_wr / cpu_msg_idx / cpu_msg_data.
        --------------------------------------------------------------------
        cpu_wr      : in std_logic;
        cpu_addr    : in integer range 0 to 15;
        cpu_mob     : in integer range 0 to 15;
        cpu_data    : in std_logic_vector(7 downto 0);

        cpu_msg_wr   : in std_logic;
        cpu_msg_idx  : in integer range 0 to 7;
        cpu_msg_data : in std_logic_vector(7 downto 0);

        --------------------------------------------------------------------
        -- CPU-visible register state
        --------------------------------------------------------------------
        mobs_o       : out mob_array_t;
        mob_data_o   : out data_array_t;

        can_en1_o    : out std_logic_vector(7 downto 0);
        can_en2_o    : out std_logic_vector(7 downto 0);
        mob_intr_o   : out std_logic_vector(MOB_COUNT - 1 downto 0);

        --------------------------------------------------------------------
        -- RX acceptance filter interface
        --
        -- rx_filter_strobe is asserted once when the frame identifier,
        -- IDE, and RTR are available.
        --------------------------------------------------------------------
        rx_filter_strobe : in std_logic;
        rx_id_11         : in std_logic_vector(10 downto 0);
        rx_id_29         : in std_logic_vector(28 downto 0);
        rx_ide           : in std_logic;
        rx_rtr           : in std_logic;

        rx_filter_hit    : out std_logic;
        rx_filter_idx    : out integer range -1 to MOB_COUNT - 1;

        rx_match_active  : out std_logic;
        rx_matched_idx   : out integer range -1 to MOB_COUNT - 1;

        --------------------------------------------------------------------
        -- RX DLC update
        --------------------------------------------------------------------
        rx_dlc_strobe : in std_logic;
        rx_dlc        : in integer range 0 to 15;

        --------------------------------------------------------------------
        -- RX frame completion
        --
        -- rx_success = '1'  : frame completed without error.
        -- rx_success = '0'  : frame aborted with error. rx_error_bits
        --                     are written into the matched MOb if a match
        --                     was active.
        --
        -- rx_error_bits mapping:
        --   bit 4 : BERR
        --   bit 3 : SERR
        --   bit 2 : CERR
        --   bit 1 : FERR
        --   bit 0 : AERR
        --------------------------------------------------------------------
        rx_complete_strobe : in std_logic;
        rx_success         : in std_logic;
        rx_data            : in std_logic_vector(63 downto 0);
        rx_error_bits      : in std_logic_vector(4 downto 0);

        --------------------------------------------------------------------
        -- TX interface
        --------------------------------------------------------------------
        tx_ready       : in  std_logic;
        tx_start       : out std_logic;
        tx_mob_idx     : out integer range -1 to MOB_COUNT - 1;

        tx_id_29       : out std_logic_vector(28 downto 0);
        tx_ide         : out std_logic;
        tx_rtr         : out std_logic;
        tx_dlc         : out integer range 0 to 8;
        tx_data        : out std_logic_vector(63 downto 0);

        tx_done        : in std_logic;

        -- tx_error_bits uses the same mapping as rx_error_bits.
        tx_error_strobe : in std_logic;
        tx_error_bits   : in std_logic_vector(4 downto 0);

        -- Abort request, normally driven by CANGCON.ABRQ.
        tx_abort_req : in  std_logic;
        tx_abort     : out std_logic;

        -- Standby command, normally driven by ENA/STB entering standby.
        standby_strobe : in std_logic;

        --------------------------------------------------------------------
        -- Frame buffer BXOK interface
        --------------------------------------------------------------------
        bxok_flag_o           : out std_logic;
        bxok_clear_req        : in  std_logic;
        bxok_clear_allowed_o  : out std_logic;

        --------------------------------------------------------------------
        -- Timer timestamp
        --------------------------------------------------------------------
        cantim : in std_logic_vector(15 downto 0)
    );
end entity mob_manager;

architecture rtl of mob_manager is

    -- --------------------------------------------------------------------
    -- Local helpers, avoiding numeric_std and Natural.
    -- --------------------------------------------------------------------
    function slv4_to_int(
        v : std_logic_vector(3 downto 0)
    ) return integer is
        variable r : integer range 0 to 15;
    begin
        r := 0;
        for i in 3 downto 0 loop
            r := r * 2;
            if v(i) = '1' then
                r := r + 1;
            end if;
        end loop;
        return r;
    end function;

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

begin

    mob_proc : process (clk, reset_n)
        --------------------------------------------------------------------
        -- MOb register and data state
        --------------------------------------------------------------------
        variable mobs_v      : mob_array_t;
        variable mob_data_v  : data_array_t;
        variable mob_en_v    : std_logic_vector(MOB_COUNT - 1 downto 0);

        --------------------------------------------------------------------
        -- TX state
        --------------------------------------------------------------------
        variable tx_active_v  : boolean;
        variable tx_current_v : integer range -1 to MOB_COUNT - 1;

        variable tx_id_v   : std_logic_vector(28 downto 0);
        variable tx_ide_v  : std_logic;
        variable tx_rtr_v  : std_logic;
        variable tx_dlc_v  : integer range 0 to 8;
        variable tx_data_v : std_logic_vector(63 downto 0);

        --------------------------------------------------------------------
        -- RX state
        --------------------------------------------------------------------
        variable rx_active_v  : boolean;
        variable rx_current_v : integer range -1 to MOB_COUNT - 1;

        variable rx_mode_v      : std_logic_vector(1 downto 0);
        variable rx_rplv_v      : std_logic;
        variable rx_ide_latch_v : std_logic;
        variable rx_rtr_latch_v : std_logic;
        variable rx_dlc_latch_v : integer range 0 to 8;

        --------------------------------------------------------------------
        -- Frame buffer state
        --------------------------------------------------------------------
        variable fbuf_received_v  : std_logic_vector(MOB_COUNT - 1 downto 0);
        variable fbuf_set_v       : std_logic_vector(MOB_COUNT - 1 downto 0);
        variable fbuf_rewritten_v : std_logic_vector(MOB_COUNT - 1 downto 0);
        variable bxok_pending_v   : boolean;

        --------------------------------------------------------------------
        -- Command edge tracking
        --------------------------------------------------------------------
        variable last_abort_v   : std_logic;
        variable last_standby_v : std_logic;

        --------------------------------------------------------------------
        -- Work variables
        --------------------------------------------------------------------
        variable idx_v : integer range -1 to MOB_COUNT - 1;
        variable j_v   : integer range -1 to MOB_COUNT - 1;

        variable mode_v     : std_logic_vector(1 downto 0);
        variable match_v    : boolean;
        variable ide_match_v : boolean;
        variable rtr_match_v : boolean;
        variable id_match_v  : boolean;

        variable tag11_v  : std_logic_vector(10 downto 0);
        variable mask11_v : std_logic_vector(10 downto 0);

        variable tag29_v  : std_logic_vector(28 downto 0);
        variable mask29_v : std_logic_vector(28 downto 0);

        variable dlc_v       : integer range 0 to 15;
        variable exp_dlc_v   : integer range 0 to 15;
        variable rx_dlc_eff_v : integer range 0 to 8;

        variable fbuf_mask_v     : std_logic_vector(MOB_COUNT - 1 downto 0);
        variable any_fbuf_v      : boolean;
        variable all_received_v  : boolean;
        variable bxok_allowed_v  : boolean;

        variable abort_now_v   : boolean;
        variable standby_now_v : boolean;

        variable en1_v : std_logic_vector(7 downto 0);
        variable en2_v : std_logic_vector(7 downto 0);
        variable mob_intr_v : std_logic_vector(MOB_COUNT - 1 downto 0);

    begin
        if reset_n = '0' then
            ----------------------------------------------------------------
            -- Reset MOb registers and buffers.
            -- PDF says MOb registers have no default value after reset,
            -- but for deterministic silicon behavior they are initialized
            -- to disabled/zero here.
            ----------------------------------------------------------------
            for i in 0 to MOB_COUNT - 1 loop
                mobs_v(i).stmob := (others => '0');
                mobs_v(i).cdmob := (others => '0');

                mobs_v(i).idt1 := (others => '0');
                mobs_v(i).idt2 := (others => '0');
                mobs_v(i).idt3 := (others => '0');
                mobs_v(i).idt4 := (others => '0');

                mobs_v(i).idm1 := (others => '0');
                mobs_v(i).idm2 := (others => '0');
                mobs_v(i).idm3 := (others => '0');
                mobs_v(i).idm4 := (others => '0');

                mobs_v(i).stml := (others => '0');
                mobs_v(i).stmh := (others => '0');

                mob_data_v(i) := (others => '0');
            end loop;

            mob_en_v := (others => '0');

            tx_active_v  := false;
            tx_current_v := -1;

            tx_id_v   := (others => '0');
            tx_ide_v  := '0';
            tx_rtr_v  := '0';
            tx_dlc_v  := 0;
            tx_data_v := (others => '0');

            rx_active_v  := false;
            rx_current_v := -1;

            rx_mode_v      := MOB_DISABLED;
            rx_rplv_v      := '0';
            rx_ide_latch_v := '0';
            rx_rtr_latch_v := '0';
            rx_dlc_latch_v := 0;

            fbuf_received_v  := (others => '0');
            fbuf_set_v       := (others => '0');
            fbuf_rewritten_v := (others => '0');
            bxok_pending_v   := false;

            last_abort_v   := '0';
            last_standby_v := '0';

            mobs_o       <= mobs_v;
            mob_data_o   <= mob_data_v;
            can_en1_o    <= (others => '0');
            can_en2_o    <= (others => '0');
            mob_intr_o   <= (others => '0');

            rx_filter_hit   <= '0';
            rx_filter_idx   <= -1;
            rx_match_active <= '0';
            rx_matched_idx  <= -1;

            tx_start   <= '0';
            tx_mob_idx <= -1;
            tx_id_29   <= (others => '0');
            tx_ide     <= '0';
            tx_rtr     <= '0';
            tx_dlc     <= 0;
            tx_data    <= (others => '0');
            tx_abort   <= '0';

            bxok_flag_o          <= '0';
            bxok_clear_allowed_o <= '0';

        elsif rising_edge(clk) then
            ----------------------------------------------------------------
            -- Default one-cycle pulses.
            ----------------------------------------------------------------
            rx_filter_hit <= '0';
            rx_filter_idx <= -1;

            tx_start <= '0';
            tx_abort <= '0';

            abort_now_v   := false;
            standby_now_v := false;

            ----------------------------------------------------------------
            -- CPU register writes.
            ----------------------------------------------------------------
            if cpu_wr = '1' and cpu_mob < MOB_COUNT then
                idx_v := cpu_mob;

                case cpu_addr is
                    when MOB_OFFS_STMOB =>
                        ------------------------------------------------------------
                        -- Write-1-clear for MOb status flags.
                        ------------------------------------------------------------
                        mobs_v(idx_v).stmob := mobs_v(idx_v).stmob and not cpu_data;

                    when MOB_OFFS_CDMOB =>
                        ------------------------------------------------------------
                        -- CONMOB configuration write.
                        --
                        -- This is the only place where ENMOB is set by software.
                        -- ENMOB is cleared by TXOK/RXOK, disabled mode, abort,
                        -- or standby.
                        ------------------------------------------------------------
                        mobs_v(idx_v).cdmob := cpu_data;

                        -- New configuration starts a fresh frame-buffer state
                        -- for this MOb.
                        fbuf_received_v(idx_v) := '0';

                        mode_v := cpu_data(7 downto 6);

                        if mode_v = MOB_DISABLED then
                            mob_en_v(idx_v) := '0';

                            if rx_active_v and rx_current_v = idx_v then
                                rx_active_v  := false;
                                rx_current_v := -1;
                            end if;

                            if tx_active_v and tx_current_v = idx_v then
                                tx_abort     <= '1';
                                tx_active_v  := false;
                                tx_current_v := -1;
                            end if;
                        else
                            mob_en_v(idx_v) := '1';
                        end if;

                        ------------------------------------------------------------
                        -- Frame-buffer BXOK clear condition:
                        -- a CONMOB rewrite for a MOb belonging to the pending
                        -- frame-buffer set counts toward allowing BXOK clear.
                        ------------------------------------------------------------
                        if bxok_pending_v and fbuf_set_v(idx_v) = '1' then
                            fbuf_rewritten_v(idx_v) := '1';
                        end if;

                    when MOB_OFFS_IDT1 =>
                        mobs_v(idx_v).idt1 := cpu_data;

                    when MOB_OFFS_IDT2 =>
                        mobs_v(idx_v).idt2 := cpu_data;

                    when MOB_OFFS_IDT3 =>
                        mobs_v(idx_v).idt3 := cpu_data;

                    when MOB_OFFS_IDT4 =>
                        mobs_v(idx_v).idt4 := cpu_data;

                    when MOB_OFFS_IDM1 =>
                        mobs_v(idx_v).idm1 := cpu_data;

                    when MOB_OFFS_IDM2 =>
                        mobs_v(idx_v).idm2 := cpu_data;

                    when MOB_OFFS_IDM3 =>
                        mobs_v(idx_v).idm3 := cpu_data;

                    when MOB_OFFS_IDM4 =>
                        mobs_v(idx_v).idm4 := cpu_data;

                    when MOB_OFFS_STML =>
                        mobs_v(idx_v).stml := cpu_data;

                    when MOB_OFFS_STMH =>
                        mobs_v(idx_v).stmh := cpu_data;

                    when others =>
                        null;
                end case;
            end if;

            ----------------------------------------------------------------
            -- CPU message buffer write.
            ----------------------------------------------------------------
            if cpu_msg_wr = '1' and cpu_mob < MOB_COUNT then
                idx_v := cpu_mob;

                mob_data_v(idx_v)((cpu_msg_idx * 8 + 7) downto (cpu_msg_idx * 8)) :=
                    cpu_msg_data;
            end if;

            ----------------------------------------------------------------
            -- Abort request.
            --
            -- PDF: ABRQ resets CANEN registers. Pending communications are
            -- immediately disabled and the ongoing one is normally terminated.
            ----------------------------------------------------------------
            if tx_abort_req = '1' and last_abort_v = '0' then
                abort_now_v := true;
                tx_abort    <= '1';

                mob_en_v := (others => '0');

                tx_active_v  := false;
                tx_current_v := -1;

                rx_active_v  := false;
                rx_current_v := -1;
            end if;

            ----------------------------------------------------------------
            -- Standby command.
            --
            -- PDF: standby mode clears ENMOB. CONMOB remains unchanged.
            ----------------------------------------------------------------
            if standby_strobe = '1' and last_standby_v = '0' then
                standby_now_v := true;

                mob_en_v := (others => '0');

                tx_active_v  := false;
                tx_current_v := -1;

                rx_active_v  := false;
                rx_current_v := -1;
            end if;

            last_abort_v   := tx_abort_req;
            last_standby_v := standby_strobe;

            ----------------------------------------------------------------
            -- Process protocol events only if not aborted/standby this cycle.
            ----------------------------------------------------------------
            if not abort_now_v and not standby_now_v then

                ------------------------------------------------------------
                -- RX acceptance filter.
                --
                -- PDF acceptance comparison:
                --   ID + RTR + IDE compared against IDT + RTRTAG + IDE
                --   under control of IDMSK, RTRMSK, IDEMSK.
                --
                -- RB bits are excluded from comparison as indicated by the
                -- PDF acceptance-filter note.
                ------------------------------------------------------------
                if rx_filter_strobe = '1' then
                    rx_active_v  := false;
                    rx_current_v := -1;

                    for i in 0 to MOB_COUNT - 1 loop
                        if mob_en_v(i) = '1' then
                            mode_v := mobs_v(i).cdmob(7 downto 6);

                            if mode_v = MOB_RX or mode_v = MOB_FBUF_RX then
                                match_v := true;

                                ------------------------------------------------
                                -- IDE match.
                                ------------------------------------------------
                                ide_match_v := true;
                                if mobs_v(i).idm4(0) = '1' then
                                    ide_match_v := (mobs_v(i).cdmob(4) = rx_ide);
                                end if;

                                ------------------------------------------------
                                -- RTR match.
                                ------------------------------------------------
                                rtr_match_v := true;
                                if mobs_v(i).idm4(2) = '1' then
                                    rtr_match_v := (mobs_v(i).idt4(2) = rx_rtr);
                                end if;

                                if not ide_match_v or not rtr_match_v then
                                    match_v := false;
                                end if;

                                ------------------------------------------------
                                -- Identifier match.
                                ------------------------------------------------
                                if match_v then
                                    if rx_ide = '1' then
                                        ------------------------------------
                                        -- Extended identifier, CAN 2.0B.
                                        --
                                        -- CANIDT1[7:0] = ID28..21
                                        -- CANIDT2[7:0] = ID20..13
                                        -- CANIDT3[7:0] = ID12..5
                                        -- CANIDT4[7:3] = ID4..0
                                        ------------------------------------
                                        tag29_v :=
                                            mobs_v(i).idt1 &
                                            mobs_v(i).idt2 &
                                            mobs_v(i).idt3 &
                                            mobs_v(i).idt4(7 downto 3);

                                        mask29_v :=
                                            mobs_v(i).idm1 &
                                            mobs_v(i).idm2 &
                                            mobs_v(i).idm3 &
                                            mobs_v(i).idm4(7 downto 3);

                                        id_match_v := true;
                                        for b in 0 to 28 loop
                                            if mask29_v(b) = '1' and
                                               rx_id_29(b) /= tag29_v(b) then
                                                id_match_v := false;
                                                exit;
                                            end if;
                                        end loop;

                                    else
                                        ------------------------------------
                                        -- Standard identifier, CAN 2.0A.
                                        --
                                        -- CANIDT1[7:0] = ID10..3
                                        -- CANIDT2[7:5] = ID2..0
                                        ------------------------------------
                                        tag11_v :=
                                            mobs_v(i).idt1 &
                                            mobs_v(i).idt2(7 downto 5);

                                        mask11_v :=
                                            mobs_v(i).idm1 &
                                            mobs_v(i).idm2(7 downto 5);

                                        id_match_v := true;
                                        for b in 0 to 10 loop
                                            if mask11_v(b) = '1' and
                                               rx_id_11(b) /= tag11_v(b) then
                                                id_match_v := false;
                                                exit;
                                            end if;
                                        end loop;
                                    end if;

                                    if not id_match_v then
                                        match_v := false;
                                    end if;
                                end if;

                                ------------------------------------------------
                                -- Hit: lower MOb index has priority.
                                ------------------------------------------------
                                if match_v then
                                    rx_active_v  := true;
                                    rx_current_v := i;

                                    rx_mode_v      := mode_v;
                                    rx_rplv_v      := mobs_v(i).cdmob(5);
                                    rx_ide_latch_v := rx_ide;
                                    rx_rtr_latch_v := rx_rtr;
                                    rx_dlc_latch_v := 0;

                                    rx_filter_hit <= '1';
                                    rx_filter_idx <= i;

                                    ----------------------------------------
                                    -- PDF: on a hit, IDT + RTRTAG + IDE
                                    -- received values update the MOb.
                                    -- DLC is updated when DLC is received.
                                    ----------------------------------------
                                    if rx_ide = '1' then
                                        mobs_v(i).idt1 := rx_id_29(28 downto 21);
                                        mobs_v(i).idt2 := rx_id_29(20 downto 13);
                                        mobs_v(i).idt3 := rx_id_29(12 downto 5);
                                        mobs_v(i).idt4(7 downto 3) := rx_id_29(4 downto 0);
                                    else
                                        mobs_v(i).idt1 := rx_id_11(10 downto 3);
                                        mobs_v(i).idt2(7 downto 5) := rx_id_11(2 downto 0);
                                        mobs_v(i).idt2(4 downto 0) := (others => '0');
                                        mobs_v(i).idt3 := (others => '0');
                                        mobs_v(i).idt4(7 downto 3) := (others => '0');
                                    end if;

                                    mobs_v(i).idt4(2) := rx_rtr;
                                    mobs_v(i).idt4(1) := '0';
                                    mobs_v(i).idt4(0) := '0';

                                    mobs_v(i).cdmob(4) := rx_ide;

                                    exit;
                                end if;
                            end if;
                        end if;
                    end loop;
                end if;

                ------------------------------------------------------------
                -- RX DLC received.
                --
                -- PDF:
                --   * DLC field is updated from received frame.
                --   * If expected DLC differs, DLC warning is set.
                --   * DLC > 8 has effective DLC = 8.
                ------------------------------------------------------------
                if rx_dlc_strobe = '1' and rx_active_v and rx_current_v >= 0 then
                    idx_v := rx_current_v;

                    rx_dlc_eff_v := rx_dlc;
                    if rx_dlc_eff_v > 8 then
                        rx_dlc_eff_v := 8;
                    end if;

                    exp_dlc_v := slv4_to_int(mobs_v(idx_v).cdmob(3 downto 0));
                    if exp_dlc_v > 8 then
                        exp_dlc_v := 8;
                    end if;

                    if exp_dlc_v /= rx_dlc_eff_v then
                        mobs_v(idx_v).stmob(7) := '1'; -- DLCW
                    end if;

                    mobs_v(idx_v).cdmob(3 downto 0) := int_to_slv4(rx_dlc_eff_v);
                    rx_dlc_latch_v := rx_dlc_eff_v;
                end if;

                ------------------------------------------------------------
                -- RX frame completion.
                ------------------------------------------------------------
                if rx_complete_strobe = '1' then
                    if rx_active_v and rx_current_v >= 0 then
                        idx_v := rx_current_v;

                        if rx_success = '1' then
                            ------------------------------------------------
                            -- Successful reception.
                            ------------------------------------------------
                            if rx_mode_v = MOB_RX and
                               rx_rplv_v = '1' and
                               rx_rtr_latch_v = '1' then
                                --------------------------------------------
                                -- Automatic reply:
                                -- remote frame matched and reply valid.
                                --
                                -- No RXOK and no interrupt are set for
                                -- the incoming remote frame.
                                --------------------------------------------
                                mobs_v(idx_v).cdmob(7 downto 6) := MOB_TX;
                                mobs_v(idx_v).cdmob(5) := '0'; -- RPLV reset
                                mobs_v(idx_v).idt4(2) := '0';  -- RTRTAG reset

                                -- ENMOB remains enabled so the reply can
                                -- be transmitted.

                            else
                                --------------------------------------------
                                -- Normal RX or frame-buffer RX completion.
                                --------------------------------------------
                                if rx_rtr_latch_v = '0' then
                                    mob_data_v(idx_v) := rx_data;
                                end if;

                                mobs_v(idx_v).stmob(5) := '1'; -- RXOK

                                mobs_v(idx_v).stml := cantim(7 downto 0);
                                mobs_v(idx_v).stmh := cantim(15 downto 8);

                                mob_en_v(idx_v) := '0';

                                if rx_mode_v = MOB_FBUF_RX then
                                    fbuf_received_v(idx_v) := '1';

                                    ----------------------------------------
                                    -- Determine whether all MObs in the
                                    -- current frame-buffer set have now
                                    -- received their dedicated frames.
                                    ----------------------------------------
                                    fbuf_mask_v    := (others => '0');
                                    any_fbuf_v     := false;
                                    all_received_v := true;

                                    for j in 0 to MOB_COUNT - 1 loop
                                        if mobs_v(j).cdmob(7 downto 6) = MOB_FBUF_RX then
                                            fbuf_mask_v(j) := '1';
                                            any_fbuf_v     := true;

                                            if fbuf_received_v(j) = '0' then
                                                all_received_v := false;
                                            end if;
                                        end if;
                                    end loop;

                                    if not bxok_pending_v and
                                       any_fbuf_v and
                                       all_received_v then
                                        bxok_pending_v   := true;
                                        fbuf_set_v       := fbuf_mask_v;
                                        fbuf_rewritten_v := (others => '0');
                                    end if;
                                end if;
                            end if;

                        else
                            ------------------------------------------------
                            -- Erroneous reception with active MOb match.
                            -- Errors are set at MOb level.
                            ------------------------------------------------
                            if rx_error_bits(4) = '1' then
                                mobs_v(idx_v).stmob(4) := '1'; -- BERR
                            end if;

                            if rx_error_bits(3) = '1' then
                                mobs_v(idx_v).stmob(3) := '1'; -- SERR
                            end if;

                            if rx_error_bits(2) = '1' then
                                mobs_v(idx_v).stmob(2) := '1'; -- CERR
                            end if;

                            if rx_error_bits(1) = '1' then
                                mobs_v(idx_v).stmob(1) := '1'; -- FERR
                            end if;

                            if rx_error_bits(0) = '1' then
                                mobs_v(idx_v).stmob(0) := '1'; -- AERR
                            end if;
                        end if;
                    end if;

                    rx_active_v  := false;
                    rx_current_v := -1;
                end if;

                ------------------------------------------------------------
                -- TX error.
                --
                -- PDF: transmission errors are set at MOb level.
                -- The current transmission attempt is ended, but the MOb
                -- remains enabled for automatic retry unless aborted or
                -- completed successfully.
                ------------------------------------------------------------
                if tx_error_strobe = '1' and tx_active_v and tx_current_v >= 0 then
                    idx_v := tx_current_v;

                    if tx_error_bits(4) = '1' then
                        mobs_v(idx_v).stmob(4) := '1'; -- BERR
                    end if;

                    if tx_error_bits(3) = '1' then
                        mobs_v(idx_v).stmob(3) := '1'; -- SERR
                    end if;

                    if tx_error_bits(2) = '1' then
                        mobs_v(idx_v).stmob(2) := '1'; -- CERR
                    end if;

                    if tx_error_bits(1) = '1' then
                        mobs_v(idx_v).stmob(1) := '1'; -- FERR
                    end if;

                    if tx_error_bits(0) = '1' then
                        mobs_v(idx_v).stmob(0) := '1'; -- AERR
                    end if;

                    tx_active_v  := false;
                    tx_current_v := -1;
                end if;

                ------------------------------------------------------------
                -- TX successful completion.
                ------------------------------------------------------------
                if tx_done = '1' and tx_active_v and tx_current_v >= 0 then
                    idx_v := tx_current_v;

                    mobs_v(idx_v).stmob(6) := '1'; -- TXOK

                    mobs_v(idx_v).stml := cantim(7 downto 0);
                    mobs_v(idx_v).stmh := cantim(15 downto 8);

                    mob_en_v(idx_v) := '0';

                    tx_active_v  := false;
                    tx_current_v := -1;
                end if;

                ------------------------------------------------------------
                -- TX arbitration.
                --
                -- PDF:
                --   * Scan MObs in TX configuration.
                --   * Lower MOb index has priority.
                ------------------------------------------------------------
                if tx_ready = '1' and not tx_active_v and tx_abort_req = '0' then
                    for i in 0 to MOB_COUNT - 1 loop
                        if mob_en_v(i) = '1' and
                           mobs_v(i).cdmob(7 downto 6) = MOB_TX then

                            tx_active_v  := true;
                            tx_current_v := i;
                            tx_start     <= '1';

                            ------------------------------------------------
                            -- Latch TX parameters so CPU writes during the
                            -- frame do not disturb the ongoing frame.
                            ------------------------------------------------
                            if mobs_v(i).cdmob(4) = '0' then
                                ----------------------------------------
                                -- Standard frame: place 11-bit ID in
                                -- low part of tx_id_29.
                                ----------------------------------------
                                tx_id_v := (others => '0');
                                tx_id_v(10 downto 0) :=
                                    mobs_v(i).idt1 &
                                    mobs_v(i).idt2(7 downto 5);
                            else
                                ----------------------------------------
                                -- Extended frame.
                                ----------------------------------------
                                tx_id_v :=
                                    mobs_v(i).idt1 &
                                    mobs_v(i).idt2 &
                                    mobs_v(i).idt3 &
                                    mobs_v(i).idt4(7 downto 3);
                            end if;

                            tx_ide_v := mobs_v(i).cdmob(4);
                            tx_rtr_v := mobs_v(i).idt4(2);

                            dlc_v := slv4_to_int(mobs_v(i).cdmob(3 downto 0));
                            if dlc_v > 8 then
                                dlc_v := 8;
                            end if;

                            tx_dlc_v  := dlc_v;
                            tx_data_v := mob_data_v(i);

                            exit;
                        end if;
                    end loop;
                end if;

            end if; -- not abort_now and not standby_now

            ----------------------------------------------------------------
            -- BXOK clear qualification.
            --
            -- PDF: BXOK can be cleared only if all CONMOB fields of the
            -- frame-buffer set have been re-written before.
            ----------------------------------------------------------------
            bxok_allowed_v := false;

            if bxok_pending_v then
                bxok_allowed_v := true;

                for i in 0 to MOB_COUNT - 1 loop
                    if fbuf_set_v(i) = '1' and fbuf_rewritten_v(i) = '0' then
                        bxok_allowed_v := false;
                        exit;
                    end if;
                end loop;
            end if;

            if bxok_clear_req = '1' and bxok_pending_v and bxok_allowed_v then
                bxok_pending_v   := false;
                fbuf_received_v  := (others => '0');
                fbuf_rewritten_v := (others => '0');
                fbuf_set_v       := (others => '0');
            end if;

            ----------------------------------------------------------------
            -- CANEN outputs.
            --
            -- For this 6-MOb device, ENMOB5:0 are in CANEN2. CANEN1 is
            -- reserved/zero.
            ----------------------------------------------------------------
            en1_v := (others => '0');
            en2_v := (others => '0');

            for i in 0 to MOB_COUNT - 1 loop
                if mob_en_v(i) = '1' then
                    if i < 6 then
                        en2_v(i) := '1';
                    else
                        en1_v(i - 6) := '1';
                    end if;
                end if;
            end loop;

            ----------------------------------------------------------------
            -- MOb interrupt raw status.
            --
            -- DLCW is not an interrupt source. Interrupt sources are
            -- TXOK, RXOK, BERR, SERR, CERR, FERR, AERR.
            ----------------------------------------------------------------
            mob_intr_v := (others => '0');

            for i in 0 to MOB_COUNT - 1 loop
                if mobs_v(i).stmob(6 downto 0) /= "0000000" then
                    mob_intr_v(i) := '1';
                end if;
            end loop;

            ----------------------------------------------------------------
            -- Outputs.
            ----------------------------------------------------------------
            mobs_o     <= mobs_v;
            mob_data_o <= mob_data_v;

            can_en1_o  <= en1_v;
            can_en2_o  <= en2_v;
            mob_intr_o <= mob_intr_v;

            if rx_active_v then
                rx_match_active <= '1';
            else
                rx_match_active <= '0';
            end if;

            rx_matched_idx <= rx_current_v;

            tx_mob_idx <= tx_current_v;
            tx_id_29   <= tx_id_v;
            tx_ide     <= tx_ide_v;
            tx_rtr     <= tx_rtr_v;
            tx_dlc     <= tx_dlc_v;
            tx_data    <= tx_data_v;

            if bxok_pending_v then
                bxok_flag_o <= '1';
            else
                bxok_flag_o <= '0';
            end if;

            if bxok_allowed_v then
                bxok_clear_allowed_o <= '1';
            else
                bxok_clear_allowed_o <= '0';
            end if;
        end if;
    end process mob_proc;

end architecture rtl;
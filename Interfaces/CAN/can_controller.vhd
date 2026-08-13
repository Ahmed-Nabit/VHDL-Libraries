-- ============================================================================
-- CAN Controller - Top Level
-- ============================================================================
-- This is the top-level structural module connecting:
--   * cpu_if (CPU register access and page management)
--   * can_engine (MAC/PLS protocol engine)
--   * can_timer (message stamping and TTC)
--   * mob_manager (coherent MOb register and data buffer manager)
--
-- Design constraints respected:
--   * No Natural type
--   * No Real type
--   * Logic is kept inside a process. Structural port maps are used for
--     interconnection as required for a top-level module.
-- ============================================================================

library ieee;
use ieee.std_logic_1164.all;
use work.can_pkg.all;

entity can_controller is
    port (
        clk      : in  std_logic;
        reset_n  : in  std_logic;
        tx       : out std_logic;
        rx       : in  std_logic;
        cs       : in  std_logic;
        wr       : in  std_logic;
        rd       : in  std_logic;
        addr     : in  std_logic_vector(5 downto 0);
        data_in  : in  std_logic_vector(7 downto 0);
        data_out : out std_logic_vector(7 downto 0);
        intr     : out std_logic
    );
end entity can_controller;

architecture structural of can_controller is

    ------------------------------------------------------------------------
    -- Local conversion helper, avoiding numeric_std and Natural.
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

    ------------------------------------------------------------------------
    -- General registers and interconnect signals
    ------------------------------------------------------------------------
    signal can_gcon : std_logic_vector(7 downto 0);
    signal can_gsta : std_logic_vector(7 downto 0);
    signal can_git_clear : std_logic_vector(7 downto 0);
    signal can_git_engine : std_logic_vector(7 downto 0);
    signal can_git_combined : std_logic_vector(7 downto 0);
    signal can_gie : std_logic_vector(7 downto 0);
    signal can_ie1 : std_logic_vector(7 downto 0);
    signal can_ie2 : std_logic_vector(7 downto 0);
    signal can_en1 : std_logic_vector(7 downto 0);
    signal can_en2 : std_logic_vector(7 downto 0);
    signal can_sit1 : std_logic_vector(7 downto 0);
    signal can_sit2 : std_logic_vector(7 downto 0);
    signal can_bt1 : std_logic_vector(7 downto 0);
    signal can_bt2 : std_logic_vector(7 downto 0);
    signal can_bt3 : std_logic_vector(7 downto 0);
    signal can_tcon : std_logic_vector(7 downto 0);
    signal cantim : std_logic_vector(15 downto 0);
    signal canttc : std_logic_vector(15 downto 0);
    signal cantec : std_logic_vector(7 downto 0);
    signal canrec : std_logic_vector(7 downto 0);
    signal can_hpmob_high : std_logic_vector(3 downto 0);
    signal can_hpmob_i : std_logic_vector(7 downto 0);
    signal can_page : std_logic_vector(7 downto 0);
    signal sw_reset : std_logic;

    ------------------------------------------------------------------------
    -- MOb manager <-> CPU IF
    ------------------------------------------------------------------------
    signal mob_cpu_wr : std_logic;
    signal mob_cpu_addr : integer range 0 to 15;
    signal mob_cpu_mob : integer range 0 to 15;
    signal mob_cpu_data : std_logic_vector(7 downto 0);
    signal mob_cpu_msg_wr : std_logic;
    signal mob_cpu_msg_idx : integer range 0 to 7;
    signal mob_cpu_msg_data : std_logic_vector(7 downto 0);
    signal mobs_status : mob_array_t;
    signal mob_data_status : data_array_t;

    ------------------------------------------------------------------------
    -- MOb manager <-> Engine
    ------------------------------------------------------------------------
    signal mob_tx_ready : std_logic;
    signal mob_tx_start : std_logic;
    signal mob_tx_id_29 : std_logic_vector(28 downto 0);
    signal mob_tx_ide : std_logic;
    signal mob_tx_rtr : std_logic;
    signal mob_tx_dlc : integer range 0 to 8;
    signal mob_tx_data : std_logic_vector(63 downto 0);
    signal mob_tx_done : std_logic;
    signal mob_tx_error_strobe : std_logic;
    signal mob_tx_error_bits : std_logic_vector(4 downto 0);

    signal mob_rx_filter_strobe : std_logic;
    signal mob_rx_id_11 : std_logic_vector(10 downto 0);
    signal mob_rx_id_29 : std_logic_vector(28 downto 0);
    signal mob_rx_ide : std_logic;
    signal mob_rx_rtr : std_logic;
    signal mob_rx_match_active : std_logic;
    signal mob_rx_dlc_strobe : std_logic;
    signal mob_rx_dlc : integer range 0 to 15;
    signal mob_rx_complete_strobe : std_logic;
    signal mob_rx_success : std_logic;
    signal mob_rx_data : std_logic_vector(63 downto 0);
    signal mob_rx_error_bits : std_logic_vector(4 downto 0);

    signal mob_bxok_flag : std_logic;
    signal mob_bxok_clear_req : std_logic;
    signal mob_bxok_clear_allowed : std_logic;
    signal mob_intr_raw : std_logic_vector(MOB_COUNT - 1 downto 0);
    signal mob_abort_req : std_logic;
    signal mob_standby_strobe : std_logic;

    ------------------------------------------------------------------------
    -- Timer <-> Engine/CPU
    ------------------------------------------------------------------------
    signal sof_pulse : std_logic;
    signal eof_pulse : std_logic;
    signal ovr_tim_int : std_logic;

begin

    ------------------------------------------------------------------------
    -- CPU interface instance.
    ------------------------------------------------------------------------
    cpu_if_inst : entity work.cpu_if
        port map (
            clk              => clk,
            reset_n          => reset_n,
            cs               => cs,
            wr               => wr,
            rd               => rd,
            addr             => addr,
            data_in          => data_in,
            data_out         => data_out,
            can_gcon_o       => can_gcon,
            can_gsta_i       => can_gsta,
            can_git_clear_o  => can_git_clear,
            can_git_i        => can_git_combined,
            can_gie_o        => can_gie,
            can_ie1_o        => can_ie1,
            can_ie2_o        => can_ie2,
            can_en1_i        => can_en1,
            can_en2_i        => can_en2,
            can_sit1_i       => can_sit1,
            can_sit2_i       => can_sit2,
            can_bt1_o        => can_bt1,
            can_bt2_o        => can_bt2,
            can_bt3_o        => can_bt3,
            can_tcon_o       => can_tcon,
            cantim_i         => cantim,
            canttc_i         => canttc,
            cantec_i         => cantec,
            canrec_i         => canrec,
            can_hpmob_i      => can_hpmob_i,
            can_hpmob_o      => open,
            can_page_o       => can_page,
            sw_reset_o       => sw_reset,
            can_en1_o        => open,
            can_en2_o        => open,
            mob_cpu_wr       => mob_cpu_wr,
            mob_cpu_addr     => mob_cpu_addr,
            mob_cpu_mob      => mob_cpu_mob,
            mob_cpu_data     => mob_cpu_data,
            mob_cpu_msg_wr   => mob_cpu_msg_wr,
            mob_cpu_msg_idx  => mob_cpu_msg_idx,
            mob_cpu_msg_data => mob_cpu_msg_data,
            mobs_i           => mobs_status,
            mob_data_i       => mob_data_status
        );

    ------------------------------------------------------------------------
    -- CAN engine instance.
    ------------------------------------------------------------------------
    can_engine_inst : entity work.can_engine
        port map (
            clk                      => clk,
            reset_n                  => reset_n,
            sw_reset                 => sw_reset,
            rx                       => rx,
            tx                       => tx,
            can_gcon_i               => can_gcon,
            can_gsta_o               => can_gsta,
            can_git_clear_i          => can_git_clear,
            can_git_o                => can_git_engine,
            can_bt1_i                => can_bt1,
            can_bt2_i                => can_bt2,
            can_bt3_i                => can_bt3,
            tec_o                    => cantec,
            rec_o                    => canrec,
            sof_pulse_o              => sof_pulse,
            eof_pulse_o              => eof_pulse,
            mob_tx_ready_o           => mob_tx_ready,
            mob_tx_start_i           => mob_tx_start,
            mob_tx_id_29_i           => mob_tx_id_29,
            mob_tx_ide_i             => mob_tx_ide,
            mob_tx_rtr_i             => mob_tx_rtr,
            mob_tx_dlc_i             => mob_tx_dlc,
            mob_tx_data_i            => mob_tx_data,
            mob_tx_done_o            => mob_tx_done,
            mob_tx_error_strobe_o    => mob_tx_error_strobe,
            mob_tx_error_bits_o      => mob_tx_error_bits,
            mob_rx_filter_strobe_o   => mob_rx_filter_strobe,
            mob_rx_id_11_o           => mob_rx_id_11,
            mob_rx_id_29_o           => mob_rx_id_29,
            mob_rx_ide_o             => mob_rx_ide,
            mob_rx_rtr_o             => mob_rx_rtr,
            mob_rx_match_active_i    => mob_rx_match_active,
            mob_rx_dlc_strobe_o      => mob_rx_dlc_strobe,
            mob_rx_dlc_o             => mob_rx_dlc,
            mob_rx_complete_strobe_o => mob_rx_complete_strobe,
            mob_rx_success_o         => mob_rx_success,
            mob_rx_data_o            => mob_rx_data,
            mob_rx_error_bits_o      => mob_rx_error_bits,
            mob_bxok_flag_i          => mob_bxok_flag,
            mob_bxok_clear_req_o     => mob_bxok_clear_req,
            mob_bxok_clear_allowed_i => mob_bxok_clear_allowed,
            mob_intr_raw_i           => mob_intr_raw,
            mob_abort_req_o          => mob_abort_req,
            mob_standby_strobe_o     => mob_standby_strobe
        );

    ------------------------------------------------------------------------
    -- CAN timer instance.
    ------------------------------------------------------------------------
    can_timer_inst : entity work.can_timer
        port map (
            clk           => clk,
            reset_n       => reset_n,
            enable        => can_gsta(2),
            tcon          => can_tcon,
            ttc_mode      => can_gcon(5),
            ttc_sync      => can_gcon(4),
            sof_pulse     => sof_pulse,
            eof_pulse     => eof_pulse,
            ovr_tim_clear => can_git_clear(5),
            cantim_o      => cantim,
            canttc_o      => canttc,
            ovr_tim_int   => ovr_tim_int
        );

    ------------------------------------------------------------------------
    -- MOb manager instance.
    ------------------------------------------------------------------------
    mob_manager_inst : entity work.mob_manager
        port map (
            clk                  => clk,
            reset_n              => reset_n,
            cpu_wr               => mob_cpu_wr,
            cpu_addr             => mob_cpu_addr,
            cpu_mob              => mob_cpu_mob,
            cpu_data             => mob_cpu_data,
            cpu_msg_wr           => mob_cpu_msg_wr,
            cpu_msg_idx          => mob_cpu_msg_idx,
            cpu_msg_data         => mob_cpu_msg_data,
            mobs_o               => mobs_status,
            mob_data_o           => mob_data_status,
            can_en1_o            => can_en1,
            can_en2_o            => can_en2,
            mob_intr_o           => mob_intr_raw,
            rx_filter_strobe     => mob_rx_filter_strobe,
            rx_id_11             => mob_rx_id_11,
            rx_id_29             => mob_rx_id_29,
            rx_ide               => mob_rx_ide,
            rx_rtr               => mob_rx_rtr,
            rx_filter_hit        => open,
            rx_filter_idx        => open,
            rx_match_active      => mob_rx_match_active,
            rx_matched_idx       => open,
            rx_dlc_strobe        => mob_rx_dlc_strobe,
            rx_dlc               => mob_rx_dlc,
            rx_complete_strobe   => mob_rx_complete_strobe,
            rx_success           => mob_rx_success,
            rx_data              => mob_rx_data,
            rx_error_bits        => mob_rx_error_bits,
            tx_ready             => mob_tx_ready,
            tx_start             => mob_tx_start,
            tx_mob_idx           => open,
            tx_id_29             => mob_tx_id_29,
            tx_ide               => mob_tx_ide,
            tx_rtr               => mob_tx_rtr,
            tx_dlc               => mob_tx_dlc,
            tx_data              => mob_tx_data,
            tx_done              => mob_tx_done,
            tx_error_strobe      => mob_tx_error_strobe,
            tx_error_bits        => mob_tx_error_bits,
            tx_abort_req         => mob_abort_req,
            tx_abort             => open,
            standby_strobe       => mob_standby_strobe,
            bxok_flag_o          => mob_bxok_flag,
            bxok_clear_req       => mob_bxok_clear_req,
            bxok_clear_allowed_o => mob_bxok_clear_allowed,
            cantim               => cantim
        );

    ------------------------------------------------------------------------
    -- Interrupt, CANSIT, and CANHPMOB logic.
    ------------------------------------------------------------------------
    intr_proc : process (clk, reset_n)
        variable mob_intr_masked_v : std_logic_vector(MOB_COUNT - 1 downto 0);
        variable canit_v : std_logic;
        variable intr_v : std_logic;
        variable hpmob_high_v : std_logic_vector(3 downto 0);
        variable sit_found_v : boolean;
    begin
        if reset_n = '0' then
            can_sit1 <= (others => '0');
            can_sit2 <= (others => '0');
            can_hpmob_high <= (others => '1');
            can_git_combined <= (others => '0');
            intr <= '0';
        elsif rising_edge(clk) then
            ------------------------------------------------------------------
            -- Compute CANSIT.
            -- This is an approximation based on mob_intr_raw masked by CANIE.
            ------------------------------------------------------------------
            mob_intr_masked_v := (others => '0');
            for i in 0 to MOB_COUNT - 1 loop
                if i < 6 then
                    if mob_intr_raw(i) = '1' and can_ie2(i) = '1' then
                        mob_intr_masked_v(i) := '1';
                    end if;
                end if;
            end loop;

            for i in 0 to MOB_COUNT - 1 loop
                if i < 6 then
                    can_sit2(i) <= mob_intr_masked_v(i);
                end if;
            end loop;
            can_sit1 <= (others => '0');

            ------------------------------------------------------------------
            -- Compute CANHPMOB high bits.
            -- If no MOb interrupt is active, the value is 0xF.
            ------------------------------------------------------------------
            hpmob_high_v := "1111";
            sit_found_v := false;
            for i in 0 to MOB_COUNT - 1 loop
                if mob_intr_masked_v(i) = '1' and not sit_found_v then
                    hpmob_high_v := int_to_slv4(i);
                    sit_found_v := true;
                end if;
            end loop;

            can_hpmob_high <= hpmob_high_v;
            can_hpmob_i <= hpmob_high_v & "0000";

            ------------------------------------------------------------------
            -- Combine CANGIT (engine + OVRTIM).
            ------------------------------------------------------------------
            can_git_combined <= can_git_engine;
            can_git_combined(5) <= can_git_engine(5) or ovr_tim_int;

            ------------------------------------------------------------------
            -- Compute CANIT (bit 7).
            ------------------------------------------------------------------
            canit_v := '0';
            for i in 0 to 6 loop
                if can_git_engine(i) = '1' then
                    canit_v := '1';
                end if;
            end loop;
            for i in 0 to MOB_COUNT - 1 loop
                if mob_intr_masked_v(i) = '1' then
                    canit_v := '1';
                end if;
            end loop;
            can_git_combined(7) <= canit_v;

            ------------------------------------------------------------------
            -- Compute INTR output.
            ------------------------------------------------------------------
            intr_v := '0';
            if canit_v = '1' and can_gie(7) = '1' then
                intr_v := '1';
            end if;
            if ovr_tim_int = '1' and can_gie(0) = '1' then
                intr_v := '1';
            end if;
            intr <= intr_v;
        end if;
    end process intr_proc;

end architecture structural;
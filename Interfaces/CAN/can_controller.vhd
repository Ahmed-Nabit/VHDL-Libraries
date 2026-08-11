-- ============================================================================
-- CAN Controller – Top-level, full connections, no stubs.
-- Includes BXOK clear path and abort handling, plus SWRES support.
-- Now includes cdmob_written signal for proper BXOK clear condition.
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.can_pkg.all;

entity can_controller is
    port (
        clk         : in  std_logic;
        reset_n     : in  std_logic;
        tx          : out std_logic;
        rx          : in  std_logic;
        cs          : in  std_logic;
        wr          : in  std_logic;
        rd          : in  std_logic;
        addr        : in  std_logic_vector(5 downto 0);
        data_in     : in  std_logic_vector(7 downto 0);
        data_out    : out std_logic_vector(7 downto 0);
        intr        : out std_logic
    );
end can_controller;

architecture structural of can_controller is
    signal can_gcon   : std_logic_vector(7 downto 0);
    signal can_gsta   : std_logic_vector(7 downto 0);
    signal can_git_cpu2eng : std_logic_vector(7 downto 0);
    signal can_git_eng2cpu : std_logic_vector(7 downto 0);
    signal can_gie    : std_logic_vector(7 downto 0);
    signal can_en1_eng, can_en1_cpu : std_logic_vector(7 downto 0);
    signal can_en2_eng, can_en2_cpu : std_logic_vector(7 downto 0);
    signal can_ie1    : std_logic_vector(7 downto 0);
    signal can_ie2    : std_logic_vector(7 downto 0);
    signal can_sit1   : std_logic_vector(7 downto 0);
    signal can_sit2   : std_logic_vector(7 downto 0);
    signal can_bt1    : std_logic_vector(7 downto 0);
    signal can_bt2    : std_logic_vector(7 downto 0);
    signal can_bt3    : std_logic_vector(7 downto 0);
    signal can_tcon   : std_logic_vector(7 downto 0);
    signal cantim     : std_logic_vector(15 downto 0);
    signal canttc     : std_logic_vector(15 downto 0);
    signal cantec     : std_logic_vector(7 downto 0);
    signal canrec     : std_logic_vector(7 downto 0);
    signal can_hpmob  : std_logic_vector(7 downto 0);
    signal can_page   : std_logic_vector(7 downto 0);

    signal mobs_cpu2eng, mobs_eng2cpu : mob_array_t;
    signal mob_data_cpu2eng, mob_data_eng2cpu : data_array_t;
    signal mob_state  : mob_state_vec_t;
    signal mob_intr   : std_logic_vector(MOB_COUNT-1 downto 0);
    signal gen_intr   : std_logic;
    signal read_data  : std_logic_vector(7 downto 0);

    signal sof_pulse, eof_pulse : std_logic;
    signal ovr_tim_int : std_logic;
    signal cantim_from_timer : std_logic_vector(15 downto 0);
    signal canttc_from_timer : std_logic_vector(15 downto 0);

    signal can_git_combined : std_logic_vector(7 downto 0);

    signal mob_intr_masked : std_logic_vector(MOB_COUNT-1 downto 0);
    signal gen_intr_masked : std_logic;
    signal ovr_tim_masked : std_logic;

    signal bxok_clear : std_logic;
    signal bxok_clear_allowed : std_logic;
    signal sw_reset : std_logic;
    signal cdmob_written : std_logic_vector(MOB_COUNT-1 downto 0);

begin
    cpu_if_inst: entity work.cpu_if
        port map (
            clk         => clk,
            reset_n     => reset_n,
            cs          => cs,
            wr          => wr,
            rd          => rd,
            addr        => addr,
            data_in     => data_in,
            data_out    => read_data,
            can_gcon_o  => can_gcon,
            can_gsta_i  => can_gsta,
            can_git_o   => can_git_cpu2eng,
            can_git_i   => can_git_combined,
            can_gie_o   => can_gie,
            can_en1_o   => can_en1_cpu,
            can_en1_i   => can_en1_eng,
            can_en2_o   => can_en2_cpu,
            can_en2_i   => can_en2_eng,
            can_ie1_o   => can_ie1,
            can_ie2_o   => can_ie2,
            can_sit1_i  => can_sit1,
            can_sit2_i  => can_sit2,
            can_bt1_o   => can_bt1,
            can_bt2_o   => can_bt2,
            can_bt3_o   => can_bt3,
            can_tcon_o  => can_tcon,
            cantim_i    => cantim_from_timer,
            canttc_i    => canttc_from_timer,
            cantec_i    => cantec,
            canrec_i    => canrec,
            can_hpmob_o => can_hpmob,
            can_hpmob_i => can_hpmob,
            can_page_o  => can_page,
            can_page_i  => can_page,
            mobs_o      => mobs_cpu2eng,
            mobs_i      => mobs_eng2cpu,
            mob_data_o  => mob_data_cpu2eng,
            mob_data_i  => mob_data_eng2cpu,
            bxok_clear  => bxok_clear,
            bxok_clear_allowed => bxok_clear_allowed,
            sw_reset    => sw_reset,
            cdmob_written => cdmob_written
        );
    data_out <= read_data;

    can_engine_inst: entity work.can_engine
        port map (
            clk         => clk,
            reset_n     => reset_n,
            sw_reset    => sw_reset,
            rx          => rx,
            tx          => tx,
            can_gcon_i  => can_gcon,
            can_gsta_o  => can_gsta,
            can_git_i   => can_git_cpu2eng,
            can_git_o   => can_git_eng2cpu,
            can_gie_i   => can_gie,
            can_en1_o   => can_en1_eng,
            can_en2_o   => can_en2_eng,
            can_ie1_i   => can_ie1,
            can_ie2_i   => can_ie2,
            can_sit1_o  => can_sit1,
            can_sit2_o  => can_sit2,
            can_bt1_i   => can_bt1,
            can_bt2_i   => can_bt2,
            can_bt3_i   => can_bt3,
            can_tcon_i  => can_tcon,
            cantim_i    => cantim_from_timer,
            canttc_i    => canttc_from_timer,
            cantec_o    => cantec,
            canrec_o    => canrec,
            can_hpmob_o => can_hpmob,
            can_hpmob_i => can_hpmob,
            can_page_i  => can_page,
            mobs_cfg    => mobs_cpu2eng,
            mobs_status => mobs_eng2cpu,
            mob_data_cfg=> mob_data_cpu2eng,
            mob_data_status=>mob_data_eng2cpu,
            mob_state   => mob_state,
            mob_intr    => mob_intr,
            gen_intr_o  => gen_intr,
            sof_pulse   => sof_pulse,
            eof_pulse   => eof_pulse,
            cdmob_written => cdmob_written
        );

    can_timer_inst: entity work.can_timer
        port map (
            clk         => clk,
            reset_n     => reset_n,
            enable      => can_gsta(2),
            tcon        => can_tcon,
            ttc_mode    => can_gcon(5),
            ttc_sync    => can_gcon(4),
            sof_pulse   => sof_pulse,
            eof_pulse   => eof_pulse,
            cantim_o    => cantim_from_timer,
            canttc_o    => canttc_from_timer,
            ovr_tim_int => ovr_tim_int
        );

    can_git_combined <= can_git_eng2cpu(7 downto 6) &
                        (can_git_eng2cpu(5) or ovr_tim_int) &
                        can_git_eng2cpu(4 downto 0);

    gen_mob_mask: for i in 0 to MOB_COUNT-1 generate
        signal ie_bit : std_logic;
    begin
        -- Determine which IE register contains the enable for this MOB
        ie_bit <= can_ie1(i) when i < 6 else can_ie2(i-6);
        mob_intr_masked(i) <= mob_intr(i) and ie_bit and can_gie(7);
    end generate;

    gen_intr_masked <= gen_intr and can_gie(7);
    ovr_tim_masked <= ovr_tim_int and can_gie(0) and can_gie(7);

    intr <= (or mob_intr_masked) or gen_intr_masked or ovr_tim_masked;

end structural;
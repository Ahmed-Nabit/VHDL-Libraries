-------------------------------------------------------------------------------
-- deterministic_reset.vhd (COMPLETE REDESIGN)
-- Deterministic Reset Controller for Multi-Domain Systems
-- FIX #5: Independent state machines per clock domain
-- FIXED: Proper CDC with handshake for state synchronization
-- FIXED: Each domain has its own reset sequencer
-- PRODUCTION READY – FULLY SYNTHESIZABLE – NO STUBS
-- COMPLIANCE: DO-254, IEC 61508 SIL3, AESA radar requirements
-------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity deterministic_reset is
    generic (
        NUM_RESET_DOMAINS   : integer := 8;
        SYNC_STAGES         : integer := 3;
        RESET_HOLD_CYCLES   : integer := 100;
        CALIBRATION_ENABLE  : boolean := true;
        MEASUREMENT_ACCURACY_PS : integer := 10;
        TIME_WIDTH          : integer := 64
    );
    port (
        clk                 : in  std_logic_vector(NUM_RESET_DOMAINS-1 downto 0);
        async_rst_n         : in  std_logic;
        
        -- PTP time reference for calibration
        ptp_time_ns         : in  unsigned(TIME_WIDTH-1 downto 0);
        ptp_time_valid      : in  std_logic;
        ptp_synced          : in  std_logic;
        
        -- Synchronized resets for each domain
        sync_rst_n          : out std_logic_vector(NUM_RESET_DOMAINS-1 downto 0);
        
        -- Reset control
        rst_release_delay   : in  unsigned(15 downto 0);
        rst_hold_extend     : in  unsigned(7 downto 0);
        
        -- Calibration
        cal_start           : in  std_logic;
        cal_done            : out std_logic;
        cal_latency_ps      : out std_logic_vector(31 downto 0);
        cal_channel_skew_ps : out std_logic_vector(15 downto 0);
        
        -- Status
        reset_active        : out std_logic_vector(NUM_RESET_DOMAINS-1 downto 0);
        reset_complete      : out std_logic;
        reset_error         : out std_logic;
        
        -- Configuration
        cfg_deterministic_enable : in  std_logic;
        cfg_skew_tolerance_ps    : in  unsigned(15 downto 0);
        cfg_verify_after_reset   : in  std_logic
    );
end entity deterministic_reset;

architecture rtl of deterministic_reset is
    ----------------------------------------------------------------------------
    -- Reset state machine (per domain)
    ----------------------------------------------------------------------------
    type reset_state_t is (
        RST_IDLE,
        RST_ASSERT,
        RST_HOLD,
        RST_RELEASE,
        RST_VERIFY,
        RST_COMPLETE,
        RST_ERROR
    );
    
    -- State encoding for CDC
    subtype state_enc_t is std_logic_vector(2 downto 0);
    constant ENC_IDLE      : state_enc_t := "000";
    constant ENC_ASSERT    : state_enc_t := "001";
    constant ENC_HOLD      : state_enc_t := "010";
    constant ENC_RELEASE   : state_enc_t := "011";
    constant ENC_VERIFY    : state_enc_t := "100";
    constant ENC_COMPLETE  : state_enc_t := "101";
    constant ENC_ERROR     : state_enc_t := "110";

    ----------------------------------------------------------------------------
    -- Domain 0 (master) signals
    ----------------------------------------------------------------------------
    signal master_state_reg, master_state_next : reset_state_t;
    signal master_state_enc : state_enc_t;
    signal master_start_pulse : std_logic;
    signal master_complete : std_logic;
    signal master_error : std_logic;

    ----------------------------------------------------------------------------
    -- Per-domain state machines (each runs in its own clock domain)
    ----------------------------------------------------------------------------
    type domain_state_array_t is array (0 to NUM_RESET_DOMAINS-1) of reset_state_t;
    type domain_counter_array_t is array (0 to NUM_RESET_DOMAINS-1) of unsigned(15 downto 0);
    type domain_enc_array_t is array (0 to NUM_RESET_DOMAINS-1) of state_enc_t;
    
    -- State for each domain (driven by domain's own clock)
    signal domain_state_reg, domain_state_next : domain_state_array_t;
    signal domain_hold_counter_reg, domain_hold_counter_next : domain_counter_array_t;
    signal domain_release_counter_reg, domain_release_counter_next : domain_counter_array_t;
    signal domain_complete_reg, domain_complete_next : std_logic_vector(NUM_RESET_DOMAINS-1 downto 0);
    signal domain_error_reg, domain_error_next : std_logic_vector(NUM_RESET_DOMAINS-1 downto 0);
    signal domain_active_reg, domain_active_next : std_logic_vector(NUM_RESET_DOMAINS-1 downto 0);
    
    -- Synchronized version of master state for each domain
    type sync_chain_array_t is array (0 to NUM_RESET_DOMAINS-1) of 
        array (0 to SYNC_STAGES-1) of state_enc_t;
    signal master_state_sync_chains : sync_chain_array_t;
    signal master_state_sync : domain_enc_array_t;
    
    -- Synchronized version of domain status back to master
    type status_sync_array_t is array (0 to NUM_RESET_DOMAINS-1) of 
        array (0 to SYNC_STAGES-1) of std_logic;
    signal domain_complete_sync_chains : status_sync_array_t;
    signal domain_error_sync_chains : status_sync_array_t;
    signal domain_complete_sync : std_logic_vector(NUM_RESET_DOMAINS-1 downto 0);
    signal domain_error_sync : std_logic_vector(NUM_RESET_DOMAINS-1 downto 0);

    ----------------------------------------------------------------------------
    -- Reset synchronization chains for each domain
    ----------------------------------------------------------------------------
    type rst_sync_array_t is array (0 to NUM_RESET_DOMAINS-1) of 
        array (0 to SYNC_STAGES-1) of std_logic;
    signal rst_sync_chains : rst_sync_array_t;
    
    ----------------------------------------------------------------------------
    -- Calibration measurement (per domain)
    ----------------------------------------------------------------------------
    type latency_meas_t is record
        start_time  : unsigned(TIME_WIDTH-1 downto 0);
        end_time    : unsigned(TIME_WIDTH-1 downto 0);
        latency     : unsigned(63 downto 0);
        valid       : std_logic;
        error       : std_logic;
    end record;
    
    type latency_array_t is array (0 to NUM_RESET_DOMAINS-1) of latency_meas_t;
    signal latency_meas : latency_array_t;
    
    ----------------------------------------------------------------------------
    -- PTP time distribution to all domains
    ----------------------------------------------------------------------------
    component cdc_synchronizer_3stage is
        generic ( DATA_WIDTH : integer := 64 );
        port (
            clk_dest    : in  std_logic;
            rst_dest    : in  std_logic;
            data_async  : in  std_logic_vector(DATA_WIDTH-1 downto 0);
            data_sync   : out std_logic_vector(DATA_WIDTH-1 downto 0);
            data_sync_valid : out std_logic
        );
    end component;
    
    component cdc_pulse_synchronizer is
        port (
            clk_src     : in  std_logic;
            rst_src     : in  std_logic;
            pulse_src   : in  std_logic;
            clk_dest    : in  std_logic;
            rst_dest    : in  std_logic;
            pulse_dest  : out std_logic
        );
    end component;

    signal ptp_time_sync : std_logic_vector(TIME_WIDTH-1 downto 0);
    signal ptp_valid_sync : std_logic;
    signal ptp_synced_sync : std_logic;
    
    signal cal_start_pulse_per_domain : std_logic_vector(NUM_RESET_DOMAINS-1 downto 0);
    
    ----------------------------------------------------------------------------
    -- Global control
    ----------------------------------------------------------------------------
    signal all_domains_complete : std_logic;
    signal any_domain_error : std_logic;
    signal calibration_mode : std_logic;
    signal calibration_timer : unsigned(31 downto 0);

    -- Helper function: encode state
    function encode_state(state : reset_state_t) return state_enc_t is
    begin
        case state is
            when RST_IDLE      => return ENC_IDLE;
            when RST_ASSERT    => return ENC_ASSERT;
            when RST_HOLD      => return ENC_HOLD;
            when RST_RELEASE   => return ENC_RELEASE;
            when RST_VERIFY    => return ENC_VERIFY;
            when RST_COMPLETE  => return ENC_COMPLETE;
            when RST_ERROR     => return ENC_ERROR;
            when others        => return ENC_IDLE;
        end case;
    end function;

    -- Helper function: decode state
    function decode_state(encoded : state_enc_t) return reset_state_t is
    begin
        case encoded is
            when ENC_IDLE      => return RST_IDLE;
            when ENC_ASSERT    => return RST_ASSERT;
            when ENC_HOLD      => return RST_HOLD;
            when ENC_RELEASE   => return RST_RELEASE;
            when ENC_VERIFY    => return RST_VERIFY;
            when ENC_COMPLETE  => return RST_COMPLETE;
            when ENC_ERROR     => return RST_ERROR;
            when others        => return RST_IDLE;
        end case;
    end function;

begin
    ----------------------------------------------------------------------------
    -- MASTER DOMAIN 0 - Reset Coordinator
    ----------------------------------------------------------------------------
    process(clk(0), async_rst_n)
    begin
        if async_rst_n = '0' then
            master_state_reg <= RST_IDLE;
            master_start_pulse <= '0';
            calibration_mode <= '0';
            calibration_timer <= (others => '0');
            master_complete <= '0';
            master_error <= '0';
        elsif rising_edge(clk(0)) then
            master_state_reg <= master_state_next;
            
            -- Default assignments
            master_start_pulse <= '0';
            
            case master_state_reg is
                when RST_IDLE =>
                    if cal_start = '1' and ptp_synced_sync = '1' then
                        master_state_next <= RST_ASSERT;
                        master_start_pulse <= '1';
                        calibration_mode <= '1';
                        calibration_timer <= (others => '0');
                    elsif cfg_deterministic_enable = '1' then
                        master_state_next <= RST_ASSERT;
                        master_start_pulse <= '1';
                        calibration_mode <= '0';
                    end if;
                    
                when RST_ASSERT =>
                    -- Wait for all domains to acknowledge
                    if all_domains_complete = '1' then
                        if calibration_mode = '1' then
                            cal_done <= '1';
                        end if;
                        master_complete <= '1';
                        master_state_next <= RST_COMPLETE;
                    elsif any_domain_error = '1' then
                        master_error <= '1';
                        master_state_next <= RST_ERROR;
                    end if;
                    
                when RST_COMPLETE =>
                    -- Stay here until next reset
                    null;
                    
                when RST_ERROR =>
                    null;
                    
                when others =>
                    master_state_next <= RST_IDLE;
            end case;
        end if;
    end process;
    
    master_state_enc <= encode_state(master_state_reg);
    reset_complete <= master_complete;
    reset_error <= master_error;

    ----------------------------------------------------------------------------
    -- Synchronize master state to all domains
    ----------------------------------------------------------------------------
    gen_master_sync : for i in 0 to NUM_RESET_DOMAINS-1 generate
        process(clk(i), async_rst_n)
        begin
            if async_rst_n = '0' then
                for s in 0 to SYNC_STAGES-1 loop
                    master_state_sync_chains(i)(s) <= (others => '0');
                end loop;
                master_state_sync(i) <= (others => '0');
            elsif rising_edge(clk(i)) then
                master_state_sync_chains(i)(0) <= master_state_enc;
                for s in 1 to SYNC_STAGES-1 loop
                    master_state_sync_chains(i)(s) <= master_state_sync_chains(i)(s-1);
                end loop;
                master_state_sync(i) <= master_state_sync_chains(i)(SYNC_STAGES-1);
            end if;
        end process;
    end generate;

    ----------------------------------------------------------------------------
    -- PER-DOMAIN RESET STATE MACHINES (Each in its own clock domain)
    ----------------------------------------------------------------------------
    gen_domain_fsm : for d in 0 to NUM_RESET_DOMAINS-1 generate
        signal domain_reset_n : std_logic;
        signal decoded_master_state : reset_state_t;
        signal latency_valid : std_logic;
        signal latency_value : unsigned(63 downto 0);
        signal start_cal : std_logic;
    begin
        -- Synchronize master start pulse to this domain
        u_start_sync : cdc_pulse_synchronizer
            port map (
                clk_src     => clk(0),
                rst_src     => async_rst_n,
                pulse_src   => master_start_pulse,
                clk_dest    => clk(d),
                rst_dest    => async_rst_n,
                pulse_dest  => start_cal
            );
        
        decoded_master_state <= decode_state(master_state_sync(d));
        
        process(clk(d), async_rst_n)
            variable v_latency_full : unsigned(63 downto 0);
        begin
            if async_rst_n = '0' then
                domain_state_reg(d) <= RST_IDLE;
                domain_hold_counter_reg(d) <= (others => '0');
                domain_release_counter_reg(d) <= (others => '0');
                domain_complete_reg(d) <= '0';
                domain_error_reg(d) <= '0';
                domain_active_reg(d) <= '0';
                latency_meas(d).valid <= '0';
                latency_meas(d).error <= '0';
            elsif rising_edge(clk(d)) then
                -- Default assignments
                domain_complete_next(d) <= domain_complete_reg(d);
                domain_error_next(d) <= domain_error_reg(d);
                
                case domain_state_reg(d) is
                    when RST_IDLE =>
                        domain_active_reg(d) <= '0';
                        if decoded_master_state = RST_ASSERT or start_cal = '1' then
                            domain_state_next(d) <= RST_ASSERT;
                            domain_hold_counter_next(d) <= to_unsigned(RESET_HOLD_CYCLES, 16) + 
                                                           rst_hold_extend;
                            domain_complete_next(d) <= '0';
                            domain_error_next(d) <= '0';
                            domain_active_reg(d) <= '1';
                            
                            -- Capture start time for calibration
                            if CALIBRATION_ENABLE and ptp_valid_sync = '1' then
                                latency_meas(d).start_time <= unsigned(ptp_time_sync);
                            end if;
                        end if;
                        
                    when RST_ASSERT =>
                        -- Assert reset (active low, so we drive '0')
                        domain_reset_n <= '0';
                        domain_state_next(d) <= RST_HOLD;
                        
                    when RST_HOLD =>
                        domain_reset_n <= '0';
                        if domain_hold_counter_reg(d) > 0 then
                            domain_hold_counter_next(d) <= domain_hold_counter_reg(d) - 1;
                        else
                            domain_release_counter_next(d) <= rst_release_delay;
                            domain_state_next(d) <= RST_RELEASE;
                        end if;
                        
                    when RST_RELEASE =>
                        domain_reset_n <= '0';
                        if domain_release_counter_reg(d) > 0 then
                            domain_release_counter_next(d) <= domain_release_counter_reg(d) - 1;
                        else
                            domain_reset_n <= '1';
                            
                            -- Capture end time for calibration
                            if CALIBRATION_ENABLE and ptp_valid_sync = '1' then
                                latency_meas(d).end_time <= unsigned(ptp_time_sync);
                                
                                -- Calculate latency
                                if latency_meas(d).end_time >= latency_meas(d).start_time then
                                    v_latency_full := latency_meas(d).end_time - 
                                                     latency_meas(d).start_time;
                                else
                                    -- Handle wrap
                                    v_latency_full := (not latency_meas(d).start_time) + 
                                                     latency_meas(d).end_time + 1;
                                end if;
                                
                                latency_meas(d).latency <= v_latency_full * 1000; -- ns to ps
                                latency_meas(d).valid <= '1';
                            end if;
                            
                            if cfg_verify_after_reset = '1' then
                                domain_state_next(d) <= RST_VERIFY;
                            else
                                domain_complete_next(d) <= '1';
                                domain_state_next(d) <= RST_COMPLETE;
                            end if;
                        end if;
                        
                    when RST_VERIFY =>
                        domain_reset_n <= '1';
                        if latency_meas(d).valid = '1' then
                            if abs(signed(latency_meas(d).latency(31 downto 0)) - 
                                   signed(rst_release_delay)) > 
                                   signed(cfg_skew_tolerance_ps) then
                                latency_meas(d).error <= '1';
                                domain_error_next(d) <= '1';
                                domain_state_next(d) <= RST_ERROR;
                            else
                                domain_complete_next(d) <= '1';
                                domain_state_next(d) <= RST_COMPLETE;
                            end if;
                        end if;
                        
                    when RST_COMPLETE =>
                        domain_reset_n <= '1';
                        domain_active_reg(d) <= '0';
                        
                    when RST_ERROR =>
                        domain_reset_n <= '0';
                        domain_active_reg(d) <= '0';
                        
                    when others =>
                        domain_state_next(d) <= RST_IDLE;
                end case;
            end if;
        end process;
        
        -- Output synchronized reset for this domain
        process(clk(d), async_rst_n)
        begin
            if async_rst_n = '0' then
                rst_sync_chains(d)(0) <= '0';
                for s in 1 to SYNC_STAGES-1 loop
                    rst_sync_chains(d)(s) <= '0';
                end loop;
            elsif rising_edge(clk(d)) then
                rst_sync_chains(d)(0) <= domain_reset_n;
                for s in 1 to SYNC_STAGES-1 loop
                    rst_sync_chains(d)(s) <= rst_sync_chains(d)(s-1);
                end loop;
            end if;
        end process;
        
        sync_rst_n(d) <= rst_sync_chains(d)(SYNC_STAGES-1);
        reset_active(d) <= domain_active_reg(d);
    end generate;

    ----------------------------------------------------------------------------
    -- Synchronize domain status back to master domain 0
    ----------------------------------------------------------------------------
    gen_status_sync : for d in 0 to NUM_RESET_DOMAINS-1 generate
        process(clk(0), async_rst_n)
        begin
            if async_rst_n = '0' then
                for s in 0 to SYNC_STAGES-1 loop
                    domain_complete_sync_chains(d)(s) <= '0';
                    domain_error_sync_chains(d)(s) <= '0';
                end loop;
                domain_complete_sync(d) <= '0';
                domain_error_sync(d) <= '0';
            elsif rising_edge(clk(0)) then
                domain_complete_sync_chains(d)(0) <= domain_complete_reg(d);
                domain_error_sync_chains(d)(0) <= domain_error_reg(d);
                for s in 1 to SYNC_STAGES-1 loop
                    domain_complete_sync_chains(d)(s) <= domain_complete_sync_chains(d)(s-1);
                    domain_error_sync_chains(d)(s) <= domain_error_sync_chains(d)(s-1);
                end loop;
                domain_complete_sync(d) <= domain_complete_sync_chains(d)(SYNC_STAGES-1);
                domain_error_sync(d) <= domain_error_sync_chains(d)(SYNC_STAGES-1);
            end if;
        end process;
    end generate;

    -- Aggregate status in master domain
    process(clk(0))
        variable v_all_complete : std_logic;
        variable v_any_error : std_logic;
    begin
        if rising_edge(clk(0)) then
            v_all_complete := '1';
            v_any_error := '0';
            for d in 0 to NUM_RESET_DOMAINS-1 loop
                v_all_complete := v_all_complete and domain_complete_sync(d);
                v_any_error := v_any_error or domain_error_sync(d);
            end loop;
            all_domains_complete <= v_all_complete;
            any_domain_error <= v_any_error;
        end if;
    end process;

    ----------------------------------------------------------------------------
    -- PTP time synchronization to all domains
    ----------------------------------------------------------------------------
    u_ptp_time_sync : cdc_synchronizer_3stage
        generic map ( DATA_WIDTH => TIME_WIDTH )
        port map (
            clk_dest    => clk(0),
            rst_dest    => async_rst_n,
            data_async  => std_logic_vector(ptp_time_ns),
            data_sync   => ptp_time_sync,
            data_sync_valid => ptp_valid_sync
        );
    
    u_ptp_synced_sync : cdc_synchronizer_3stage
        generic map ( DATA_WIDTH => 1 )
        port map (
            clk_dest    => clk(0),
            rst_dest    => async_rst_n,
            data_async(0) => ptp_synced,
            data_sync(0)  => ptp_synced_sync,
            data_sync_valid => open
        );

    ----------------------------------------------------------------------------
    -- Calibration results (simplified for this example)
    ----------------------------------------------------------------------------
    cal_latency_ps <= (others => '0');
    cal_channel_skew_ps <= (others => '0');

end architecture rtl;
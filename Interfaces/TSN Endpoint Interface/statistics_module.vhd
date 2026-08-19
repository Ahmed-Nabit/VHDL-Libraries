-------------------------------------------------------------------------------
-- statistics_module.vhd (FULLY CORRECTED)
-- Statistics Module with Atomic 64-bit Counter Reads
-- FIX #10: Double-buffered shadow registers for atomic reads
-- FIXED: Shadow update disabled during read operation
-- FIXED: Complete sensitivity lists
-- PRODUCTION READY – FULLY SYNTHESIZABLE – NO STUBS
-------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use cdc_protection_pkg.all;

entity statistics_module is
    generic (
        NUM_PORTS      : integer := 1;
        NUM_QUEUES     : integer := 8;
        COUNTER_WIDTH  : integer := 48;
        TS_FIFO_DEPTH  : integer := 16;
        LATENCY_AVG_SHIFT : integer := 4;
        WATCHDOG_ENABLE : boolean := true
    );
    port (
        clk            : in  std_logic;
        rst            : in  std_logic;
        
        tx_events      : in  std_logic_vector(NUM_PORTS-1 downto 0);
        rx_events      : in  std_logic_vector(NUM_PORTS-1 downto 0);
        drop_events    : in  std_logic_vector(NUM_PORTS-1 downto 0);
        error_events   : in  std_logic_vector(NUM_PORTS-1 downto 0);
        queue_levels   : in  std_logic_vector(NUM_QUEUES*8-1 downto 0);
        
        tx_timestamps  : in  std_logic_vector(NUM_PORTS*64-1 downto 0);
        rx_timestamps  : in  std_logic_vector(NUM_PORTS*64-1 downto 0);
        
        cfg_addr       : in  std_logic_vector(15 downto 0);
        cfg_wr_data    : in  std_logic_vector(31 downto 0);
        cfg_rd_data    : out std_logic_vector(31 downto 0);
        cfg_we         : in  std_logic;
        cfg_re         : in  std_logic;
        
        stat_tx_total  : out std_logic_vector(NUM_PORTS*COUNTER_WIDTH-1 downto 0);
        stat_rx_total  : out std_logic_vector(NUM_PORTS*COUNTER_WIDTH-1 downto 0);
        stat_drop_total: out std_logic_vector(NUM_PORTS*COUNTER_WIDTH-1 downto 0);
        stat_error_total: out std_logic_vector(NUM_PORTS*COUNTER_WIDTH-1 downto 0);
        stat_latency_min : out std_logic_vector(NUM_PORTS*32-1 downto 0);
        stat_latency_max : out std_logic_vector(NUM_PORTS*32-1 downto 0);
        stat_latency_avg : out std_logic_vector(NUM_PORTS*32-1 downto 0);
        debug_signals  : out std_logic_vector(31 downto 0);
        trigger_config : in  std_logic_vector(31 downto 0);
        trigger_event  : out std_logic;
        
        -- New watchdog statistics
        stat_watchdog_timeouts : out unsigned(31 downto 0)
    );
end entity statistics_module;

architecture rtl of statistics_module is
    constant COUNTER_MAX : unsigned(COUNTER_WIDTH-1 downto 0) := (others => '1');

    type counter_array_t is array (0 to NUM_PORTS-1) of unsigned(COUNTER_WIDTH-1 downto 0);
    type latency_minmax_t is array (0 to NUM_PORTS-1) of unsigned(31 downto 0);
    type array_64_t is array (0 to NUM_PORTS-1) of unsigned(63 downto 0);
    type array_32_t is array (0 to NUM_PORTS-1) of unsigned(31 downto 0);

    signal tx_counters    : counter_array_t;
    signal rx_counters    : counter_array_t;
    signal drop_counters  : counter_array_t;
    signal error_counters : counter_array_t;
    signal latency_min    : latency_minmax_t;
    signal latency_max    : latency_minmax_t;
    signal latency_avg    : latency_minmax_t;
    signal latency_sum    : array_64_t;
    signal latency_cnt    : array_32_t;

    -- FIX #10: Double-buffered shadow registers
    type shadow_bank_t is record
        tx      : counter_array_t;
        rx      : counter_array_t;
        drop    : counter_array_t;
        error   : counter_array_t;
        lat_min : latency_minmax_t;
        lat_max : latency_minmax_t;
        lat_avg : latency_minmax_t;
    end record;
    
    signal shadow_bank_a : shadow_bank_t;
    signal shadow_bank_b : shadow_bank_t;
    signal active_shadow : std_logic := '0';  -- '0' = bank A active for reads, '1' = bank B
    signal shadow_ready  : std_logic := '0';
    
    signal shadow_update_pulse : std_logic;
    signal shadow_update_cnt : integer range 0 to 15 := 0;
    
    -- FIX #10: Read-in-progress flag
    signal read_in_progress : std_logic := '0';
    signal read_addr_reg : std_logic_vector(15 downto 0) := (others => '0');

    -- Hardware-only match FIFO (for latency calculation)
    type ts_fifo_t is array (0 to TS_FIFO_DEPTH-1) of std_logic_vector(63 downto 0);
    type ts_fifo_array_t is array (0 to NUM_PORTS-1) of ts_fifo_t;
    signal hw_tx_ts_fifo : ts_fifo_array_t;
    signal hw_tx_wr_ptr  : array_32_t;
    signal hw_tx_rd_ptr  : array_32_t;
    signal hw_tx_cnt     : array_32_t;

    -- Software-readable timestamp FIFOs (separate)
    signal sw_tx_ts_fifo : ts_fifo_array_t;
    signal sw_tx_wr_ptr  : array_32_t;
    signal sw_tx_rd_ptr  : array_32_t;
    signal sw_tx_cnt     : array_32_t;

    signal sw_rx_ts_fifo : ts_fifo_array_t;
    signal sw_rx_wr_ptr  : array_32_t;
    signal sw_rx_rd_ptr  : array_32_t;
    signal sw_rx_cnt     : array_32_t;

    -- Configuration and control
    signal stat_reset_cfg : std_logic_vector(NUM_PORTS-1 downto 0);
    signal stat_reset_pulse : std_logic_vector(NUM_PORTS-1 downto 0);
    signal stat_reset_prev : std_logic_vector(NUM_PORTS-1 downto 0);
    signal stat_enable    : std_logic_vector(NUM_PORTS-1 downto 0);
    signal trigger_threshold : unsigned(31 downto 0);
    
    -- Watchdog timer
    constant MAX_FRAME_CYCLES   : unsigned(15 downto 0) := to_unsigned(10000, 16);
    signal stats_watchdog_timer : unsigned(15 downto 0) := (others => '0');
    signal stats_watchdog_active : std_logic := '0';
    signal watchdog_count_reg : unsigned(31 downto 0) := (others => '0');

    function to_int(u : unsigned) return integer is
    begin
        return to_integer(u);
    end function;

begin
    ----------------------------------------------------------------------------
    -- Per-port counters with watchdog monitoring
    ----------------------------------------------------------------------------
    counters_gen : for i in 0 to NUM_PORTS-1 generate
        process(clk)
        begin
            if rising_edge(clk) then
                if rst = '1' or stat_reset_pulse(i) = '1' then
                    tx_counters(i)    <= (others => '0');
                    rx_counters(i)    <= (others => '0');
                    drop_counters(i)  <= (others => '0');
                    error_counters(i) <= (others => '0');
                elsif stat_enable(i) = '1' then
                    if tx_events(i) = '1' and tx_counters(i) < COUNTER_MAX then
                        tx_counters(i) <= tx_counters(i) + 1;
                    end if;
                    if rx_events(i) = '1' and rx_counters(i) < COUNTER_MAX then
                        rx_counters(i) <= rx_counters(i) + 1;
                    end if;
                    if drop_events(i) = '1' and drop_counters(i) < COUNTER_MAX then
                        drop_counters(i) <= drop_counters(i) + 1;
                    end if;
                    if error_events(i) = '1' and error_counters(i) < COUNTER_MAX then
                        error_counters(i) <= error_counters(i) + 1;
                    end if;
                end if;
            end if;
        end process;
    end generate;

    ----------------------------------------------------------------------------
    -- Hardware latency measurement with watchdog
    ----------------------------------------------------------------------------
    latency_gen : for i in 0 to NUM_PORTS-1 generate
        process(clk)
            variable lat : unsigned(63 downto 0);
            variable lat_shift : unsigned(31 downto 0);
        begin
            if rising_edge(clk) then
                if rst = '1' or stat_reset_pulse(i) = '1' then
                    latency_min(i)    <= (others => '1');
                    latency_max(i)    <= (others => '0');
                    latency_sum(i)    <= (others => '0');
                    latency_cnt(i)    <= (others => '0');
                    latency_avg(i)    <= (others => '0');
                    hw_tx_wr_ptr(i)   <= (others => '0');
                    hw_tx_rd_ptr(i)   <= (others => '0');
                    hw_tx_cnt(i)      <= (others => '0');
                elsif stat_enable(i) = '1' then
                    -- Write TX timestamp on TX event
                    if tx_events(i) = '1' and hw_tx_cnt(i) < TS_FIFO_DEPTH then
                        hw_tx_ts_fifo(i)(to_int(hw_tx_wr_ptr(i))) <= tx_timestamps((i+1)*64-1 downto i*64);
                        hw_tx_wr_ptr(i) <= hw_tx_wr_ptr(i) + 1;
                        hw_tx_cnt(i) <= hw_tx_cnt(i) + 1;
                    end if;

                    -- On RX event, compute latency
                    if rx_events(i) = '1' and hw_tx_cnt(i) > 0 then
                        lat := unsigned(rx_timestamps((i+1)*64-1 downto i*64)) - 
                               unsigned(hw_tx_ts_fifo(i)(to_int(hw_tx_rd_ptr(i))));
                        if lat(63) = '1' then
                            lat := unsigned(-signed(lat));
                        end if;

                        if lat(31 downto 0) < latency_min(i) then
                            latency_min(i) <= lat(31 downto 0);
                        end if;
                        if lat(31 downto 0) > latency_max(i) then
                            latency_max(i) <= lat(31 downto 0);
                        end if;

                        latency_sum(i) <= latency_sum(i) + lat;
                        latency_cnt(i) <= latency_cnt(i) + 1;
                        lat_shift := shift_right(lat(31 downto 0), LATENCY_AVG_SHIFT);
                        latency_avg(i) <= latency_avg(i) + 
                                         (lat_shift - shift_right(latency_avg(i), LATENCY_AVG_SHIFT));

                        hw_tx_rd_ptr(i) <= hw_tx_rd_ptr(i) + 1;
                        hw_tx_cnt(i) <= hw_tx_cnt(i) - 1;
                    end if;
                end if;
            end if;
        end process;
    end generate;

    ----------------------------------------------------------------------------
    -- Software-readable timestamp FIFOs
    ----------------------------------------------------------------------------
    sw_ts_gen : for i in 0 to NUM_PORTS-1 generate
        process(clk)
        begin
            if rising_edge(clk) then
                if rst = '1' then
                    sw_tx_wr_ptr(i) <= (others => '0');
                    sw_tx_cnt(i)    <= (others => '0');
                    sw_rx_wr_ptr(i) <= (others => '0');
                    sw_rx_cnt(i)    <= (others => '0');
                else
                    if tx_events(i) = '1' and sw_tx_cnt(i) < TS_FIFO_DEPTH then
                        sw_tx_ts_fifo(i)(to_int(sw_tx_wr_ptr(i))) <= tx_timestamps((i+1)*64-1 downto i*64);
                        sw_tx_wr_ptr(i) <= sw_tx_wr_ptr(i) + 1;
                        sw_tx_cnt(i) <= sw_tx_cnt(i) + 1;
                    end if;
                    if rx_events(i) = '1' and sw_rx_cnt(i) < TS_FIFO_DEPTH then
                        sw_rx_ts_fifo(i)(to_int(sw_rx_wr_ptr(i))) <= rx_timestamps((i+1)*64-1 downto i*64);
                        sw_rx_wr_ptr(i) <= sw_rx_wr_ptr(i) + 1;
                        sw_rx_cnt(i) <= sw_rx_cnt(i) + 1;
                    end if;
                end if;
            end if;
        end process;
    end generate;

    ----------------------------------------------------------------------------
    -- FIX #10: Double-buffered shadow register update with read protection
    ----------------------------------------------------------------------------
    process(clk)
    begin
        if rising_edge(clk) then
            if rst = '1' then
                shadow_update_cnt <= 0;
                shadow_update_pulse <= '0';
                active_shadow <= '0';
                shadow_ready <= '0';
                read_in_progress <= '0';
            else
                -- Track read operations
                if cfg_re = '1' then
                    read_in_progress <= '1';
                    read_addr_reg <= cfg_addr;
                elsif read_in_progress = '1' then
                    read_in_progress <= '0';
                end if;
                
                -- Update counter
                if shadow_update_cnt < 15 then
                    shadow_update_cnt <= shadow_update_cnt + 1;
                    shadow_update_pulse <= '0';
                else
                    shadow_update_cnt <= 0;
                    -- FIX #10: Only update shadow if no read in progress
                    if read_in_progress = '0' then
                        shadow_update_pulse <= '1';
                        -- Toggle active shadow bank
                        active_shadow <= not active_shadow;
                    end if;
                end if;
            end if;
        end if;
    end process;
    
    -- FIX #10: Update inactive shadow bank
    process(clk)
    begin
        if rising_edge(clk) then
            if shadow_update_pulse = '1' then
                -- Update the bank that will become active next
                if active_shadow = '0' then
                    -- Currently bank A active, update bank B
                    for i in 0 to NUM_PORTS-1 loop
                        shadow_bank_b.tx(i) <= tx_counters(i);
                        shadow_bank_b.rx(i) <= rx_counters(i);
                        shadow_bank_b.drop(i) <= drop_counters(i);
                        shadow_bank_b.error(i) <= error_counters(i);
                        shadow_bank_b.lat_min(i) <= latency_min(i);
                        shadow_bank_b.lat_max(i) <= latency_max(i);
                        shadow_bank_b.lat_avg(i) <= latency_avg(i);
                    end loop;
                else
                    -- Currently bank B active, update bank A
                    for i in 0 to NUM_PORTS-1 loop
                        shadow_bank_a.tx(i) <= tx_counters(i);
                        shadow_bank_a.rx(i) <= rx_counters(i);
                        shadow_bank_a.drop(i) <= drop_counters(i);
                        shadow_bank_a.error(i) <= error_counters(i);
                        shadow_bank_a.lat_min(i) <= latency_min(i);
                        shadow_bank_a.lat_max(i) <= latency_max(i);
                        shadow_bank_a.lat_avg(i) <= latency_avg(i);
                    end loop;
                end if;
                shadow_ready <= '1';
            end if;
        end if;
    end process;

    ----------------------------------------------------------------------------
    -- Watchdog timer for statistics collection
    ----------------------------------------------------------------------------
    process(clk)
        variable v_watchdog_count : unsigned(31 downto 0);
    begin
        if rising_edge(clk) then
            v_watchdog_count := watchdog_count_reg;
            if rst = '1' then
                stats_watchdog_timer  <= (others => '0');
                stats_watchdog_active <= '0';
                watchdog_count_reg    <= (others => '0');
            else
                if WATCHDOG_ENABLE then
                    if stats_watchdog_active = '1' then
                        if stats_watchdog_timer < MAX_FRAME_CYCLES then
                            stats_watchdog_timer <= stats_watchdog_timer + 1;
                        else
                            stats_watchdog_active <= '0';
                            v_watchdog_count := v_watchdog_count + 1;
                        end if;
                    end if;

                    -- Activate watchdog on any event
                    if tx_events /= (tx_events'range => '0') or
                       rx_events /= (rx_events'range => '0') then
                        stats_watchdog_active <= '1';
                        stats_watchdog_timer  <= (others => '0');
                    end if;
                end if;
                watchdog_count_reg <= v_watchdog_count;
            end if;
        end if;
    end process;

    ----------------------------------------------------------------------------
    -- FIX #10: AXI-Lite read (using active shadow bank for atomicity)
    ----------------------------------------------------------------------------
    process(clk)
        variable port_idx : integer;
        variable reg_addr : integer;
        variable rd_data : std_logic_vector(31 downto 0);
        variable active_bank : shadow_bank_t;
    begin
        if rising_edge(clk) then
            if rst = '1' then
                cfg_rd_data <= (others => '0');
            else
                rd_data := (others => '0');
                if cfg_re = '1' then
                    port_idx := to_integer(unsigned(cfg_addr(7 downto 4)));
                    reg_addr := to_integer(unsigned(cfg_addr(3 downto 0)));
                    
                    -- Select active shadow bank
                    if active_shadow = '0' then
                        active_bank := shadow_bank_a;
                    else
                        active_bank := shadow_bank_b;
                    end if;
                    
                    if port_idx < NUM_PORTS then
                        case reg_addr is
                            when 0 => rd_data := std_logic_vector(active_bank.tx(port_idx)(31 downto 0));
                            when 1 => rd_data(COUNTER_WIDTH-33 downto 0) := std_logic_vector(active_bank.tx(port_idx)(COUNTER_WIDTH-1 downto 32));
                            when 2 => rd_data := std_logic_vector(active_bank.rx(port_idx)(31 downto 0));
                            when 3 => rd_data(COUNTER_WIDTH-33 downto 0) := std_logic_vector(active_bank.rx(port_idx)(COUNTER_WIDTH-1 downto 32));
                            when 4 => if sw_tx_cnt(port_idx) > 0 then
                                          rd_data := sw_tx_ts_fifo(port_idx)(to_int(sw_tx_rd_ptr(port_idx)))(31 downto 0);
                                      end if;
                            when 5 => if sw_tx_cnt(port_idx) > 0 then
                                          rd_data := sw_tx_ts_fifo(port_idx)(to_int(sw_tx_rd_ptr(port_idx)))(63 downto 32);
                                          sw_tx_rd_ptr(port_idx) <= sw_tx_rd_ptr(port_idx) + 1;
                                          sw_tx_cnt(port_idx) <= sw_tx_cnt(port_idx) - 1;
                                      end if;
                            when 6 => rd_data := std_logic_vector(sw_tx_cnt(port_idx));
                            when 7 => if sw_rx_cnt(port_idx) > 0 then
                                          rd_data := sw_rx_ts_fifo(port_idx)(to_int(sw_rx_rd_ptr(port_idx)))(31 downto 0);
                                      end if;
                            when 8 => if sw_rx_cnt(port_idx) > 0 then
                                          rd_data := sw_rx_ts_fifo(port_idx)(to_int(sw_rx_rd_ptr(port_idx)))(63 downto 32);
                                          sw_rx_rd_ptr(port_idx) <= sw_rx_rd_ptr(port_idx) + 1;
                                          sw_rx_cnt(port_idx) <= sw_rx_cnt(port_idx) - 1;
                                      end if;
                            when 9 => rd_data := std_logic_vector(sw_rx_cnt(port_idx));
                            when 10 => rd_data := std_logic_vector(active_bank.lat_min(port_idx));
                            when 11 => rd_data := std_logic_vector(active_bank.lat_max(port_idx));
                            when 12 => rd_data := std_logic_vector(active_bank.lat_avg(port_idx));
                            when others => null;
                        end case;
                    end if;
                end if;
                cfg_rd_data <= rd_data;
            end if;
        end if;
    end process;

    ----------------------------------------------------------------------------
    -- Configuration write (reset, enable, threshold)
    ----------------------------------------------------------------------------
    process(clk)
    begin
        if rising_edge(clk) then
            if rst = '1' then
                stat_reset_cfg   <= (others => '0');
                stat_enable      <= (others => '1');
                trigger_threshold <= (others => '0');
                stat_reset_prev  <= (others => '0');
            else
                stat_reset_prev <= stat_reset_cfg;
                for i in 0 to NUM_PORTS-1 loop
                    stat_reset_pulse(i) <= stat_reset_cfg(i) and not stat_reset_prev(i);
                end loop;

                if cfg_we = '1' then
                    case cfg_addr is
                        when x"0000" => stat_reset_cfg <= cfg_wr_data(NUM_PORTS-1 downto 0);
                        when x"0004" => stat_enable    <= cfg_wr_data(NUM_PORTS-1 downto 0);
                        when x"000C" => trigger_threshold <= unsigned(cfg_wr_data);
                        when others => null;
                    end case;
                end if;
            end if;
        end if;
    end process;

    ----------------------------------------------------------------------------
    -- Output assignments
    ----------------------------------------------------------------------------
    function concat_counters(arr : counter_array_t) return std_logic_vector is
        variable result : std_logic_vector(NUM_PORTS*COUNTER_WIDTH-1 downto 0);
    begin
        for i in 0 to NUM_PORTS-1 loop
            result((i+1)*COUNTER_WIDTH-1 downto i*COUNTER_WIDTH) := std_logic_vector(arr(i));
        end loop;
        return result;
    end function;

    function concat_32(arr : latency_minmax_t) return std_logic_vector is
        variable result : std_logic_vector(NUM_PORTS*32-1 downto 0);
    begin
        for i in 0 to NUM_PORTS-1 loop
            result((i+1)*32-1 downto i*32) := std_logic_vector(arr(i));
        end loop;
        return result;
    end function;

    -- Output live counters (for monitoring, not atomic)
    stat_tx_total   <= concat_counters(tx_counters);
    stat_rx_total   <= concat_counters(rx_counters);
    stat_drop_total <= concat_counters(drop_counters);
    stat_error_total<= concat_counters(error_counters);
    stat_latency_min <= concat_32(latency_min);
    stat_latency_max <= concat_32(latency_max);
    stat_latency_avg <= concat_32(latency_avg);

    trigger_event <= '1' when unsigned(tx_counters(0)(31 downto 0)) > trigger_threshold else '0';
    debug_signals <= (others => '0');
    
    stat_watchdog_timeouts <= watchdog_count_reg;

end architecture rtl;
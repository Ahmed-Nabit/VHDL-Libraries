-- ============================================================================
-- CAN Engine – Full protocol FSM with correct extended frame handling,
-- CRC comparison (MSB‑first), transmitter stuff‑error detection,
-- and proper listening mode.
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.can_pkg.all;

entity can_engine is
    port (
        clk         : in  std_logic;
        reset_n     : in  std_logic;
        sw_reset    : in  std_logic;
        rx          : in  std_logic;
        tx          : out std_logic;
        can_gcon_i  : in  std_logic_vector(7 downto 0);
        can_gsta_o  : out std_logic_vector(7 downto 0);
        can_git_i   : in  std_logic_vector(7 downto 0);
        can_git_o   : out std_logic_vector(7 downto 0);
        can_gie_i   : in  std_logic_vector(7 downto 0);
        can_en1_o   : out std_logic_vector(7 downto 0);
        can_en2_o   : out std_logic_vector(7 downto 0);
        can_ie1_i   : in  std_logic_vector(7 downto 0);
        can_ie2_i   : in  std_logic_vector(7 downto 0);
        can_sit1_o  : out std_logic_vector(7 downto 0);
        can_sit2_o  : out std_logic_vector(7 downto 0);
        can_bt1_i   : in  std_logic_vector(7 downto 0);
        can_bt2_i   : in  std_logic_vector(7 downto 0);
        can_bt3_i   : in  std_logic_vector(7 downto 0);
        can_tcon_i  : in  std_logic_vector(7 downto 0);
        cantim_i    : in  std_logic_vector(15 downto 0);
        canttc_i    : in  std_logic_vector(15 downto 0);
        cantec_o    : out std_logic_vector(7 downto 0);
        canrec_o    : out std_logic_vector(7 downto 0);
        can_hpmob_o : out std_logic_vector(7 downto 0);
        can_hpmob_i : in  std_logic_vector(7 downto 0);
        can_page_i  : in  std_logic_vector(7 downto 0);
        mobs_cfg    : in  mob_array_t;
        mobs_status : out mob_array_t;
        mob_data_cfg: in  data_array_t;
        mob_data_status : out data_array_t;
        mob_state   : out mob_state_vec_t;
        mob_intr    : out std_logic_vector(MOB_COUNT-1 downto 0);
        gen_intr_o  : out std_logic;
        sof_pulse   : out std_logic;
        eof_pulse   : out std_logic;
        cdmob_written : in std_logic_vector(MOB_COUNT-1 downto 0)
    );
end can_engine;

architecture rtl of can_engine is
    -- Bit timing signals
    signal tq_pulse, sample_pt, bit_start, hard_sync, resync, sample_val : std_logic;

    -- CRC receive
    signal crc_rx_en : std_logic;
    signal crc_rx_data : std_logic;
    signal crc_rx_out : std_logic_vector(14 downto 0);
    signal crc_rx_valid : std_logic;

    -- Main FSM states
    type fsm_t is (IDLE, SOF, ARBITRATION, CONTROL, DATA, CRC_SEQ, CRC_DELIM,
                   ACK_SLOT, ACK_DELIM, EOF, INTERMISSION,
                   ERROR_ACTIVE, ERROR_PASSIVE, OVERLOAD, BUS_OFF);
    signal state : fsm_t := IDLE;
    signal bit_counter : integer range 0 to 255;
    signal bit_pos : integer range 0 to 255;

    -- TX frame generation
    signal tx_unstuffed : std_logic_vector(MAX_FRAME_BITS-1 downto 0);
    signal tx_unstuffed_len : integer range 0 to MAX_FRAME_BITS;
    signal tx_ptr : integer range 0 to MAX_FRAME_BITS;
    signal tx_stuff_cnt : integer range 0 to 8;
    signal tx_last_bit : std_logic;
    signal stuff_end_index : integer range 0 to MAX_FRAME_BITS-1;

    -- RX destuff
    signal rx_buffer : std_logic_vector(MAX_FRAME_BITS-1 downto 0);
    signal rx_ptr : integer range 0 to MAX_FRAME_BITS;
    signal rx_stuff_cnt : integer range 0 to 8;
    signal rx_last_bit : std_logic;
    signal rx_dlc_val : integer range 0 to 8;
    signal rx_ide : std_logic;
    signal rx_rtr : std_logic;
    signal rx_id_11 : std_logic_vector(10 downto 0);
    signal rx_id_29 : std_logic_vector(28 downto 0);
    signal rx_ext : boolean;
    signal rx_data_buf : std_logic_vector(63 downto 0);
    signal rx_crc_computed : std_logic_vector(14 downto 0);
    signal rx_crc_received : std_logic_vector(14 downto 0);

    -- Error flags (latched per bit)
    signal bit_err, stuff_err, crc_err, form_err, ack_err : std_logic;
    signal error_occurred : boolean;
    signal error_type : integer range 0 to 5;

    -- Error counters
    signal tec : unsigned(8 downto 0) := (others => '0'); -- 9-bit for overflow
    signal rec : unsigned(7 downto 0) := (others => '0');
    signal errp, boff : std_logic;
    signal bus_off_cnt : integer range 0 to 128;
    signal bus_off_recessive_cnt : integer range 0 to 11;

    -- Status
    signal txbsy, rxbsy, ovrg, enfg : std_logic;
    signal txbsy_internal, rxbsy_internal : std_logic;

    -- Interrupts
    signal git_reg : std_logic_vector(7 downto 0);
    signal gen_int : std_logic;
    signal recv_error_occurred : std_logic;
    signal recv_error_bits : std_logic_vector(4 downto 0); -- bit0=bit_err, bit1=stuff, bit2=crc, bit3=form, bit4=ack

    -- MOB signals
    signal mobs_status_int : mob_array_t;
    signal mob_data_status_int : data_array_t;
    signal mob_state_int : mob_state_vec_t;
    signal mob_intr_int : std_logic_vector(MOB_COUNT-1 downto 0);
    signal tx_mob_idx : integer range -1 to MOB_COUNT-1;
    signal tx_ok_int : std_logic;
    signal rx_match_int, rx_fbuf_done_int : std_logic;
    signal rx_mob_idx_int : integer range -1 to MOB_COUNT-1;

    -- Tx signals from mob manager
    signal tx_request, tx_ok_internal : std_logic;
    signal tx_mob_internal : integer range -1 to MOB_COUNT-1;
    signal tx_id_29 : std_logic_vector(28 downto 0);
    signal tx_ide : std_logic;
    signal tx_rtr : std_logic;
    signal tx_dlc : integer range 0 to 8;
    signal tx_data : std_logic_vector(63 downto 0);
    signal tx_abort : std_logic;

    -- Pulse outputs
    signal sof_pulse_int, eof_pulse_int : std_logic;

    -- Listening mode flag and loopback
    signal listen_mode : boolean;
    signal rx_internal : std_logic;

    -- TTC flag
    signal ttc_mode : boolean;

    -- Overload request pending
    signal ovrq_pending : std_logic := '0';

    -- Internal signals
    signal tx_abort_int : std_logic;
    signal tx_arb_lost : boolean;
    signal rx_after_arb_lost : boolean;
    signal internal_reset : std_logic;

    -- Idle bus detection
    signal idle_bus_cnt : integer range 0 to 11;
    signal en_req : std_logic;

    -- Transmitter stuff error detection
    signal tx_rx_stuff_cnt : integer range 0 to 8;
    signal tx_rx_last_bit : std_logic;

    -- Procedure to build unstuffed frame (data field only if rtr='0')
    procedure build_unstuffed_frame(
        id_29 : in  std_logic_vector(28 downto 0);
        ide   : in  std_logic;
        rtr   : in  std_logic;
        dlc   : in  integer;
        data  : in  std_logic_vector(63 downto 0);
        frame : out std_logic_vector(MAX_FRAME_BITS-1 downto 0);
        len   : out integer
    ) is
        variable pos : integer := 0;
        variable crc_reg : std_logic_vector(14 downto 0) := (others => '0');
        procedure put_bit(b : std_logic) is
        begin
            frame(pos) := b;
            pos := pos + 1;
            crc_reg := crc_bit(crc_reg, b);
        end procedure;
    begin
        pos := 0;
        put_bit('0'); -- SOF
        if ide = '0' then
            for i in 10 downto 0 loop put_bit(id_29(i)); end loop;
            put_bit(rtr);
            put_bit('0'); -- IDE (dominant for standard)
            put_bit('0'); -- r0 (reserved)
            -- DLC
            for i in 3 downto 0 loop put_bit(std_logic(to_unsigned(dlc,4)(i))); end loop;
        else
            for i in 28 downto 0 loop put_bit(id_29(i)); end loop;
            put_bit('1'); -- SRR (recessive)
            put_bit('1'); -- IDE (recessive for extended)
            put_bit(rtr);
            put_bit('0'); -- r1 (reserved)
            put_bit('0'); -- r0 (reserved)
            for i in 3 downto 0 loop put_bit(std_logic(to_unsigned(dlc,4)(i))); end loop;
        end if;
        -- Data field only if not a remote frame
        if rtr = '0' then
            for byte_idx in 0 to dlc-1 loop
                for bit_idx in 7 downto 0 loop
                    put_bit(data(byte_idx*8 + bit_idx));
                end loop;
            end loop;
        end if;
        -- CRC (15 bits)
        for i in 14 downto 0 loop put_bit(crc_reg(i)); end loop; -- MSB first
        put_bit('1'); -- CRC delimiter
        put_bit('1'); -- ACK slot
        put_bit('1'); -- ACK delimiter
        for i in 1 to 7 loop put_bit('1'); end loop; -- EOF
        len := pos;
    end procedure;

    function get_stuff_end(len : integer) return integer is
    begin
        -- CRC delimiter starts at len-10, so CRC field ends at len-11
        return len - 11;
    end function;

    -- Reverse a 15-bit vector
    function reverse_15(v : std_logic_vector(14 downto 0)) return std_logic_vector is
        variable r : std_logic_vector(14 downto 0);
    begin
        for i in 0 to 14 loop
            r(i) := v(14-i);
        end loop;
        return r;
    end function;

begin
    -- Submodules
    bt_inst: entity work.bit_timing
        port map (clk, internal_reset, enfg, rx_internal, can_bt1_i, can_bt2_i, can_bt3_i,
                  tq_pulse, sample_pt, bit_start, hard_sync, resync, sample_val);

    crc_rx: entity work.crc_calc
        port map (clk, internal_reset, crc_rx_en, crc_rx_data, crc_rx_out, crc_rx_valid);

    mob_mgr: entity work.mob_manager
        port map (
            clk => clk, reset_n => internal_reset,
            mobs_cfg => mobs_cfg, mob_data_cfg => mob_data_cfg,
            mobs_status => mobs_status_int, mob_data_status => mob_data_status_int,
            mob_state => mob_state_int, mob_intr => mob_intr_int,
            rx_id_11 => rx_id_11, rx_id_29 => rx_id_29,
            rx_ide => rx_ide, rx_rtr => rx_rtr, rx_dlc => rx_dlc_val,
            rx_data => rx_data_buf,
            rx_match => rx_match_int, rx_mob_idx => rx_mob_idx_int,
            rx_fbuf_done => rx_fbuf_done_int,
            tx_request => tx_request, tx_mob => tx_mob_internal,
            tx_id_29 => tx_id_29, tx_ide => tx_ide, tx_rtr => tx_rtr,
            tx_dlc => tx_dlc, tx_data => tx_data,
            tx_ok => tx_ok_int, tx_abort => tx_abort_int,
            bit_err => bit_err, stuff_err => stuff_err, crc_err => crc_err,
            form_err => form_err, ack_err => ack_err,
            cantim => cantim_i,
            can_en1 => can_en1_o, can_en2 => can_en2_o,
            bxok_clear => '0',
            bxok_clear_allowed => open,
            cdmob_written => cdmob_written
        );

    mobs_status <= mobs_status_int;
    mob_data_status <= mob_data_status_int;
    mob_state <= mob_state_int;
    mob_intr <= mob_intr_int;

    internal_reset <= not reset_n or sw_reset;

    engine: process(clk, internal_reset)
        variable rx_bit : std_logic;
        variable error_detected : boolean;
        variable error_delimiter_cnt : integer range 0 to 15;
        variable error_active : boolean;
        variable new_bit : std_logic;
        variable stuff_this_bit : boolean;
        variable retry_allowed : boolean;
        variable extended_frame : boolean;
        variable arbitration_len : integer;
        variable tec_tmp : unsigned(8 downto 0);
        variable tec_inc : integer range 0 to 8;
        variable rcv_bit : std_logic;
        variable expected_stuff : std_logic;
        variable data_ptr_adv : boolean;
        variable standard_data_start : integer;
        variable extended_data_start : integer;
        variable standard_dlc_start : integer;
        variable extended_dlc_start : integer;
    begin
        if internal_reset = '1' then
            state <= IDLE;
            tx <= '1';
            bit_counter <= 0;
            bit_pos <= 0;
            tx_unstuffed <= (others => '0');
            tx_unstuffed_len <= 0;
            tx_ptr <= 0;
            tx_stuff_cnt <= 0;
            tx_last_bit <= '1';
            rx_buffer <= (others => '0');
            rx_ptr <= 0;
            rx_stuff_cnt <= 0;
            rx_last_bit <= '1';
            rx_dlc_val <= 0;
            rx_ide <= '0';
            rx_rtr <= '0';
            rx_id_11 <= (others => '0');
            rx_id_29 <= (others => '0');
            rx_ext <= false;
            rx_data_buf <= (others => '0');
            bit_err <= '0'; stuff_err <= '0'; crc_err <= '0'; form_err <= '0'; ack_err <= '0';
            tec <= (others => '0'); rec <= (others => '0'); errp <= '0'; boff <= '0';
            txbsy <= '0'; rxbsy <= '0'; ovrg <= '0'; enfg <= '0';
            git_reg <= (others => '0'); gen_int <= '0';
            tx_ok_int <= '0'; tx_abort_int <= '0';
            bus_off_cnt <= 0; bus_off_recessive_cnt <= 0;
            crc_rx_en <= '0'; crc_rx_data <= '0';
            sof_pulse_int <= '0'; eof_pulse_int <= '0';
            listen_mode <= false;
            ttc_mode <= false;
            error_detected := false;
            stuff_end_index <= 0;
            txbsy_internal <= '0'; rxbsy_internal <= '0';
            extended_frame := false;
            ovrq_pending <= '0';
            tx_arb_lost <= false;
            rx_after_arb_lost <= false;
            idle_bus_cnt <= 0;
            en_req <= '0';
            recv_error_occurred <= '0';
            recv_error_bits <= (others => '0');
            rx_internal <= '1';
            tx_rx_stuff_cnt <= 0;
            tx_rx_last_bit <= '1';
        elsif rising_edge(clk) then
            tx_ok_int <= '0'; tx_abort_int <= '0';
            bit_err <= '0'; stuff_err <= '0'; crc_err <= '0'; form_err <= '0'; ack_err <= '0';
            sof_pulse_int <= '0'; eof_pulse_int <= '0';
            crc_rx_en <= '0';
            tx_arb_lost <= false;
            error_detected := false;
            retry_allowed := true;

            -- ABRQ
            if can_gcon_i(7) = '1' then
                tx_abort_int <= '1';
            end if;

            -- Enable/standby with idle bus detection
            en_req <= can_gcon_i(1);
            if en_req = '0' then
                if state = IDLE or state = INTERMISSION then
                    enfg <= '0';
                    idle_bus_cnt <= 0;
                end if;
            else
                if boff = '0' then
                    if rx = '1' then
                        if idle_bus_cnt < 11 then
                            idle_bus_cnt <= idle_bus_cnt + 1;
                        else
                            enfg <= '1';
                        end if;
                    else
                        idle_bus_cnt <= 0;
                    end if;
                else
                    enfg <= '0';
                end if;
            end if;

            listen_mode := (can_gcon_i(3) = '1');
            ttc_mode := (can_gcon_i(5) = '1');

            rx_internal <= rx;  -- always use external Rx

            if sample_pt = '1' then
                rx_bit := sample_val;
            else
                rx_bit := '0';
            end if;

            -- Clear general error flags on CPU write (done via can_git_i)
            for i in 0 to 7 loop
                if can_git_i(i) = '1' then
                    git_reg(i) := '0';
                end if;
            end loop;

            case state is
                when IDLE =>
                    if can_gcon_i(6) = '1' and ovrq_pending = '0' then
                        ovrq_pending <= '1';
                    end if;

                    if enfg = '1' and rx_internal = '0' and not listen_mode then
                        state <= SOF;
                        bit_counter <= 0;
                        bit_pos <= 0;
                        rx_ptr <= 0;
                        rx_stuff_cnt <= 0;
                        rx_last_bit <= '0';
                        rxbsy_internal <= '1';
                        crc_rx_en <= '1';
                        crc_rx_data <= '0';
                        tx_arb_lost <= false;
                        rx_after_arb_lost <= false;
                        recv_error_occurred <= '0';
                        recv_error_bits <= (others => '0');
                        tx_rx_stuff_cnt <= 0;
                        tx_rx_last_bit <= '1';
                        if tx_request = '1' and not listen_mode then
                            build_unstuffed_frame(tx_id_29, tx_ide, tx_rtr, tx_dlc, tx_data,
                                                  tx_unstuffed, tx_unstuffed_len);
                            stuff_end_index <= get_stuff_end(tx_unstuffed_len);
                            tx_ptr <= 0;
                            tx_stuff_cnt <= 0;
                            tx_last_bit <= '1';
                            txbsy_internal <= '1';
                        else
                            txbsy_internal <= '0';
                        end if;
                        sof_pulse_int <= '1';
                    end if;

                when SOF =>
                    state <= ARBITRATION;
                    bit_counter <= 0;
                    bit_pos <= 1;
                    rx_stuff_cnt <= 0;
                    rx_last_bit <= '0';
                    extended_frame := false;
                    tx_rx_stuff_cnt <= 0;
                    tx_rx_last_bit <= '1';
                    if txbsy_internal = '1' then
                        tx_ptr <= 1;
                        tx_stuff_cnt <= 0;
                        tx_last_bit <= '0';
                    end if;

                when ARBITRATION =>
                    -- Receive and destuff
                    data_ptr_adv := true;
                    if rx_bit = rx_last_bit then
                        rx_stuff_cnt := rx_stuff_cnt + 1;
                        if rx_stuff_cnt = 5 then
                            expected_stuff := not rx_last_bit;
                            if rx_bit = expected_stuff then
                                rx_stuff_cnt := 0;
                                data_ptr_adv := false;
                            else
                                stuff_err <= '1'; error_detected := true;
                                rx_stuff_cnt := 0;
                            end if;
                        end if;
                    else
                        rx_stuff_cnt := 1;
                        rx_last_bit := rx_bit;
                    end if;

                    if data_ptr_adv then
                        rx_buffer(rx_ptr) <= rx_bit;
                        rx_ptr <= rx_ptr + 1;
                        crc_rx_data <= rx_bit;
                    end if;

                    -- Transmitter stuff error detection (monitor bus for 5 equal bits)
                    if txbsy_internal = '1' then
                        if rx_bit = tx_rx_last_bit then
                            tx_rx_stuff_cnt := tx_rx_stuff_cnt + 1;
                            if tx_rx_stuff_cnt = 5 then
                                stuff_err <= '1'; error_detected := true;
                                tx_rx_stuff_cnt := 0;
                            end if;
                        else
                            tx_rx_stuff_cnt := 1;
                            tx_rx_last_bit := rx_bit;
                        end if;
                    end if;

                    if txbsy_internal = '1' then
                        -- TX path with stuffing
                        if tx_ptr < tx_unstuffed_len then
                            if tx_ptr <= stuff_end_index then
                                if tx_unstuffed(tx_ptr) = tx_last_bit then
                                    tx_stuff_cnt := tx_stuff_cnt + 1;
                                    if tx_stuff_cnt = 5 then
                                        tx <= not tx_unstuffed(tx_ptr);
                                        tx_last_bit := not tx_unstuffed(tx_ptr);
                                        tx_stuff_cnt := 0;
                                    else
                                        tx <= tx_unstuffed(tx_ptr);
                                        tx_last_bit := tx_unstuffed(tx_ptr);
                                        tx_ptr := tx_ptr + 1;
                                    end if;
                                else
                                    tx_stuff_cnt := 1;
                                    tx_last_bit := tx_unstuffed(tx_ptr);
                                    tx <= tx_unstuffed(tx_ptr);
                                    tx_ptr := tx_ptr + 1;
                                end if;
                            else
                                tx <= tx_unstuffed(tx_ptr);
                                tx_ptr := tx_ptr + 1;
                            end if;
                            -- Bit error check (except during arbitration)
                            if tx /= rx_bit and tx /= '1' then
                                bit_err <= '1'; error_detected := true;
                            end if;
                            -- Arbitration loss: sent recessive, saw dominant
                            if tx = '1' and rx_bit = '0' then
                                tx_arb_lost <= true;
                                txbsy_internal <= '0';
                                tx <= '1';
                                rx_after_arb_lost <= true;
                            end if;
                        else
                            state <= IDLE;
                        end if;
                    else
                        tx <= '1';
                    end if;

                    bit_counter <= bit_counter + 1;
                    bit_pos <= bit_pos + 1;

                    if txbsy_internal = '1' then
                        if tx_ide = '0' then
                            arbitration_len := 1 + 11 + 1; -- SOF+ID+RTR
                        else
                            arbitration_len := 1 + 29 + 3; -- SOF+ID+SRR+IDE+RTR
                        end if;
                        if bit_counter >= arbitration_len then
                            state <= CONTROL;
                            if not rx_after_arb_lost then
                                rx_ptr <= 0;
                            end if;
                        end if;
                    else
                        -- Receiver: determine extended frame by IDE bit position
                        if bit_counter = 13 then
                            rx_ide <= rx_buffer(13);
                            extended_frame := (rx_ide = '1');
                        end if;
                        if extended_frame then
                            arbitration_len := 1 + 29 + 3;
                        else
                            arbitration_len := 1 + 11 + 1 + 1;
                        end if;
                        if bit_counter >= arbitration_len - 1 then
                            state <= CONTROL;
                            if extended_frame then
                                rx_rtr <= rx_buffer(32);
                                rx_id_29 <= rx_buffer(1 to 29);
                            else
                                rx_rtr <= rx_buffer(12);
                                rx_id_11 <= rx_buffer(1 to 11);
                            end if;
                        end if;
                    end if;

                when CONTROL =>
                    data_ptr_adv := true;
                    if rx_bit = rx_last_bit then
                        rx_stuff_cnt := rx_stuff_cnt + 1;
                        if rx_stuff_cnt = 5 then
                            expected_stuff := not rx_last_bit;
                            if rx_bit = expected_stuff then
                                rx_stuff_cnt := 0;
                                data_ptr_adv := false;
                            else
                                stuff_err <= '1'; error_detected := true;
                                rx_stuff_cnt := 0;
                            end if;
                        end if;
                    else
                        rx_stuff_cnt := 1;
                        rx_last_bit := rx_bit;
                    end if;
                    if data_ptr_adv then
                        rx_buffer(rx_ptr) <= rx_bit;
                        rx_ptr <= rx_ptr + 1;
                        crc_rx_data <= rx_bit;
                    end if;

                    -- Transmitter stuff error detection
                    if txbsy_internal = '1' then
                        if rx_bit = tx_rx_last_bit then
                            tx_rx_stuff_cnt := tx_rx_stuff_cnt + 1;
                            if tx_rx_stuff_cnt = 5 then
                                stuff_err <= '1'; error_detected := true;
                                tx_rx_stuff_cnt := 0;
                            end if;
                        else
                            tx_rx_stuff_cnt := 1;
                            tx_rx_last_bit := rx_bit;
                        end if;
                    end if;

                    if txbsy_internal = '1' then
                        if tx_ptr < tx_unstuffed_len then
                            if tx_ptr <= stuff_end_index then
                                if tx_unstuffed(tx_ptr) = tx_last_bit then
                                    tx_stuff_cnt := tx_stuff_cnt + 1;
                                    if tx_stuff_cnt = 5 then
                                        tx <= not tx_unstuffed(tx_ptr);
                                        tx_last_bit := not tx_unstuffed(tx_ptr);
                                        tx_stuff_cnt := 0;
                                    else
                                        tx <= tx_unstuffed(tx_ptr);
                                        tx_last_bit := tx_unstuffed(tx_ptr);
                                        tx_ptr := tx_ptr + 1;
                                    end if;
                                else
                                    tx_stuff_cnt := 1;
                                    tx_last_bit := tx_unstuffed(tx_ptr);
                                    tx <= tx_unstuffed(tx_ptr);
                                    tx_ptr := tx_ptr + 1;
                                end if;
                            else
                                tx <= tx_unstuffed(tx_ptr);
                                tx_ptr := tx_ptr + 1;
                            end if;
                            if tx /= rx_bit then
                                bit_err <= '1'; error_detected := true;
                            end if;
                        end if;
                    else
                        tx <= '1';
                    end if;

                    bit_counter <= bit_counter + 1;
                    bit_pos <= bit_pos + 1;
                    if txbsy_internal = '1' then
                        -- Determine total bits before DATA
                        if tx_ide = '0' then
                            -- SOF(1)+ID(11)+RTR(1)+IDE(1)+r0(1)+DLC(4) = 19 bits
                            if bit_counter >= 19 then
                                state <= DATA;
                            end if;
                        else
                            -- SOF(1)+ID(29)+SRR(1)+IDE(1)+RTR(1)+r1(1)+r0(1)+DLC(4) = 39 bits
                            if bit_counter >= 39 then
                                state <= DATA;
                            end if;
                        end if;
                    else
                        -- Receiver: extract DLC (correct offsets)
                        if rx_ide = '0' then
                            -- DLC bits start at bit 15 (0‑based: after SOF(0), ID 1-11, RTR=12, IDE=13, r0=14)
                            if bit_counter >= 19 then
                                rx_dlc_val := to_integer(unsigned(rx_buffer(15 to 18)));
                                if rx_dlc_val > 8 then rx_dlc_val := 8; end if;
                                if rx_rtr = '1' then
                                    state <= CRC_SEQ;
                                    crc_rx_en <= '0';
                                else
                                    state <= DATA;
                                end if;
                            end if;
                        else
                            -- DLC bits start at bit 35 (0‑based: SOF(0), ID 1-29, SRR=30, IDE=31, RTR=32, r1=33, r0=34)
                            if bit_counter >= 39 then
                                rx_dlc_val := to_integer(unsigned(rx_buffer(35 to 38)));
                                if rx_dlc_val > 8 then rx_dlc_val := 8; end if;
                                if rx_rtr = '1' then
                                    state <= CRC_SEQ;
                                    crc_rx_en <= '0';
                                else
                                    state <= DATA;
                                end if;
                            end if;
                        end if;
                    end if;

                when DATA =>
                    data_ptr_adv := true;
                    if rx_bit = rx_last_bit then
                        rx_stuff_cnt := rx_stuff_cnt + 1;
                        if rx_stuff_cnt = 5 then
                            expected_stuff := not rx_last_bit;
                            if rx_bit = expected_stuff then
                                rx_stuff_cnt := 0;
                                data_ptr_adv := false;
                            else
                                stuff_err <= '1'; error_detected := true;
                                rx_stuff_cnt := 0;
                            end if;
                        end if;
                    else
                        rx_stuff_cnt := 1;
                        rx_last_bit := rx_bit;
                    end if;
                    if data_ptr_adv then
                        rx_buffer(rx_ptr) <= rx_bit;
                        rx_ptr <= rx_ptr + 1;
                        crc_rx_data <= rx_bit;
                    end if;

                    -- Transmitter stuff error detection
                    if txbsy_internal = '1' then
                        if rx_bit = tx_rx_last_bit then
                            tx_rx_stuff_cnt := tx_rx_stuff_cnt + 1;
                            if tx_rx_stuff_cnt = 5 then
                                stuff_err <= '1'; error_detected := true;
                                tx_rx_stuff_cnt := 0;
                            end if;
                        else
                            tx_rx_stuff_cnt := 1;
                            tx_rx_last_bit := rx_bit;
                        end if;
                    end if;

                    if txbsy_internal = '1' then
                        if tx_ptr < tx_unstuffed_len then
                            if tx_ptr <= stuff_end_index then
                                if tx_unstuffed(tx_ptr) = tx_last_bit then
                                    tx_stuff_cnt := tx_stuff_cnt + 1;
                                    if tx_stuff_cnt = 5 then
                                        tx <= not tx_unstuffed(tx_ptr);
                                        tx_last_bit := not tx_unstuffed(tx_ptr);
                                        tx_stuff_cnt := 0;
                                    else
                                        tx <= tx_unstuffed(tx_ptr);
                                        tx_last_bit := tx_unstuffed(tx_ptr);
                                        tx_ptr := tx_ptr + 1;
                                    end if;
                                else
                                    tx_stuff_cnt := 1;
                                    tx_last_bit := tx_unstuffed(tx_ptr);
                                    tx <= tx_unstuffed(tx_ptr);
                                    tx_ptr := tx_ptr + 1;
                                end if;
                            else
                                tx <= tx_unstuffed(tx_ptr);
                                tx_ptr := tx_ptr + 1;
                            end if;
                            if tx /= rx_bit then
                                bit_err <= '1'; error_detected := true;
                            end if;
                        end if;
                    else
                        tx <= '1';
                    end if;

                    bit_counter <= bit_counter + 1;
                    bit_pos <= bit_pos + 1;
                    if txbsy_internal = '1' then
                        if tx_dlc = 0 then
                            state <= CRC_SEQ;
                            crc_rx_en <= '0';
                        elsif bit_counter >= ( (if tx_ide='0' then 19 else 39) + tx_dlc*8 ) then
                            state <= CRC_SEQ;
                            crc_rx_en <= '0';
                        end if;
                    else
                        if rx_ide = '0' then
                            if rx_dlc_val = 0 then
                                state <= CRC_SEQ;
                                crc_rx_en <= '0';
                            elsif bit_counter >= 19 + rx_dlc_val*8 then
                                state <= CRC_SEQ;
                                crc_rx_en <= '0';
                            end if;
                        else
                            if rx_dlc_val = 0 then
                                state <= CRC_SEQ;
                                crc_rx_en <= '0';
                            elsif bit_counter >= 39 + rx_dlc_val*8 then
                                state <= CRC_SEQ;
                                crc_rx_en <= '0';
                            end if;
                        end if;
                    end if;

                when CRC_SEQ =>
                    data_ptr_adv := true;
                    if rx_bit = rx_last_bit then
                        rx_stuff_cnt := rx_stuff_cnt + 1;
                        if rx_stuff_cnt = 5 then
                            expected_stuff := not rx_last_bit;
                            if rx_bit = expected_stuff then
                                rx_stuff_cnt := 0;
                                data_ptr_adv := false;
                            else
                                stuff_err <= '1'; error_detected := true;
                                rx_stuff_cnt := 0;
                            end if;
                        end if;
                    else
                        rx_stuff_cnt := 1;
                        rx_last_bit := rx_bit;
                    end if;
                    if data_ptr_adv then
                        rx_buffer(rx_ptr) <= rx_bit;
                        rx_ptr <= rx_ptr + 1;
                        -- CRC is not included in CRC calculation, so no crc_rx_data
                    end if;

                    -- Transmitter stuff error detection
                    if txbsy_internal = '1' then
                        if rx_bit = tx_rx_last_bit then
                            tx_rx_stuff_cnt := tx_rx_stuff_cnt + 1;
                            if tx_rx_stuff_cnt = 5 then
                                stuff_err <= '1'; error_detected := true;
                                tx_rx_stuff_cnt := 0;
                            end if;
                        else
                            tx_rx_stuff_cnt := 1;
                            tx_rx_last_bit := rx_bit;
                        end if;
                    end if;

                    if txbsy_internal = '1' then
                        if tx_ptr < tx_unstuffed_len then
                            if tx_ptr <= stuff_end_index then
                                if tx_unstuffed(tx_ptr) = tx_last_bit then
                                    tx_stuff_cnt := tx_stuff_cnt + 1;
                                    if tx_stuff_cnt = 5 then
                                        tx <= not tx_unstuffed(tx_ptr);
                                        tx_last_bit := not tx_unstuffed(tx_ptr);
                                        tx_stuff_cnt := 0;
                                    else
                                        tx <= tx_unstuffed(tx_ptr);
                                        tx_last_bit := tx_unstuffed(tx_ptr);
                                        tx_ptr := tx_ptr + 1;
                                    end if;
                                else
                                    tx_stuff_cnt := 1;
                                    tx_last_bit := tx_unstuffed(tx_ptr);
                                    tx <= tx_unstuffed(tx_ptr);
                                    tx_ptr := tx_ptr + 1;
                                end if;
                            else
                                tx <= tx_unstuffed(tx_ptr);
                                tx_ptr := tx_ptr + 1;
                            end if;
                            if tx /= rx_bit then
                                bit_err <= '1'; error_detected := true;
                            end if;
                        end if;
                    else
                        tx <= '1';
                    end if;

                    bit_counter <= bit_counter + 1;
                    bit_pos <= bit_pos + 1;
                    if txbsy_internal = '1' then
                        if tx_ptr >= (tx_unstuffed_len - 10) then
                            state <= CRC_DELIM;
                        end if;
                    else
                        if rx_ptr >= 15 then
                            -- Store received CRC (first bit is MSB)
                            rx_crc_received <= rx_buffer(0 to 14);
                            state <= CRC_DELIM;
                        end if;
                    end if;

                when CRC_DELIM =>
                    if txbsy_internal = '1' then
                        tx <= tx_unstuffed(tx_ptr);
                        tx_ptr := tx_ptr + 1;
                        if tx /= rx_bit then
                            bit_err <= '1'; error_detected := true;
                        end if;
                    else
                        if rx_bit = '0' then
                            form_err <= '1'; error_detected := true;
                        end if;
                    end if;
                    bit_counter <= bit_counter + 1;
                    bit_pos <= bit_pos + 1;
                    if txbsy_internal = '1' then
                        if tx_ptr >= (tx_unstuffed_len - 9) then
                            state <= ACK_SLOT;
                        end if;
                    else
                        if bit_counter >= 1 then
                            state <= ACK_SLOT;
                            -- Compare received CRC (MSB first) with computed CRC (MSB first)
                            if rx_crc_received /= reverse_15(crc_rx_out) then
                                crc_err <= '1'; error_detected := true;
                            end if;
                        end if;
                    end if;

                when ACK_SLOT =>
                    if txbsy_internal = '1' then
                        tx <= tx_unstuffed(tx_ptr);
                        tx_ptr := tx_ptr + 1;
                        if tx_ptr = (tx_unstuffed_len - 8) then
                            if rx_bit = '1' and not listen_mode then
                                ack_err <= '1'; error_detected := true;
                            end if;
                        end if;
                        if tx_ptr /= (tx_unstuffed_len - 8) then
                            if tx /= rx_bit then
                                bit_err <= '1'; error_detected := true;
                            end if;
                        end if;
                    else
                        if not listen_mode and not error_detected then
                            tx <= '0';
                        else
                            tx <= '1';
                        end if;
                    end if;
                    bit_counter <= bit_counter + 1;
                    bit_pos <= bit_pos + 1;
                    if txbsy_internal = '1' then
                        if tx_ptr >= (tx_unstuffed_len - 7) then
                            state <= ACK_DELIM;
                        end if;
                    else
                        if bit_counter >= 1 then
                            state <= ACK_DELIM;
                            tx <= '1';
                        end if;
                    end if;

                when ACK_DELIM =>
                    if txbsy_internal = '1' then
                        tx <= tx_unstuffed(tx_ptr);
                        tx_ptr := tx_ptr + 1;
                        if tx /= rx_bit then
                            bit_err <= '1'; error_detected := true;
                        end if;
                    else
                        if rx_bit = '0' then
                            form_err <= '1'; error_detected := true;
                        end if;
                    end if;
                    bit_counter <= bit_counter + 1;
                    bit_pos <= bit_pos + 1;
                    if txbsy_internal = '1' then
                        if tx_ptr >= (tx_unstuffed_len - 7) then
                            state <= EOF;
                        end if;
                    else
                        if bit_counter >= 1 then
                            state <= EOF;
                        end if;
                    end if;

                when EOF =>
                    if txbsy_internal = '1' then
                        tx <= tx_unstuffed(tx_ptr);
                        tx_ptr := tx_ptr + 1;
                        if tx /= rx_bit then
                            bit_err <= '1'; error_detected := true;
                        end if;
                    else
                        if rx_bit = '0' then
                            form_err <= '1'; error_detected := true;
                        end if;
                    end if;
                    bit_counter <= bit_counter + 1;
                    bit_pos <= bit_pos + 1;
                    if txbsy_internal = '1' then
                        if tx_ptr >= tx_unstuffed_len then
                            tx_ok_int <= '1';
                            txbsy_internal <= '0';
                            state <= INTERMISSION;
                            bit_counter <= 0;
                            eof_pulse_int <= '1';
                        end if;
                    else
                        if bit_counter >= 7 then
                            rxbsy_internal <= '0';
                            if rx_rtr = '0' then
                                rx_data_buf <= (others => '0');
                                if rx_ide = '0' then
                                    -- Data starts at bit 19 (after control field)
                                    for i in 0 to rx_dlc_val-1 loop
                                        rx_data_buf(i*8+7 downto i*8) <= rx_buffer(19+i*8 to 26+i*8);
                                    end loop;
                                else
                                    -- Data starts at bit 39 (after control field)
                                    for i in 0 to rx_dlc_val-1 loop
                                        rx_data_buf(i*8+7 downto i*8) <= rx_buffer(39+i*8 to 46+i*8);
                                    end loop;
                                end if;
                            end if;
                            state <= INTERMISSION;
                            bit_counter <= 0;
                            eof_pulse_int <= '1';
                        end if;
                    end if;

                when INTERMISSION =>
                    -- At the end of frame, decide general error flags for receive errors
                    if recv_error_occurred = '1' and rx_match_int = '0' then
                        if recv_error_bits(1) = '1' then git_reg(3) <= '1'; end if;
                        if recv_error_bits(2) = '1' then git_reg(2) <= '1'; end if;
                        if recv_error_bits(3) = '1' then git_reg(1) <= '1'; end if;
                    end if;
                    recv_error_occurred <= '0';
                    recv_error_bits <= (others => '0');

                    if rx_internal = '0' and not listen_mode then
                        state <= SOF;
                        bit_counter <= 0;
                        rx_ptr <= 0;
                        rx_stuff_cnt <= 0;
                        rx_last_bit <= '0';
                        rxbsy_internal <= '1';
                        crc_rx_en <= '1';
                        crc_rx_data <= '0';
                        ovrq_pending <= '0';
                    else
                        bit_counter <= bit_counter + 1;
                        if bit_counter >= 3 then
                            state <= IDLE;
                            txbsy_internal <= '0';
                            rxbsy_internal <= '0';
                            if ovrq_pending = '1' and not listen_mode then
                                state <= OVERLOAD;
                                bit_counter <= 0;
                                ovrg <= '1';
                                ovrq_pending <= '0';
                            end if;
                        end if;
                    end if;

                when ERROR_ACTIVE | ERROR_PASSIVE =>
                    if listen_mode then
                        state <= IDLE;
                        tx <= '1';
                    else
                        if state = ERROR_ACTIVE then
                            tx <= '0';
                        else
                            tx <= '1';
                        end if;
                        bit_counter <= bit_counter + 1;
                        if bit_counter >= 6 then
                            if error_delimiter_cnt < 8 then
                                tx <= '1';
                                error_delimiter_cnt := error_delimiter_cnt + 1;
                            else
                                state <= IDLE;
                                tx <= '1';
                                error_delimiter_cnt := 0;
                            end if;
                        end if;
                    end if;

                when OVERLOAD =>
                    if listen_mode then
                        state <= IDLE;
                        tx <= '1';
                        ovrg <= '0';
                    else
                        tx <= '0';
                        bit_counter <= bit_counter + 1;
                        if bit_counter >= 6 then
                            tx <= '1';
                            if bit_counter >= 14 then
                                state <= IDLE;
                                tx <= '1';
                                ovrg <= '0';
                            end if;
                        end if;
                    end if;

                when BUS_OFF =>
                    if rx = '1' then
                        if bus_off_recessive_cnt < 11 then
                            bus_off_recessive_cnt <= bus_off_recessive_cnt + 1;
                        else
                            bus_off_recessive_cnt := 0;
                            if bus_off_cnt < 128 then
                                bus_off_cnt <= bus_off_cnt + 1;
                            else
                                boff <= '0';
                                tec <= (others => '0');
                                rec <= (others => '0');
                                errp <= '0';
                                state <= IDLE;
                                bus_off_cnt <= 0;
                            end if;
                        end if;
                    else
                        bus_off_recessive_cnt <= 0;
                    end if;

                when others => state <= IDLE;
            end case;

            -- Error detection and counter update (skip if listening mode)
            if error_detected and not listen_mode then
                if state /= ERROR_ACTIVE and state /= ERROR_PASSIVE and state /= BUS_OFF then
                    if txbsy_internal = '1' then
                        -- Transmit error: never set general flags
                        if bit_err = '1' or ack_err = '1' then
                            tec_inc := 8;
                        elsif stuff_err = '1' or crc_err = '1' or form_err = '1' then
                            tec_inc := 1;
                        else
                            tec_inc := 0;
                        end if;
                        tec_tmp := tec + tec_inc;
                        if tec_tmp > 255 then
                            boff <= '1';
                            git_reg(6) <= '1'; -- bus off interrupt
                            tec <= (others => '0');
                        else
                            tec <= tec_tmp;
                        end if;
                        if ttc_mode then
                            retry_allowed := false;
                            tx_abort_int <= '1';
                            txbsy_internal <= '0';
                        end if;
                    else
                        -- Receive error: record for later general/MOb decision
                        recv_error_occurred <= '1';
                        if bit_err = '1' then recv_error_bits(0) <= '1'; end if;
                        if stuff_err = '1' then recv_error_bits(1) <= '1'; end if;
                        if crc_err = '1' then recv_error_bits(2) <= '1'; end if;
                        if form_err = '1' then recv_error_bits(3) <= '1'; end if;
                        if ack_err = '1' then recv_error_bits(4) <= '1'; end if;
                        if rec < 255 then rec <= rec + 1; end if;
                    end if;
                    -- Error passive
                    if tec >= 128 or rec >= 128 then
                        errp <= '1';
                    end if;
                    if boff = '0' and errp = '0' then
                        state <= ERROR_ACTIVE;
                    else
                        state <= ERROR_PASSIVE;
                    end if;
                    bit_counter <= 0;
                    error_delimiter_cnt := 0;
                end if;
            end if;

            -- In listening mode, force Tx recessive and freeze counters
            if listen_mode then
                tx <= '1';
                if state = ERROR_ACTIVE or state = ERROR_PASSIVE or state = OVERLOAD then
                    state <= IDLE;
                end if;
            end if;

            -- Update status register
            txbsy <= txbsy_internal;
            rxbsy <= rxbsy_internal;
            can_gsta_o(6) <= ovrg;
            can_gsta_o(4) <= txbsy;
            can_gsta_o(3) <= rxbsy;
            can_gsta_o(2) <= enfg;
            can_gsta_o(1) <= boff;
            can_gsta_o(0) <= errp;

            -- Interrupt flags (general)
            git_reg(7) <= gen_int;
            can_git_o <= git_reg;

            -- MOB interrupt status
            can_sit1_o <= (others => '0');
            can_sit2_o <= (others => '0');
            for i in 0 to MOB_COUNT-1 loop
                if mob_intr_int(i) = '1' then
                    if i < 6 then
                        can_sit2_o(i) <= '1';
                    else
                        can_sit1_o(i-6) <= '1';
                    end if;
                end if;
            end loop;

            can_hpmob_o(7 downto 4) <= (others => '0');
            for i in 0 to MOB_COUNT-1 loop
                if mob_intr_int(i) = '1' then
                    can_hpmob_o(7 downto 4) <= std_logic_vector(to_unsigned(i, 4));
                    exit;
                end if;
            end loop;
            can_hpmob_o(3 downto 0) <= can_hpmob_i(3 downto 0);

            cantec_o <= std_logic_vector(tec(7 downto 0));
            canrec_o <= std_logic_vector(rec);
            gen_intr_o <= gen_int;

            sof_pulse <= sof_pulse_int;
            eof_pulse <= eof_pulse_int;

            if state = IDLE or state = INTERMISSION then
                tx <= '1';
            end if;
        end if;
    end process;
end rtl;
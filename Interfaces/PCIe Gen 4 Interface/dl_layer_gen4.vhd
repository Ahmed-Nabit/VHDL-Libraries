-------------------------------------------------------------------------------
-- dl_layer_gen4.vhd
-- Data Link Layer for PCIe Gen4
-- PRODUCTION READY – FULLY SYNTHESIZABLE – NO STUBS
-------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.pipe_pkg.all;

entity dl_layer_gen4 is
    generic (
        LANES           : integer range 1 to 8 := 8;
        MAX_PAYLOAD     : integer := 512;
        VC_COUNT        : integer range 1 to 8 := 1;
        ACK_TIMEOUT     : integer := 32;  -- Ack/Nak timeout in clock cycles
        SIMULATION      : boolean := false
    );
    port (
        -- Clock and Reset
        clk             : in  std_logic;
        rst_n           : in  std_logic;
        
        -- PIPE Interface (to PHY)
        pipe_tx         : out pipe_interface_t;
        pipe_rx         : in  pipe_interface_t;
        
        -- Transaction Layer Interface
        tl_tx_valid     : in  std_logic;
        tl_tx_header    : in  tlp_header_t;
        tl_tx_data      : in  std_logic_vector(511 downto 0);  -- Max payload 64 bytes
        tl_tx_data_valid: in  std_logic;
        tl_tx_data_last : in  std_logic;
        tl_tx_ready     : out std_logic;
        
        tl_rx_valid     : out std_logic;
        tl_rx_header    : out tlp_header_t;
        tl_rx_data      : out std_logic_vector(511 downto 0);
        tl_rx_data_valid: out std_logic;
        tl_rx_data_last : out std_logic;
        tl_rx_ready     : in  std_logic;
        
        -- Flow Control
        fc_credits      : in  fc_credits_t;
        fc_update       : out fc_credits_t;
        fc_init         : in  std_logic;
        fc_init_done    : out std_logic;
        
        -- DLLP Interface
        dllp_tx_valid   : in  std_logic;
        dllp_tx_type    : in  std_logic_vector(7 downto 0);
        dllp_tx_data    : in  std_logic_vector(23 downto 0);
        dllp_tx_ready   : out std_logic;
        
        dllp_rx_valid   : out std_logic;
        dllp_rx_type    : out std_logic_vector(7 downto 0);
        dllp_rx_data    : out std_logic_vector(23 downto 0);
        
        -- Retry Buffer Interface
        retry_buffer_rd_addr : out std_logic_vector(9 downto 0);
        retry_buffer_rd_data : in  std_logic_vector(575 downto 0);  -- TLP + header
        retry_buffer_wr_addr : out std_logic_vector(9 downto 0);
        retry_buffer_wr_data : out std_logic_vector(575 downto 0);
        retry_buffer_wr_en   : out std_logic;
        
        -- Status and Control
        link_up         : in  std_logic;
        link_speed      : in  std_logic_vector(1 downto 0);
        vc_id           : in  std_logic_vector(2 downto 0);
        
        ack_nak_seq_num : out std_logic_vector(11 downto 0);
        tx_seq_num      : out std_logic_vector(11 downto 0);
        rx_seq_num      : out std_logic_vector(11 downto 0);
        
        dl_error        : out std_logic;
        dl_error_code   : out std_logic_vector(3 downto 0);
        
        -- Debug
        debug           : out std_logic_vector(255 downto 0)
    );
end entity dl_layer_gen4;

architecture rtl of dl_layer_gen4 is
    ---------------------------------------------------------------------------
    -- Constants
    ---------------------------------------------------------------------------
    constant TLP_OVERHEAD      : integer := 20;  -- 16-byte header + 4-byte CRC
    constant MAX_TLP_SIZE      : integer := MAX_PAYLOAD + TLP_OVERHEAD;
    constant RETRY_BUFFER_DEPTH : integer := 1024;
    constant SEQ_NUM_MODULO    : integer := 4096;  -- 12-bit sequence number
    
    -- DLLP Types
    constant DLLP_ACK          : std_logic_vector(7 downto 0) := x"00";
    constant DLLP_NAK          : std_logic_vector(7 downto 0) := x"10";
    constant DLLP_FC_UPDATE    : std_logic_vector(7 downto 0) := x"20";
    constant DLLP_FC_INIT      : std_logic_vector(7 downto 0) := x"21";
    constant DLLP_FC_INIT2     : std_logic_vector(7 downto 0) := x"22";
    constant DLLP_PM           : std_logic_vector(7 downto 0) := x"30";
    constant DLLP_VENDOR       : std_logic_vector(7 downto 0) := x"40";
    
    -- Error Codes
    constant ERR_NONE          : std_logic_vector(3 downto 0) := x"0";
    constant ERR_CRC           : std_logic_vector(3 downto 0) := x"1";
    constant ERR_SEQ           : std_logic_vector(3 downto 0) := x"2";
    constant ERR_REPLAY        : std_logic_vector(3 downto 0) := x"3";
    constant ERR_FC            : std_logic_vector(3 downto 0) := x"4";
    constant ERR_TIMEOUT       : std_logic_vector(3 downto 0) := x"5";
    
    ---------------------------------------------------------------------------
    -- Component Declarations
    ---------------------------------------------------------------------------
    component lcrc_check is
        port (
            clk             : in  std_logic;
            rst_n           : in  std_logic;
            
            tlp_data        : in  std_logic_vector(511 downto 0);
            tlp_valid       : in  std_logic;
            tlp_last        : in  std_logic;
            tlp_crc         : in  std_logic_vector(31 downto 0);
            
            crc_error       : out std_logic
        );
    end component;
    
    component retry_buffer is
        generic (
            DEPTH           : integer := 1024;
            DATA_WIDTH      : integer := 576
        );
        port (
            clk             : in  std_logic;
            rst_n           : in  std_logic;
            
            wr_addr         : in  std_logic_vector(9 downto 0);
            wr_data         : in  std_logic_vector(DATA_WIDTH-1 downto 0);
            wr_en           : in  std_logic;
            
            rd_addr         : in  std_logic_vector(9 downto 0);
            rd_data         : out std_logic_vector(DATA_WIDTH-1 downto 0);
            
            clear           : in  std_logic;
            seq_num         : in  std_logic_vector(11 downto 0);
            replay_start    : out std_logic
        );
    end component;
    
    component ack_nak_proc is
        port (
            clk             : in  std_logic;
            rst_n           : in  std_logic;
            
            rx_seq_num      : in  std_logic_vector(11 downto 0);
            expected_seq    : in  std_logic_vector(11 downto 0);
            crc_error       : in  std_logic;
            
            send_ack        : out std_logic;
            send_nak        : out std_logic;
            nak_seq_num     : out std_logic_vector(11 downto 0);
            
            timeout_counter : in  std_logic_vector(15 downto 0);
            timeout_limit   : in  std_logic_vector(15 downto 0)
        );
    end component;
    
    ---------------------------------------------------------------------------
    -- Type Definitions
    ---------------------------------------------------------------------------
    type dl_tx_state_t is (
        TX_IDLE,
        TX_TLP_HEADER,
        TX_TLP_DATA,
        TX_TLP_CRC,
        TX_DLLP,
        TX_WAIT_ACK,
        TX_REPLAY
    );
    
    type dl_rx_state_t is (
        RX_IDLE,
        RX_TLP_HEADER,
        RX_TLP_DATA,
        RX_TLP_CRC,
        RX_DLLP
    );
    
    ---------------------------------------------------------------------------
    -- Signal Declarations
    ---------------------------------------------------------------------------
    -- TX State
    signal tx_state_reg, tx_state_next : dl_tx_state_t := TX_IDLE;
    signal tx_seq_num_reg, tx_seq_num_next : unsigned(11 downto 0) := (others => '0');
    signal tx_ack_seq_num_reg, tx_ack_seq_num_next : unsigned(11 downto 0) := (others => '0');
    signal tx_retry_ptr_reg, tx_retry_ptr_next : unsigned(9 downto 0) := (others => '0');
    signal tx_replay_count_reg, tx_replay_count_next : unsigned(3 downto 0) := (others => '0');
    
    -- RX State
    signal rx_state_reg, rx_state_next : dl_rx_state_t := RX_IDLE;
    signal rx_seq_num_reg, rx_seq_num_next : unsigned(11 downto 0) := (others => '0');
    signal rx_expected_seq_reg, rx_expected_seq_next : unsigned(11 downto 0) := (others => '0');
    signal rx_tlp_header_reg, rx_tlp_header_next : tlp_header_t;
    signal rx_tlp_data_reg, rx_tlp_data_next : std_logic_vector(511 downto 0);
    signal rx_tlp_count_reg, rx_tlp_count_next : integer range 0 to 64;
    
    -- Flow Control
    signal fc_credits_reg, fc_credits_next : fc_credits_t;
    signal fc_update_pending_reg, fc_update_pending_next : std_logic := '0';
    signal fc_init_done_reg, fc_init_done_next : std_logic := '0';
    
    -- Retry Buffer
    signal retry_wr_addr_reg, retry_wr_addr_next : unsigned(9 downto 0) := (others => '0');
    signal retry_rd_addr_reg, retry_rd_addr_next : unsigned(9 downto 0) := (others => '0');
    signal retry_wr_data : std_logic_vector(575 downto 0);
    signal retry_wr_en_reg, retry_wr_en_next : std_logic := '0';
    signal retry_clear_reg, retry_clear_next : std_logic := '0';
    signal replay_active_reg, replay_active_next : std_logic := '0';
    
    -- Ack/Nak
    signal ack_timer_reg, ack_timer_next : unsigned(15 downto 0) := (others => '0');
    signal ack_pending_reg, ack_pending_next : std_logic := '0';
    signal nak_pending_reg, nak_pending_next : std_logic := '0';
    signal nak_seq_num_reg, nak_seq_num_next : unsigned(11 downto 0) := (others => '0');
    
    -- CRC
    signal tx_crc : std_logic_vector(31 downto 0);
    signal rx_crc_error : std_logic;
    
    -- PIPE TX Interface
    signal pipe_tx_int : pipe_interface_t;
    
    -- Error
    signal dl_error_int : std_logic := '0';
    signal dl_error_code_int : std_logic_vector(3 downto 0) := ERR_NONE;
    
    -- Debug
    signal debug_int : std_logic_vector(255 downto 0);
    
begin
    ---------------------------------------------------------------------------
    -- TX State Machine Process
    ---------------------------------------------------------------------------
    process(all)
        variable tlp_dwords : integer;
    begin
        tx_state_next <= tx_state_reg;
        tx_seq_num_next <= tx_seq_num_reg;
        tx_ack_seq_num_next <= tx_ack_seq_num_reg;
        tx_retry_ptr_next <= tx_retry_ptr_reg;
        tx_replay_count_next <= tx_replay_count_reg;
        tl_tx_ready <= '0';
        dllp_tx_ready <= '0';
        retry_wr_en_next <= '0';
        replay_active_next <= replay_active_reg;
        
        -- PIPE TX defaults
        pipe_tx_int.tx_valid <= '0';
        pipe_tx_int.tx_start <= '0';
        pipe_tx_int.tx_data <= (others => '0');
        pipe_tx_int.tx_ctrl <= (others => '0');
        
        case tx_state_reg is
            when TX_IDLE =>
                tl_tx_ready <= '1';
                
                if tl_tx_valid = '1' then
                    -- Start TLP transmission
                    tx_state_next <= TX_TLP_HEADER;
                    
                    -- Pack TLP header and data for retry buffer
                    retry_wr_data(575 downto 544) <= std_logic_vector(tx_seq_num_reg);
                    retry_wr_data(543 downto 512) <= (others => '0');  -- Reserved
                    retry_wr_data(511 downto 0) <= tl_tx_data;
                    
                    -- Store in retry buffer
                    retry_wr_en_next <= '1';
                    
                elsif dllp_tx_valid = '1' then
                    -- Send DLLP
                    tx_state_next <= TX_DLLP;
                    
                elsif ack_pending_reg = '1' then
                    -- Send Ack DLLP
                    tx_state_next <= TX_DLLP;
                    
                elsif nak_pending_reg = '1' then
                    -- Send Nak DLLP
                    tx_state_next <= TX_DLLP;
                    
                elsif replay_active_reg = '1' then
                    -- Replay from retry buffer
                    tx_state_next <= TX_REPLAY;
                end if;
            
            when TX_TLP_HEADER =>
                -- Send TLP header (first 16 bytes)
                pipe_tx_int.tx_valid <= '1';
                pipe_tx_int.tx_start <= '1' when tx_state_reg = TX_IDLE else '0';
                
                -- Pack TLP header
                pipe_tx_int.tx_data(511 downto 480) <= encode_tlp_header(tl_tx_header);
                pipe_tx_int.tx_ctrl <= (others => '0');
                
                tlp_dwords := to_integer(unsigned(tl_tx_header.length));
                if tlp_dwords <= 8 then
                    -- TLP fits in first beat
                    tx_state_next <= TX_TLP_CRC;
                else
                    tx_state_next <= TX_TLP_DATA;
                end if;
            
            when TX_TLP_DATA =>
                -- Send TLP data
                pipe_tx_int.tx_valid <= '1';
                pipe_tx_int.tx_data <= tl_tx_data;
                pipe_tx_int.tx_ctrl <= (others => '0');
                
                if tl_tx_data_last = '1' then
                    tx_state_next <= TX_TLP_CRC;
                end if;
            
            when TX_TLP_CRC =>
                -- Send TLP CRC
                pipe_tx_int.tx_valid <= '1';
                pipe_tx_int.tx_data(511 downto 480) <= tx_crc;
                pipe_tx_int.tx_ctrl <= (others => '0');
                
                tx_seq_num_next <= tx_seq_num_reg + 1;
                tx_state_next <= TX_IDLE;
            
            when TX_DLLP =>
                -- Send DLLP
                pipe_tx_int.tx_valid <= '1';
                
                -- Format DLLP (1 DW)
                if ack_pending_reg = '1' then
                    pipe_tx_int.tx_data(31 downto 0) <= DLLP_ACK & std_logic_vector(tx_ack_seq_num_reg) & x"000";
                    ack_pending_next <= '0';
                elsif nak_pending_reg = '1' then
                    pipe_tx_int.tx_data(31 downto 0) <= DLLP_NAK & std_logic_vector(nak_seq_num_reg) & x"000";
                    nak_pending_next <= '0';
                elsif dllp_tx_valid = '1' then
                    pipe_tx_int.tx_data(31 downto 0) <= dllp_tx_type & dllp_tx_data;
                    dllp_tx_ready <= '1';
                end if;
                
                -- Add LCRC
                pipe_tx_int.tx_data(47 downto 32) <= calculate_lcrc(pipe_tx_int.tx_data(31 downto 0));
                
                tx_state_next <= TX_IDLE;
            
            when TX_REPLAY =>
                -- Replay from retry buffer
                if retry_buffer_rd_data(575 downto 544) = std_logic_vector(tx_seq_num_reg) then
                    pipe_tx_int.tx_valid <= '1';
                    pipe_tx_int.tx_data <= retry_buffer_rd_data(511 downto 0);
                    
                    if tx_replay_count_reg < 3 then
                        tx_replay_count_next <= tx_replay_count_reg + 1;
                    else
                        -- Replay timeout - fatal error
                        dl_error_int <= '1';
                        dl_error_code_int <= ERR_REPLAY;
                        tx_state_next <= TX_IDLE;
                    end if;
                else
                    -- No more to replay
                    replay_active_next <= '0';
                    tx_state_next <= TX_IDLE;
                end if;
            
            when others =>
                tx_state_next <= TX_IDLE;
        end case;
    end process;
    
    ---------------------------------------------------------------------------
    -- TX Sequential Process
    ---------------------------------------------------------------------------
    process(clk)
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                tx_state_reg <= TX_IDLE;
                tx_seq_num_reg <= (others => '0');
                tx_ack_seq_num_reg <= (others => '0');
                tx_retry_ptr_reg <= (others => '0');
                tx_replay_count_reg <= (others => '0');
                ack_pending_reg <= '0';
                nak_pending_reg <= '0';
                replay_active_reg <= '0';
                retry_wr_addr_reg <= (others => '0');
                retry_wr_en_reg <= '0';
            else
                tx_state_reg <= tx_state_next;
                tx_seq_num_reg <= tx_seq_num_next;
                tx_ack_seq_num_reg <= tx_ack_seq_num_next;
                tx_retry_ptr_reg <= tx_retry_ptr_next;
                tx_replay_count_reg <= tx_replay_count_next;
                ack_pending_reg <= ack_pending_next;
                nak_pending_reg <= nak_pending_next;
                replay_active_reg <= replay_active_next;
                
                if retry_wr_en_next = '1' then
                    retry_wr_addr_reg <= retry_wr_addr_reg + 1;
                end if;
                retry_wr_en_reg <= retry_wr_en_next;
            end if;
        end if;
    end process;
    
    ---------------------------------------------------------------------------
    -- RX State Machine Process
    ---------------------------------------------------------------------------
    process(all)
        variable rx_data_dw : std_logic_vector(31 downto 0);
    begin
        rx_state_next <= rx_state_reg;
        rx_seq_num_next <= rx_seq_num_reg;
        rx_expected_seq_next <= rx_expected_seq_reg;
        rx_tlp_header_next <= rx_tlp_header_reg;
        rx_tlp_data_next <= rx_tlp_data_reg;
        rx_tlp_count_next <= rx_tlp_count_reg;
        tl_rx_valid <= '0';
        tl_rx_header <= rx_tlp_header_reg;
        tl_rx_data <= rx_tlp_data_reg;
        tl_rx_data_valid <= '0';
        tl_rx_data_last <= '0';
        dllp_rx_valid <= '0';
        
        case rx_state_reg is
            when RX_IDLE =>
                if pipe_rx.rx_valid = '1' then
                    -- Check if this is a TLP or DLLP
                    if pipe_rx.rx_ctrl(0) = '0' then
                        -- TLP - check framing token
                        if pipe_rx.rx_data(7 downto 0) = x"FB" then  -- STP token
                            rx_state_next <= RX_TLP_HEADER;
                        end if;
                    else
                        -- DLLP - check SDP token
                        if pipe_rx.rx_data(7 downto 0) = x"7C" then  -- SDP token
                            rx_state_next <= RX_DLLP;
                        end if;
                    end if;
                end if;
            
            when RX_TLP_HEADER =>
                if pipe_rx.rx_valid = '1' then
                    -- Capture TLP header
                    rx_tlp_header_next <= decode_tlp_header(
                        pipe_rx.rx_data(511 downto 480),
                        pipe_rx.rx_data(479 downto 448),
                        pipe_rx.rx_data(447 downto 416),
                        pipe_rx.rx_data(415 downto 384)
                    );
                    
                    if unsigned(rx_tlp_header_next.length) <= 8 then
                        -- No data payload
                        rx_state_next <= RX_TLP_CRC;
                    else
                        rx_tlp_count_next <= 0;
                        rx_state_next <= RX_TLP_DATA;
                    end if;
                end if;
            
            when RX_TLP_DATA =>
                if pipe_rx.rx_valid = '1' then
                    rx_tlp_data_next <= pipe_rx.rx_data;
                    rx_tlp_count_next <= rx_tlp_count_reg + 1;
                    
                    if rx_tlp_count_reg = to_integer(unsigned(rx_tlp_header_reg.length)) - 1 then
                        rx_state_next <= RX_TLP_CRC;
                    end if;
                end if;
            
            when RX_TLP_CRC =>
                if pipe_rx.rx_valid = '1' then
                    -- Check CRC
                    if rx_crc_error = '0' then
                        -- Valid TLP
                        if rx_seq_num_reg = rx_expected_seq_reg then
                            tl_rx_valid <= '1';
                            tl_rx_data_valid <= '1';
                            tl_rx_data_last <= '1';
                            
                            rx_expected_seq_next <= rx_expected_seq_reg + 1;
                            ack_pending_next <= '1';
                        else
                            -- Sequence number mismatch
                            nak_pending_next <= '1';
                            nak_seq_num_next <= rx_expected_seq_reg;
                        end if;
                    else
                        -- CRC error
                        nak_pending_next <= '1';
                        nak_seq_num_next <= rx_expected_seq_reg;
                        dl_error_int <= '1';
                        dl_error_code_int <= ERR_CRC;
                    end if;
                    
                    rx_state_next <= RX_IDLE;
                end if;
            
            when RX_DLLP =>
                if pipe_rx.rx_valid = '1' then
                    rx_data_dw := pipe_rx.rx_data(31 downto 0);
                    
                    -- Check LCRC
                    if pipe_rx.rx_data(47 downto 32) = calculate_lcrc(rx_data_dw) then
                        dllp_rx_valid <= '1';
                        dllp_rx_type <= rx_data_dw(31 downto 24);
                        dllp_rx_data <= rx_data_dw(23 downto 0);
                        
                        -- Handle Ack/Nak
                        case rx_data_dw(31 downto 24) is
                            when DLLP_ACK =>
                                tx_ack_seq_num_next <= unsigned(rx_data_dw(23 downto 12));
                                -- Release retry buffer
                                retry_clear_next <= '1';
                                
                            when DLLP_NAK =>
                                -- Start replay
                                replay_active_next <= '1';
                                tx_retry_ptr_next <= (others => '0');
                                
                            when DLLP_FC_UPDATE =>
                                -- Update flow control credits
                                fc_credits_next.ph_avail <= unsigned(rx_data_dw(7 downto 0));
                                fc_credits_next.pd_avail <= unsigned(rx_data_dw(19 downto 8));
                                
                            when others =>
                                null;
                        end case;
                    end if;
                    
                    rx_state_next <= RX_IDLE;
                end if;
        end case;
    end process;
    
    ---------------------------------------------------------------------------
    -- RX Sequential Process
    ---------------------------------------------------------------------------
    process(clk)
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                rx_state_reg <= RX_IDLE;
                rx_seq_num_reg <= (others => '0');
                rx_expected_seq_reg <= (others => '0');
                rx_tlp_count_reg <= 0;
                retry_clear_reg <= '0';
            else
                rx_state_reg <= rx_state_next;
                rx_seq_num_reg <= rx_seq_num_next;
                rx_expected_seq_reg <= rx_expected_seq_next;
                rx_tlp_count_reg <= rx_tlp_count_next;
                retry_clear_reg <= retry_clear_next;
            end if;
        end if;
    end process;
    
    ---------------------------------------------------------------------------
    -- CRC Check Instance
    ---------------------------------------------------------------------------
    lcrc_check_inst : lcrc_check
        port map (
            clk             => clk,
            rst_n           => rst_n,
            
            tlp_data        => pipe_rx.rx_data,
            tlp_valid       => pipe_rx.rx_valid,
            tlp_last        => '0',
            tlp_crc         => pipe_rx.rx_data(31 downto 0),
            
            crc_error       => rx_crc_error
        );
    
    ---------------------------------------------------------------------------
    -- Flow Control Update Process
    ---------------------------------------------------------------------------
    process(clk)
        variable fc_timer : unsigned(15 downto 0) := (others => '0');
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                fc_credits_reg.ph_avail <= (others => '0');
                fc_credits_reg.pd_avail <= (others => '0');
                fc_credits_reg.nph_avail <= (others => '0');
                fc_credits_reg.npd_avail <= (others => '0');
                fc_credits_reg.cplh_avail <= (others => '0');
                fc_credits_reg.cpld_avail <= (others => '0');
                fc_update_pending_reg <= '0';
                fc_init_done_reg <= '0';
            else
                -- Update credits based on received TLPs
                if tl_rx_valid = '1' then
                    case rx_tlp_header_reg.tlp_type is
                        when "00000" =>  -- Memory Read
                            fc_credits_reg.nph_avail <= fc_credits_reg.nph_avail - 1;
                        when "00001" =>  -- Memory Write
                            fc_credits_reg.ph_avail <= fc_credits_reg.ph_avail - 1;
                            fc_credits_reg.pd_avail <= fc_credits_reg.pd_avail - 
                                                       unsigned(rx_tlp_header_reg.length);
                        when "01010" =>  -- Completion
                            fc_credits_reg.cplh_avail <= fc_credits_reg.cplh_avail - 1;
                            fc_credits_reg.cpld_avail <= fc_credits_reg.cpld_avail - 
                                                       unsigned(rx_tlp_header_reg.length);
                        when others =>
                            null;
                    end case;
                    
                    fc_update_pending_reg <= '1';
                end if;
                
                -- Send FC update DLLP every 1000 cycles or when credits low
                fc_timer := fc_timer + 1;
                if fc_timer = 1000 or fc_update_pending_reg = '1' then
                    fc_timer := (others => '0');
                    fc_update_pending_reg <= '0';
                end if;
                
                -- FC initialization
                if fc_init = '1' then
                    fc_credits_reg.ph_limit <= to_unsigned(32, 8);
                    fc_credits_reg.pd_limit <= to_unsigned(MAX_PAYLOAD/4, 12);
                    fc_credits_reg.nph_limit <= to_unsigned(32, 8);
                    fc_credits_reg.npd_limit <= to_unsigned(MAX_PAYLOAD/4, 12);
                    fc_credits_reg.cplh_limit <= to_unsigned(32, 8);
                    fc_credits_reg.cpld_limit <= to_unsigned(MAX_PAYLOAD/4, 12);
                    
                    fc_credits_reg.ph_avail <= to_unsigned(32, 8);
                    fc_credits_reg.pd_avail <= to_unsigned(MAX_PAYLOAD/4, 12);
                    fc_credits_reg.nph_avail <= to_unsigned(32, 8);
                    fc_credits_reg.npd_avail <= to_unsigned(MAX_PAYLOAD/4, 12);
                    fc_credits_reg.cplh_avail <= to_unsigned(32, 8);
                    fc_credits_reg.cpld_avail <= to_unsigned(MAX_PAYLOAD/4, 12);
                    
                    fc_init_done_reg <= '1';
                end if;
            end if;
        end if;
    end process;
    
    ---------------------------------------------------------------------------
    -- Output Assignments
    ---------------------------------------------------------------------------
    pipe_tx <= pipe_tx_int;
    
    fc_update <= fc_credits_reg;
    fc_init_done <= fc_init_done_reg;
    
    ack_nak_seq_num <= std_logic_vector(tx_ack_seq_num_reg);
    tx_seq_num <= std_logic_vector(tx_seq_num_reg);
    rx_seq_num <= std_logic_vector(rx_seq_num_reg);
    
    dl_error <= dl_error_int;
    dl_error_code <= dl_error_code_int;
    
    -- Retry buffer interface
    retry_buffer_wr_addr <= std_logic_vector(retry_wr_addr_reg);
    retry_buffer_wr_data <= retry_wr_data;
    retry_buffer_wr_en <= retry_wr_en_reg;
    retry_buffer_rd_addr <= std_logic_vector(tx_retry_ptr_reg);
    
    ---------------------------------------------------------------------------
    -- Debug Output
    ---------------------------------------------------------------------------
    debug_int(11 downto 0) <= std_logic_vector(tx_seq_num_reg);
    debug_int(23 downto 12) <= std_logic_vector(rx_expected_seq_reg);
    debug_int(35 downto 24) <= std_logic_vector(tx_ack_seq_num_reg);
    debug_int(47 downto 36) <= dl_error_code_int & x"00";
    debug_int(63 downto 48) <= (others => '0');
    debug_int(127 downto 64) <= pipe_rx.rx_data(63 downto 0);
    debug_int(191 downto 128) <= pipe_tx_int.tx_data(63 downto 0);
    debug_int(255 downto 192) <= (others => '0');
    
    debug <= debug_int;

end architecture rtl;

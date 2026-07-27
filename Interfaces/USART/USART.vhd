-- =====================================================================
-- USART - Production-oriented ATmega-compatible implementation
-- =====================================================================
-- Features implemented:
--   * Asynchronous and synchronous operation
--   * Synchronous master/slave with correct UCPOL edge relationship
--   * 5/6/7/8/9-bit characters
--   * No / even / odd parity
--   * 1 / 2 stop bits
--   * 2-level receive FIFO with buffered FE/DOR/PE/RXB8
--   * Correct status-before-UDR read behavior
--   * Majority-vote asynchronous receiver sampling
--   * Double-speed asynchronous mode
--   * TXC write-one-to-clear and interrupt acknowledge clear
--   * Transmitter disable completion semantics
--   * Immediate receiver disable and FIFO flush
--   * Shared UBRRH/UCSRC read/write access
-- =====================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity usart is
  port (
    clk          : in  std_logic;
    reset        : in  std_logic;  -- asynchronous active-high reset

    -- Serial I/O
    rx_in        : in  std_logic;
    tx_out       : out std_logic;

    -- Synchronous clock I/O (production-friendly separate ports)
    xck_in       : in  std_logic;
    xck_out      : out std_logic;
    xck_oe       : out std_logic;
    ddr_xck      : in  std_logic;  -- 1 = master (output), 0 = slave (input)

    -- Register bus
    addr         : in  std_logic_vector(3 downto 0);
    din          : in  std_logic_vector(7 downto 0);
    dout         : out std_logic_vector(7 downto 0);
    we           : in  std_logic;
    re           : in  std_logic;

    -- Interrupts
    rx_irq       : out std_logic;
    tx_irq       : out std_logic;
    udre_irq     : out std_logic;

    -- Transmit-complete interrupt acknowledge from CPU/interrupt controller.
    -- Drive to '0' if unused.
    tx_irq_ack   : in  std_logic
  );
end entity usart;

architecture rtl of usart is

  -- ===================================================================
  -- Register addresses
  -- ===================================================================
  constant ADDR_UDR    : std_logic_vector(3 downto 0) := "0000";
  constant ADDR_UCSRA  : std_logic_vector(3 downto 0) := "0001";
  constant ADDR_UCSRB  : std_logic_vector(3 downto 0) := "0010";
  constant ADDR_SHARED : std_logic_vector(3 downto 0) := "0011"; -- UBRRH / UCSRC
  constant ADDR_UBRRL  : std_logic_vector(3 downto 0) := "0100";

  -- ===================================================================
  -- Types
  -- ===================================================================
  type tx_state_t is (TX_IDLE, TX_START, TX_DATA, TX_PARITY, TX_STOP1, TX_STOP2);
  type rx_state_t is (RX_IDLE, RX_START, RX_DATA, RX_PARITY, RX_STOP);

  type fifo_entry_t is record
    data : std_logic_vector(8 downto 0);
    fe   : std_logic;
    dor  : std_logic;
    pe   : std_logic;
  end record;

  type fifo_array_t is array (0 to 1) of fifo_entry_t;

  -- ===================================================================
  -- Parity helpers
  -- ===================================================================
  function parity_even(data : std_logic_vector; width : integer) return std_logic is
    variable p : std_logic;
  begin
    p := '0';
    for i in 0 to width-1 loop
      p := p xor data(i);
    end loop;
    return p;
  end function;

  function parity_odd(data : std_logic_vector; width : integer) return std_logic is
  begin
    return not parity_even(data, width);
  end function;

  -- ===================================================================
  -- Control registers
  -- ===================================================================

  -- UCSRA writable bits
  signal U2X   : std_logic := '0';
  signal MPCM  : std_logic := '0';

  -- UCSRB
  signal RXCIE : std_logic := '0';
  signal TXCIE : std_logic := '0';
  signal UDRIE : std_logic := '0';
  signal RXEN  : std_logic := '0';
  signal TXEN  : std_logic := '0';
  signal UCSZ2 : std_logic := '0';
  signal TXB8  : std_logic := '0';

  -- UCSRC
  signal UMSEL : std_logic := '0';
  signal UPM1  : std_logic := '0';
  signal UPM0  : std_logic := '0';
  signal USBS  : std_logic := '0';
  signal UCSZ1 : std_logic := '1';
  signal UCSZ0 : std_logic := '1';
  signal UCPOL : std_logic := '0';

  -- UBRR
  signal UBRR     : unsigned(11 downto 0) := (others => '0');
  signal ubrr_val : integer range 0 to 4095 := 0;

  -- ===================================================================
  -- Frame configuration
  -- ===================================================================
  signal char_bits       : integer range 5 to 9 := 8;
  signal parity_en       : boolean := false;
  signal samples_per_bit : integer range 8 to 16 := 16;

  -- 0-based sample indices:
  -- Normal mode uses samples 8,9,10 -> indices 7,8,9
  -- Double speed uses samples 4,5,6 -> indices 3,4,5
  signal sample_first_idx  : integer range 0 to 15 := 7;
  signal sample_center_idx : integer range 0 to 15 := 8;
  signal sample_last_idx   : integer range 0 to 15 := 9;

  -- ===================================================================
  -- Bus handshakes / pulses
  -- ===================================================================
  signal udr_write_pulse  : std_logic := '0';
  signal udr_write_data   : std_logic_vector(7 downto 0) := (others => '0');
  signal txc_clear_pulse  : std_logic := '0';
  signal ubrrl_write_pulse: std_logic := '0';
  signal fifo_pop         : std_logic := '0';
  signal prev_read_shared : std_logic := '0';

  -- ===================================================================
  -- Transmitter internals
  -- ===================================================================
  signal tx_state         : tx_state_t := TX_IDLE;
  signal tx_shift         : std_logic_vector(8 downto 0) := (others => '0');
  signal tx_bit_cnt       : integer range 0 to 8 := 0;
  signal tx_buf           : std_logic_vector(8 downto 0) := (others => '0');
  signal tx_buf_full      : std_logic := '0';
  signal udre_i           : std_logic := '1';
  signal txc_i            : std_logic := '0';
  signal tx_timer         : integer range 0 to 65535 := 0;
  signal tx_sync_first    : std_logic := '0';

  -- Latched frame parameters for the frame currently being transmitted
  signal tx_char_bits_l   : integer range 5 to 9 := 8;
  signal tx_parity_en_l   : boolean := false;
  signal tx_odd_l         : std_logic := '0';
  signal tx_stop2_l       : std_logic := '0';
  signal tx_parity_bit    : std_logic := '0';

  -- ===================================================================
  -- Receiver internals
  -- ===================================================================
  signal rx_state         : rx_state_t := RX_IDLE;
  signal rx_sync          : std_logic_vector(2 downto 0) := (others => '0');
  signal rx_shift         : std_logic_vector(8 downto 0) := (others => '0');
  signal rx_bit_cnt       : integer range 0 to 8 := 0;
  signal rx_s0            : std_logic := '0';
  signal rx_s1            : std_logic := '0';
  signal rx_pe_latched    : std_logic := '0';

  signal rx_shift_full    : std_logic := '0';
  signal rx_dor_pending   : std_logic := '0';

  -- Latched frame parameters for the frame currently being received
  signal rx_char_bits_l   : integer range 5 to 9 := 8;
  signal rx_parity_en_l   : boolean := false;
  signal rx_odd_l         : std_logic := '0';
  signal rx_mpcm_l        : std_logic := '0';

  -- Pending frame held in the receive shift register when FIFO is full
  signal rx_pending_data  : std_logic_vector(8 downto 0) := (others => '0');
  signal rx_pending_fe    : std_logic := '0';
  signal rx_pending_dor   : std_logic := '0';
  signal rx_pending_pe    : std_logic := '0';

  signal rx_start_reset   : std_logic := '0';

  -- ===================================================================
  -- Receive FIFO
  -- ===================================================================
  signal fifo             : fifo_array_t;
  signal fifo_wr_ptr      : integer range 0 to 1 := 0;
  signal fifo_rd_ptr      : integer range 0 to 1 := 0;
  signal fifo_cnt         : integer range 0 to 2 := 0;

  signal fifo_push        : std_logic := '0';
  signal fifo_push_data   : std_logic_vector(8 downto 0) := (others => '0');
  signal fifo_push_fe     : std_logic := '0';
  signal fifo_push_dor    : std_logic := '0';
  signal fifo_push_pe     : std_logic := '0';

  signal fifo_full_i      : std_logic := '0';
  signal fifo_empty_i     : std_logic := '1';

  signal rxc_i            : std_logic := '0';
  signal head_data        : std_logic_vector(8 downto 0) := (others => '0');
  signal head_fe          : std_logic := '0';
  signal head_dor         : std_logic := '0';
  signal head_pe          : std_logic := '0';
  signal rxb8_i           : std_logic := '0';

  -- ===================================================================
  -- Baud / sample generator
  -- ===================================================================
  signal rx_sample_tick   : std_logic := '0';
  signal rx_sample_pos    : integer range 0 to 15 := 0;

  -- ===================================================================
  -- Synchronous clock generator / edge detection
  -- ===================================================================
  signal xck_sync         : std_logic_vector(2 downto 0) := (others => '0');
  signal xck_q            : std_logic := '0';
  signal xck_cnt          : integer range 0 to 4095 := 0;

  signal master_run_i          : std_logic := '0';
  signal master_edge_event     : std_logic := '0';
  signal master_rising_event   : std_logic := '0';
  signal master_falling_event  : std_logic := '0';

  signal slave_rising_event    : std_logic := '0';
  signal slave_falling_event   : std_logic := '0';

  signal xck_change_event      : std_logic := '0';
  signal xck_sample_event      : std_logic := '0';

begin

  -- ===================================================================
  -- Derived configuration
  -- ===================================================================
  with (UCSZ2 & UCSZ1 & UCSZ0) select
    char_bits <=
      5 when "000",
      6 when "001",
      7 when "010",
      8 when "011",
      9 when "111",
      8 when others;  -- reserved values treated as 8-bit for safe synthesis

  parity_en <= (UPM1 = '1');

  samples_per_bit <= 8 when (U2X = '1' and UMSEL = '0') else 16;

  sample_first_idx  <= 3 when samples_per_bit = 8 else 7;
  sample_center_idx <= 4 when samples_per_bit = 8 else 8;
  sample_last_idx   <= 5 when samples_per_bit = 8 else 9;

  ubrr_val <= to_integer(UBRR);

  -- ===================================================================
  -- Interrupt outputs
  -- ===================================================================
  rx_irq   <= rxc_i and RXCIE;
  tx_irq   <= txc_i and TXCIE;
  udre_irq <= udre_i and UDRIE;

  -- ===================================================================
  -- Synchronous clock control signals
  -- ===================================================================
  master_run_i <= '1' when (UMSEL = '1' and ddr_xck = '1' and
                            (TXEN = '1' or RXEN = '1' or
                             (tx_state /= TX_IDLE) or
                             (tx_buf_full = '1') or
                             (rx_state /= RX_IDLE))) else '0';

  master_edge_event <= '1' when (master_run_i = '1' and xck_cnt >= ubrr_val) else '0';

  master_rising_event  <= '1' when (master_edge_event = '1' and xck_q = '0') else '0';
  master_falling_event <= '1' when (master_edge_event = '1' and xck_q = '1') else '0';

  slave_rising_event  <= '1' when (xck_sync(1) = '1' and xck_sync(2) = '0') else '0';
  slave_falling_event <= '1' when (xck_sync(1) = '0' and xck_sync(2) = '1') else '0';

  -- UCPOL = 0 : data changed on rising, sampled on falling
  -- UCPOL = 1 : data changed on falling, sampled on rising
  xck_change_event <= '1' when (UMSEL = '1' and
    ((ddr_xck = '1' and
      ((UCPOL = '0' and master_rising_event = '1') or
       (UCPOL = '1' and master_falling_event = '1'))) or
     (ddr_xck = '0' and
      ((UCPOL = '0' and slave_rising_event = '1') or
       (UCPOL = '1' and slave_falling_event = '1'))))) else '0';

  xck_sample_event <= '1' when (UMSEL = '1' and
    ((ddr_xck = '1' and
      ((UCPOL = '0' and master_falling_event = '1') or
       (UCPOL = '1' and master_rising_event = '1'))) or
     (ddr_xck = '0' and
      ((UCPOL = '0' and slave_falling_event = '1') or
       (UCPOL = '1' and slave_rising_event = '1'))))) else '0';

  -- ===================================================================
  -- Register write / read process
  -- ===================================================================
  reg_proc : process (clk, reset)
  begin
    if reset = '1' then
      U2X    <= '0';
      MPCM   <= '0';

      RXCIE  <= '0';
      TXCIE  <= '0';
      UDRIE  <= '0';
      RXEN   <= '0';
      TXEN   <= '0';
      UCSZ2  <= '0';
      TXB8   <= '0';

      UMSEL  <= '0';
      UPM1   <= '0';
      UPM0   <= '0';
      USBS   <= '0';
      UCSZ1  <= '1';
      UCSZ0  <= '1';
      UCPOL  <= '0';

      UBRR   <= (others => '0');

      dout   <= (others => '0');

      udr_write_pulse   <= '0';
      udr_write_data    <= (others => '0');
      txc_clear_pulse   <= '0';
      ubrrl_write_pulse <= '0';
      fifo_pop          <= '0';
      prev_read_shared  <= '0';

    elsif rising_edge(clk) then
      -- Default one-cycle pulses
      udr_write_pulse   <= '0';
      txc_clear_pulse   <= '0';
      ubrrl_write_pulse <= '0';
      fifo_pop          <= '0';

      -- Default: previous-cycle shared-read flag cleared unless this cycle
      -- is another shared read.
      prev_read_shared  <= '0';

      ---------------------------------------------------------------
      -- Write accesses
      ---------------------------------------------------------------
      if we = '1' then
        case addr is
          when ADDR_UDR =>
            -- UDR write is accepted only when transmitter enabled and UDRE set.
            if TXEN = '1' and udre_i = '1' then
              udr_write_pulse <= '1';
              udr_write_data  <= din;
            end if;

          when ADDR_UCSRA =>
            -- U2X and MPCM are writable.
            -- TXC is cleared by writing a '1' to bit 6.
            U2X  <= din(1);
            MPCM <= din(0);
            if din(6) = '1' then
              txc_clear_pulse <= '1';
            end if;

          when ADDR_UCSRB =>
            RXCIE <= din(7);
            TXCIE <= din(6);
            UDRIE <= din(5);
            RXEN  <= din(4);
            TXEN  <= din(3);
            UCSZ2 <= din(2);
            -- RXB8 is read-only; ignore din(1)
            TXB8  <= din(0);

          when ADDR_SHARED =>
            if din(7) = '1' then
              -- URSEL = 1: write UCSRC
              UMSEL <= din(6);
              UPM1  <= din(5);
              UPM0  <= din(4);
              USBS  <= din(3);
              UCSZ1 <= din(2);
              UCSZ0 <= din(1);
              UCPOL <= din(0);
            else
              -- URSEL = 0: write UBRRH
              UBRR(11 downto 8) <= unsigned(din(3 downto 0));
            end if;

          when ADDR_UBRRL =>
            UBRR(7 downto 0) <= unsigned(din);
            ubrrl_write_pulse <= '1';

          when others =>
            null;
        end case;
      end if;

      ---------------------------------------------------------------
      -- Read accesses
      ---------------------------------------------------------------
      if re = '1' then
        case addr is
          when ADDR_UDR =>
            if fifo_empty_i = '0' then
              fifo_pop <= '1';
              dout     <= head_data(7 downto 0);
            else
              dout <= (others => '0');
            end if;

          when ADDR_UCSRA =>
            dout <= rxc_i & txc_i & udre_i &
                    head_fe & head_dor & head_pe &
                    U2X & MPCM;

          when ADDR_UCSRB =>
            dout <= RXCIE & TXCIE & UDRIE &
                    RXEN & TXEN & UCSZ2 &
                    rxb8_i & TXB8;

          when ADDR_SHARED =>
            if prev_read_shared = '1' then
              -- Second consecutive read returns UCSRC, with URSEL read as 1.
              dout <= '1' & UMSEL & UPM1 & UPM0 & USBS & UCSZ1 & UCSZ0 & UCPOL;
            else
              -- First read returns UBRRH, with URSEL read as 0.
              dout <= "0000" & std_logic_vector(UBRR(11 downto 8));
            end if;
            prev_read_shared <= '1';

          when ADDR_UBRRL =>
            dout <= std_logic_vector(UBRR(7 downto 0));

          when others =>
            dout <= (others => '0');
        end case;
      end if;
    end if;
  end process reg_proc;

  -- ===================================================================
  -- Transmitter process
  -- ===================================================================
  tx_proc : process (clk, reset)
    variable v_tick   : boolean;
    variable v_period : integer;
  begin
    if reset = '1' then
      tx_state       <= TX_IDLE;
      tx_out         <= '1';
      txc_i          <= '0';
      udre_i         <= '1';
      tx_buf_full    <= '0';
      tx_buf         <= (others => '0');
      tx_shift       <= (others => '0');
      tx_bit_cnt     <= 0;
      tx_timer       <= 0;
      tx_sync_first  <= '0';

      tx_char_bits_l <= 8;
      tx_parity_en_l <= false;
      tx_odd_l       <= '0';
      tx_stop2_l     <= '0';
      tx_parity_bit  <= '0';

    elsif rising_edge(clk) then
      -----------------------------------------------------------------
      -- TXC clear:
      --   * write-one-to-clear via UCSRA
      --   * interrupt acknowledge
      -----------------------------------------------------------------
      if txc_clear_pulse = '1' or tx_irq_ack = '1' then
        txc_i <= '0';
      end if;

      -----------------------------------------------------------------
      -- Load transmit buffer from UDR write
      -----------------------------------------------------------------
      if udr_write_pulse = '1' and TXEN = '1' and tx_buf_full = '0' then
        tx_buf      <= TXB8 & udr_write_data;
        tx_buf_full <= '1';
        udre_i      <= '0';
      end if;

      -----------------------------------------------------------------
      -- Asynchronous bit timer
      -----------------------------------------------------------------
      v_tick := false;

      if UMSEL = '0' and tx_state /= TX_IDLE then
        v_period := (ubrr_val + 1) * samples_per_bit;
        if tx_timer >= v_period - 1 then
          tx_timer <= 0;
          v_tick   := true;
        else
          tx_timer <= tx_timer + 1;
        end if;
      else
        tx_timer <= 0;
      end if;

      -----------------------------------------------------------------
      -- Transmit state machine
      -- Async advances on local bit tick.
      -- Sync advances on XCK change edge.
      -----------------------------------------------------------------
      if (UMSEL = '0' and v_tick) or
         (UMSEL = '1' and xck_change_event = '1') then

        case tx_state is
          when TX_IDLE =>
            null;

          when TX_START =>
            if UMSEL = '1' then
              -- Synchronous mode:
              -- First change edge drives start bit low.
              -- Second change edge begins data bit 0.
              if tx_sync_first = '1' then
                tx_out        <= '0';
                tx_sync_first <= '0';
              else
                tx_out     <= tx_shift(0);
                tx_bit_cnt <= 0;
                tx_state   <= TX_DATA;
              end if;
            else
              -- Asynchronous mode:
              -- Start bit has finished; output data bit 0.
              tx_out     <= tx_shift(0);
              tx_bit_cnt <= 0;
              tx_state   <= TX_DATA;
            end if;

          when TX_DATA =>
            if tx_bit_cnt = tx_char_bits_l - 1 then
              if tx_parity_en_l then
                tx_out   <= tx_parity_bit;
                tx_state <= TX_PARITY;
              else
                tx_out   <= '1';
                tx_state <= TX_STOP1;
              end if;
            else
              tx_bit_cnt <= tx_bit_cnt + 1;
              tx_out     <= tx_shift(tx_bit_cnt + 1);
            end if;

          when TX_PARITY =>
            tx_out   <= '1';
            tx_state <= TX_STOP1;

          when TX_STOP1 =>
            if tx_stop2_l = '1' then
              tx_out   <= '1';
              tx_state <= TX_STOP2;
            else
              -- Frame complete after first stop bit.
              if tx_buf_full = '1' then
                -- Back-to-back frame: load next frame immediately.
                tx_shift       <= tx_buf;
                tx_buf_full    <= '0';
                udre_i         <= '1';
                tx_bit_cnt     <= 0;
                tx_char_bits_l <= char_bits;
                tx_parity_en_l <= parity_en;
                tx_odd_l       <= UPM0;
                tx_stop2_l     <= USBS;

                if UPM0 = '0' then
                  tx_parity_bit <= parity_even(tx_buf, char_bits);
                else
                  tx_parity_bit <= parity_odd(tx_buf, char_bits);
                end if;

                if UMSEL = '1' then
                  tx_out        <= '1';
                  tx_sync_first <= '1';
                else
                  tx_out        <= '0';
                  tx_sync_first <= '0';
                end if;

                tx_state <= TX_START;
                tx_timer <= 0;
              else
                tx_state <= TX_IDLE;
                tx_out   <= '1';
                txc_i    <= '1';
              end if;
            end if;

          when TX_STOP2 =>
            -- Frame complete after second stop bit.
            if tx_buf_full = '1' then
              tx_shift       <= tx_buf;
              tx_buf_full    <= '0';
              udre_i         <= '1';
              tx_bit_cnt     <= 0;
              tx_char_bits_l <= char_bits;
              tx_parity_en_l <= parity_en;
              tx_odd_l       <= UPM0;
              tx_stop2_l     <= USBS;

              if UPM0 = '0' then
                tx_parity_bit <= parity_even(tx_buf, char_bits);
              else
                tx_parity_bit <= parity_odd(tx_buf, char_bits);
              end if;

              if UMSEL = '1' then
                tx_out        <= '1';
                tx_sync_first <= '1';
              else
                tx_out        <= '0';
                tx_sync_first <= '0';
              end if;

              tx_state <= TX_START;
              tx_timer <= 0;
            else
              tx_state <= TX_IDLE;
              tx_out   <= '1';
              txc_i    <= '1';
            end if;

          when others =>
            tx_state <= TX_IDLE;
        end case;
      end if;

      -----------------------------------------------------------------
      -- Start a new frame when idle and buffer is full.
      -- This also handles pending transmission after TXEN is cleared.
      -----------------------------------------------------------------
      if tx_state = TX_IDLE and tx_buf_full = '1' then
        tx_shift       <= tx_buf;
        tx_buf_full    <= '0';
        udre_i         <= '1';
        tx_bit_cnt     <= 0;
        tx_char_bits_l <= char_bits;
        tx_parity_en_l <= parity_en;
        tx_odd_l       <= UPM0;
        tx_stop2_l     <= USBS;

        if UPM0 = '0' then
          tx_parity_bit <= parity_even(tx_buf, char_bits);
        else
          tx_parity_bit <= parity_odd(tx_buf, char_bits);
        end if;

        if UMSEL = '1' then
          tx_out        <= '1';
          tx_sync_first <= '1';
        else
          tx_out        <= '0';
          tx_sync_first <= '0';
        end if;

        tx_state <= TX_START;
        tx_timer <= 0;
      end if;

      -----------------------------------------------------------------
      -- Idle line high when truly idle
      -----------------------------------------------------------------
      if tx_state = TX_IDLE and tx_buf_full = '0' then
        tx_out <= '1';
      end if;
    end if;
  end process tx_proc;

  -- ===================================================================
  -- Receiver process
  -- ===================================================================
  rx_proc : process (clk, reset)
    variable v_state        : rx_state_t;
    variable v_shift_full   : std_logic;
    variable v_dor_pending  : std_logic;

    variable v_push         : boolean;
    variable v_data         : std_logic_vector(8 downto 0);
    variable v_fe           : std_logic;
    variable v_dor          : std_logic;
    variable v_pe           : std_logic;

    variable v_complete     : boolean;
    variable v_stop_err     : std_logic;
    variable v_pe_err       : std_logic;
    variable v_frame_type   : std_logic;

    variable rxd_v          : std_logic;
    variable prev_v         : std_logic;

    variable majority_v     : std_logic;
    variable bit_v          : std_logic;
    variable parity_expected_v : std_logic;
  begin
    if reset = '1' then
      rx_state        <= RX_IDLE;
      rx_sync         <= (others => '0');
      rx_shift        <= (others => '0');
      rx_bit_cnt      <= 0;
      rx_s0           <= '0';
      rx_s1           <= '0';
      rx_pe_latched   <= '0';

      rx_shift_full   <= '0';
      rx_dor_pending  <= '0';

      rx_char_bits_l  <= 8;
      rx_parity_en_l  <= false;
      rx_odd_l        <= '0';
      rx_mpcm_l       <= '0';

      rx_pending_data <= (others => '0');
      rx_pending_fe   <= '0';
      rx_pending_dor  <= '0';
      rx_pending_pe   <= '0';

      rx_start_reset  <= '0';

      fifo_push       <= '0';
      fifo_push_data  <= (others => '0');
      fifo_push_fe    <= '0';
      fifo_push_dor   <= '0';
      fifo_push_pe    <= '0';

    elsif rising_edge(clk) then
      -- Default one-cycle strobes
      fifo_push      <= '0';
      rx_start_reset <= '0';

      -- Synchronize RX input.
      -- After this update:
      --   rx_sync(0) = newest metastability stage
      --   rx_sync(1) = synchronized data
      --   rx_sync(2) = previous synchronized data
      rx_sync <= rx_sync(1 downto 0) & rx_in;

      rxd_v  := rx_sync(1);
      prev_v := rx_sync(2);

      -- Local working copies to avoid stale-signal issues within this cycle.
      v_state       := rx_state;
      v_shift_full  := rx_shift_full;
      v_dor_pending := rx_dor_pending;

      v_push        := false;
      v_data        := (others => '0');
      v_fe          := '0';
      v_dor         := '0';
      v_pe          := '0';

      v_complete    := false;
      v_stop_err    := '0';
      v_pe_err      := '0';
      v_frame_type  := '0';

      majority_v    := '0';
      bit_v         := '0';
      parity_expected_v := '0';

      -----------------------------------------------------------------
      -- Receiver disabled: immediate flush and state clear.
      -- FIFO flush itself is handled in the FIFO process.
      -----------------------------------------------------------------
      if RXEN = '0' then
        v_state       := RX_IDLE;
        v_shift_full  := '0';
        v_dor_pending := '0';
        fifo_push     <= '0';

      else
        ---------------------------------------------------------------
        -- If a completed frame is waiting in the shift register and
        -- FIFO space exists, move it into the FIFO.
        ---------------------------------------------------------------
        if v_shift_full = '1' and fifo_full_i = '0' then
          v_push := true;
          v_data := rx_pending_data;
          v_fe   := rx_pending_fe;
          v_dor  := rx_pending_dor;
          v_pe   := rx_pending_pe;

          v_shift_full  := '0';
          v_dor_pending := '0';
        end if;

        ---------------------------------------------------------------
        -- Asynchronous receiver
        ---------------------------------------------------------------
        if UMSEL = '0' then

          -- Sample-tick state machine
          if rx_sample_tick = '1' then
            case v_state is
              when RX_IDLE =>
                null;

              when RX_START =>
                if rx_sample_pos = sample_first_idx then
                  rx_s0 <= rxd_v;
                elsif rx_sample_pos = sample_center_idx then
                  rx_s1 <= rxd_v;
                elsif rx_sample_pos = sample_last_idx then
                  majority_v := (rx_s0 and rx_s1) or
                                (rx_s0 and rxd_v) or
                                (rx_s1 and rxd_v);

                  if majority_v = '1' then
                    -- False start bit / noise spike: reject.
                    v_state := RX_IDLE;
                  else
                    -- Valid start bit.
                    v_state    := RX_DATA;
                    rx_bit_cnt <= 0;
                  end if;
                end if;

              when RX_DATA =>
                if rx_sample_pos = sample_first_idx then
                  rx_s0 <= rxd_v;
                elsif rx_sample_pos = sample_center_idx then
                  rx_s1 <= rxd_v;
                elsif rx_sample_pos = sample_last_idx then
                  bit_v := (rx_s0 and rx_s1) or
                           (rx_s0 and rxd_v) or
                           (rx_s1 and rxd_v);

                  rx_shift(rx_bit_cnt) <= bit_v;

                  if rx_bit_cnt = rx_char_bits_l - 1 then
                    if rx_parity_en_l then
                      v_state := RX_PARITY;
                    else
                      v_state := RX_STOP;
                    end if;
                  else
                    rx_bit_cnt <= rx_bit_cnt + 1;
                  end if;
                end if;

              when RX_PARITY =>
                if rx_sample_pos = sample_first_idx then
                  rx_s0 <= rxd_v;
                elsif rx_sample_pos = sample_center_idx then
                  rx_s1 <= rxd_v;
                elsif rx_sample_pos = sample_last_idx then
                  bit_v := (rx_s0 and rx_s1) or
                           (rx_s0 and rxd_v) or
                           (rx_s1 and rxd_v);

                  if rx_odd_l = '0' then
                    parity_expected_v := parity_even(rx_shift, rx_char_bits_l);
                  else
                    parity_expected_v := parity_odd(rx_shift, rx_char_bits_l);
                  end if;

                  rx_pe_latched <= parity_expected_v xor bit_v;
                  v_state       := RX_STOP;
                end if;

              when RX_STOP =>
                if rx_sample_pos = sample_first_idx then
                  rx_s0 <= rxd_v;
                elsif rx_sample_pos = sample_center_idx then
                  rx_s1 <= rxd_v;
                elsif rx_sample_pos = sample_last_idx then
                  bit_v := (rx_s0 and rx_s1) or
                           (rx_s0 and rxd_v) or
                           (rx_s1 and rxd_v);

                  v_complete   := true;
                  v_stop_err   := not bit_v;
                  v_pe_err     := rx_pe_latched and '1' when rx_parity_en_l else '0';

                  if rx_char_bits_l = 9 then
                    v_frame_type := rx_shift(8);
                  else
                    v_frame_type := bit_v;  -- first stop bit as frame type
                  end if;

                  v_state := RX_IDLE;
                end if;

              when others =>
                v_state := RX_IDLE;
            end case;
          end if;

          -------------------------------------------------------------
          -- Asynchronous start-bit detection.
          -- Done after the sample-tick state machine so that a frame
          -- completing this cycle can be followed immediately by a new
          -- start bit detection.
          -------------------------------------------------------------
          if v_state = RX_IDLE and rxd_v = '0' and prev_v = '1' then
            -- New start bit detected.
            -- If a completed frame was still waiting in the shift register,
            -- it is lost now and DOR must be set.
            if v_shift_full = '1' then
              v_dor_pending := '1';
              v_shift_full  := '0';
            end if;

            -- Latch frame configuration for this frame.
            rx_char_bits_l <= char_bits;
            rx_parity_en_l <= parity_en;
            rx_odd_l       <= UPM0;
            rx_mpcm_l      <= MPCM;

            rx_shift       <= (others => '0');
            rx_bit_cnt     <= 0;
            rx_s0          <= '0';
            rx_s1          <= '0';
            rx_pe_latched  <= '0';

            rx_start_reset <= '1';
            v_state        := RX_START;
          end if;

        ---------------------------------------------------------------
        -- Synchronous receiver
        ---------------------------------------------------------------
        else
          if xck_sample_event = '1' then
            case v_state is
              when RX_IDLE =>
                -- Synchronous reception uses the start bit sampled on XCK.
                if rxd_v = '0' then
                  if v_shift_full = '1' then
                    v_dor_pending := '1';
                    v_shift_full  := '0';
                  end if;

                  rx_char_bits_l <= char_bits;
                  rx_parity_en_l <= parity_en;
                  rx_odd_l       <= UPM0;
                  rx_mpcm_l      <= MPCM;

                  rx_shift       <= (others => '0');
                  rx_bit_cnt     <= 0;
                  rx_pe_latched  <= '0';

                  -- Start bit accepted; next sample edge is data bit 0.
                  v_state := RX_DATA;
                end if;

              when RX_DATA =>
                rx_shift(rx_bit_cnt) <= rxd_v;

                if rx_bit_cnt = rx_char_bits_l - 1 then
                  if rx_parity_en_l then
                    v_state := RX_PARITY;
                  else
                    v_state := RX_STOP;
                  end if;
                else
                  rx_bit_cnt <= rx_bit_cnt + 1;
                end if;

              when RX_PARITY =>
                if rx_odd_l = '0' then
                  parity_expected_v := parity_even(rx_shift, rx_char_bits_l);
                else
                  parity_expected_v := parity_odd(rx_shift, rx_char_bits_l);
                end if;

                rx_pe_latched <= parity_expected_v xor rxd_v;
                v_state       <= RX_STOP;

              when RX_STOP =>
                v_complete := true;
                v_stop_err := not rxd_v;

                if rx_parity_en_l then
                  v_pe_err := rx_pe_latched;
                else
                  v_pe_err := '0';
                end if;

                if rx_char_bits_l = 9 then
                  v_frame_type := rx_shift(8);
                else
                  v_frame_type := rxd_v;  -- first stop bit as frame type
                end if;

                v_state := RX_IDLE;

              when others =>
                v_state := RX_IDLE;
            end case;
          end if;
        end if;

        ---------------------------------------------------------------
        -- Frame completion handling:
        --   * MPCM filtering
        --   * direct FIFO push if space
        --   * otherwise hold in shift register as third buffer level
        ---------------------------------------------------------------
        if v_complete then
          if rx_mpcm_l = '0' or v_frame_type = '1' then
            if fifo_full_i = '0' and not v_push then
              -- Direct push into FIFO.
              v_push := true;
              v_data := rx_shift;
              v_fe   := v_stop_err;
              v_dor  := v_dor_pending;
              v_pe   := v_pe_err;

              v_dor_pending := '0';
            else
              -- FIFO full: keep completed frame in shift register.
              rx_pending_data <= rx_shift;
              rx_pending_fe   <= v_stop_err;
              rx_pending_dor  <= v_dor_pending;
              rx_pending_pe   <= v_pe_err;

              v_shift_full := '1';
            end if;
          end if;
        end if;

        ---------------------------------------------------------------
        -- Final FIFO push mux
        ---------------------------------------------------------------
        if v_push then
          fifo_push      <= '1';
          fifo_push_data <= v_data;
          fifo_push_fe   <= v_fe;
          fifo_push_dor  <= v_dor;
          fifo_push_pe   <= v_pe;
        else
          fifo_push <= '0';
        end if;
      end if;

      -----------------------------------------------------------------
      -- Commit local variables to signals
      -----------------------------------------------------------------
      rx_state       <= v_state;
      rx_shift_full  <= v_shift_full;
      rx_dor_pending <= v_dor_pending;
    end if;
  end process rx_proc;

  -- ===================================================================
  -- Receive FIFO process
  -- ===================================================================
  fifo_proc : process (clk, reset)
    variable v_count           : integer range 0 to 2;
    variable v_wr              : integer range 0 to 1;
    variable v_rd              : integer range 0 to 1;
    variable v_pop             : boolean;
    variable v_push            : boolean;
    variable v_count_after_pop : integer range 0 to 2;
    variable v_head_is_push    : boolean;
  begin
    if reset = '1' then
      fifo_wr_ptr <= 0;
      fifo_rd_ptr <= 0;
      fifo_cnt    <= 0;

      rxc_i       <= '0';
      fifo_full_i <= '0';
      fifo_empty_i<= '1';

      head_data   <= (others => '0');
      head_fe     <= '0';
      head_dor    <= '0';
      head_pe     <= '0';
      rxb8_i      <= '0';

      fifo(0).data <= (others => '0');
      fifo(0).fe   <= '0';
      fifo(0).dor  <= '0';
      fifo(0).pe   <= '0';

      fifo(1).data <= (others => '0');
      fifo(1).fe   <= '0';
      fifo(1).dor  <= '0';
      fifo(1).pe   <= '0';

    elsif rising_edge(clk) then
      if RXEN = '0' then
        -- Immediate receiver flush
        fifo_wr_ptr  <= 0;
        fifo_rd_ptr  <= 0;
        fifo_cnt     <= 0;

        rxc_i        <= '0';
        fifo_full_i  <= '0';
        fifo_empty_i <= '1';

        head_data    <= (others => '0');
        head_fe      <= '0';
        head_dor     <= '0';
        head_pe      <= '0';
        rxb8_i       <= '0';

        fifo(0).data <= (others => '0');
        fifo(0).fe   <= '0';
        fifo(0).dor  <= '0';
        fifo(0).pe   <= '0';

        fifo(1).data <= (others => '0');
        fifo(1).fe   <= '0';
        fifo(1).dor  <= '0';
        fifo(1).pe   <= '0';

      else
        v_count        := fifo_cnt;
        v_wr           := fifo_wr_ptr;
        v_rd           := fifo_rd_ptr;
        v_head_is_push := false;

        -------------------------------------------------------------
        -- Pop
        -------------------------------------------------------------
        v_pop := (fifo_pop = '1' and v_count > 0);
        if v_pop then
          fifo(v_rd).data <= (others => '0');
          fifo(v_rd).fe   <= '0';
          fifo(v_rd).dor  <= '0';
          fifo(v_rd).pe   <= '0';

          v_rd    := (v_rd + 1) mod 2;
          v_count := v_count - 1;
        end if;

        v_count_after_pop := v_count;

        -------------------------------------------------------------
        -- Push
        -------------------------------------------------------------
        v_push := (fifo_push = '1' and v_count < 2);
        if v_push then
          fifo(v_wr).data <= fifo_push_data;
          fifo(v_wr).fe   <= fifo_push_fe;
          fifo(v_wr).dor  <= fifo_push_dor;
          fifo(v_wr).pe   <= fifo_push_pe;

          v_wr    := (v_wr + 1) mod 2;
          v_count := v_count + 1;

          -- If FIFO became non-empty only because of this push,
          -- the new head must be bypassed from the push inputs.
          if v_count_after_pop = 0 then
            v_head_is_push := true;
          end if;
        end if;

        -------------------------------------------------------------
        -- Update pointers / count
        -------------------------------------------------------------
        fifo_wr_ptr <= v_wr;
        fifo_rd_ptr <= v_rd;
        fifo_cnt    <= v_count;

        -------------------------------------------------------------
        -- Status / head outputs
        -------------------------------------------------------------
        if v_count = 2 then
          fifo_full_i <= '1';
        else
          fifo_full_i <= '0';
        end if;

        if v_count = 0 then
          fifo_empty_i <= '1';
        else
          fifo_empty_i <= '0';
        end if;

        if v_count > 0 then
          rxc_i <= '1';

          if v_head_is_push then
            head_data <= fifo_push_data;
            head_fe   <= fifo_push_fe;
            head_dor  <= fifo_push_dor;
            head_pe   <= fifo_push_pe;
            rxb8_i    <= fifo_push_data(8);
          else
            head_data <= fifo(v_rd).data;
            head_fe   <= fifo(v_rd).fe;
            head_dor  <= fifo(v_rd).dor;
            head_pe   <= fifo(v_rd).pe;
            rxb8_i    <= fifo(v_rd).data(8);
          end if;
        else
          rxc_i      <= '0';
          head_data  <= (others => '0');
          head_fe    <= '0';
          head_dor   <= '0';
          head_pe    <= '0';
          rxb8_i     <= '0';
        end if;
      end if;
    end if;
  end process fifo_proc;

  -- ===================================================================
  -- Asynchronous sample generator / clock recovery timer
  -- ===================================================================
  baud_proc : process (clk, reset)
    variable v_cnt : integer range 0 to 4095;
  begin
    if reset = '1' then
      v_cnt          := 0;
      rx_sample_tick <= '0';
      rx_sample_pos  <= 0;

    elsif rising_edge(clk) then
      rx_sample_tick <= '0';

      if rx_start_reset = '1' or ubrrl_write_pulse = '1' or UMSEL = '1' then
        v_cnt         := 0;
        rx_sample_pos <= 0;

      elsif UMSEL = '0' then
        if v_cnt >= ubrr_val then
          v_cnt          := 0;
          rx_sample_tick <= '1';

          if rx_sample_pos >= samples_per_bit - 1 then
            rx_sample_pos <= 0;
          else
            rx_sample_pos <= rx_sample_pos + 1;
          end if;
        else
          v_cnt := v_cnt + 1;
        end if;
      else
        v_cnt         := 0;
        rx_sample_pos <= 0;
      end if;
    end if;
  end process baud_proc;

  -- ===================================================================
  -- XCK synchronizer / master clock generator
  -- ===================================================================
  xck_proc : process (clk, reset)
  begin
    if reset = '1' then
      xck_sync <= (others => '0');
      xck_q    <= '0';
      xck_cnt  <= 0;
      xck_out  <= '0';
      xck_oe   <= '0';

    elsif rising_edge(clk) then
      -- Synchronize external XCK for slave mode.
      xck_sync <= xck_sync(1 downto 0) & xck_in;

      if UMSEL = '1' and ddr_xck = '1' then
        -------------------------------------------------------------
        -- Master mode: drive XCK output enable
        -------------------------------------------------------------
        xck_oe <= '1';

        if master_run_i = '1' then
          if master_edge_event = '1' then
            xck_q   <= not xck_q;
            xck_out <= not xck_q;
            xck_cnt <= 0;
          else
            xck_out <= xck_q;
            xck_cnt <= xck_cnt + 1;
          end if;
        else
          -- Idle level chosen so that the first generated edge is a
          -- data-change edge for the selected UCPOL.
          xck_cnt <= 0;
          if UCPOL = '1' then
            xck_q   <= '1';
            xck_out <= '1';
          else
            xck_q   <= '0';
            xck_out <= '0';
          end if;
        end if;

      else
        -------------------------------------------------------------
        -- Slave mode or asynchronous mode: do not drive XCK
        -------------------------------------------------------------
        xck_oe  <= '0';
        xck_out <= '0';
        xck_cnt <= 0;

        if UCPOL = '1' then
          xck_q <= '1';
        else
          xck_q <= '0';
        end if;
      end if;
    end if;
  end process xck_proc;

end architecture rtl;
-- =============================================================================
-- TWI (Two-Wire Interface) Module
-- Fixed, production-oriented RTL for ATmega16(L)-style TWI behavior.
--
-- Implementation constraints respected:
--   * No NATURAL or REAL types are used.
--   * No concurrent signal assignment statements are used.
--   * All signal assignments are made inside processes.
--   * Variables are used for intermediate calculations and decisions.
--
-- Features:
--   * TWBR, TWCR, TWSR, TWDR, TWAR register model
--   * Master Transmitter / Master Receiver
--   * Slave Transmitter / Slave Receiver
--   * 7-bit addressing
--   * General call recognition
--   * START / REPEATED START / STOP generation
--   * SCL stretching while TWINT is set
--   * Multi-master arbitration detection
--   * Arbitration-lost address monitor
--   * Bus error detection and recovery
--   * Open-drain SCL/SDA
--   * Not-addressed slave ignores data until START/STOP
--
-- Notes:
--   * External pull-ups are required unless SoC pads provide them.
--   * Reset is active-high.
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity twi_interface is
  port (
    clk   : in    std_logic;
    reset : in    std_logic;
    addr  : in    std_logic_vector(2 downto 0);
    din   : in    std_logic_vector(7 downto 0);
    dout  : out   std_logic_vector(7 downto 0);
    wr    : in    std_logic;
    rd    : in    std_logic;
    irq   : out   std_logic;
    scl   : inout std_logic;
    sda   : inout std_logic
  );
end entity twi_interface;

architecture rtl of twi_interface is

  ---------------------------------------------------------------------------
  -- Status codes, prescaler bits masked to zero in TWSR[7:3]
  ---------------------------------------------------------------------------
  constant ST_START            : std_logic_vector(7 downto 0) := x"08";
  constant ST_REP_START        : std_logic_vector(7 downto 0) := x"10";
  constant ST_MT_SLA_ACK       : std_logic_vector(7 downto 0) := x"18";
  constant ST_MT_SLA_NACK      : std_logic_vector(7 downto 0) := x"20";
  constant ST_MT_DATA_ACK      : std_logic_vector(7 downto 0) := x"28";
  constant ST_MT_DATA_NACK     : std_logic_vector(7 downto 0) := x"30";
  constant ST_MT_ARB_LOST      : std_logic_vector(7 downto 0) := x"38";
  constant ST_MR_SLA_ACK       : std_logic_vector(7 downto 0) := x"40";
  constant ST_MR_SLA_NACK      : std_logic_vector(7 downto 0) := x"48";
  constant ST_MR_DATA_ACK      : std_logic_vector(7 downto 0) := x"50";
  constant ST_MR_DATA_NACK     : std_logic_vector(7 downto 0) := x"58";
  constant ST_SR_SLA_ACK       : std_logic_vector(7 downto 0) := x"60";
  constant ST_SR_ARB_LOST      : std_logic_vector(7 downto 0) := x"68";
  constant ST_SR_GCA_ACK       : std_logic_vector(7 downto 0) := x"70";
  constant ST_SR_GCA_ARB       : std_logic_vector(7 downto 0) := x"78";
  constant ST_SR_DATA_ACK      : std_logic_vector(7 downto 0) := x"80";
  constant ST_SR_DATA_NACK     : std_logic_vector(7 downto 0) := x"88";
  constant ST_SR_GCA_DATA_ACK  : std_logic_vector(7 downto 0) := x"90";
  constant ST_SR_GCA_DATA_NACK : std_logic_vector(7 downto 0) := x"98";
  constant ST_SR_STOP          : std_logic_vector(7 downto 0) := x"A0";
  constant ST_ST_SLA_ACK       : std_logic_vector(7 downto 0) := x"A8";
  constant ST_ST_ARB_LOST      : std_logic_vector(7 downto 0) := x"B0";
  constant ST_ST_DATA_ACK      : std_logic_vector(7 downto 0) := x"B8";
  constant ST_ST_DATA_NACK     : std_logic_vector(7 downto 0) := x"C0";
  constant ST_ST_LAST_DATA     : std_logic_vector(7 downto 0) := x"C8";
  constant ST_NO_INFO          : std_logic_vector(7 downto 0) := x"F8";
  constant ST_BUS_ERROR        : std_logic_vector(7 downto 0) := x"00";

  ---------------------------------------------------------------------------
  -- Helper function
  ---------------------------------------------------------------------------
  function prescaler_value(ps : std_logic_vector(1 downto 0)) return integer is
  begin
    case ps is
      when "00"   => return 1;
      when "01"   => return 4;
      when "10"   => return 16;
      when others => return 64;
    end case;
  end function;

  ---------------------------------------------------------------------------
  -- FSM types
  ---------------------------------------------------------------------------
  type state_type is (
    IDLE,
    WAIT_CMD,

    START_WAIT_FREE,
    START_PRE,
    START_SCL_HIGH,
    START_SDA_LOW,
    START_SCL_LOW,

    STOP_PRE,
    STOP_SCL_HIGH,
    STOP_SDA_HIGH,

    M_LOW,
    M_HIGH_WAIT,
    M_HIGH,
    M_SHIFT,

    M_ACK_LOW,
    M_ACK_HIGH_WAIT,
    M_ACK_HIGH,
    M_ACK_NEXT,

    ARB_LOST_MONITOR,

    S_LISTEN,
    S_IGNORE,
    S_ADDR_ACK_DRIVE,
    S_ADDR_ACK_WAIT,

    S_RX_DATA,
    S_RX_ACK_DRIVE,
    S_RX_ACK_WAIT,

    S_TX_SETUP,
    S_TX_DATA,
    S_TX_ACK_WAIT,

    BUS_ERR_ST
  );

  type mode_type is (
    MODE_IDLE,
    MODE_MT,
    MODE_MR,
    MODE_SR,
    MODE_ST
  );

  ---------------------------------------------------------------------------
  -- CPU-visible registers / register fragments
  ---------------------------------------------------------------------------
  signal twbr_reg   : std_logic_vector(7 downto 0);
  signal twar_reg   : std_logic_vector(7 downto 0);
  signal twdr_reg   : std_logic_vector(7 downto 0);
  signal twcr_ea    : std_logic;
  signal twcr_sta   : std_logic;
  signal twcr_sto   : std_logic;
  signal twcr_en    : std_logic;
  signal twcr_ie    : std_logic;
  signal twsr_ps    : std_logic_vector(1 downto 0);
  signal twwc       : std_logic;
  signal twint_flag : std_logic;

  signal twcr_read : std_logic_vector(7 downto 0);
  signal twsr_read : std_logic_vector(7 downto 0);

  ---------------------------------------------------------------------------
  -- Synchronizers and edge detection
  ---------------------------------------------------------------------------
  signal scl_meta : std_logic;
  signal scl_sync : std_logic;
  signal scl_prev : std_logic;

  signal sda_meta : std_logic;
  signal sda_sync : std_logic;
  signal sda_prev : std_logic;

  signal scl_rise  : std_logic;
  signal scl_fall  : std_logic;
  signal sda_rise  : std_logic;
  signal sda_fall  : std_logic;
  signal start_det : std_logic;
  signal stop_det  : std_logic;

  ---------------------------------------------------------------------------
  -- Bus control
  ---------------------------------------------------------------------------
  signal scl_oe : std_logic;
  signal sda_oe : std_logic;

  ---------------------------------------------------------------------------
  -- Internal TWI engine state
  ---------------------------------------------------------------------------
  signal state      : state_type;
  signal mode       : mode_type;
  signal status_reg : std_logic_vector(7 downto 0);

  signal master          : std_logic;
  signal bus_busy        : std_logic;
  signal addressed       : std_logic;
  signal gca             : std_logic;
  signal slave_dir_read  : std_logic;
  signal arb_lost_as_master       : std_logic;
  signal pending_start_after_stop : std_logic;
  signal rep_start_pending        : std_logic;

  signal shift_reg : std_logic_vector(7 downto 0);
  signal byte_reg  : std_logic_vector(7 downto 0);
  signal bit_cnt   : integer range 0 to 9;
  signal delay_cnt : integer range 0 to 65535;

  signal tx_byte        : std_logic;
  signal byte_is_addr   : std_logic;
  signal master_rx_next : std_logic;
  signal ack_send       : std_logic;
  signal sampled_sda    : std_logic;
  signal ack_received   : std_logic;

  signal arb_byte : std_logic_vector(7 downto 0);
  signal arb_bit  : integer range 0 to 9;

begin

  ---------------------------------------------------------------------------
  -- Input synchronizers
  ---------------------------------------------------------------------------
  sync_proc : process(clk, reset)
  begin
    if reset = '1' then
      scl_meta <= '1';
      scl_sync <= '1';
      scl_prev <= '1';
      sda_meta <= '1';
      sda_sync <= '1';
      sda_prev <= '1';
    elsif rising_edge(clk) then
      scl_meta <= scl;
      scl_sync <= scl_meta;
      scl_prev <= scl_sync;

      sda_meta <= sda;
      sda_sync <= sda_meta;
      sda_prev <= sda_sync;
    end if;
  end process sync_proc;

  ---------------------------------------------------------------------------
  -- Edge and START/STOP detection
  ---------------------------------------------------------------------------
  edge_proc : process(scl_sync, scl_prev, sda_sync, sda_prev) is
    variable v_scl_rise : std_logic;
    variable v_scl_fall : std_logic;
    variable v_sda_rise : std_logic;
    variable v_sda_fall : std_logic;
    variable v_start    : std_logic;
    variable v_stop     : std_logic;
  begin
    v_scl_rise := '0';
    v_scl_fall := '0';
    v_sda_rise := '0';
    v_sda_fall := '0';
    v_start    := '0';
    v_stop     := '0';

    if scl_sync = '1' and scl_prev = '0' then
      v_scl_rise := '1';
    end if;

    if scl_sync = '0' and scl_prev = '1' then
      v_scl_fall := '1';
    end if;

    if sda_sync = '1' and sda_prev = '0' then
      v_sda_rise := '1';
    end if;

    if sda_sync = '0' and sda_prev = '1' then
      v_sda_fall := '1';
    end if;

    if v_sda_fall = '1' and scl_sync = '1' then
      v_start := '1';
    end if;

    if v_sda_rise = '1' and scl_sync = '1' then
      v_stop := '1';
    end if;

    scl_rise  <= v_scl_rise;
    scl_fall  <= v_scl_fall;
    sda_rise  <= v_sda_rise;
    sda_fall  <= v_sda_fall;
    start_det <= v_start;
    stop_det  <= v_stop;
  end process edge_proc;

  ---------------------------------------------------------------------------
  -- Readable register helpers
  ---------------------------------------------------------------------------
  read_sig_proc : process(
    twint_flag,
    twcr_ea,
    twcr_sta,
    twcr_sto,
    twwc,
    twcr_en,
    twcr_ie,
    status_reg,
    twsr_ps
  ) is
    variable v_twsr : std_logic_vector(7 downto 0);
  begin
    twcr_read <= twint_flag & twcr_ea & twcr_sta & twcr_sto &
                 twwc & twcr_en & '0' & twcr_ie;

    if twint_flag = '1' then
      v_twsr := status_reg(7 downto 3) & '0' & twsr_ps;
    else
      v_twsr := ST_NO_INFO(7 downto 3) & '0' & twsr_ps;
    end if;

    twsr_read <= v_twsr;
  end process read_sig_proc;

  ---------------------------------------------------------------------------
  -- Data output multiplexer
  ---------------------------------------------------------------------------
  read_out_proc : process(
    addr,
    rd,
    twbr_reg,
    twcr_read,
    twsr_read,
    twdr_reg,
    twar_reg
  ) is
    variable v_data : std_logic_vector(7 downto 0);
  begin
    v_data := x"00";

    case addr is
      when "000" =>
        v_data := twbr_reg;
      when "001" =>
        v_data := twcr_read;
      when "010" =>
        v_data := twsr_read;
      when "011" =>
        v_data := twdr_reg;
      when "100" =>
        v_data := twar_reg;
      when others =>
        v_data := x"00";
    end case;

    if rd = '1' then
      dout <= v_data;
    else
      dout <= x"00";
    end if;
  end process read_out_proc;

  ---------------------------------------------------------------------------
  -- Interrupt output
  ---------------------------------------------------------------------------
  irq_proc : process(twcr_ie, twint_flag) is
    variable v_irq : std_logic;
  begin
    v_irq := '0';
    if twcr_ie = '1' and twint_flag = '1' then
      v_irq := '1';
    end if;
    irq <= v_irq;
  end process irq_proc;

  ---------------------------------------------------------------------------
  -- Open-drain bus drivers
  ---------------------------------------------------------------------------
  io_proc : process(scl_oe, sda_oe) is
  begin
    if scl_oe = '1' then
      scl <= '0';
    else
      scl <= 'Z';
    end if;

    if sda_oe = '1' then
      sda <= '0';
    else
      sda <= 'Z';
    end if;
  end process io_proc;

  ---------------------------------------------------------------------------
  -- Main TWI register/FSM process
  ---------------------------------------------------------------------------
  main_proc : process(clk, reset) is
    variable v_half      : integer;
    variable v_setup     : integer;
    variable v_en_eff    : std_logic;
    variable v_cmd       : boolean;
    variable v_cmd_ea    : std_logic;
    variable v_cmd_sta   : std_logic;
    variable v_cmd_sto   : std_logic;

    variable v_own_ss      : boolean;
    variable v_byte_active : boolean;
    variable v_handled     : boolean;

    variable v_addr_match : boolean;
    variable v_gca_match  : boolean;

    variable v_byte : std_logic_vector(7 downto 0);
    variable v_idx  : integer;
  begin
    if reset = '1' then
      state      <= IDLE;
      mode       <= MODE_IDLE;
      status_reg <= ST_NO_INFO;

      master          <= '0';
      bus_busy        <= '0';
      addressed       <= '0';
      gca             <= '0';
      slave_dir_read  <= '0';
      arb_lost_as_master       <= '0';
      pending_start_after_stop <= '0';
      rep_start_pending        <= '0';

      shift_reg <= x"00";
      byte_reg  <= x"00";
      bit_cnt   <= 0;
      delay_cnt <= 0;

      tx_byte        <= '0';
      byte_is_addr   <= '0';
      master_rx_next <= '0';
      ack_send       <= '0';
      sampled_sda    <= '1';
      ack_received   <= '0';

      arb_byte <= x"00";
      arb_bit  <= 0;

      scl_oe <= '0';
      sda_oe <= '0';

      twbr_reg   <= x"00";
      twar_reg   <= x"FE";
      twdr_reg   <= x"FF";
      twcr_ea    <= '0';
      twcr_sta   <= '0';
      twcr_sto   <= '0';
      twcr_en    <= '0';
      twcr_ie    <= '0';
      twsr_ps    <= "00";
      twwc       <= '0';
      twint_flag <= '0';

    elsif rising_edge(clk) then

      -----------------------------------------------------------------------
      -- Decode CPU command write to TWCR before register updates.
      -----------------------------------------------------------------------
      v_cmd     := false;
      v_cmd_ea  := twcr_ea;
      v_cmd_sta := twcr_sta;
      v_cmd_sto := twcr_sto;
      v_en_eff  := twcr_en;

      if wr = '1' and addr = "001" then
        v_cmd_ea  := din(6);
        v_cmd_sta := din(5);
        v_cmd_sto := din(4);
        v_en_eff  := din(2);
        if din(7) = '1' then
          v_cmd := true;
        end if;
      end if;

      -----------------------------------------------------------------------
      -- Register writes
      -----------------------------------------------------------------------
      if wr = '1' then
        case addr is
          when "000" =>
            twbr_reg <= din;

          when "001" =>
            twcr_ea  <= din(6);
            twcr_sta <= din(5);
            twcr_sto <= din(4);
            twcr_en  <= din(2);
            twcr_ie  <= din(0);
            if din(7) = '1' then
              twint_flag <= '0';
            end if;

          when "010" =>
            twsr_ps <= din(1 downto 0);

          when "011" =>
            if twcr_en = '1' then
              if twint_flag = '1' then
                twdr_reg <= din;
                twwc     <= '0';
              else
                twwc <= '1';
              end if;
            end if;

          when "100" =>
            twar_reg <= din;

          when others =>
            null;
        end case;
      end if;

      -----------------------------------------------------------------------
      -- Defaults for this clock cycle
      -----------------------------------------------------------------------
      v_handled := false;

      -----------------------------------------------------------------------
      -- Bit-rate half-period calculation:
      --   f_SCL = f_CPU / (16 + 2 * TWBR * 4^TWPS)
      --   half_period = 8 + TWBR * prescaler
      -----------------------------------------------------------------------
      v_half := 8 + to_integer(unsigned(twbr_reg)) * prescaler_value(twsr_ps);
      if v_half < 2 then
        v_half := 2;
      end if;

      v_setup := v_half / 4;
      if v_setup < 1 then
        v_setup := 1;
      end if;

      -----------------------------------------------------------------------
      -- TWEN = 0: switch off TWI immediately
      -----------------------------------------------------------------------
      if v_en_eff = '0' then
        state      <= IDLE;
        mode       <= MODE_IDLE;
        master     <= '0';
        bus_busy   <= '0';
        addressed  <= '0';
        gca        <= '0';
        slave_dir_read <= '0';

        arb_lost_as_master       <= '0';
        pending_start_after_stop <= '0';
        rep_start_pending        <= '0';

        status_reg <= ST_NO_INFO;
        twint_flag <= '0';
        twwc       <= '0';

        scl_oe <= '0';
        sda_oe <= '0';

        tx_byte   <= '0';
        bit_cnt   <= 0;
        delay_cnt <= 0;

        v_handled := true;
      end if;

      -----------------------------------------------------------------------
      -- Classify current state for START/STOP detection
      -----------------------------------------------------------------------
      v_own_ss := false;
      case state is
        when START_PRE | START_SCL_HIGH | START_SDA_LOW | START_SCL_LOW |
             STOP_PRE  | STOP_SCL_HIGH  | STOP_SDA_HIGH =>
          v_own_ss := true;
        when others =>
          v_own_ss := false;
      end case;

      v_byte_active := false;
      case state is
        when M_LOW | M_HIGH_WAIT | M_HIGH | M_SHIFT |
             M_ACK_LOW | M_ACK_HIGH_WAIT | M_ACK_HIGH | M_ACK_NEXT |
             S_ADDR_ACK_DRIVE | S_ADDR_ACK_WAIT |
             S_RX_DATA | S_RX_ACK_DRIVE | S_RX_ACK_WAIT |
             S_TX_SETUP | S_TX_DATA | S_TX_ACK_WAIT =>
          v_byte_active := true;
        when others =>
          v_byte_active := false;
      end case;

      -----------------------------------------------------------------------
      -- START condition detection
      -----------------------------------------------------------------------
      if not v_handled and start_det = '1' and not v_own_ss then
        bus_busy  <= '1';
        bit_cnt   <= 0;
        shift_reg <= x"00";

        if master = '1' then
          if v_byte_active then
            status_reg <= ST_BUS_ERROR;
            twint_flag <= '1';
            master     <= '0';
            mode       <= MODE_IDLE;
            tx_byte    <= '0';
            scl_oe     <= '1';
            sda_oe     <= '0';
            state      <= BUS_ERR_ST;
          end if;
          v_handled := true;
        else
          if addressed = '1' then
            status_reg <= ST_SR_STOP;
            twint_flag <= '1';
            addressed  <= '0';
            scl_oe     <= '1';
            sda_oe     <= '0';
            state      <= WAIT_CMD;
            v_handled  := true;
          elsif arb_lost_as_master = '1' then
            status_reg         <= ST_MT_ARB_LOST;
            twint_flag         <= '1';
            arb_lost_as_master <= '0';
            scl_oe             <= '0';
            sda_oe             <= '0';
            state              <= WAIT_CMD;
            v_handled          := true;
          else
            if state /= BUS_ERR_ST then
              state <= S_LISTEN;
            end if;
            v_handled := true;
          end if;
        end if;
      end if;

      -----------------------------------------------------------------------
      -- STOP condition detection
      -----------------------------------------------------------------------
      if not v_handled and stop_det = '1' and not v_own_ss then
        bus_busy  <= '0';
        bit_cnt   <= 0;
        shift_reg <= x"00";

        if master = '1' then
          if v_byte_active then
            status_reg <= ST_BUS_ERROR;
            twint_flag <= '1';
            master     <= '0';
            mode       <= MODE_IDLE;
            tx_byte    <= '0';
            scl_oe     <= '1';
            sda_oe     <= '0';
            state      <= BUS_ERR_ST;
          end if;
          v_handled := true;
        else
          if addressed = '1' then
            status_reg <= ST_SR_STOP;
            twint_flag <= '1';
            addressed  <= '0';
            scl_oe     <= '1';
            sda_oe     <= '0';
            state      <= WAIT_CMD;
            v_handled  := true;
          elsif arb_lost_as_master = '1' then
            status_reg         <= ST_MT_ARB_LOST;
            twint_flag         <= '1';
            arb_lost_as_master <= '0';
            scl_oe             <= '0';
            sda_oe             <= '0';
            state              <= WAIT_CMD;
            v_handled          := true;
          else
            if state /= BUS_ERR_ST then
              state <= S_LISTEN;
            end if;
            v_handled := true;
          end if;
        end if;
      end if;

      -----------------------------------------------------------------------
      -- Main state machine
      -----------------------------------------------------------------------
      if not v_handled then
        case state is

          -------------------------------------------------------------------
          -- Idle / slave listen entry
          -------------------------------------------------------------------
          when IDLE =>
            scl_oe <= '0';
            sda_oe <= '0';
            master <= '0';
            tx_byte <= '0';
            bit_cnt <= 0;
            delay_cnt <= 0;

            if v_cmd then
              if v_cmd_sta = '1' and v_cmd_sto = '1' then
                if master = '1' then
                  pending_start_after_stop <= '1';
                  rep_start_pending        <= '0';
                  state                    <= STOP_PRE;
                else
                  twcr_sto          <= '0';
                  master            <= '1';
                  pending_start_after_stop <= '0';
                  rep_start_pending <= '0';
                  state             <= START_WAIT_FREE;
                end if;

              elsif v_cmd_sta = '1' then
                if master = '1' and bus_busy = '1' then
                  rep_start_pending <= '1';
                  state             <= START_PRE;
                else
                  rep_start_pending <= '0';
                  master            <= '1';
                  state             <= START_WAIT_FREE;
                end if;

              elsif v_cmd_sto = '1' then
                if master = '1' then
                  pending_start_after_stop <= '0';
                  state                    <= STOP_PRE;
                else
                  twcr_sto   <= '0';
                  status_reg <= ST_NO_INFO;
                  state      <= S_LISTEN;
                end if;

              else
                if bus_busy = '1' then
                  state <= S_IGNORE;
                else
                  state <= S_LISTEN;
                end if;
              end if;
            else
              if bus_busy = '1' then
                state <= S_IGNORE;
              else
                state <= S_LISTEN;
              end if;
            end if;

          -------------------------------------------------------------------
          -- Wait for software command while TWINT is set
          -------------------------------------------------------------------
          when WAIT_CMD =>
            if status_reg = ST_MT_ARB_LOST and addressed = '0' then
              scl_oe <= '0';
            else
              scl_oe <= '1';
            end if;
            sda_oe <= '0';

            if v_cmd then
              if v_cmd_sta = '1' and v_cmd_sto = '1' then
                if master = '1' then
                  pending_start_after_stop <= '1';
                  rep_start_pending        <= '0';
                  state                    <= STOP_PRE;
                else
                  twcr_sto          <= '0';
                  master            <= '1';
                  pending_start_after_stop <= '0';
                  rep_start_pending <= '0';
                  state             <= START_WAIT_FREE;
                end if;

              elsif v_cmd_sta = '1' then
                if master = '1' and bus_busy = '1' then
                  rep_start_pending <= '1';
                  state             <= START_PRE;
                else
                  rep_start_pending <= '0';
                  master            <= '1';
                  state             <= START_WAIT_FREE;
                end if;

              elsif v_cmd_sto = '1' then
                if master = '1' then
                  pending_start_after_stop <= '0';
                  state                    <= STOP_PRE;
                else
                  twcr_sto   <= '0';
                  status_reg <= ST_NO_INFO;
                  state      <= S_LISTEN;
                end if;

              else
                -------------------------------------------------------------
                -- Decode next action from current status
                -------------------------------------------------------------
                case status_reg is

                  when ST_START | ST_REP_START =>
                    -- Send SLA+R/W from TWDR.
                    shift_reg      <= twdr_reg;
                    byte_reg       <= twdr_reg;
                    bit_cnt        <= 0;
                    tx_byte        <= '1';
                    byte_is_addr   <= '1';
                    master_rx_next <= twdr_reg(0);
                    master         <= '1';
                    mode           <= MODE_MT;
                    delay_cnt      <= 0;
                    scl_oe         <= '1';
                    state          <= M_LOW;

                  when ST_MT_SLA_ACK | ST_MT_SLA_NACK |
                       ST_MT_DATA_ACK | ST_MT_DATA_NACK =>
                    -- Master transmitter: send next data byte.
                    shift_reg      <= twdr_reg;
                    byte_reg       <= twdr_reg;
                    bit_cnt        <= 0;
                    tx_byte        <= '1';
                    byte_is_addr   <= '0';
                    master_rx_next <= '0';
                    master         <= '1';
                    mode           <= MODE_MT;
                    delay_cnt      <= 0;
                    scl_oe         <= '1';
                    state          <= M_LOW;

                  when ST_MR_SLA_ACK | ST_MR_DATA_ACK =>
                    -- Master receiver: receive next data byte.
                    shift_reg      <= x"00";
                    bit_cnt        <= 0;
                    tx_byte        <= '0';
                    byte_is_addr   <= '0';
                    master_rx_next <= '1';
                    master         <= '1';
                    mode           <= MODE_MR;
                    delay_cnt      <= 0;
                    scl_oe         <= '1';
                    state          <= M_LOW;

                  when ST_MR_SLA_NACK | ST_MR_DATA_NACK =>
                    -- Safe bus release if software gives no explicit START.
                    pending_start_after_stop <= '0';
                    master <= '1';
                    state  <= STOP_PRE;

                  when ST_SR_SLA_ACK | ST_SR_GCA_ACK |
                       ST_SR_ARB_LOST | ST_SR_GCA_ARB =>
                    if slave_dir_read = '1' then
                      -- Slave transmitter: load TWDR and send.
                      shift_reg <= twdr_reg;
                      bit_cnt   <= 0;
                      sda_oe    <= not twdr_reg(7);
                      scl_oe    <= '1';
                      delay_cnt <= 0;
                      mode      <= MODE_ST;
                      state     <= S_TX_SETUP;
                    else
                      -- Slave receiver: receive next data byte.
                      shift_reg <= x"00";
                      bit_cnt   <= 0;
                      scl_oe    <= '0';
                      sda_oe    <= '0';
                      mode      <= MODE_SR;
                      state     <= S_RX_DATA;
                    end if;

                  when ST_ST_SLA_ACK | ST_ST_ARB_LOST | ST_ST_DATA_ACK =>
                    -- Slave transmitter: send next byte.
                    shift_reg <= twdr_reg;
                    bit_cnt   <= 0;
                    sda_oe    <= not twdr_reg(7);
                    scl_oe    <= '1';
                    delay_cnt <= 0;
                    mode      <= MODE_ST;
                    state     <= S_TX_SETUP;

                  when ST_SR_DATA_ACK | ST_SR_GCA_DATA_ACK =>
                    -- Slave receiver: receive next byte.
                    shift_reg <= x"00";
                    bit_cnt   <= 0;
                    scl_oe    <= '0';
                    sda_oe    <= '0';
                    mode      <= MODE_SR;
                    state     <= S_RX_DATA;

                  when ST_SR_STOP =>
                    -- A repeated START may already be present.
                    addressed <= '0';
                    gca       <= '0';
                    bit_cnt   <= 0;
                    scl_oe    <= '0';
                    sda_oe    <= '0';
                    state     <= S_LISTEN;

                  when ST_SR_DATA_NACK | ST_SR_GCA_DATA_NACK |
                       ST_ST_DATA_NACK | ST_ST_LAST_DATA =>
                    -- Not addressed; ignore bus until START/STOP.
                    addressed <= '0';
                    gca       <= '0';
                    bit_cnt   <= 0;
                    scl_oe    <= '0';
                    sda_oe    <= '0';
                    state     <= S_IGNORE;

                  when others =>
                    -- Includes arbitration lost and no-info fallback.
                    arb_lost_as_master <= '0';
                    addressed          <= '0';
                    gca                <= '0';
                    bit_cnt            <= 0;
                    scl_oe             <= '0';
                    sda_oe             <= '0';
                    state              <= S_IGNORE;

                end case;
              end if;
            else
              if twint_flag = '0' then
                state <= IDLE;
              end if;
            end if;

          -------------------------------------------------------------------
          -- Wait for free bus before initial START
          -------------------------------------------------------------------
          when START_WAIT_FREE =>
            scl_oe <= '0';
            sda_oe <= '0';
            master <= '1';

            if bus_busy = '0' and scl_sync = '1' and sda_sync = '1' then
              if delay_cnt = v_half - 1 then
                delay_cnt <= 0;
                state     <= START_PRE;
              else
                delay_cnt <= delay_cnt + 1;
              end if;
            else
              delay_cnt <= 0;
            end if;

          -------------------------------------------------------------------
          -- START generation
          -------------------------------------------------------------------
          when START_PRE =>
            -- Force SCL low and release SDA high.
            scl_oe <= '1';
            sda_oe <= '0';

            if scl_sync = '0' then
              if delay_cnt = v_half - 1 then
                delay_cnt <= 0;
                state     <= START_SCL_HIGH;
              else
                delay_cnt <= delay_cnt + 1;
              end if;
            else
              delay_cnt <= 0;
            end if;

          when START_SCL_HIGH =>
            -- Release SCL and wait for actual SCL high.
            scl_oe <= '0';
            sda_oe <= '0';

            if scl_sync = '1' then
              if delay_cnt = v_half - 1 then
                delay_cnt <= 0;
                state     <= START_SDA_LOW;
              else
                delay_cnt <= delay_cnt + 1;
              end if;
            else
              delay_cnt <= 0;
            end if;

          when START_SDA_LOW =>
            -- Pull SDA low while SCL is high: START condition.
            scl_oe <= '0';
            sda_oe <= '1';

            if delay_cnt = v_half - 1 then
              delay_cnt <= 0;
              state     <= START_SCL_LOW;
            else
              delay_cnt <= delay_cnt + 1;
            end if;

          when START_SCL_LOW =>
            -- Pull SCL low and finish START.
            scl_oe <= '1';
            sda_oe <= '1';

            if scl_sync = '0' then
              if delay_cnt = v_half - 1 then
                delay_cnt <= 0;
                bus_busy  <= '1';

                arb_lost_as_master <= '0';

                if rep_start_pending = '1' then
                  status_reg <= ST_REP_START;
                else
                  status_reg <= ST_START;
                end if;

                rep_start_pending <= '0';
                master            <= '1';
                twint_flag        <= '1';
                state             <= WAIT_CMD;
              else
                delay_cnt <= delay_cnt + 1;
              end if;
            else
              delay_cnt <= 0;
            end if;

          -------------------------------------------------------------------
          -- STOP generation
          -------------------------------------------------------------------
          when STOP_PRE =>
            -- SCL low, SDA low.
            scl_oe <= '1';
            sda_oe <= '1';

            if scl_sync = '0' then
              if delay_cnt = v_half - 1 then
                delay_cnt <= 0;
                state     <= STOP_SCL_HIGH;
              else
                delay_cnt <= delay_cnt + 1;
              end if;
            else
              delay_cnt <= 0;
            end if;

          when STOP_SCL_HIGH =>
            -- Release SCL, keep SDA low.
            scl_oe <= '0';
            sda_oe <= '1';

            if scl_sync = '1' then
              if delay_cnt = v_half - 1 then
                delay_cnt <= 0;
                state     <= STOP_SDA_HIGH;
              else
                delay_cnt <= delay_cnt + 1;
              end if;
            else
              delay_cnt <= 0;
            end if;

          when STOP_SDA_HIGH =>
            -- Release SDA while SCL high: STOP condition.
            scl_oe <= '0';
            sda_oe <= '0';

            if delay_cnt = v_half - 1 then
              delay_cnt  <= 0;
              bus_busy   <= '0';
              twcr_sto   <= '0';
              status_reg <= ST_NO_INFO;

              if pending_start_after_stop = '1' then
                pending_start_after_stop <= '0';
                master            <= '1';
                rep_start_pending <= '0';
                state             <= START_WAIT_FREE;
              else
                pending_start_after_stop <= '0';
                master <= '0';
                state  <= IDLE;
              end if;
            else
              delay_cnt <= delay_cnt + 1;
            end if;

          -------------------------------------------------------------------
          -- Master byte transmission / reception: SCL low phase
          -------------------------------------------------------------------
          when M_LOW =>
            scl_oe <= '1';

            if tx_byte = '1' then
              sda_oe <= not shift_reg(7);
            else
              sda_oe <= '0';
            end if;

            if scl_sync = '0' then
              if delay_cnt = v_half - 1 then
                delay_cnt <= 0;
                state     <= M_HIGH_WAIT;
              else
                delay_cnt <= delay_cnt + 1;
              end if;
            else
              delay_cnt <= 0;
            end if;

          -------------------------------------------------------------------
          -- Master byte: release SCL and wait for actual high
          -------------------------------------------------------------------
          when M_HIGH_WAIT =>
            scl_oe <= '0';

            if tx_byte = '1' then
              sda_oe <= not shift_reg(7);
            else
              sda_oe <= '0';
            end if;

            if scl_sync = '1' then
              delay_cnt <= 0;
              state     <= M_HIGH;
            else
              delay_cnt <= 0;
            end if;

          -------------------------------------------------------------------
          -- Master byte: SCL high phase
          -------------------------------------------------------------------
          when M_HIGH =>
            scl_oe <= '0';

            if tx_byte = '1' then
              sda_oe <= not shift_reg(7);

              -- Arbitration:
              -- A master loses if it outputs SDA high but reads SDA low.
              if shift_reg(7) = '1' and sda_sync = '0' and scl_sync = '1' then
                master  <= '0';
                mode    <= MODE_IDLE;
                tx_byte <= '0';
                scl_oe  <= '0';
                sda_oe  <= '0';

                if byte_is_addr = '1' then
                  v_byte := byte_reg;
                  v_byte(7 - bit_cnt) := '0';

                  arb_byte           <= v_byte;
                  arb_bit            <= bit_cnt;
                  arb_lost_as_master <= '1';
                  status_reg         <= ST_MT_ARB_LOST;

                  if bit_cnt = 7 then
                    shift_reg <= v_byte;
                    state     <= S_ADDR_ACK_DRIVE;
                  else
                    state <= ARB_LOST_MONITOR;
                  end if;
                else
                  status_reg         <= ST_MT_ARB_LOST;
                  twint_flag         <= '1';
                  arb_lost_as_master <= '0';
                  state              <= WAIT_CMD;
                end if;

              elsif scl_sync = '0' then
                sampled_sda <= sda_sync;
                delay_cnt   <= 0;
                scl_oe      <= '1';
                state       <= M_SHIFT;

              elsif delay_cnt = v_half - 1 then
                sampled_sda <= sda_sync;
                delay_cnt   <= 0;
                scl_oe      <= '1';
                state       <= M_SHIFT;

              else
                delay_cnt <= delay_cnt + 1;
              end if;

            else
              sda_oe <= '0';

              if scl_sync = '0' then
                sampled_sda <= sda_sync;
                delay_cnt   <= 0;
                scl_oe      <= '1';
                state       <= M_SHIFT;

              elsif delay_cnt = v_half - 1 then
                sampled_sda <= sda_sync;
                delay_cnt   <= 0;
                scl_oe      <= '1';
                state       <= M_SHIFT;

              else
                delay_cnt <= delay_cnt + 1;
              end if;
            end if;

          -------------------------------------------------------------------
          -- Master byte: advance bit counter / shift register
          -------------------------------------------------------------------
          when M_SHIFT =>
            scl_oe <= '1';

            if tx_byte = '1' then
              if bit_cnt = 7 then
                sda_oe <= '0';
              else
                sda_oe <= not shift_reg(6);
              end if;
              shift_reg <= shift_reg(6 downto 0) & '1';
            else
              sda_oe    <= '0';
              shift_reg <= shift_reg(6 downto 0) & sampled_sda;
            end if;

            if bit_cnt = 7 then
              bit_cnt   <= 0;
              delay_cnt <= 0;
              state     <= M_ACK_LOW;
            else
              bit_cnt   <= bit_cnt + 1;
              delay_cnt <= 0;
              state     <= M_LOW;
            end if;

          -------------------------------------------------------------------
          -- Master ACK/NACK phase: SCL low
          -------------------------------------------------------------------
          when M_ACK_LOW =>
            scl_oe <= '1';

            if tx_byte = '1' then
              -- Master transmitter: release SDA so slave can ACK/NACK.
              sda_oe <= '0';
            else
              -- Master receiver: drive ACK/NACK.
              ack_send <= twcr_ea;
              sda_oe   <= twcr_ea;
            end if;

            if scl_sync = '0' then
              if delay_cnt = v_half - 1 then
                delay_cnt <= 0;
                state     <= M_ACK_HIGH_WAIT;
              else
                delay_cnt <= delay_cnt + 1;
              end if;
            else
              delay_cnt <= 0;
            end if;

          -------------------------------------------------------------------
          -- Master ACK/NACK phase: wait SCL high
          -------------------------------------------------------------------
          when M_ACK_HIGH_WAIT =>
            scl_oe <= '0';

            if tx_byte = '1' then
              sda_oe <= '0';
            else
              sda_oe <= ack_send;
            end if;

            if scl_sync = '1' then
              delay_cnt <= 0;
              state     <= M_ACK_HIGH;
            else
              delay_cnt <= 0;
            end if;

          -------------------------------------------------------------------
          -- Master ACK/NACK phase: SCL high
          -------------------------------------------------------------------
          when M_ACK_HIGH =>
            scl_oe <= '0';

            if tx_byte = '1' then
              sda_oe <= '0';
            else
              sda_oe <= ack_send;
            end if;

            -- Arbitration in Master Receiver NOT ACK bit only.
            if tx_byte = '0' and ack_send = '0' and
               sda_sync = '0' and scl_sync = '1' then
              status_reg         <= ST_MT_ARB_LOST;
              twint_flag         <= '1';
              master             <= '0';
              mode               <= MODE_IDLE;
              tx_byte            <= '0';
              scl_oe             <= '0';
              sda_oe             <= '0';
              arb_lost_as_master <= '0';
              state              <= WAIT_CMD;

            elsif scl_sync = '0' then
              sampled_sda <= sda_sync;
              delay_cnt   <= 0;
              scl_oe      <= '1';
              state       <= M_ACK_NEXT;

            elsif delay_cnt = v_half - 1 then
              sampled_sda <= sda_sync;
              delay_cnt   <= 0;
              scl_oe      <= '1';
              state       <= M_ACK_NEXT;

            else
              delay_cnt <= delay_cnt + 1;
            end if;

          -------------------------------------------------------------------
          -- Master ACK/NACK phase: completion and status
          -------------------------------------------------------------------
          when M_ACK_NEXT =>
            scl_oe <= '1';
            sda_oe <= '0';

            if tx_byte = '1' then
              ack_received <= not sampled_sda;

              if sampled_sda = '0' then
                -- ACK received.
                if byte_is_addr = '1' then
                  if master_rx_next = '1' then
                    status_reg <= ST_MR_SLA_ACK;
                    mode       <= MODE_MR;
                  else
                    status_reg <= ST_MT_SLA_ACK;
                    mode       <= MODE_MT;
                  end if;
                else
                  status_reg <= ST_MT_DATA_ACK;
                  mode       <= MODE_MT;
                end if;
              else
                -- NACK received.
                if byte_is_addr = '1' then
                  if master_rx_next = '1' then
                    status_reg <= ST_MR_SLA_NACK;
                    mode       <= MODE_MR;
                  else
                    status_reg <= ST_MT_SLA_NACK;
                    mode       <= MODE_MT;
                  end if;
                else
                  status_reg <= ST_MT_DATA_NACK;
                  mode       <= MODE_MT;
                end if;
              end if;

            else
              -- Master received data byte, ACK/NACK already sent.
              twdr_reg <= shift_reg;

              if ack_send = '1' then
                status_reg <= ST_MR_DATA_ACK;
              else
                status_reg <= ST_MR_DATA_NACK;
              end if;

              mode <= MODE_MR;
            end if;

            twint_flag <= '1';
            state      <= WAIT_CMD;

          -------------------------------------------------------------------
          -- Arbitration lost monitor:
          -- Continue sampling winning master's address byte after losing
          -- arbitration during an address transmission.
          -------------------------------------------------------------------
          when ARB_LOST_MONITOR =>
            scl_oe <= '0';
            sda_oe <= '0';

            if arb_bit >= 7 then
              shift_reg <= arb_byte;
              state     <= S_ADDR_ACK_DRIVE;
            elsif scl_rise = '1' then
              v_idx  := arb_bit + 1;
              v_byte := arb_byte;

              if v_idx >= 0 and v_idx <= 7 then
                v_byte(7 - v_idx) := sda_sync;
              end if;

              arb_byte <= v_byte;
              arb_bit  <= v_idx;

              if v_idx = 7 then
                shift_reg <= v_byte;
                state     <= S_ADDR_ACK_DRIVE;
              end if;
            end if;

          -------------------------------------------------------------------
          -- Slave listen / address shift-in
          -------------------------------------------------------------------
          when S_LISTEN =>
            scl_oe <= '0';
            sda_oe <= '0';
            master <= '0';
            tx_byte <= '0';
            mode    <= MODE_IDLE;

            if bus_busy = '0' then
              bit_cnt <= 0;
            end if;

            if bus_busy = '1' and scl_rise = '1' and bit_cnt < 8 then
              shift_reg <= shift_reg(6 downto 0) & sda_sync;

              if bit_cnt = 7 then
                bit_cnt <= 0;
                state   <= S_ADDR_ACK_DRIVE;
              else
                bit_cnt <= bit_cnt + 1;
              end if;
            end if;

          -------------------------------------------------------------------
          -- Slave ignore:
          -- Not addressed and not allowed to interpret data as address.
          -- Wait for START/STOP.
          -------------------------------------------------------------------
          when S_IGNORE =>
            scl_oe <= '0';
            sda_oe <= '0';
            master <= '0';
            tx_byte <= '0';
            mode    <= MODE_IDLE;
            bit_cnt <= 0;
            delay_cnt <= 0;

            if bus_busy = '0' then
              state <= S_LISTEN;
            end if;

          -------------------------------------------------------------------
          -- Slave address ACK drive: drive ACK on falling edge after 8 bits
          -------------------------------------------------------------------
          when S_ADDR_ACK_DRIVE =>
            scl_oe <= '0';

            if scl_fall = '1' then
              v_addr_match := (shift_reg(7 downto 1) = twar_reg(7 downto 1));
              v_gca_match  := (twar_reg(0) = '1' and
                               shift_reg(7 downto 1) = "0000000" and
                               shift_reg(0) = '0');

              if (v_addr_match or v_gca_match) and twcr_ea = '1' then
                sda_oe <= '1';
              else
                sda_oe <= '0';
              end if;

              state <= S_ADDR_ACK_WAIT;
            end if;

          -------------------------------------------------------------------
          -- Slave address ACK wait: complete on 9th falling edge
          -------------------------------------------------------------------
          when S_ADDR_ACK_WAIT =>
            scl_oe <= '0';

            if scl_fall = '1' then
              v_addr_match := (shift_reg(7 downto 1) = twar_reg(7 downto 1));
              v_gca_match  := (twar_reg(0) = '1' and
                               shift_reg(7 downto 1) = "0000000" and
                               shift_reg(0) = '0');

              if (v_addr_match or v_gca_match) and twcr_ea = '1' then
                addressed      <= '1';
                slave_dir_read <= shift_reg(0);

                if v_gca_match then
                  gca <= '1';
                else
                  gca <= '0';
                end if;

                if arb_lost_as_master = '1' then
                  if v_gca_match then
                    status_reg <= ST_SR_GCA_ARB;
                    mode       <= MODE_SR;
                  elsif shift_reg(0) = '0' then
                    status_reg <= ST_SR_ARB_LOST;
                    mode       <= MODE_SR;
                  else
                    status_reg <= ST_ST_ARB_LOST;
                    mode       <= MODE_ST;
                  end if;
                  arb_lost_as_master <= '0';
                else
                  if v_gca_match then
                    status_reg <= ST_SR_GCA_ACK;
                    mode       <= MODE_SR;
                  elsif shift_reg(0) = '0' then
                    status_reg <= ST_SR_SLA_ACK;
                    mode       <= MODE_SR;
                  else
                    status_reg <= ST_ST_SLA_ACK;
                    mode       <= MODE_ST;
                  end if;
                end if;

                twint_flag <= '1';
                scl_oe     <= '1';
                sda_oe     <= '0';
                bit_cnt    <= 0;
                state      <= WAIT_CMD;

              else
                sda_oe <= '0';
                scl_oe <= '0';
                bit_cnt <= 0;

                if arb_lost_as_master = '1' then
                  status_reg         <= ST_MT_ARB_LOST;
                  twint_flag         <= '1';
                  arb_lost_as_master <= '0';
                  state              <= WAIT_CMD;
                else
                  state <= S_IGNORE;
                end if;
              end if;
            end if;

          -------------------------------------------------------------------
          -- Slave receiver: data shift-in
          -------------------------------------------------------------------
          when S_RX_DATA =>
            scl_oe <= '0';
            sda_oe <= '0';

            if scl_rise = '1' and bit_cnt < 8 then
              shift_reg <= shift_reg(6 downto 0) & sda_sync;

              if bit_cnt = 7 then
                bit_cnt <= 0;
                state   <= S_RX_ACK_DRIVE;
              else
                bit_cnt <= bit_cnt + 1;
              end if;
            end if;

          -------------------------------------------------------------------
          -- Slave receiver: drive ACK/NACK after 8 data bits
          -------------------------------------------------------------------
          when S_RX_ACK_DRIVE =>
            scl_oe <= '0';

            if scl_fall = '1' then
              ack_send <= twcr_ea;

              if twcr_ea = '1' then
                sda_oe <= '1';
              else
                sda_oe <= '0';
              end if;

              state <= S_RX_ACK_WAIT;
            end if;

          -------------------------------------------------------------------
          -- Slave receiver: complete ACK/NACK bit
          -------------------------------------------------------------------
          when S_RX_ACK_WAIT =>
            scl_oe <= '0';

            if scl_fall = '1' then
              twdr_reg <= shift_reg;

              if ack_send = '1' then
                if gca = '1' then
                  status_reg <= ST_SR_GCA_DATA_ACK;
                else
                  status_reg <= ST_SR_DATA_ACK;
                end if;
              else
                if gca = '1' then
                  status_reg <= ST_SR_GCA_DATA_NACK;
                else
                  status_reg <= ST_SR_DATA_NACK;
                end if;

                addressed <= '0';
                gca       <= '0';
              end if;

              twint_flag <= '1';
              scl_oe     <= '1';
              sda_oe     <= '0';
              bit_cnt    <= 0;
              state      <= WAIT_CMD;
            end if;

          -------------------------------------------------------------------
          -- Slave transmitter: SDA setup before releasing SCL
          -------------------------------------------------------------------
          when S_TX_SETUP =>
            scl_oe <= '1';
            sda_oe <= not shift_reg(7);

            if delay_cnt = v_setup - 1 then
              delay_cnt <= 0;
              scl_oe    <= '0';
              state     <= S_TX_DATA;
            else
              delay_cnt <= delay_cnt + 1;
            end if;

          -------------------------------------------------------------------
          -- Slave transmitter: send data bits on SCL falling edges
          -------------------------------------------------------------------
          when S_TX_DATA =>
            scl_oe <= '0';

            if bit_cnt = 0 then
              sda_oe <= not shift_reg(7);
            end if;

            if scl_fall = '1' then
              if bit_cnt < 7 then
                sda_oe    <= not shift_reg(6);
                shift_reg <= shift_reg(6 downto 0) & '1';
                bit_cnt   <= bit_cnt + 1;
              else
                -- Eight bits sent; release SDA for master ACK/NACK.
                sda_oe       <= '0';
                ack_received <= '0';
                state        <= S_TX_ACK_WAIT;
              end if;
            end if;

          -------------------------------------------------------------------
          -- Slave transmitter: sample master ACK/NACK
          -------------------------------------------------------------------
          when S_TX_ACK_WAIT =>
            scl_oe <= '0';
            sda_oe <= '0';

            if scl_rise = '1' then
              ack_received <= not sda_sync;
            end if;

            if scl_fall = '1' then
              if ack_received = '1' then
                if twcr_ea = '0' then
                  status_reg <= ST_ST_LAST_DATA;
                  addressed  <= '0';
                else
                  status_reg <= ST_ST_DATA_ACK;
                end if;
              else
                status_reg <= ST_ST_DATA_NACK;
                addressed  <= '0';
              end if;

              twint_flag <= '1';
              scl_oe     <= '1';
              sda_oe     <= '0';
              bit_cnt    <= 0;
              state      <= WAIT_CMD;
            end if;

          -------------------------------------------------------------------
          -- Bus error recovery state
          -------------------------------------------------------------------
          when BUS_ERR_ST =>
            scl_oe <= '1';
            sda_oe <= '0';

            if v_cmd and v_cmd_sto = '1' then
              twcr_sto   <= '0';
              scl_oe     <= '0';
              sda_oe     <= '0';
              status_reg <= ST_NO_INFO;
              master     <= '0';
              addressed  <= '0';
              gca        <= '0';
              bus_busy   <= '0';
              state      <= S_IGNORE;
            end if;

          when others =>
            state <= IDLE;

        end case;
      end if;
    end if;
  end process main_proc;

end architecture rtl;
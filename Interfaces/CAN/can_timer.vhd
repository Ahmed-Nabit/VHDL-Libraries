-- ============================================================================
-- CAN Timer
-- ============================================================================
-- PDF-compliant behavior implemented:
--   * 16-bit timer counts from 0x0000 when CAN controller is enabled (ENFG).
--   * Prescaler equation: TclkCANTIM = TclkIO x 8 x (CANTCON[7:0] + 1).
--   * Overrun interrupt flag (OVRTIM) is latched when the timer rolls over
--     from 0xFFFF to 0x0000, and is cleared by a write-1-clear from the CPU.
--   * TTC capture on SOF or EOF depending on SYNTTC configuration.
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

entity can_timer is
    port (
        clk           : in  std_logic;
        reset_n       : in  std_logic;

        -- ENFG status from CANGSTA
        enable        : in  std_logic;

        -- CANTCON register
        tcon          : in  std_logic_vector(7 downto 0);

        -- TTC configuration
        ttc_mode      : in  std_logic;   -- TTC bit
        ttc_sync      : in  std_logic;   -- SYNTTC (0=SOF, 1=EOF)

        -- Frame pulses from engine
        sof_pulse     : in  std_logic;
        eof_pulse     : in  std_logic;

        -- Write-1-clear for OVRTIM flag from CPU interface
        ovr_tim_clear : in  std_logic;

        -- Timer outputs
        cantim_o      : out std_logic_vector(15 downto 0);
        canttc_o      : out std_logic_vector(15 downto 0);
        ovr_tim_int   : out std_logic
    );
end entity can_timer;

architecture rtl of can_timer is
begin

    timer_proc : process (clk, reset_n)
        variable prescaler_cnt_v : integer range 0 to 2048;
        variable timer_cnt_v     : integer range 0 to 65535;
        variable ttc_cnt_v       : integer range 0 to 65535;
        variable ovr_int_v       : std_logic;
        variable prescaler_max_v : integer range 0 to 2048;
        variable div8_cnt_v      : integer range 0 to 7;
        variable tcon_int_v      : integer range 0 to 255;
    begin
        if reset_n = '0' then
            prescaler_cnt_v := 0;
            div8_cnt_v      := 0;
            timer_cnt_v     := 0;
            ttc_cnt_v       := 0;
            ovr_int_v       := '0';
            prescaler_max_v := 8;
            tcon_int_v      := 0;

            cantim_o    <= (others => '0');
            canttc_o    <= (others => '0');
            ovr_tim_int <= '0';

        elsif rising_edge(clk) then
            --------------------------------------------------------------------
            -- Decode CANTCON register to integer.
            --------------------------------------------------------------------
            tcon_int_v := 0;
            for i in 7 downto 0 loop
                tcon_int_v := tcon_int_v * 2;
                if tcon(i) = '1' then
                    tcon_int_v := tcon_int_v + 1;
                end if;
            end loop;

            --------------------------------------------------------------------
            -- Calculate maximum prescaler count.
            -- TclkCANTIM = TclkIO x 8 x (CANTCON[7:0] + 1)
            -- Maximum value is 8 * (255 + 1) = 2048.
            --------------------------------------------------------------------
            prescaler_max_v := 8 * (tcon_int_v + 1);

            --------------------------------------------------------------------
            -- Timer counting logic.
            -- The timer only counts when the CAN controller is enabled (ENFG).
            --------------------------------------------------------------------
            if enable = '1' then
                --------------------------------------------------------------
                -- First stage: fixed divide-by-8.
                --------------------------------------------------------------
                if div8_cnt_v < 7 then
                    div8_cnt_v := div8_cnt_v + 1;
                else
                    div8_cnt_v := 0;

                    ----------------------------------------------------------
                    -- Second stage: programmable divide-by-(TCON + 1).
                    ----------------------------------------------------------
                    if prescaler_cnt_v >= prescaler_max_v - 1 then
                        prescaler_cnt_v := 0;

                        ------------------------------------------------------
                        -- 16-bit timer increment and overrun detection.
                        ------------------------------------------------------
                        if timer_cnt_v = 65535 then
                            timer_cnt_v := 0;
                            ovr_int_v   := '1';
                        else
                            timer_cnt_v := timer_cnt_v + 1;
                        end if;
                    else
                        prescaler_cnt_v := prescaler_cnt_v + 1;
                    end if;
                end if;
            else
                --------------------------------------------------------------
                -- Reset counters when disabled.
                --------------------------------------------------------------
                prescaler_cnt_v := 0;
                div8_cnt_v      := 0;
                timer_cnt_v     := 0;
            end if;

            --------------------------------------------------------------------
            -- OVRTIM flag clear.
            -- The flag is sticky and remains set until explicitly cleared by
            -- the CPU writing a 1 to CANGIT.OVRTIM.
            --------------------------------------------------------------------
            if ovr_tim_clear = '1' then
                ovr_int_v := '0';
            end if;

            --------------------------------------------------------------------
            -- TTC capture logic.
            -- Captures the current timer value into the TTC register on SOF
            -- or EOF depending on the SYNTTC configuration bit.
            --------------------------------------------------------------------
            if ttc_mode = '1' then
                if (ttc_sync = '0' and sof_pulse = '1') or
                   (ttc_sync = '1' and eof_pulse = '1') then
                    ttc_cnt_v := timer_cnt_v;
                end if;
            end if;

            --------------------------------------------------------------------
            -- Outputs.
            --------------------------------------------------------------------
            cantim_o    <= int_to_slv16(timer_cnt_v);
            canttc_o    <= int_to_slv16(ttc_cnt_v);
            ovr_tim_int <= ovr_int_v;
        end if;
    end process timer_proc;

end architecture rtl;
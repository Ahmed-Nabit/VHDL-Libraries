-- ============================================================================
-- CAN Package – Constants, types, and CRC function (corrected polynomial)
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

package can_pkg is
    constant MOB_COUNT : integer := 6;
    subtype mob_idx is integer range 0 to MOB_COUNT-1;

    -- MOB configuration codes
    constant MOB_DISABLED   : std_logic_vector(1 downto 0) := "00";
    constant MOB_TX         : std_logic_vector(1 downto 0) := "01";
    constant MOB_RX         : std_logic_vector(1 downto 0) := "10";
    constant MOB_FBUF_RX    : std_logic_vector(1 downto 0) := "11";

    -- Register addresses (6-bit)
    subtype reg_addr is std_logic_vector(5 downto 0);
    constant ADDR_CANGCON   : reg_addr := "000000";
    constant ADDR_CANGSTA   : reg_addr := "000001";
    constant ADDR_CANGIT    : reg_addr := "000010";
    constant ADDR_CANGIE    : reg_addr := "000011";
    constant ADDR_CANEN1    : reg_addr := "000100";
    constant ADDR_CANEN2    : reg_addr := "000101";
    constant ADDR_CANIE1    : reg_addr := "000110";
    constant ADDR_CANIE2    : reg_addr := "000111";
    constant ADDR_CANSIT1   : reg_addr := "001000";
    constant ADDR_CANSIT2   : reg_addr := "001001";
    constant ADDR_CANBT1    : reg_addr := "001010";
    constant ADDR_CANBT2    : reg_addr := "001011";
    constant ADDR_CANBT3    : reg_addr := "001100";
    constant ADDR_CANTCON   : reg_addr := "001101";
    constant ADDR_CANTIML   : reg_addr := "001110";
    constant ADDR_CANTIMH   : reg_addr := "001111";
    constant ADDR_CANTTCL   : reg_addr := "010000";
    constant ADDR_CANTTCH   : reg_addr := "010001";
    constant ADDR_CANTEC    : reg_addr := "010010";
    constant ADDR_CANREC    : reg_addr := "010011";
    constant ADDR_CANHPMOB  : reg_addr := "010100";
    constant ADDR_CANPAGE   : reg_addr := "010101";
    constant ADDR_MOB_BASE  : reg_addr := "100000";

    -- MOB register offsets within page
    constant MOB_OFFS_STMOB : integer := 0;
    constant MOB_OFFS_CDMOB : integer := 1;
    constant MOB_OFFS_IDT1  : integer := 2;
    constant MOB_OFFS_IDT2  : integer := 3;
    constant MOB_OFFS_IDT3  : integer := 4;
    constant MOB_OFFS_IDT4  : integer := 5;
    constant MOB_OFFS_IDM1  : integer := 6;
    constant MOB_OFFS_IDM2  : integer := 7;
    constant MOB_OFFS_IDM3  : integer := 8;
    constant MOB_OFFS_IDM4  : integer := 9;
    constant MOB_OFFS_STML  : integer := 10;
    constant MOB_OFFS_STMH  : integer := 11;
    constant MOB_OFFS_MSG   : integer := 12;

    -- MOB register record
    type mob_reg_t is record
        stmob : std_logic_vector(7 downto 0);
        cdmob : std_logic_vector(7 downto 0);
        idt1  : std_logic_vector(7 downto 0);
        idt2  : std_logic_vector(7 downto 0);
        idt3  : std_logic_vector(7 downto 0);
        idt4  : std_logic_vector(7 downto 0);
        idm1  : std_logic_vector(7 downto 0);
        idm2  : std_logic_vector(7 downto 0);
        idm3  : std_logic_vector(7 downto 0);
        idm4  : std_logic_vector(7 downto 0);
        stml  : std_logic_vector(7 downto 0);
        stmh  : std_logic_vector(7 downto 0);
    end record;
    type mob_array_t is array (0 to MOB_COUNT-1) of mob_reg_t;
    type data_array_t is array (0 to MOB_COUNT-1) of std_logic_vector(63 downto 0);

    -- State of each MOB (internal)
    type mob_state_t is (IDLE, TX_PENDING, RX_MATCHED, AUTO_REPLY, FBUF_WAIT);
    type mob_state_vec_t is array (0 to MOB_COUNT-1) of mob_state_t;

    -- CRC function (polynomial x^15 + x^14 + x^10 + x^8 + x^7 + x^4 + x^3 + 1)
    function crc_bit(crc_in : std_logic_vector(14 downto 0); data : std_logic) return std_logic_vector;

    -- Maximum frame length (SOF to EOF) = 128 bits for extended with 8 data bytes
    constant MAX_FRAME_BITS : integer := 128;
end package;

package body can_pkg is
    function crc_bit(crc_in : std_logic_vector(14 downto 0); data : std_logic) return std_logic_vector is
        variable nc : std_logic_vector(14 downto 0);
        variable fb : std_logic;
    begin
        fb := crc_in(14) xor data;
        nc(0) := fb;
        nc(1) := crc_in(0);
        nc(2) := crc_in(1);
        nc(3) := crc_in(2) xor fb;
        nc(4) := crc_in(3) xor fb;
        nc(5) := crc_in(4);
        nc(6) := crc_in(5);
        nc(7) := crc_in(6) xor fb;
        nc(8) := crc_in(7) xor fb;
        nc(9) := crc_in(8);
        nc(10):= crc_in(9) xor fb;
        nc(11):= crc_in(10);
        nc(12):= crc_in(11);
        nc(13):= crc_in(12);
        nc(14):= crc_in(13) xor fb;   -- CRITICAL FIX: add feedback to bit14
        return nc;
    end function;
end package body;
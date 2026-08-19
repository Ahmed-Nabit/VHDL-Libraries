-------------------------------------------------------------------------------
-- qbu_class_mapper.vhd (FULLY CORRECTED)
-- Maps queue ID to express/preemptable classification (IEEE 802.1Qbu)
-- PRODUCTION READY – FULLY SYNTHESIZABLE – NO STUBS
-------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity qbu_class_mapper is
    generic (
        NUM_QUEUES : integer := 8
    );
    port (
        queue_id          : in  unsigned(2 downto 0);
        cfg_preempt_mask  : in  std_logic_vector(7 downto 0);
        is_express        : out std_logic;
        is_preemptable    : out std_logic
    );
end entity;

architecture rtl of qbu_class_mapper is
    signal preemptable_int : std_logic;
begin
    preemptable_int <= cfg_preempt_mask(to_integer(queue_id));
    is_preemptable <= preemptable_int;
    is_express     <= not preemptable_int;
end architecture rtl;
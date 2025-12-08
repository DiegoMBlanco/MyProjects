library ieee;
use ieee.std_logic_1164.all;

entity BIN_7SEG is 
	port(binario : in std_logic_vector(13 downto 0); seg1, seg2, seg3, seg4 : out std_logic_vector (6 downto 0));
end BIN_7SEG;

architecture behav of BIN_7SEG is

	component BINARIO_A_BCD
		port( binario : in std_logic_vector (13 downto 0); bcd1, bcd2, bcd3, bcd4 : out std_logic_vector (3 downto 0));
	end component;
	
	component BCD_WHEN_A01665010
		port( A0, A1, A2, A3 , LT, RBI, RBO : in std_logic; salidas_abcdefg : out std_logic_vector (6 downto 0));
	end component;
	
	signal display1, display2, display3, display4 : std_logic_vector (3 downto 0);
	signal binario_signal : std_logic_vector(13 downto 0);
	
	begin
		
		U0 : BINARIO_A_BCD port map(binario, display1, display2, display3, display4);
		U1 : BCD_WHEN_A01665010 port map(display1(0), display1(1), display1(2), display1(3), '1', '1', '1', seg1);
		U2 : BCD_WHEN_A01665010 port map(display2(0), display2(1), display2(2), display2(3), '1', '1', '1', seg2);
		U3 : BCD_WHEN_A01665010 port map(display3(0), display3(1), display3(2), display3(3), '1', '1', '1', seg3);
		U4 : BCD_WHEN_A01665010 port map(display4(0), display4(1), display4(2), display4(3), '1', '1', '1', seg4);
	

	
	
	
end behav;
library ieee;
use ieee.std_logic_1164.all;

entity BCD_WHEN_A01665010 is
	port( A0, A1, A2, A3 , LT, RBI, RBO : in std_logic; salidas_abcdefg : out std_logic_vector (6 downto 0));
end BCD_WHEN_A01665010;

architecture behav of BCD_WHEN_A01665010 is
	signal inputs : std_logic_vector (3 downto 0);
	
	begin
		inputs <= A3 & A2 & A1 & A0;
		salidas_abcdefg <= "1000000" when inputs = "0000" else 
								 "1111001" when inputs = "0001" else
								 "0100100" when inputs = "0010" else
								 "0110000" when inputs = "0011" else
								 
								 "0011001" when inputs = "0100" else
								 "0010010" when inputs = "0101" else 
								 "0000010" when inputs = "0110" else
								 "1111000" when inputs = "0111" else
								 "0000000" when inputs = "1000" else
								 
								 "0011000" when inputs = "1001" else
								 "1111111";
			
	
end behav;
library ieee;
use ieee.std_logic_1164.all;

entity DIVISOR_CLOCK is

	generic( pulsos : integer := 1);
	port(clk_in: in std_logic; clk_out : out std_logic);
	
end DIVISOR_CLOCK;

architecture behav of DIVISOR_CLOCK is
	signal Z : std_logic;
	begin
		process (clk_in)
			variable contador : integer := 0;
				
			begin
				if rising_edge (clk_in) then
					contador := contador+1;
					if contador = pulsos then
						contador := 0;
						Z <= not Z;
					else
						Z <= Z;
					end if;
				else
					Z <= Z;
				end if;
					
		end process;
	clk_out <= Z;	
end behav;
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all; -- esta librería nos permite convertir 

entity BINARIO_A_BCD is
	port( binario : in std_logic_vector (13 downto 0); bcd1, bcd2, bcd3, bcd4 : out std_logic_vector (3 downto 0));
end BINARIO_A_BCD;

architecture behav of BINARIO_A_BCD is 
	begin
	process (binario)
		variable num : integer; -- numero real al que le vamos a restar
		begin 
			num := to_integer(unsigned(binario));
		-- millar
			if (num < 1000) then
				num := num - 0;
				bcd4 <= "0000";
			elsif (num < 2000) then
				num := num -1000;
				bcd4 <= "0001";
			elsif (num < 3000) then
				num := num -2000;
				bcd4 <= "0010";
			elsif (num < 4000) then
				num := num -3000;
				bcd4 <= "0011";
			elsif (num < 5000) then
				num := num -4000;
				bcd4 <= "0100";
			elsif (num < 6000) then
				num := num -5000;
				bcd4 <= "0101";
			elsif (num < 7000) then
				num := num -6000;
				bcd4 <= "0110";
			elsif (num < 8000) then
				num := num -7000;
				bcd4 <= "0111";
			elsif num < 9000 then
				num := num -8000;
				bcd4 <= "1000";
			else 
				num := num -9000;
				bcd4 <= "1001";
			end if;
			
		-- centena
			if num < 100 then
				num := num - 0;
				bcd3 <= "0000";
			elsif num < 200 then
				num := num -100;
				bcd3 <= "0001";
			elsif num < 300 then
				num := num -200;
				bcd3 <= "0010";
			elsif num < 400 then
				num := num -300;
				bcd3 <= "0011";
			elsif num < 500 then
				num := num -400;
				bcd3 <= "0100";
			elsif num < 600 then
				num := num -500;
				bcd3 <= "0101";
			elsif num < 700 then
				num := num -600;
				bcd3 <= "0110";
			elsif num < 800 then
				num := num -700;
				bcd3 <= "0111";
			elsif num < 900 then
				num := num -800;
				bcd3 <= "1000";
			else 
				num := num -900;
				bcd3 <= "1001";
			end if;
			
		-- decena
			if num < 10 then
				num := num - 0;
				bcd2 <= "0000";
			elsif num < 20 then
				num := num -10;
				bcd2 <= "0001";
			elsif num < 30 then
				num := num -20;
				bcd2 <= "0010";
			elsif num < 40 then
				num := num -30;
				bcd2 <= "0011";
			elsif num < 50 then
				num := num -40;
				bcd2 <= "0100";
			elsif num < 60 then
				num := num -50;
				bcd2 <= "0101";
			elsif num < 70 then
				num := num -60;
				bcd2 <= "0110";
			elsif num < 80 then
				num := num -70;
				bcd2 <= "0111";
			elsif num < 90 then
				num := num -80;
				bcd2 <= "1000";
			else 
				num := num -90;
				bcd2 <= "1001";
			end if;
			
			bcd1 <= std_logic_vector(to_unsigned(num, 4));
			
						
				
	end process;
end behav;
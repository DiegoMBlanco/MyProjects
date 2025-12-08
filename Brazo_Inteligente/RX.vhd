library ieee;
use ieee.std_logic_1164.all;

entity RX is
	port(
		entradaRX : in std_logic;
		CLK_div   : in std_logic;
		salida    : out std_logic_vector(7 downto 0); num, B0, B1, B2, B3, B4, B5, B6, B7, B8, B9 : out std_logic_vector(7 downto 0)
	);
end RX;

architecture behav of RX is

	type rx_type is (reposo, d0, d1, d2, d3, d4, d5, d6, d7, bit_stop);
	signal rx_estados : rx_type := reposo;
	signal salida_temp : std_logic_vector(7 downto 0) := (others => '0');
	signal bit_flag, bit_n : std_logic_vector (7 downto 0);

begin

	-- Lógica de recepción
	process (CLK_div)
	variable contador : integer := 0;
	begin
		if rising_edge(CLK_div) then
			case rx_estados is
				when reposo =>
					if entradaRX = '0' then  -- Detecta bit de inicio
						rx_estados <= d0;
					end if;
				
				when d0 =>
					salida_temp(0) <= entradaRX;
					rx_estados <= d1;
					
				when d1 =>
					salida_temp(1) <= entradaRX;
					rx_estados <= d2;

				when d2 =>
					salida_temp(2) <= entradaRX;
					rx_estados <= d3;

				when d3 =>
					salida_temp(3) <= entradaRX;
					rx_estados <= d4;

				when d4 =>
					salida_temp(4) <= entradaRX;
					rx_estados <= d5;

				when d5 =>
					salida_temp(5) <= entradaRX;
					rx_estados <= d6;

				when d6 =>
					salida_temp(6) <= entradaRX;
					rx_estados <= d7;

				when d7 =>
					salida_temp(7) <= entradaRX;
					 
					rx_estados <= bit_stop;

				when bit_stop =>
					if entradaRX = '1' then  -- Bit de parada
						bit_flag <= salida_temp;
						if bit_flag = "01101110" then
							bit_n <= "01101110"; -- n
						else
							if bit_n = "01101110" and contador < 11 then
								if contador = 0 then
									num <= salida_temp;
									contador := contador + 1;
								elsif contador = 1 then
									B0 <= salida_temp;
									contador := contador +1;
								elsif contador = 2 then
									B1 <= salida_temp;
									contador := contador +1;
								elsif contador = 3 then
									B2 <= salida_temp;
									contador := contador +1;
								elsif contador = 4 then
									B3 <= salida_temp;
									contador := contador +1;
								elsif contador = 5 then
									B4 <= salida_temp;
									contador := contador +1;
								elsif contador = 6 then
									B5 <= salida_temp;
									contador := contador +1;
								elsif contador = 7 then
									B6 <= salida_temp;
									contador := contador +1;
								elsif contador = 8 then
									B7 <= salida_temp;
									contador := contador +1;
								elsif contador = 9 then
									B8 <= salida_temp;
									contador := contador +1;
								elsif contador = 10 then
									B9 <= salida_temp;
									contador := 0;
									bit_n <= "00000000";
								else
									
									
								end if;
								
							end if;
						end if;
						salida <= salida_temp;  -- Transferencia final del dato recibido
						rx_estados <= reposo;
					else
						rx_estados <= reposo; -- Si hay error, vuelve a reposo
					end if;

				when others =>
					rx_estados <= reposo;

			end case;
		end if;
	end process;

end behav;

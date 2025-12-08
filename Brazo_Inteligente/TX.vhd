library ieee;
use ieee.std_logic_1164.all;

entity TX is
	port(data_in : in std_logic_vector (7 downto 0);
		  boton : in std_logic;
		  CLK_div : in std_logic;
		  data_out : out std_logic);
end TX;

architecture behav of TX is

	type tx_type is (reposo, bit_inicio, d0, d1, d2, d3, d4, d5, d6, d7, bit_stop); -- reposo es mi boton
	signal tx_estados : tx_type;
	
	begin
	
	--lógica de transmisión
		process(CLK_div, boton)
			begin
				if rising_edge (CLK_div) then
					case tx_estados is
						when reposo =>
							if boton = '0' then
								tx_estados <= bit_inicio;
							else
								tx_estados <= tx_estados;
							end if;
							
						when bit_inicio =>tx_estados <= d0;
						when d0 =>tx_estados <= d1;
						when d1 =>tx_estados <= d2;
						when d2 =>tx_estados <= d3;
						when d3 =>tx_estados <= d4;
						when d4 =>tx_estados <= d5;
						when d5 =>tx_estados <= d6;
						when d6 =>tx_estados <= d7;
						when d7 =>tx_estados <= bit_stop;
							
						when bit_stop =>
							if boton = '0' then -- así garantizamos que funicone el botón
								tx_estados <= tx_estados;
							else
								tx_estados <= reposo;
							end if;
						when others => tx_estados <= reposo;
					end case;
				else	
					tx_estados <= tx_estados;
				end if;
		end process;
		
		--logica de salidas
		process(tx_estados)
			begin
				case tx_estados is
					when reposo =>data_out <= '1';
					when bit_inicio => data_out<= '0'; --indica bit de inicio
					when d0 => data_out<= data_in(0); -- se envía del menos significativo al más significativo (el de la derecha)
					when d1 => data_out<= data_in(1);
					when d2 => data_out<= data_in(2);
					when d3 => data_out<= data_in(3);
					when d4 => data_out<= data_in(4);
					when d5 => data_out<= data_in(5);
					when d6 => data_out<= data_in(6);
					when d7 => data_out<= data_in(7);
					when bit_stop => data_out<= '1';
					when others => data_out<= '0';
				end case;
		
		end process;
		
		
end behav;

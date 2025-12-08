library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity PRUEBA_VGA is
	port(CLK, boton, reset : in std_logic; R, G, B : out std_logic_vector(3 downto 0); switches : in std_logic_vector (7 downto 0);
			RX_signal, btn : in std_logic;TX_signal, pwm1, pwm2, pwm3, pwm4, pwm5: out std_logic; bits : out std_logic_vector (7 downto 0);
        VS, HS : out std_logic;
		  display1, display2, display3, display4 : out std_logic_vector (6 downto 0));
end PRUEBA_VGA;

architecture behav of PRUEBA_VGA is

	component top_level is
    port (
        clk     : in  std_logic;
        reset   : in  std_logic;
		  number : in std_logic_vector (3 downto 0);
        servo1_pwm : out std_logic;
        servo2_pwm : out std_logic;
		  servo3_pwm : out std_logic;
        servo4_pwm : out std_logic
    );
   end component;

	component VGA_RETO is
    port(
        CLK : in std_logic; B0, B1, B2, B3, B4, B5, B6, B7, B8, B9 : in std_logic_vector(7 downto 0);
        R, G, B : out std_logic_vector(3 downto 0); 
        VS, HS : out std_logic
    );
	end component;
	
	component SERIAL is
	port(RX_signal, CLK, btn : in std_logic;
		  switches : in std_logic_vector (7 downto 0);
		  leds : out std_logic_vector (7 downto 0);
	     TX_signal : out std_logic; num, B0, B1, B2, B3, B4, B5, B6, B7, B8, B9 : out std_logic_vector(7 downto 0));
	end component;
	
	component BIN_7SEG is 
		port(binario : in std_logic_vector(13 downto 0); seg1, seg2, seg3, seg4 : out std_logic_vector (6 downto 0));
	end component;
	
	component MOTORES is
		 Port (
			  clk : in STD_LOGIC;  -- Reloj de 50 MHz
			  numero : in STD_LOGIC_VECTOR(3 downto 0); -- Posición de 0 a 9
			  pwm_out : out STD_LOGIC  -- Señal PWM para el servomotor
		 );
	end component;
	
	
	
	signal B0, B1, B2, B3, B4, B5, B6, B7, B8, B9 : std_logic_vector(7 downto 0);
	signal num : std_logic_vector (7 downto 0);
	signal num_temp : integer := 0;
	signal numero_adivinado : std_logic_vector (13 downto 0);
	signal numero_enviado_motor : std_logic_vector (3 downto 0);

	begin
	
	U0: VGA_RETO port map (CLK, B0, B1, B2, B3, B4, B5, B6, B7, B8, B9, R, G, B, Vs, Hs);
	U1: SERIAL port map (RX_signal, CLK, btn, switches, bits, TX_signal,num,B0, B1, B2, B3, B4, B5, B6, B7, B8, B9);
	U2: BIN_7SEG port map (numero_adivinado, display1, display2, display3, display4);
	U3: MOTORES port map (CLK, numero_enviado_motor, pwm5);
	U4 : top_level port map (CLK, reset, numero_enviado_motor, pwm1, pwm2, pwm3, pwm4);

	num_temp <= to_integer(unsigned("000000" & num)) - 48;
	numero_adivinado <= std_logic_vector(to_unsigned(num_temp, 14));
	
	process (boton)
		begin
			if boton = '1' then
				numero_enviado_motor <= std_logic_vector(to_unsigned(num_temp, 4));
			else
				numero_enviado_motor <= "1111";
			end if;
	end process;
	
	
end behav;


















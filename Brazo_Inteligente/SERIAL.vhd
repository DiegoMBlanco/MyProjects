library ieee;
use ieee.std_logic_1164.all;

entity SERIAL is
	port(RX_signal, CLK, btn : in std_logic;
		  switches : in std_logic_vector (7 downto 0);
		  leds : out std_logic_vector (7 downto 0);
	     TX_signal : out std_logic; num, B0, B1, B2, B3, B4, B5, B6, B7, B8, B9 : out std_logic_vector(7 downto 0));
end SERIAL;

architecture behav of SERIAL is

	component DIVISOR_CLOCK is
		generic( pulsos : integer := (50000000)/(2*9600)); -- 9600 baudios
		port(clk_in: in std_logic; clk_out : out std_logic);
	end component;

	component TX is
		port(data_in : in std_logic_vector (7 downto 0);
			  boton : in std_logic;
			  CLK_div : in std_logic;
			  data_out : out std_logic);
	end component;
	
	component RX is
	port(
		entradaRX : in std_logic;
		CLK_div   : in std_logic;
		salida    : out std_logic_vector(7 downto 0); num, B0, B1, B2, B3, B4, B5, B6, B7, B8, B9 : out std_logic_vector(7 downto 0)
	);
end component;
	
	
	signal CLK_9600 : std_logic;
	
	begin
	
		U0: DIVISOR_CLOCK port map(CLK, CLK_9600);
		U1: TX port map (switches, btn, CLK_9600, TX_signal);
		U2: RX port map (RX_signal, CLK_9600, leds,num, B0, B1, B2, B3, B4, B5, B6, B7, B8, B9);
		
		
end behav;
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

entity top_level is
    port (
        clk     : in  std_logic;
        reset   : in  std_logic;
		  number : in std_logic_vector (3 downto 0);
        servo1_pwm : out std_logic;
        servo2_pwm : out std_logic;
		  servo3_pwm : out std_logic;
        servo4_pwm : out std_logic
    );
end top_level;

architecture Behavioral of top_level is

	component DIVISOR_CLOCK is

	generic( pulsos : integer := 25000000);
	port(clk_in: in std_logic; clk_out : out std_logic);
	
	end component;

 
    signal target1 : integer range 544 to 2400 := 1500; -- mano
    signal speed1  : integer range 0 to 255 := 50;
    signal target2 : integer range 544 to 2400 := 1500; -- codo
    signal speed2  : integer range 0 to 255 := 50;
	 signal target3 : integer range 544 to 2400 := 1500; -- hombro
    signal speed3  : integer range 0 to 255 := 50;
    signal target4 : integer range 544 to 2400 := 1500; -- base
    signal speed4  : integer range 0 to 255 := 50;
    signal write_en : std_logic := '1'; 
	 signal target_value : integer;
	 signal clk_div : std_logic;
	 
	 type numeros_type is (zero, a_inicio, a_bajar, a_pintar, a_levantar, 
	                             b_inicio, b_bajar, b_pintar, b_levantar,
										  c_inicio, c_bajar, c_pintar, c_levantar,
										  d_inicio, d_bajar, d_pintar, d_levantar,
										  e_inicio, e_bajar, e_pintar, e_levantar,
										  f_inicio, f_bajar, f_pintar, f_levantar,
										  g_inicio, g_bajar, g_pintar, g_levantar);
	signal estados : numeros_type;

begin

	U0 : DIVISOR_CLOCK port map (clk, clk_div);
	
	
	
	
	
	
	
	
	process(clk_div, number)
	begin
		if rising_edge (clk_div) then
			case estados is
				when zero =>
					 target1 <= 1400;
					 target2 <= 544;
					 target3 <= 1300;
					 target4 <= 1350;
					 if  number = "0000" then
						estados <= e_inicio;
					 
					 elsif number = "0001" then
						estados <= b_inicio;
						
					 elsif number = "0010" then
						estados <= a_inicio;
					 
					 elsif number = "0011" then
						estados <= a_inicio;
					 
					 elsif number = "0100" then
						estados <= f_inicio;
					 
					 elsif number = "0101" then
						estados <= a_inicio;
					 
					 elsif number = "0110" then
						estados <= a_inicio;
					 
					elsif number = "0111" then
						estados <= a_inicio;
						
					 elsif number = "1000" then
						estados <= e_inicio;
						
					 elsif number = "1001" then
						estados <= f_inicio;
						
					 elsif number = "1111" then
						estados <= zero;
					 else 
						estados <= zero;
					end if;
					
				when a_inicio =>
					target1 <= 1400;
	            target2 <= 1100;
	            target3 <= 644;
	            target4 <= 1330;
					
					estados <= a_bajar;
					
				when a_bajar =>
					target1 <= 1400;
	            target2 <= 1100;
	            target3 <= 544;
	            target4 <= 1330;
					
					estados <= a_pintar;
					
				when a_pintar =>
					target1 <= 1400;
	            target2 <= 700;
	            target3 <= 700;
	            target4 <= 1350;
					estados <= a_levantar;
						
						
				when a_levantar =>
					 
					 if  number = "0000" then
						estados <= b_inicio;
						
					 elsif number = "0010" then
						estados <= b_inicio;
					 
					 elsif number = "0011" then
						estados <= b_inicio;
			
					 
					 elsif number = "0101" then
						estados <= f_inicio;
					 
					 elsif number = "0110" then
						estados <= f_inicio;
					 
					elsif number = "0111" then
						estados <= b_inicio;
						
					 elsif number = "1000" then
						estados <= b_inicio;
						
					 elsif number = "1001" then
						estados <= b_inicio;
						
					 elsif number = "1111" then
						estados <= zero;
						target1 <= 1400;
					   target2 <= 700;
	               target3 <= 900;
	               target4 <= 1350;
					 else 
						estados <= zero;
						target1 <= 1400;
					   target2 <= 700;
	               target3 <= 900;
	               target4 <= 1350;
					end if;
						
					
				when b_inicio =>
					target1 <= 1450;
	            target2 <= 700;
	            target3 <= 800;
	            target4 <= 1350;
					estados <= b_bajar;
					
				when b_bajar =>
					target1 <= 1450;
	            target2 <= 700;
	            target3 <= 700;
	            target4 <= 1350;
					
					estados <= b_pintar;
					
				when b_pintar =>
					target1 <= 1450;
	            target2 <= 700;
	            target3 <= 700;
	            target4 <= 1450;
					estados <= b_levantar;
						
					
				when b_levantar =>
					 
					 if  number = "0000" then
						estados <= c_inicio;
					 
					 elsif number = "0001" then
						estados <= c_inicio;
						
					 elsif number = "0010" then
						estados <= g_inicio;
					 
					 elsif number = "0011" then
						estados <= g_inicio;
					 
					 elsif number = "0100" then
						estados <= c_inicio;
					
					 
					elsif number = "0111" then
						estados <= c_inicio;
						
					 elsif number = "1000" then
						estados <= g_inicio;
						
					 elsif number = "1001" then
						estados <= g_inicio;
						
					 elsif number = "1111" then
						estados <= zero;
						target1 <= 1450;
					   target2 <= 700;
	               target3 <= 900;
	               target4 <= 1450;
					 else 
						estados <= zero;
						target1 <= 1450;
					   target2 <= 700;
	               target3 <= 900;
	               target4 <= 1450;
					end if;
					 
					 
				when c_inicio =>
					target1 <= 1450;
	            target2 <= 700;
	            target3 <= 800;
	            target4 <= 1450;
					estados <= c_bajar;
					
				when c_bajar =>
					target1 <= 1450;
	            target2 <= 700;
	            target3 <= 700;
	            target4 <= 1450;
					
					estados <= c_pintar;
					
				when c_pintar =>
					target1 <= 1450;
	            target2 <= 700;
	            target3 <= 700;
	            target4 <= 1570;
					estados <= c_levantar;
						
						
				when c_levantar =>
					 
					 if  number = "0000" then
						estados <= d_inicio;
					 
					 elsif number = "0001" then
						estados <= zero;
						target1 <= 1450;
					   target2 <= 700;
	               target3 <= 900;
	               target4 <= 1500;
					 
					 elsif number = "0011" then
						estados <= d_inicio;
					 
					 elsif number = "0100" then
						estados <= zero;
						target1 <= 1450;
					   target2 <= 700;
	               target3 <= 900;
	               target4 <= 1500;
					 
					 elsif number = "0101" then
						estados <= d_inicio;
					 
					 elsif number = "0110" then
						estados <= d_inicio;
					 
					elsif number = "0111" then
						estados <= zero;
						target1 <= 1450;
					   target2 <= 700;
	               target3 <= 900;
	               target4 <= 1500;
						
					 elsif number = "1000" then
						estados <= d_inicio;
						
					 elsif number = "1001" then
						estados <= zero;
						target1 <= 1450;
					   target2 <= 700;
	               target3 <= 900;
	               target4 <= 1500;
						
					 elsif number = "1111" then
						estados <= zero;
						target1 <= 1450;
					   target2 <= 700;
	               target3 <= 900;
	               target4 <= 1500;
					 else 
						estados <= zero;
						target1 <= 1450;
					   target2 <= 700;
	               target3 <= 900;
	               target4 <= 1500;
					end if;
						
						
						
				when d_inicio =>
					target1 <= 1400;
	            target2 <= 1100;
	            target3 <= 644;
	            target4 <= 1570;
					estados <= d_bajar;
					
				when d_bajar =>
					target1 <= 1400;
	            target2 <= 1100;
	            target3 <= 544;
	            target4 <= 1570;
					estados <= d_pintar;
					
				when d_pintar =>
					target1 <= 1400;
	            target2 <= 700;
	            target3 <= 700;
	            target4 <= 1570;
					estados <= d_levantar;
					
						
						
				when d_levantar =>
					
					 if  number = "0000" then
						estados <= zero;
						target1 <= 1400;
					   target2 <= 700;
	               target3 <= 900;
	               target4 <= 1500;
					 

						
					 elsif number = "0010" then
						estados <= zero;
						target1 <= 1400;
					   target2 <= 700;
	               target3 <= 900;
	               target4 <= 1500;
						
					 
					 elsif number = "0011" then
						estados <= zero;
						target1 <= 1400;
					   target2 <= 700;
	               target3 <= 900;
	               target4 <= 1500;
					 
					 
					 elsif number = "0101" then
						estados <= zero;
						target1 <= 1400;
					   target2 <= 700;
	               target3 <= 900;
	               target4 <= 1500;
					 
					 elsif number = "0110" then
						estados <= zero;
						target1 <= 1400;
					   target2 <= 700;
	               target3 <= 900;
	               target4 <= 1500;
					 

						
					 elsif number = "1000" then
						estados <= zero;
						target1 <= 1400;
					   target2 <= 700;
	               target3 <= 900;
	               target4 <= 1500;
						
			
						
					 elsif number = "1111" then
						estados <= zero;
						target1 <= 1400;
					   target2 <= 700;
	               target3 <= 900;
	               target4 <= 1500;
					 else 
						estados <= zero;
						target1 <= 1400;
					   target2 <= 700;
	               target3 <= 900;
	               target4 <= 1500;
					end if;
					 
					 
					 
				when e_inicio =>
					target1 <= 1500;
	            target2 <= 1100;
	            target3 <= 644;
	            target4 <= 1450;
					estados <= e_bajar;
					
				when e_bajar =>
					target1 <= 1400;
	            target2 <= 1100;
	            target3 <= 544;
	            target4 <= 1450;
					estados <= e_pintar;
					
					
				when e_pintar =>
					target1 <= 1400;
	            target2 <= 1100;
	            target3 <= 544;
	            target4 <= 1570;
					estados <= e_levantar;
						
						
				when e_levantar =>
					 
					 if  number = "0000" then
						estados <= f_inicio;
					 
			
						
					 elsif number = "0010" then
						estados <= d_inicio;
					 
			
					 
					 elsif number = "0110" then
						estados <= g_inicio;
					 
		
						
					 elsif number = "1000" then
						estados <= f_inicio;
						
						
					 elsif number = "1111" then
						estados <= zero;
						target1 <= 1300;
					   target2 <= 1100;
	               target3 <= 900;
	               target4 <= 1570;
					 else 
						estados <= zero;
						target1 <= 1300;
					   target2 <= 1100;
	               target3 <= 900;
	               target4 <= 1570;
					end if;
						
						
						
				when f_inicio =>
					target1 <= 1400;
	            target2 <= 1100;
	            target3 <= 644;
	            target4 <= 1330;
					estados <= f_bajar;
					
				when f_bajar =>
					target1 <= 1400;
	            target2 <= 1100;
	            target3 <= 544;
	            target4 <= 1330;
					estados <= f_pintar;
					
					
				when f_pintar =>
					target1 <= 1400;
	            target2 <= 1100;
	            target3 <= 544;
	            target4 <= 1450;
					estados <= f_levantar;
						
						
				when f_levantar =>
					 
					 if  number = "0000" then
						estados <= a_inicio;
					 
					
						
					
				
					 elsif number = "0100" then
						estados <= g_inicio;
					 
					 elsif number = "0101" then
						estados <= g_inicio;
					 
					 elsif number = "0110" then
						estados <= e_inicio;
			
						
					 elsif number = "1000" then
						estados <= a_inicio;
						
					 elsif number = "1001" then
						estados <= a_inicio;
						
					 elsif number = "1111" then
						estados <= zero;
						target1 <= 1300;
					  target2 <= 1100;
	              target3 <= 900;
	              target4 <= 1450;
					 else 
						estados <= zero;
						target1 <= 1300;
					 target2 <= 1100;
	             target3 <= 900;
	             target4 <= 1450;
					end if;
						
						
						
				when g_inicio =>
					target1 <= 1500;
	            target2 <= 1100;
	            target3 <= 544;
	            target4 <= 1450;
					estados <= g_bajar;
					
				when g_bajar =>
					target1 <= 1450;
	            target2 <= 1100;
	            target3 <= 700;
	            target4 <= 1450;
					estados <= g_pintar;
					
				when g_pintar =>
					target1 <= 1450;
	            target2 <= 700;
	            target3 <= 700;
	            target4 <= 1450;
					estados <= g_levantar;
						
						
				when g_levantar =>
					
		
					 if number = "0010" then
						estados <= e_inicio;
					 
					 elsif number = "0011" then
						estados <= c_inicio;
					 
					 elsif number = "0100" then
						estados <= b_inicio;
					 
					 elsif number = "0101" then
						estados <= c_inicio;
					 
					 elsif number = "0110" then
						estados <= c_inicio;
					 
				
						
					 elsif number = "1000" then
						estados <= d_inicio;
						
						
					 elsif number = "1001" then
						estados <= c_inicio;
						
						
					 elsif number = "1111" then
						estados <= zero;
						target1 <= 1300;
					  target2 <= 700;
	              target3 <= 900;
	              target4 <= 1450;
					 else 
						estados <= zero;
						target1 <= 1300;
					  target2 <= 700;
	              target3 <= 900;
	              target4 <= 1450;
					end if;
					
					
				when others =>
					estados <= zero;
					
			end case;
		else
		end if;
	end process;
	


    
	 
	 
	 
				  
    -- Servo 1 Controller
    servo1: entity work.servo_controller
        port map (
            clk => clk,
            reset => reset,
            target_us => target1,
            speed_us => speed1,
            write_en => write_en,
            pwm_out => servo1_pwm
        );

    -- Servo 2 Controller
    servo2: entity work.servo_controller
        port map (
            clk => clk,
            reset => reset,
            target_us => target2,
            speed_us => speed2,
            write_en => write_en,
            pwm_out => servo2_pwm
        );
		  
		  -- Servo 3 Controller
    servo3: entity work.servo_controller
        port map (
            clk => clk,
            reset => reset,
            target_us => target3,
            speed_us => speed3,
            write_en => write_en,
            pwm_out => servo3_pwm
        );

    -- Servo 4 Controller
    servo4: entity work.servo_controller
        port map (
            clk => clk,
            reset => reset,
            target_us => target4,
            speed_us => speed4,
            write_en => write_en,
            pwm_out => servo4_pwm
        );


end Behavioral;
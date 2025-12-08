library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_ARITH.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;

entity MOTORES is
    Port (
        clk : in STD_LOGIC;  
        numero : in STD_LOGIC_VECTOR(3 downto 0); 
        pwm_out : out STD_LOGIC
    );
end MOTORES;

architecture Behavioral of MOTORES is
    constant CLK_FREQ : integer := 50000000; 
    constant PWM_PERIOD : integer := 20000000; -- 20 ms en ciclos de reloj

    signal count : integer := 0;
    signal pulse_width : integer := 0;

begin
    process (clk)
    begin
        if rising_edge(clk) then
            if count < PWM_PERIOD then
                count <= count + 1;
            else
                count <= 0;
                
                -- Tomando en cuenta que 0.7 ms equivale a 0°, y 2.3 ms a 180°, cada número de la ruleta es equivalente a 18° = 180 microsegundos
                case numero is
                    when "0000" => pulse_width <=  700  * 50;  -- 0°
                    when "0001" => pulse_width <=  880  * 50;  -- 18°
                    when "0010" => pulse_width <= 1060 * 50;  -- 36°
                    when "0011" => pulse_width <= 1240 * 50;  -- 54°
                    when "0100" => pulse_width <= 1420 * 50;  -- 72°
                    when "0101" => pulse_width <= 1500 * 50;  -- 90°
                    when "0110" => pulse_width <= 1660 * 50;  -- 108°
                    when "0111" => pulse_width <= 1840 * 50;  -- 126°
                    when "1000" => pulse_width <= 2020 * 50;  -- 144°
                    when "1001" => pulse_width <= 2300 * 50;  -- 162°
                    when others => pulse_width <= 1500 * 50;  -- Default a 90°
                end case;
            end if;

            if count < pulse_width then
                pwm_out <= '1';
            else
                pwm_out <= '0';
            end if;
        end if;
    end process;
end Behavioral;


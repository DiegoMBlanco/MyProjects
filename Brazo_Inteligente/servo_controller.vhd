library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity servo_controller is
    port (
        clk         : in  std_logic;
        reset       : in  std_logic;
        target_us   : in  integer range 544 to 2400;  
        speed_us    : in  integer range 0 to 255;     -
        write_en    : in  std_logic;                   
        pwm_out     : out std_logic                   
    );
end servo_controller;

architecture Behavioral of servo_controller is
    constant CLK_FREQ      : integer := 50000000;  
    constant PERIOD_US     : integer := 20000;       
    constant CYCLES_PER_US : integer := 50;           

    signal target_cycles   : integer := 1500 * CYCLES_PER_US;  
    signal current_cycles  : integer := 1500 * CYCLES_PER_US;
    signal speed_cycles    : integer := 0;

    signal period_counter  : integer := 0;  
    signal pwm_counter     : integer := 0;  
begin

    -- Habilita la actualización de nuestro grado objetivo y de la velocidad
    process(clk, reset)
    begin
        if reset = '1' then
            target_cycles <= 1500 * CYCLES_PER_US;
            speed_cycles <= 0;
        elsif rising_edge(clk) then
            if write_en = '1' then
                target_cycles <= target_us * CYCLES_PER_US;
                speed_cycles <= speed_us * CYCLES_PER_US;
            end if;
        end if;
    end process;

    -- actualiza los ciclos actuales cada 20 ms
    process(clk, reset)
        variable next_current : integer;
    begin
        if reset = '1' then
            current_cycles <= 1500 * CYCLES_PER_US;
            period_counter <= 0;
        elsif rising_edge(clk) then
            -
            if period_counter < (PERIOD_US * CYCLES_PER_US) - 1 then
                period_counter <= period_counter + 1;
            else
                period_counter <= 0;

                -- actualizar posición
                if speed_cycles = 0 then
                    current_cycles <= target_cycles;  
                else
                    if current_cycles < target_cycles then
                        next_current := current_cycles + speed_cycles;
                        if next_current > target_cycles then
                            current_cycles <= target_cycles;
                        else
                            current_cycles <= next_current;
                        end if;
                    else
                        next_current := current_cycles - speed_cycles;
                        if next_current < target_cycles then
                            current_cycles <= target_cycles;
                        else
                            current_cycles <= next_current;
                        end if;
                    end if;
                end if;
            end if;
        end if;
    end process;

    -- proceso para generar señal pwm
    process(clk, reset)
    begin
        if reset = '1' then
            pwm_counter <= 0;
            pwm_out <= '0';
        elsif rising_edge(clk) then
            
            if pwm_counter >= (PERIOD_US * CYCLES_PER_US) - 1 then
                pwm_counter <= 0;
            else
                pwm_counter <= pwm_counter + 1;
            end if;

            
            if pwm_counter < current_cycles then
                pwm_out <= '1';
            else
                pwm_out <= '0';
            end if;
        end if;
    end process;

end Behavioral;
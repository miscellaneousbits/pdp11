library ieee;
use ieee.std_logic_1164.all;
 
entity debounceN is
    Generic (   
        N : positive;                                                      -- input bus width
        CNT_VAL : positive := 1000000);                                           -- clock counts for debounce period
    Port (  
        clk_i : in std_logic := 'X';                                            -- system clock
        data_i : in std_logic_vector (N-1 downto 0) := (others => 'X');         -- noisy input data
        data_o : out std_logic_vector (N-1 downto 0)                           -- registered stable output data
    );                      
end debounceN;
 
architecture implementation of debounceN is
    -- datapath pipeline 
    signal reg_A, reg_B : std_logic_vector (N-1 downto 0) := (others => '0');   -- debounce edge detectors
    signal reg_out : std_logic_vector (N-1 downto 0) := (others => '0');        -- registered output
    signal dat_strb : std_logic := '0';                                         -- data transfer strobe
    --signal strb_reg : std_logic := '0';                                         -- registered strobe
    signal dat_diff : std_logic := '0';                                         -- edge detector
    -- debounce counter
    signal cnt_reg : integer range CNT_VAL + 1 downto 0 := 0;                   -- debounce period counter
    signal cnt_next : integer range CNT_VAL + 1 downto 0 := 0;                  -- combinatorial signal
	 signal strb_o : std_logic := '0';                                                 -- strobe for new data available

begin
 
    cnt_reg_proc: process (clk_i) is
    begin
        if clk_i'event and clk_i = '1' then
            cnt_reg <= cnt_next;
        end if;
    end process cnt_reg_proc;
    cnt_next_proc: cnt_next <=  0 when dat_diff = '1' or dat_strb = '1' else cnt_reg + 1;
    final_cnt_proc: dat_strb <= '1' when cnt_reg = CNT_VAL else '0';

    pipeline_proc: process (clk_i) is
    begin
        if clk_i'event and clk_i = '1' then
            reg_A <= data_i;
            reg_B <= reg_A;
        end if;
        if clk_i'event and clk_i = '1' then
            if dat_strb = '1' then
                reg_out <= reg_B;
            end if;
        end if;
    end process pipeline_proc;
    edge_detector_proc: dat_diff <= '1' when reg_A /= reg_B else '0';
 
    data_o_proc:    data_o <= reg_out;
end implementation;

library ieee;
use ieee.std_logic_1164.all;
 
entity debounce1 is
    Generic (   
        CNT_VAL : positive := 1000000);
    Port (  
        clk_i : in std_logic := 'X';
        data_i : in std_logic;
        data_o : out std_logic
    );                      
end debounce1;
 
architecture implementation of debounce1 is

	component debounceN is
	Generic (   
		N : positive;
		CNT_VAL : positive
	);
	Port (  
		clk_i : in std_logic := 'X';
		data_i : in std_logic_vector (N-1 downto 0) := (others => 'X');
		data_o : out std_logic_vector (N-1 downto 0)
	);
	end component;
	
	signal data_i_v : std_logic_vector(0 downto 0);
	signal data_o_v : std_logic_vector(0 downto 0);
	
begin
	data_i_v(0) <= data_i;
	data_o <= data_o_v(0);
	
	db: debounceN
	generic map(CNT_VAL => CNT_VAL, N => 1)
	port map(
		clk_i => clk_i,
		data_i => data_i_v,
		data_o => data_o_v
	);
	
end implementation;

library ieee;
use ieee.std_logic_1164.all;
 
entity oneshot is
    Port (  
        clk_i : in std_logic := 'X';
        data_i : in std_logic;
        data_o : out std_logic
    );                      
end oneshot;
 
architecture implementation of oneshot is
	
	signal last_data : std_logic;
	signal current_data : std_logic;
	
begin
	process(clk_i)
	begin
      if clk_i = '1' and clk_i'event then
			last_data <= current_data;
			current_data <= data_i;
			if last_data /= current_data and current_data = '1' then
				data_o <= '1';
			else
				data_o <= '0';
			end if;
		end if;
	end process;
		
end implementation;
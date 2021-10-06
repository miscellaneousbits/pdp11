
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_ARITH.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;
use ieee.numeric_std.all;

entity paneldriver is
   port(
      cons_load : out std_logic;
      cons_exa : out std_logic;
      cons_dep : out std_logic;
      cons_cont : out std_logic;
      cons_ena : out std_logic;
      cons_inst : out std_logic;
      cons_start : out std_logic;
      cons_sw : out std_logic_vector(21 downto 0);
      cons_adss_mode : out std_logic_vector(1 downto 0);
      cons_adss_id : out std_logic;
      cons_adss_cons : out std_logic;
		cons_dsel : out integer range 0 to 3;

      cons_consphy : in std_logic_vector(21 downto 0);
      cons_progphy : in std_logic_vector(21 downto 0);
      cons_shfr : in std_logic_vector(15 downto 0);
      cons_maddr : in std_logic_vector(15 downto 0);                 -- microcode address fpu/cpu
      cons_br : in std_logic_vector(15 downto 0);
      cons_dr : in std_logic_vector(15 downto 0);
      cons_parh : in std_logic;
      cons_parl : in std_logic;

      cons_adrserr : in std_logic;
      cons_run : in std_logic;
      cons_pause : in std_logic;
      cons_master : in std_logic;
      cons_kernel : in std_logic;
      cons_super : in std_logic;
      cons_user : in std_logic;
      cons_id : in std_logic;
      cons_map16 : in std_logic;
      cons_map18 : in std_logic;
      cons_map22 : in std_logic;
      cons_reset : out std_logic;                                    -- a request for a reset from the console
		
		panel_reset : in std_logic;
		panel_sw: in std_logic_vector(17 downto 0);
		panel_pbin : in std_logic;
		panel_pbsel : out std_logic_vector(2 downto 0);

      clkin : in std_logic;
      reset : in std_logic
   );
end paneldriver;

architecture implementation of paneldriver is

component debounceN is
   Generic (   
        N : positive := 8;
        CNT_VAL : positive := 100000);
	port (
	     clk_i : in std_logic := 'X';
        data_i : in std_logic_vector (N-1 downto 0) := (others => 'X');
        data_o : out std_logic_vector (N-1 downto 0)
	);
end component;

component debounce1 is
	port (
	     clk_i : in std_logic := 'X';
        data_i : in std_logic;
        data_o : out std_logic
	);
end component;

component oneshot is
    Port (  
        clk_i : in std_logic := 'X';
        data_i : in std_logic;
        data_o : out std_logic
    );                      
end component;

signal panel_sw_db : std_logic_vector(17 downto 0);

signal sel : integer range 0 to 3 := 0;
signal resetdb : std_logic;

constant intvl : integer := 500000;

signal pb_count : integer range 0 to 7 := 0;
signal intvl_count : integer range 0 to intvl - 1;
signal toggle : std_logic := '0';

signal dseldb : std_logic := '0';
signal depdb :std_logic := '0';
signal startdb :std_logic := '0';
signal instdb :std_logic := '0';
signal enadb :std_logic := '0';
signal contdb :std_logic := '0';
signal exadb  :std_logic := '0';
signal loaddb :std_logic := '0';

signal dselos :std_logic := '0';

begin

	cons_sw <= "0000" & panel_sw_db;
   cons_adss_cons <= '0';
	cons_adss_mode <= "00";
	cons_adss_id <= '0';
	cons_dsel <= sel;
	
	dbounce_reset: debounce1
	port map (
		clk_i => clkin,
      data_i => not panel_reset,
      data_o => resetdb
	);

	-- the direct reset switch
	oneshot_reset: oneshot
	port map (
		clk_i => clkin,
      data_i => resetdb,
      data_o => cons_reset
	);

	-- scanned switches
	oneshot_dep: oneshot
	port map (
		clk_i => clkin,
      data_i => depdb,
      data_o => cons_dep
	);

	oneshot_start: oneshot
	port map (
		clk_i => clkin,
      data_i => startdb,
      data_o => cons_start
	);

	oneshot_inst: oneshot
	port map (
		clk_i => clkin,
      data_i => instdb,
      data_o => cons_inst
	);

	oneshot_ena: oneshot
	port map (
		clk_i => clkin,
      data_i => enadb,
      data_o => cons_ena
	);

	oneshot_cont: oneshot
	port map (
		clk_i => clkin,
      data_i => contdb,
      data_o => cons_cont
	);

	oneshot_exa: oneshot
	port map (
		clk_i => clkin,
      data_i => exadb,
      data_o => cons_exa
	);

	oneshot_load: oneshot
	port map (
		clk_i => clkin,
      data_i => loaddb,
      data_o => cons_load
	);

	oneshot_dsel: oneshot
	port map (
		clk_i => clkin,
      data_i => dseldb,
      data_o => dselos
	);

	-- the data switches
	debounce_sw: debounceN
	generic map (N => 18)
	port map (
		clk_i => clkin,
      data_i => not panel_sw,
      data_o => panel_sw_db
	);

	-- roll the data selector
	process(clkin, reset)
	begin
      if clkin = '1' and clkin'event then
         if reset = '1' then
				sel <= 0;
			else
				if (dselos = '1') then
					if sel = 7 then
						sel <= 0;
					else
						sel <= sel + 1;
					end if;
				end if;
			end if;
		end if;
	end process;
	
	-- scan switches
	process (clkin)
	begin
      if clkin = '1' and clkin'event then
         if reset = '1' then
				pb_count <= 0;
				intvl_count <= 0;
				toggle <= '0';
			else
				if intvl_count = intvl - 1 then
					intvl_count <= 0;
				else
					intvl_count <= intvl_count + 1;
				end if;
			end if;
			if intvl_count = 0 then
				toggle <= not toggle;
				if toggle = '0' then
					if pb_count = 7 then
						pb_count <= 0;
					else
						pb_count <= pb_count + 1;
					end if;
					panel_pbsel <= std_logic_vector( to_unsigned( pb_count, panel_pbsel'length));
				else
					case pb_count is
						when 0 => depdb <= panel_pbin;
						when 1 => startdb <= panel_pbin;
						when 2 => instdb <= panel_pbin;
						when 3 => enadb <= panel_pbin;
						when 4 => contdb <= panel_pbin;
						when 5 => exadb <= panel_pbin;
						when 6 => loaddb <= panel_pbin;
						when 7 => dseldb <= panel_pbin;
					end case;
				end if;
			end if;
		end if;
	end process;
	
end implementation;

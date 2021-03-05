-----------------------------------------------------------------------------------------------------------------------
-- Repo: https://github.com/metacollin/ow.git
-- Developed by: https://orthogonalsystems.com
--
-- Copyright 2020 Orthogonal Systems LLC, Collin Anderson, Ian Wisher
-- SPDX-License-Identifier: Apache-2.0 WITH SHL-2.1
--
-- Licensed under the Solderpad Hardware License v 2.1 (the "License"); you may not use this file except in compliance
-- with the License, or, at your option, the Apache License version 2.0. You may obtain a copy of the License at
--
-- https://solderpad.org/licenses/SHL-2.1/
--
-- Unless required by applicable law or agreed to in writing, any work distributed under the License is distributed on
-- an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the
-- specific language governing permissions and limitations under the License.
-----------------------------------------------------------------------------------------------------------------------

library IEEE;
use IEEE.STD_LOGIC_1164.all;
use IEEE.NUMERIC_STD.all;
use IEEE.math_real."log2";

entity one_wire_wrapper is
  generic (
    scratchpad_size : integer := 64;
    page_size       : integer := 256;
    eeprom_size     : integer := 1024;
    read_chunk      : integer := 64
  );
  port (
    clk_1mhz        : in     std_logic;
    reset           : in     std_logic;
    one_wire        : inout  std_logic;
    serial_id       : out    std_logic_vector (63 downto 0);
    serial_id_valid : out    std_logic;
    WR              : in     std_logic;
    RD              : in     std_logic;
    MEMORY_IN       : in     std_logic_vector(eeprom_size-1 downto 0);
    MEMORY_OUT      : out    std_logic_vector(eeprom_size-1 downto 0);
    RDY             : out    std_logic;
    ERR             : out    std_logic;
    CLR             : in     std_logic
  );
end one_wire_wrapper;

architecture Behavioral of one_wire_wrapper is

  component one_wire_io
    port (
      CLK        : in std_logic;
      MR         : in std_logic;
      DQ_CONTROL : in std_logic;
      DQ_IN      : out   std_logic;
      DQ         : inout std_logic
    );
  end component;

  component one_wire_interface
    port (
      ADDRESS           : in  std_logic_vector(2 downto 0);
      ADS_bar           : in  std_logic;
      clear_interrupts  : in  std_logic;
      DIN               : in  std_logic_vector(7 downto 0);
      DQ_IN             : in  std_logic;
      EN_bar            : in  std_logic;
      FSM_CLK           : in  std_logic;
      MR                : in  std_logic;
      OneWireIO_eq_Load : in  std_logic;
      pdr               : in  std_logic;
      OW_LOW            : in  std_logic;
      OW_SHORT          : in  std_logic;
      rbf               : in  std_logic;
      rcvr_buffer       : in  std_logic_vector(7 downto 0);
      RD_bar            : in  std_logic;
      reset_owr         : in  std_logic;
      rsrf              : in  std_logic;
      temt              : in  std_logic;
      WR_bar            : in  std_logic;
      BIT_CTL           : out std_logic;
      CLK_EN            : out std_logic;
      clr_activate_intr : out std_logic;
      DDIR              : out std_logic;
      div_1             : out std_logic;
      div_2             : out std_logic;
      div_3             : out std_logic;
      DOUT              : out std_logic_vector(7 downto 0);
      EN_FOW            : out std_logic;
      EOWL              : out std_logic;
      EOWSH             : out std_logic;
      epd               : out std_logic;
      erbf              : out std_logic;
      ersf              : out std_logic;
      etbe              : out std_logic;
      etmt              : out std_logic;
      FOW               : out std_logic;
      ias               : out std_logic;
      LLM               : out std_logic;
      OD                : out std_logic;
      owr               : out std_logic;
      pd                : out std_logic;
      PPM               : out std_logic;
      pre_0             : out std_logic;
      pre_1             : out std_logic;
      rbf_reset         : out std_logic;
      sr_a              : out std_logic;
      STP_SPLY          : out std_logic;
      STPEN             : out std_logic;
      tbe               : out std_logic;
      xmit_buffer       : out std_logic_vector(7 downto 0)
    );
  end component;

  component onewiremaster
    port (
      BIT_CTL           : in  std_logic;
      clk_1us           : in  std_logic;
      clr_activate_intr : in  std_logic;
      DQ_IN             : in  std_logic;
      EN_FOW            : in  std_logic;
      EOWL              : in  std_logic;
      EOWSH             : in  std_logic;
      epd               : in  std_logic;
      erbf              : in  std_logic;
      ersf              : in  std_logic;
      etbe              : in  std_logic;
      etmt              : in  std_logic;
      FOW               : in  std_logic;
      ias               : in  std_logic;
      LLM               : in  std_logic;
      MR                : in  std_logic;
      OD                : in  std_logic;
      owr               : in  std_logic;
      pd                : in  std_logic;
      PPM               : in  std_logic;
      rbf_reset         : in  std_logic;
      sr_a              : in  std_logic;
      STP_SPLY          : in  std_logic;
      STPEN             : in  std_logic;
      tbe               : in  std_logic;
      xmit_buffer       : in  std_logic_vector (7 downto 0);
      clear_interrupts  : out std_logic;
      DQ_CONTROL        : out std_logic;
      FSM_CLK           : out std_logic;
      INTR              : out std_logic;
      OneWireIO_eq_Load : out std_logic;
      OW_LOW            : out std_logic;
      OW_SHORT          : out std_logic;
      pdr               : out std_logic;
      rbf               : out std_logic;
      rcvr_buffer       : out std_logic_vector (7 downto 0);
      reset_owr         : out std_logic;
      rsrf              : out std_logic;
      STPZ              : out std_logic;
      temt              : out std_logic
    );
  end component;

  signal DDIR              : std_logic;
  signal DOUT              : std_logic_vector(7 downto 0);
  signal DQ_CONTROL        : std_logic;
  signal DIN               : std_logic_vector(7 downto 0);
  signal DQ_IN             : std_logic;
  signal CLK_EN            : std_logic;
  signal div_1             : std_logic;
  signal div_2             : std_logic;
  signal div_3             : std_logic;
  signal pre_0             : std_logic;
  signal pre_1             : std_logic;
  signal clk_1us           : std_logic;
  signal clear_interrupts  : std_logic;
  signal fsm_clk           : std_logic;
  signal onewireio_eq_load : std_logic;
  signal pdr               : std_logic;
  signal ow_low            : std_logic;
  signal ow_short          : std_logic;
  signal rbf               : std_logic;
  signal rcvr_buffer       : std_logic_vector(7 downto 0);
  signal reset_owr         : std_logic;
  signal rsrf              : std_logic;
  signal temt              : std_logic;
  signal bit_ctl           : std_logic;
  signal clr_activate_intr : std_logic;
  signal en_fow            : std_logic;
  signal eowl              : std_logic;
  signal eowsh             : std_logic;
  signal epd               : std_logic;
  signal erbf              : std_logic;
  signal ersf              : std_logic;
  signal etbe              : std_logic;
  signal etmt              : std_logic;
  signal fow               : std_logic;
  signal ias               : std_logic;
  signal llm               : std_logic;
  signal od                : std_logic;
  signal owr               : std_logic;
  signal pd                : std_logic;
  signal ppm               : std_logic;
  signal rbf_reset         : std_logic;
  signal sr_a              : std_logic;
  signal stp_sply          : std_logic;
  signal stpen             : std_logic;
  signal tbe               : std_logic;
  signal xmit_buffer       : std_logic_vector(7 downto 0);

  type reset_t is (bus_reset);
  type wait_t is (wait_for_program);

  type one_wire_state is (
    setup,
    enabling_ow_clock,
    generating_ow_reset,
    updating_int_register,
    waiting_for_reset,
    ow_tx,
    ow_rx,
    ow_rx_byte,
    read_serial,
    waiting_for_send,
    waiting_for_response,
    load_id,
    validate_id,
    write_to_scratchpad,
    read_from_scratchpad,
    copy_scratchpad_to_memory,
    read_from_memory,
    write_page_to_eeprom,
    write_eeprom,
    wait4prog,
    load_contents,
    restart_count,
    set_restart_count,
    add_restart_count,
    set_scratchpad_contents,
    write_count_to_eeprom,
    idle);

  signal ADDRESS             : std_logic_vector(2 downto 0);
  signal ADS_bar             : std_logic;
  signal EN_bar              : std_logic;
  signal MR                  : std_logic;
  signal RD_bar              : std_logic;
  signal WR_bar              : std_logic;
  signal INTR                : std_logic;
  signal STPZ                : std_logic;
  signal DATA                : std_logic_vector(7 downto 0);
  signal DQ                  : std_logic;
  signal ow_state            : one_wire_state;
  signal next_ow_state       : one_wire_state;
  signal after_next_ow_state : one_wire_state;
  signal call_addr           : std_logic_vector(2 downto 0);
  signal data_in             : std_logic_vector(7 downto 0);
  signal cur_step            : integer;
  signal serial_crc          : std_logic_vector(7 downto 0);

  type vector_array is array (natural range <>) of std_logic_vector(7 downto 0);
  signal id_buffer            : vector_array(0 to 7);
  signal copy_status, ta1, es : std_logic_vector(7 downto 0);
  constant ta2                : std_logic_vector(7 downto 0) := x"00";
  signal scratchpad_contents  : vector_array (0 to ((scratchpad_size/8) - 1));
  
  signal index, wait_count    : integer;
  signal data_out             : std_logic_vector(7 downto 0);
  signal id_vec               : std_logic_vector(63 downto 0);
  signal restart_cnt          : std_logic_vector(31 downto 0);
  constant scratchpad_count     : natural :=  eeprom_size/scratchpad_size;
  signal scratchpad_write_count : natural :=  0;

  signal int_register   : std_logic_vector(7 downto 0);
  signal setup_count    : integer := 6;
  signal scratchpad_crc : vector_array (0 to 1);

  signal eeprom_buffer : vector_array(0 to ((eeprom_size/8) - 1));
  signal read_done     : boolean;
  signal read_buffer   : std_logic_vector(7 downto 0);
  signal write_buffer  : std_logic_vector(7 downto 0);
  signal write_done    : boolean;

  constant SKIP_ROM         : std_logic_vector(7 downto 0) := x"CC";
  constant WRITE_SCRATCHPAD : std_logic_vector(7 downto 0) := x"0F";
  constant READ_SCRATCHPAD  : std_logic_vector(7 downto 0) := x"AA";
  constant COPY_SCRATCHPAD  : std_logic_vector(7 downto 0) := x"55";
  constant READ_MEMORY      : std_logic_vector(7 downto 0) := x"F0";
  constant COPY_SUCCESSFUL  : std_logic_vector(7 downto 0) := x"AA";
  constant AUTH_CODE        : std_logic_vector(7 downto 0) := b"00000111";
  constant READ_SERIAL_ID   : std_logic_vector(7 downto 0) := x"33";

  constant CMD_REG   : std_logic_vector(2 downto 0) := b"000";
  constant TRX_REG   : std_logic_vector(2 downto 0) := b"001";
  constant INT_REG   : std_logic_vector(2 downto 0) := b"010";
  constant INT_E_REG : std_logic_vector(2 downto 0) := b"011";
  constant CLK_REG   : std_logic_vector(2 downto 0) := b"100";
  constant CTL_REG   : std_logic_vector(2 downto 0) := b"101";

begin

  clk_1us <= clk_1mhz;

  xone_wire_io : one_wire_io
  port map(
    CLK        => clk_1mhz,
    MR         => MR,
    DQ_IN      => DQ_IN,
    DQ_CONTROL => DQ_CONTROL,
    DQ         => one_wire
  );

  xone_wire_interface : one_wire_interface
  port map(
    ADDRESS           => ADDRESS,
    ADS_bar           => ADS_bar,
    clear_interrupts  => clear_interrupts,
    DIN               => DIN,
    DQ_IN             => DQ_IN,
    EN_bar            => EN_bar,
    FSM_CLK           => FSM_CLK,
    MR                => MR,
    OneWireIO_eq_Load => OneWireIO_eq_Load,
    pdr               => pdr,
    OW_LOW            => OW_LOW,
    OW_SHORT          => OW_SHORT,
    rbf               => rbf,
    rcvr_buffer       => rcvr_buffer,
    RD_bar            => RD_bar,
    reset_owr         => reset_owr,
    rsrf              => rsrf,
    temt              => temt,
    WR_bar            => WR_bar,
    BIT_CTL           => BIT_CTL,
    CLK_EN            => CLK_EN,
    clr_activate_intr => clr_activate_intr,
    DDIR              => DDIR,
    div_1             => div_1,
    div_2             => div_2,
    div_3             => div_3,
    DOUT              => DOUT,
    EN_FOW            => EN_FOW,
    EOWL              => EOWL,
    EOWSH             => EOWSH,
    epd               => epd,
    erbf              => erbf,
    ersf              => ersf,
    etbe              => etbe,
    etmt              => etmt,
    FOW               => FOW,
    ias               => ias,
    LLM               => LLM,
    OD                => OD,
    owr               => owr,
    pd                => pd,
    PPM               => PPM,
    pre_0             => pre_0,
    pre_1             => pre_1,
    rbf_reset         => rbf_reset,
    sr_a              => sr_a,
    STP_SPLY          => STP_SPLY,
    STPEN             => STPEN,
    tbe               => tbe,
    xmit_buffer       => xmit_buffer
  );

  xonewiremaster : onewiremaster
  port map(
    BIT_CTL           => BIT_CTL,
    clk_1us           => clk_1us,
    clr_activate_intr => clr_activate_intr,
    DQ_IN             => DQ_IN,
    EN_FOW            => EN_FOW,
    EOWL              => EOWL,
    EOWSH             => EOWSH,
    epd               => epd,
    erbf              => erbf,
    ersf              => ersf,
    etbe              => etbe,
    etmt              => etmt,
    FOW               => FOW,
    ias               => ias,
    LLM               => LLM,
    MR                => MR,
    OD                => OD,
    owr               => owr,
    pd                => pd,
    PPM               => PPM,
    rbf_reset         => rbf_reset,
    sr_a              => sr_a,
    STP_SPLY          => STP_SPLY,
    STPEN             => STPEN,
    tbe               => tbe,
    xmit_buffer       => xmit_buffer,
    clear_interrupts  => clear_interrupts,
    DQ_CONTROL        => DQ_CONTROL,
    FSM_CLK           => FSM_CLK,
    INTR              => INTR,
    OneWireIO_eq_Load => OneWireIO_eq_Load,
    OW_LOW            => OW_LOW,
    OW_SHORT          => OW_SHORT,
    pdr               => pdr,
    rbf               => rbf,
    rcvr_buffer       => rcvr_buffer,
    reset_owr         => reset_owr,
    rsrf              => rsrf,
    STPZ              => STPZ,
    temt              => temt
  );

  one_wire_state_machine : process (clk_1mhz)
    variable sequence_num : integer := 0;

procedure ow(constant tx : in std_logic_vector(7 downto 0);
constant addr            : in std_logic_vector(2 downto 0) := b"001") is
  begin
    if cur_step = sequence_num then
      after_next_ow_state <= ow_state;
      ow_state            <= ow_tx;
      data_in             <= tx;
      call_addr           <= addr;
      setup_count         <= 6;
      cur_step            <= cur_step + 1;
    end if;
    sequence_num := sequence_num + 1;
  end procedure;

procedure ow(constant tx : in vector_array;
constant addr            : in std_logic_vector(2 downto 0) := b"001") is
begin
  if cur_step = sequence_num then
    if write_done = true then
      if index = tx'length then
        cur_step <= cur_step + 1;
        index    <= 0;
      else
        index               <= index + 1;
        after_next_ow_state <= ow_state;
        ow_state            <= ow_tx;
        setup_count         <= 6;
      end if;
      write_done <= false;
      data_in    <= tx(index);
    end if;
    after_next_ow_state <= ow_state;
    ow_state            <= ow_tx;
    setup_count         <= 6;
  end if;
  sequence_num := sequence_num + 1;
end procedure;

procedure ow(constant ow_rst : in reset_t) is
begin
  if cur_step = sequence_num then
    after_next_ow_state <= ow_state;
    ow_state            <= generating_ow_reset;
    cur_step            <= cur_step + 1;
  end if;
  sequence_num := sequence_num + 1;
end procedure;

procedure ow(constant ow_prog_wait : in wait_t) is
begin
  if cur_step = sequence_num then
    after_next_ow_state <= ow_state;
    ow_state            <= wait4prog;
    cur_step            <= cur_step + 1;
  end if;

  sequence_num := sequence_num + 1;
end procedure;

procedure ow(signal rx : out std_logic_vector(7 downto 0)) is
begin
  if cur_step = sequence_num then
    if read_done = true then
      cur_step  <= cur_step + 1;
      read_done <= false;
      rx        <= read_buffer;
    else
      after_next_ow_state <= ow_state;
      ow_state            <= ow_rx;
      setup_count <= 6;
    end if;
  end if;
  sequence_num := sequence_num + 1;
end procedure;

procedure ow(signal rx : out vector_array) is
begin
  if cur_step = sequence_num then
    if index = rx'length + 1 then
      cur_step    <= cur_step + 1;
      setup_count <= 6;
      index       <= 0;
    else
      index               <= index + 1;
      after_next_ow_state <= ow_state;
      ow_state            <= ow_rx;
      setup_count <= 6;
    end if;
    if index > 0 then
      rx(index - 1) <= read_buffer;
    end if;
  end if;
  sequence_num := sequence_num + 1;
end procedure;

procedure ow(constant proceed_to : in one_wire_state) is
begin
  if cur_step = sequence_num then
    ow_state <= proceed_to;
    cur_step <= 0;
  end if;
  sequence_num := sequence_num + 1;
end procedure;

function crc8_56w
  (ow_id : std_logic_vector(55 downto 0))
  return std_logic_vector is

  variable crc : std_logic_vector(7 downto 0) := x"00";

  variable reflected : std_logic_vector(ow_id'range);
  alias rev          : std_logic_vector(ow_id'REVERSE_RANGE) is ow_id;
  variable id        : std_logic_vector(55 downto 0);
  variable crc_out   : std_logic_vector(7 downto 0);
  variable crc_ref   : std_logic_vector(7 downto 0);
  alias crc_rev      : std_logic_vector(crc_out'REVERSE_RANGE) is crc_out;

begin
  for i in rev'range loop
    reflected(i) := rev(i);
  end loop;

  id := reflected;

  crc_out(0) := id(53) xor id(52) xor id(51) xor id(50) xor id(49) xor id(48) xor id(45) xor id(44) xor id(42) xor id(40) xor id(39) xor id(38) xor id(34) xor 
                id(33) xor id(32) xor id(31) xor id(25) xor id(24) xor id(23) xor id(21) xor id(18) xor id(15) xor id(14) xor id(11) xor id(10) xor id(9)  xor 
                id(6)  xor id(4)  xor id(3)  xor id(0)  xor crc(0) xor crc(1) xor crc(2) xor crc(3) xor crc(4) xor crc(5);

  crc_out(1) := id(54) xor id(53) xor id(52) xor id(51) xor id(50) xor id(49) xor id(46) xor id(45) xor id(43) xor id(41) xor id(40) xor id(39) xor id(35) xor 
                id(34) xor id(33) xor id(32) xor id(26) xor id(25) xor id(24) xor id(22) xor id(19) xor id(16) xor id(15) xor id(12) xor id(11) xor id(10) xor 
                id(7)  xor id(5)  xor id(4)  xor id(1)  xor crc(1) xor crc(2) xor crc(3) xor crc(4) xor crc(5) xor crc(6);

  crc_out(2) := id(55) xor id(54) xor id(53) xor id(52) xor id(51) xor id(50) xor id(47) xor id(46) xor id(44) xor id(42) xor id(41) xor id(40) xor id(36) xor 
                id(35) xor id(34) xor id(33) xor id(27) xor id(26) xor id(25) xor id(23) xor id(20) xor id(17) xor id(16) xor id(13) xor id(12) xor id(11) xor 
                id(8)  xor id(6)  xor id(5)  xor id(2)  xor crc(2) xor crc(3) xor crc(4) xor crc(5) xor crc(6) xor crc(7);

  crc_out(3) := id(55) xor id(54) xor id(53) xor id(52) xor id(51) xor id(48) xor id(47) xor id(45) xor id(43) xor id(42) xor id(41) xor id(37) xor id(36) xor
                id(35) xor id(34) xor id(28) xor id(27) xor id(26) xor id(24) xor id(21) xor id(18) xor id(17) xor id(14) xor id(13) xor id(12) xor id(9)  xor 
                id(7)  xor id(6)  xor id(3)  xor crc(0) xor crc(3) xor crc(4) xor crc(5) xor crc(6) xor crc(7);

  crc_out(4) := id(55) xor id(54) xor id(51) xor id(50) xor id(46) xor id(45) xor id(43) xor id(40) xor id(39) xor id(37) xor id(36) xor id(35) xor id(34) xor 
                id(33) xor id(32) xor id(31) xor id(29) xor id(28) xor id(27) xor id(24) xor id(23) xor id(22) xor id(21) xor id(19) xor id(13) xor id(11) xor
                id(9)  xor id(8)  xor id(7)  xor id(6)  xor id(3)  xor id(0)  xor crc(2) xor crc(3) xor crc(6) xor crc(7);

  crc_out(5) := id(55) xor id(53) xor id(50) xor id(49) xor id(48) xor id(47) xor id(46) xor id(45) xor id(42) xor id(41) xor id(39) xor id(37) xor id(36) xor 
                id(35) xor id(31) xor id(30) xor id(29) xor id(28) xor id(22) xor id(21) xor id(20) xor id(18) xor id(15) xor id(12) xor id(11) xor id(8)  xor
                id(7)  xor id(6)  xor id(3)  xor id(1)  xor id(0)  xor crc(0) xor crc(1) xor crc(2) xor crc(5) xor crc(7);

  crc_out(6) := id(54) xor id(51) xor id(50) xor id(49) xor id(48) xor id(47) xor id(46) xor id(43) xor id(42) xor id(40) xor id(38) xor id(37) xor id(36) xor
                id(32) xor id(31) xor id(30) xor id(29) xor id(23) xor id(22) xor id(21) xor id(19) xor id(16) xor id(13) xor id(12) xor id(9)  xor id(8)  xor
                id(7)  xor id(4)  xor id(2)  xor id(1)  xor crc(0) xor crc(1) xor crc(2) xor crc(3) xor crc(6);

  crc_out(7) := id(55) xor id(52) xor id(51) xor id(50) xor id(49) xor id(48) xor id(47) xor id(44) xor id(43) xor id(41) xor id(39) xor id(38) xor id(37) xor
                id(33) xor id(32) xor id(31) xor id(30) xor id(24) xor id(23) xor id(22) xor id(20) xor id(17) xor id(14) xor id(13) xor id(10) xor id(9)  xor 
                id(8)  xor id(5)  xor id(3)  xor id(2)  xor crc(0) xor crc(1) xor crc(2) xor crc(3) xor crc(4) xor crc(7);

  for i in crc_rev'range loop
    crc_ref(i) := crc_rev(i);
  end loop;

  return crc_ref;

end function;

begin
if (rising_edge(clk_1mhz)) then
  sequence_num := 0;      
  if (reset = '1') then
    ADDRESS         <= b"000";
    ADS_bar         <= '0';
    EN_bar          <= '0';
    MR              <= '1';
    RD_bar          <= '1';
    WR_bar          <= '1';
    DIN             <= x"00";
    ow_state        <= setup;
    int_register    <= x"00";
    index           <= 0;
    ERR             <= '0';
    wait_count      <= 0;
    copy_status     <= x"00";
    ta1             <= x"00";
    es              <= x"07";
    cur_step        <= 0;
    serial_id_valid <= '0';
    serial_id       <= x"0000000000000000";
    read_done       <= false;
    write_done      <= false;
    read_buffer     <= x"00";
    RDY             <= '0';

  else
    case ow_state is
      when setup =>
        MR          <= '0';
        ADDRESS     <= b"000";
        --setup_count <= 6;
        ow_state    <= enabling_ow_clock;

        
      when enabling_ow_clock =>
        if (setup_count = 6) then
          ADS_bar     <= '0';
          setup_count <= setup_count - 1;
          ow_state    <= enabling_ow_clock;
        elsif (setup_count = 5) then
          ADDRESS     <= b"100";
          setup_count <= setup_count - 1;
          ow_state    <= enabling_ow_clock;
        elsif (setup_count = 4) then
          ADS_bar     <= '1';
          DIN         <= b"10000000";
          setup_count <= setup_count - 1;
          ow_state    <= enabling_ow_clock;
        elsif (setup_count = 3) then
          DIN         <= b"10000000";
          WR_bar      <= '0';
          setup_count <= setup_count - 1;
          ow_state    <= enabling_ow_clock;
        elsif (setup_count = 2) then
          WR_bar      <= '1';
          setup_count <= setup_count - 1;
          ow_state    <= enabling_ow_clock;
        elsif (setup_count = 1) then
          setup_count <= setup_count - 1;
          ow_state    <= enabling_ow_clock;
        else
          setup_count <= 6;
          ow_state    <= read_serial;
        end if;

        
      when wait4prog =>
        if (wait_count = 12500) then
          ow_state   <= after_next_ow_state;
          wait_count <= 0;
        else
          ow_state   <= wait4prog;
          wait_count <= wait_count + 1;
        end if;


      when generating_ow_reset =>
        if (setup_count = 6) then
          ADS_bar     <= '0';
          setup_count <= setup_count - 1;
          ow_state    <= generating_ow_reset;
        elsif (setup_count = 5) then
          ADDRESS     <= b"000";
          setup_count <= setup_count - 1;
          ow_state    <= generating_ow_reset;
        elsif (setup_count = 4) then
          ADS_bar     <= '1';
          DIN         <= x"01";
          setup_count <= setup_count - 1;
          ow_state    <= generating_ow_reset;
        elsif (setup_count = 3) then
          DIN         <= x"01";
          WR_bar      <= '0';
          setup_count <= setup_count - 1;
          ow_state    <= generating_ow_reset;
        elsif (setup_count = 2) then
          WR_bar      <= '1';
          setup_count <= setup_count - 1;
          ow_state    <= generating_ow_reset;
        elsif (setup_count = 1) then
          setup_count <= setup_count - 1;
          ow_state    <= generating_ow_reset;
        else
          setup_count <= 6;
          ow_state    <= waiting_for_reset;
        end if;


      when waiting_for_reset =>
        if (int_register(0) = '0') then
          ow_state      <= updating_int_register;
          next_ow_state <= waiting_for_reset;
        else
          ow_state <= after_next_ow_state;
        end if;


      when updating_int_register =>
        if (setup_count = 6) then
          ADS_bar     <= '0';
          setup_count <= setup_count - 1;
          ow_state    <= updating_int_register;
        elsif (setup_count = 5) then
          ADDRESS     <= b"010";
          setup_count <= setup_count - 1;
          ow_state    <= updating_int_register;
        elsif (setup_count = 4) then
          ADS_bar     <= '1';
          setup_count <= setup_count - 1;
          ow_state    <= updating_int_register;
        elsif (setup_count = 3) then
          RD_bar      <= '0';
          setup_count <= setup_count - 1;
          ow_state    <= updating_int_register;
        elsif (setup_count = 2) then
          int_register <= DOUT;
          setup_count  <= setup_count - 1;
          ow_state     <= updating_int_register;
        elsif (setup_count = 1) then
          RD_bar      <= '1';
          setup_count <= setup_count - 1;
          ow_state    <= updating_int_register;
        else
          setup_count <= 6;
          ow_state    <= next_ow_state;
        end if;


      when ow_tx =>
        if (setup_count = 6) then
          ADS_bar     <= '0';
          setup_count <= setup_count - 1;
          ow_state    <= ow_tx;
        elsif (setup_count = 5) then
          ADDRESS     <= call_addr;
          setup_count <= setup_count - 1;
          ow_state    <= ow_tx;
        elsif (setup_count = 4) then
          ADS_bar     <= '1';
          DIN         <= data_in;
          setup_count <= setup_count - 1;
          ow_state    <= ow_tx;
        elsif (setup_count = 3) then
          DIN         <= data_in;
          WR_bar      <= '0';
          setup_count <= setup_count - 1;
          ow_state    <= ow_tx;
        elsif (setup_count = 2) then
          WR_bar      <= '1';
          setup_count <= setup_count - 1;
          ow_state    <= ow_tx;
        elsif (setup_count = 1) then
          setup_count <= setup_count - 1;
          ow_state    <= ow_tx;
        else
          setup_count   <= 6;
          ow_state      <= updating_int_register;
          next_ow_state <= waiting_for_send;
          write_done    <= True;
        end if;


      when waiting_for_send =>
        if (int_register(3) = '1') then
          ow_state <= after_next_ow_state;
          --  cur_step <= cur_step + 1;
        else
          ow_state      <= updating_int_register;
          next_ow_state <= waiting_for_send;
        end if;


      when ow_rx =>
        if (setup_count = 6) then
          ADS_bar     <= '0';
          setup_count <= setup_count - 1;
          ow_state    <= ow_rx;
        elsif (setup_count = 5) then
          ADDRESS     <= b"001";
          setup_count <= setup_count - 1;
          ow_state    <= ow_rx;
        elsif (setup_count = 4) then
          ADS_bar     <= '1';
          DIN         <= x"FF";
          setup_count <= setup_count - 1;
          ow_state    <= ow_rx;
        elsif (setup_count = 3) then
          DIN         <= x"FF";
          WR_bar      <= '0';
          setup_count <= setup_count - 1;
          ow_state    <= ow_rx;
        elsif (setup_count = 2) then
          WR_bar      <= '1';
          setup_count <= setup_count - 1;
          ow_state    <= ow_rx;
        elsif (setup_count = 1) then
          setup_count <= setup_count - 1;
          ow_state    <= ow_rx;
        else
          setup_count   <= 6;
          ow_state      <= updating_int_register;
          next_ow_state <= waiting_for_response;
        end if;


      when waiting_for_response =>
        if (int_register(4) = '1' and int_register(3) = '1') then
          ow_state <= ow_rx_byte;
        else
          ow_state      <= updating_int_register;
          next_ow_state <= waiting_for_response;
        end if;


      when ow_rx_byte =>
        if (setup_count = 6) then
          ADS_bar     <= '0';
          setup_count <= setup_count - 1;
          ow_state    <= ow_rx_byte;
        elsif (setup_count = 5) then
          ADDRESS     <= b"001";
          setup_count <= setup_count - 1;
          ow_state    <= ow_rx_byte;
        elsif (setup_count = 4) then
          ADS_bar     <= '1';
          setup_count <= setup_count - 1;
          ow_state    <= ow_rx_byte;
        elsif (setup_count = 3) then
          RD_bar      <= '0';
          setup_count <= setup_count - 1;
          ow_state    <= ow_rx_byte;
        elsif (setup_count = 2) then
          setup_count <= setup_count - 1;
          ow_state    <= ow_rx_byte;
        elsif (setup_count = 1) then
          RD_bar      <= '1';
          setup_count <= setup_count - 1;
          ow_state    <= ow_rx_byte;
        else
          read_buffer <= DOUT;
          setup_count <= 6;
          ow_state    <= after_next_ow_state;
          read_done   <= true;
        end if;


      when load_id =>
        id_vec(7 downto 0)   <= id_buffer(0);
        id_vec(15 downto 8)  <= id_buffer(1);
        id_vec(23 downto 16) <= id_buffer(2);
        id_vec(31 downto 24) <= id_buffer(3);
        id_vec(39 downto 32) <= id_buffer(4);
        id_vec(47 downto 40) <= id_buffer(5);
        id_vec(55 downto 48) <= id_buffer(6);
        id_vec(63 downto 56) <= id_buffer(7);
        serial_crc           <= id_buffer(7);
        
        ow(proceed_to => validate_id);
        
        
     when validate_id =>
        if crc8_56w(id_vec(55 downto 0)) = serial_crc then
            serial_id_valid <= '1';
            serial_id       <= id_vec;
        else
            serial_id       <= id_vec;
            serial_id_valid <= '0';
            ERR             <= '1';
        end if;
        ow(proceed_to => restart_count);

      when read_from_memory =>
        RDY <= '0';
        ow(bus_reset);
        ow(tx         => SKIP_ROM);
        ow(tx         => READ_MEMORY);
        ow(tx         => ta1);
        ow(tx         => ta2);
        ow(rx         => eeprom_buffer);
        ow(proceed_to => load_contents);
        
      when restart_count =>
        RDY <= '0';
        ow(bus_reset);
        ow(tx         => SKIP_ROM);
        ow(tx         => READ_MEMORY);
        ow(tx         => ta1);
        ow(tx         => ta2);
        ow(rx         => eeprom_buffer);
        ow(proceed_to => set_restart_count); 
        
        
        when set_restart_count =>
            restart_cnt <= eeprom_buffer(3) & eeprom_buffer(2) & eeprom_buffer(1) & eeprom_buffer(0);
            ow_state <= add_restart_count;
            
        when add_restart_count =>
            restart_cnt <= std_logic_vector(unsigned(restart_cnt) + 1);
            ow_state <= set_scratchpad_contents;

        when set_scratchpad_contents =>
            scratchpad_contents(0) <= restart_cnt(7 downto 0);
            scratchpad_contents(1) <= restart_cnt(15 downto 8);
            scratchpad_contents(2) <= restart_cnt(23 downto 16);
            scratchpad_contents(3) <= restart_cnt(31 downto 24);
            for i in 4 to scratchpad_contents'length - 1 loop
                scratchpad_contents(i) <= eeprom_buffer(i);
            end loop;
            ow_state <= write_count_to_eeprom;                     

       when write_count_to_eeprom =>
        ow(bus_reset);
        ow(tx => SKIP_ROM);
        ow(tx => WRITE_SCRATCHPAD);
        ow(tx => ta1);
        ow(tx => ta2);
        ow(tx => scratchpad_contents);
        ow(rx => scratchpad_crc); -- optional
        ow(bus_reset);
        ow(tx => SKIP_ROM);
        ow(tx => COPY_SCRATCHPAD);
        ow(tx => ta1);
        ow(tx => ta2);
        ow(tx => AUTH_CODE);
        ow(wait_for_program);
        ow(bus_reset);
        ow(tx => SKIP_ROM);
        ow(tx => READ_MEMORY);
        ow(tx => ta1);
        ow(tx => ta2);
        ow(rx => eeprom_buffer);
        ow(proceed_to => load_contents);

      when load_contents =>
        for i in 0 to eeprom_buffer'length - 1 loop
          MEMORY_OUT((8 * (i + 1)) - 1 downto 8 * i) <= eeprom_buffer(i);
        end loop;
        ow_state <= idle;
                                        
      when write_page_to_eeprom =>
        ow(bus_reset);
        ow(tx => SKIP_ROM);
        ow(tx => WRITE_SCRATCHPAD);
        ow(tx => ta1);
        ow(tx => ta2);
        ow(tx => scratchpad_contents);
        ow(rx => scratchpad_crc); -- optional
        ow(bus_reset);
        ow(tx => SKIP_ROM);
        ow(tx => COPY_SCRATCHPAD);
        ow(tx => ta1);
        ow(tx => ta2);
        ow(tx => AUTH_CODE);
        ow(wait_for_program);
        ow(proceed_to => write_eeprom);
        
      when write_eeprom =>
        if scratchpad_write_count = scratchpad_count then
            ow_state <= idle;
            RDY      <= '1';
            ta1      <= x"00";
            scratchpad_write_count <= 0;
        else 
            ta1  <= std_logic_vector(to_unsigned(scratchpad_write_count*8,8));
            for i in 0 to 7 loop
                  scratchpad_contents(i) <= MEMORY_IN((8 * (8*scratchpad_write_count + 1+i)) - 1 
                                                       downto 
                                                       8 * (8*scratchpad_write_count+i));
            end loop;
            scratchpad_write_count <= scratchpad_write_count + 1;
            ow_state <= write_page_to_eeprom;
        end if;     
                               
      when read_serial =>
        ow(bus_reset);
        ow(tx         => READ_SERIAL_ID);
        ow(rx         => id_buffer);
        ow(proceed_to => load_id);

      when idle =>
        if (CLR = '1') then
          ERR             <= '0';
          serial_id       <= (others => '0');
          serial_id_valid <= '0';
          MEMORY_OUT      <= (others => '0');   
        end if;

        if (WR = '1') then
          RDY <= '0';
          ta1 <= x"00";
          scratchpad_write_count <= 0;
          ow_state <= write_eeprom;
          
        elsif (RD = '1') then
          RDY      <= '0';
          ow_state <= read_from_memory;
        else
          RDY      <= '1';
          ow_state <= idle;
        end if;

      when others =>
        ow_state <= idle;
    end case;
  end if; --reset
end if; -- edge
end process;

end Behavioral;

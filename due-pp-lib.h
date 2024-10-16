//2023.10.07  +int due_download_prog_save_to_file(int fd,due_prog_t *program);
//10.18 +int due_download_prog_save_to_file_command/data

/*
 Copyright 2018, Carl Michal

 This file is part of due-pp-lib.

 due-pp-lib is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or (at
 your option) any later version.

 This program is distributed in the hope that it will be useful, but
 WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

//#include "stdio.h"  //FILE


// MAXDATA is in 4-byte words
#define MAXDATA 23003
#define MAXSUB 4000

#define DEFAULT_PORT 0
#define ALT_PORT 1
#define DAC_PORT 2

//#define NUM_OPCODES 14
// branch and exit assigned to 15?
// this isn't a great system since the minimums don't necessarily line up
// with opcodes. Here the NUM_OPCODES element is for an ordinary event,
// and NUM_OPCODES+1 is for CALL_SUB. EXT_TRIG and TRIG_MAX
// have restrictions on both the event themselves and the preceding.
// Some of these are for the events preceding. Take the bigger of the two and
// use for both.
//int min_ticks[NUM_OPCODES+2] = {20,20,20,25,0,25,20,25,25,25,25,25,25,20,10,25};


typedef struct due_prog_type{
  unsigned int dpos,
    events, // not really important!
    queued_events,
    queue_pos,
    loop_level,
    state,
    last_ticks,
    in_sub_num;
  char in_sub,
    error,
    active_port, // 0 = default, 1=alternate, 2=dacs
    auto_shift;  // do we bit shift the outputs automatically?
  unsigned int data[MAXDATA]; // the program itself
  int sub_table[MAXSUB];//={[0 ... 3999] = 0 }; // holds the start address for the beginning of each subroutine.
  char sub_entry_port[MAXSUB];
} due_prog_t;


// public functions:
int due_init_program(due_prog_t *program, char auto_shift);
int due_add_event(due_prog_t *program, unsigned int outputs, unsigned  int ticks);
int due_start_loop(due_prog_t *program, unsigned int loops,unsigned int outputs, unsigned int ticks) ;
int due_end_loop(due_prog_t *program, unsigned int outputs, unsigned int ticks) ;
int due_exit_program(due_prog_t *program);
int due_swap_to_alt(due_prog_t *program, unsigned int outputs, unsigned int ticks);
int due_swap_to_default(due_prog_t *program, unsigned int outputs, unsigned int ticks);
int due_swap_to_dacs(due_prog_t *program, unsigned int dac0, unsigned int dac1, unsigned int ticks);
int due_finalize_program(due_prog_t *program);
int due_call_sub(due_prog_t *program, unsigned int subroutine_id, unsigned int outputs,unsigned int ticks);
int due_start_sub(due_prog_t *program, unsigned int subroutine_id);
int due_return_from_sub(due_prog_t *program, unsigned int outputs, unsigned int ticks);
int due_wait_for_trigger(due_prog_t *program, unsigned int outputs,unsigned int ticks);
int due_wait_for_trigger_max(due_prog_t *program, unsigned int outputs, unsigned int ticks);
int due_write_dacs(due_prog_t *program, unsigned int dac0, unsigned int dac1,unsigned int outputs,unsigned int ticks);
int due_write_alt(due_prog_t *program, unsigned int outputs_alt, unsigned int outputs,unsigned int ticks);
int due_write_default(due_prog_t *program, unsigned int outputs_def,unsigned int outputs,unsigned int ticks);
int due_dump_program(due_prog_t *program);


int due_open_prog(char *device);
void due_close_prog(int fd);

int due_download_prog(int fd,due_prog_t *program);
int due_download_prog_save_to_file(int fd,due_prog_t *program);
int due_download_prog_save_to_file_command(int fd,due_prog_t *program);
//int due_download_prog_save_to_file_data(int fd,due_prog_t *program);


int due_run_program(int fd, char start_command);
int due_wait_for_completion(int fd, int timeout);
int due_interrupt_program(int fd);
int due_get_status(int fd);
int due_write_dacs_now(int fd, unsigned int dac0,unsigned int dac1);
int due_write_alt_now(int fd, unsigned int outputs);
int due_read_analog(int fd, unsigned char pin);



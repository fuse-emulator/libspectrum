/* mmc.c: Emulation of the SD / MMC interface
   Copyright (c) 2017 Philip Kendall

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License along
   with this program; if not, write to the Free Software Foundation, Inc.,
   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

   Author contact information:

   E-mail: Philip Kendall <philip-fuse@shadowmagic.org.uk>

*/

#include "config.h"

#include <stdio.h>
#include <string.h>

#include "internals.h"

#define IN_IDLE_STATE_MASK   0x01
#define ERASE_RESET_MASK     0x02
#define ILLEGAL_COMMAND_MASK 0x04
#define ERASE_SEQ_ERROR_MASK 0x10
#define PARAMETER_ERROR_MASK 0x40

#define TOKEN_START_BLOCK        0xFE /* 11111110 */
#define TOKEN_START_MBWR_BLOCK   0xFC /* 11111100 */
#define TOKEN_STOP_TRAN          0xFD /* 11111101 */

/* The states while a command is being sent to the card */
enum command_state_t {
  WAITING_FOR_COMMAND,
  WAITING_FOR_DATA0,
  WAITING_FOR_DATA1,
  WAITING_FOR_DATA2,
  WAITING_FOR_DATA3,
  WAITING_FOR_CRC,
  WAITING_FOR_DATA_TOKEN,
  WAITING_FOR_DATA,
  WAITING_FOR_DATA_CRC1,
  WAITING_FOR_DATA_CRC2
};

/* The MMC commands we support */
enum command_byte {
  GO_IDLE_STATE = 0,
  SEND_IF_COND = 8,
  SEND_CSD = 9,
  SEND_CID = 10,
  STOP_TRANSMISSION = 12,
  SET_BLOCKLEN = 16,
  READ_SINGLE_BLOCK = 17,
  READ_MULTIPLE_BLOCK = 18,
  WRITE_BLOCK = 24,
  WRITE_MULTIPLE_BLOCK = 25,
  ERASE_WR_BLK_START = 32,
  ERASE_WR_BLK_END = 33,
  ERASE = 38,
  APP_CMD = 55,
  READ_OCR = 58,
  CRC_ON_OFF = 59
};

/* Application specific commands, only some implemented
   but needed for command identification */
enum application_command_byte {
  SD_STATUS = 13,
  SEND_NUM_WR_BLOCKS = 22,
  SET_WR_BLK_ERASE_COUNT = 23,
  SD_SEND_OP_COND = 41,
  SET_CLR_CARD_DETECT = 42,
  SEND_SCR = 51,
};

/* The state of an erase sequence */
enum erase_sequence_t {
  SEQ_ERASE_NONE,
  SEQ_ERASE_WR_BLK_START,
  SEQ_ERASE_WR_BLK_END,
};

struct libspectrum_mmc_card {
  /* The actual "card" data */
  libspectrum_ide_drive drive;

  /* Cache of written sectors */
  GHashTable *cache;

  /* The C_SIZE field of the card CSD */
  libspectrum_word c_size;

  /* The number of sectors (512 bytes) */
  libspectrum_dword total_sectors;

  /* R1 status of current command */
  libspectrum_byte r1_status;

  /* The current state of the command being transmitted to the card */
  enum command_state_t command_state;

  /* The most recent command index sent to the MMC */
  libspectrum_byte current_command;

  /* The argument for the current MMC command */
  libspectrum_byte current_argument[4];

  /* How much data has been sent for the current command */
  size_t data_count;

  /* The data for the current command */
  libspectrum_byte send_buffer[512];

  /* The response to the most recent command */
  libspectrum_byte response_buffer[516];

  /* One past the last valid byte in response_buffer */
  libspectrum_byte *response_buffer_end;

  /* The next byte to be returned from response_buffer */
  libspectrum_byte *response_buffer_next;

  /* CMD8/SEND_IF_COND recieved while initialising */
  int cmd8_issued;

  /* CMD55/APP_CMD issued so next command is application specific */
  int cmd55_issued;

  /* A CMD18 multiple block read is in progress: the card streams sectors
     until the last sector, an error, or CMD12/STOP_TRANSMISSION */
  int multiple_read_active;

  /* Keep progress of ongoing erase sequence */
  enum erase_sequence_t erase_sequence;

  /* Initial and end blocks of programmed erase */
  libspectrum_dword erase_block_start, erase_block_end;

};

static void
mmc_read_multiple_blocks( libspectrum_mmc_card *card );

libspectrum_mmc_card*
libspectrum_mmc_alloc( void )
{
  libspectrum_mmc_card *card = libspectrum_new( libspectrum_mmc_card, 1 );

  card->drive.disk = NULL;
  card->cache = g_hash_table_new( g_int_hash, g_int_equal );

  libspectrum_mmc_reset( card );

  return card;
}

void
libspectrum_mmc_free( libspectrum_mmc_card *card )
{
  libspectrum_mmc_eject( card );

  g_hash_table_destroy( card->cache );

  libspectrum_free( card );
}

libspectrum_error
libspectrum_mmc_insert( libspectrum_mmc_card *card, const char *filename )
{
  libspectrum_error error;
  libspectrum_dword c_size;

  libspectrum_mmc_eject( card );
  if( !filename ) return LIBSPECTRUM_ERROR_NONE;

  error = libspectrum_ide_insert_into_drive( &card->drive, filename );
  if( error ) return error;

  card->total_sectors = (libspectrum_dword)card->drive.cylinders *
    card->drive.heads * card->drive.sectors;

  if( card->drive.sector_size != 512 || card->total_sectors % 1024 ) {
      libspectrum_print_error( LIBSPECTRUM_ERROR_UNKNOWN,
        "Image size not supported" );
    return LIBSPECTRUM_ERROR_UNKNOWN;
  }

  /* memory capacity = (C_SIZE+1) * 512K bytes
     we reduce the sector count by 1024. This gives us a minimum card size
     of 512 Kb. Not too worried about that. */
  c_size = ( card->total_sectors >> 10 ) - 1;

  /* We emulate a SDHC card which has a maximum size of (32 Gb - 80 Mb) */
  if( c_size >= 65375 ) {
    libspectrum_print_error( LIBSPECTRUM_ERROR_UNKNOWN,
      "Image size too big (>32 Gb)" );
    return LIBSPECTRUM_ERROR_UNKNOWN;
  }

  card->c_size = c_size;

  return LIBSPECTRUM_ERROR_NONE;
}

void
libspectrum_mmc_eject( libspectrum_mmc_card *card )
{
  libspectrum_ide_eject_from_drive( &card->drive, card->cache );
  libspectrum_mmc_reset( card );
}

void
libspectrum_mmc_reset( libspectrum_mmc_card *card )
{
  card->r1_status = IN_IDLE_STATE_MASK;
  card->command_state = WAITING_FOR_COMMAND;
  card->response_buffer_next = card->response_buffer;
  card->response_buffer_end = card->response_buffer;
  card->cmd8_issued = 0;
  card->cmd55_issued = 0;
  card->multiple_read_active = 0;
  card->erase_sequence = SEQ_ERASE_NONE;
  card->erase_block_start = 0;
  card->erase_block_end = 0;
}

int
libspectrum_mmc_dirty( libspectrum_mmc_card *card )
{
  return g_hash_table_size( card->cache ) != 0;
}

void
libspectrum_mmc_commit( libspectrum_mmc_card *card )
{
  libspectrum_ide_commit_drive( &card->drive, card->cache );
}

libspectrum_byte
libspectrum_mmc_read( libspectrum_mmc_card *card )
{
  libspectrum_byte r = 0xff;

  if( card->response_buffer_next < card->response_buffer_end ) {
    r = *(card->response_buffer_next)++;
  } else if( card->multiple_read_active ) {
    mmc_read_multiple_blocks( card );
    /* note: even though we've filled the buffer, *this* read request returns 0xff */
  }

  return r;
}

static int
parse_command( libspectrum_mmc_card *card, libspectrum_byte b )
{
  /* All commands should have start bit == 0 and transmitter bit == 1 */
  if( ( b & 0xc0 ) != 0x40 ) return 0;  /* don't change current state */

  /* Unlike the SD Memory Card protocol, in the SPI mode, the card will always
     respond to a command. The response indicates acceptance or rejection of
     the command. Now wait for arguments and CRC */
  card->current_command = b & 0x3f;
  return 1;
}

/* static void
reset_response_buffer( libspectrum_mmc_card *card )
{
  card->response_buffer_end = card->response_buffer_next = card->response_buffer;
} */

/* static void
set_response_buffer_size( libspectrum_mmc_card *card, int size )
{
  card->response_buffer_next = card->response_buffer;
  card->response_buffer_end = card->response_buffer + size;
} */

static void
set_response_buffer( libspectrum_mmc_card *card, libspectrum_byte value )
{
  card->response_buffer[ 0 ] = value;
  card->response_buffer_next = card->response_buffer;
  card->response_buffer_end = card->response_buffer + 1;
}

static void
set_response_buffer2( libspectrum_mmc_card *card, libspectrum_byte v1, libspectrum_byte v2 )
{
  card->response_buffer[ 0 ] = v1;
  card->response_buffer[ 1 ] = v2;
  card->response_buffer_next = card->response_buffer;
  card->response_buffer_end = card->response_buffer + 2;
}

/* static void
append_response_buffer( libspectrum_mmc_card *card, libspectrum_byte value )
{
  const int pos = card->response_buffer_end - card->response_buffer_next;
  card->response_buffer[ pos ] = value;
  card->response_buffer_end++;
} */

#define set_response_buffer_r1( c ) set_response_buffer( c, c->r1_status )

static void
set_response_buffer_r7( libspectrum_mmc_card *card, libspectrum_dword value )
{
  card->response_buffer[ 0 ] = card->r1_status;
  libspectrum_write_dword_be( &card->response_buffer[ 1 ], value );
  card->response_buffer_next = card->response_buffer;
  card->response_buffer_end = card->response_buffer + 5;
}

static void
set_data_address( libspectrum_mmc_card *card, libspectrum_dword sector_number )
{
  libspectrum_write_dword_be( card->current_argument, sector_number );
}

static void
reset_erase_sequence( libspectrum_mmc_card *card )
{
  card->erase_sequence = SEQ_ERASE_NONE;
  card->erase_block_start = 0;
  card->erase_block_end = 0;
}

static void
erase_wr_blk_start( libspectrum_mmc_card *card )
{
  /* Card initialised? */
  if( card->r1_status & IN_IDLE_STATE_MASK ) {
    card->r1_status |= ILLEGAL_COMMAND_MASK;
    set_response_buffer_r1( card );
    return;
  }

  /* Check erase sequence */
  if( card->erase_sequence != SEQ_ERASE_NONE ) {
    card->r1_status |= ERASE_SEQ_ERROR_MASK;
    set_response_buffer_r1( card );
    reset_erase_sequence( card );
    return;
  }

  card->erase_block_start =
    libspectrum_read_dword_be( card->current_argument );

  /* Sector out of range */
  if( card->erase_block_start >= card->total_sectors ) {
    card->r1_status |= PARAMETER_ERROR_MASK;
    set_response_buffer_r1( card );
    reset_erase_sequence( card );
    return;
  }

  set_response_buffer_r1( card );
  card->erase_sequence = SEQ_ERASE_WR_BLK_START;
}

static void
erase_wr_blk_end( libspectrum_mmc_card *card )
{
  /* Card initialised? */
  if( card->r1_status & IN_IDLE_STATE_MASK ) {
    card->r1_status |= ILLEGAL_COMMAND_MASK;
    set_response_buffer_r1( card );
    return;
  }

  /* Check erase sequence */
  if( card->erase_sequence != SEQ_ERASE_WR_BLK_START ) {
    card->r1_status |= ERASE_SEQ_ERROR_MASK;
    set_response_buffer_r1( card );
    reset_erase_sequence( card );
    return;
  }

  card->erase_block_end =
    libspectrum_read_dword_be( card->current_argument );

  /* Sector out of range */
  if( card->erase_block_end >= card->total_sectors ) {
    card->r1_status |= PARAMETER_ERROR_MASK;
    set_response_buffer_r1( card );
    reset_erase_sequence( card );
    return;
  }

  /* Check continuous range */
  if( card->erase_block_start > card->erase_block_end ) {
    card->r1_status |= PARAMETER_ERROR_MASK;
    set_response_buffer_r1( card );
    reset_erase_sequence( card );
    return;
  }

  set_response_buffer_r1( card );
  card->erase_sequence = SEQ_ERASE_WR_BLK_END;
}

static void
erase( libspectrum_mmc_card *card )
{
  libspectrum_dword i;

  /* Card initialised? */
  if( card->r1_status & IN_IDLE_STATE_MASK ) {
    card->r1_status |= ILLEGAL_COMMAND_MASK;
    set_response_buffer_r1( card );
    return;
  }

  /* Check erase sequence */
  if( card->erase_sequence != SEQ_ERASE_WR_BLK_END ) {
    card->r1_status |= ERASE_SEQ_ERROR_MASK;
    set_response_buffer_r1( card );
    reset_erase_sequence( card );
    return;
  }

  /* The data at the card after an erase operation is either '0' or '1',
     depends on the card vendor */
  memset( card->send_buffer, 0x00, 512 );

  for( i = card->erase_block_start; i <= card->erase_block_end; i++ ) {
    libspectrum_ide_write_sector_to_hdf( &card->drive, card->cache, i,
                                         card->send_buffer );
  }

  /* R1 command response + busy flag */
  set_response_buffer2( card, card->r1_status, 0x01 );

  reset_erase_sequence( card );
}

static int
set_response_buffer_from_sector( libspectrum_mmc_card *card, libspectrum_dword sector_number, int offset )
{
  int error;

  /* First byte: Start Block token */
  card->response_buffer[ offset ] = TOKEN_START_BLOCK;
  /* Bytes 2-513 filled using libspectrum_ide_read_sector_from_hdf */
  error = libspectrum_ide_read_sector_from_hdf(
      &card->drive, card->cache, sector_number, &card->response_buffer[ offset + 1 ]
  );

  /* Data retrieval error */
  if( error ) {
    if( offset == 0 ) {
      /* 2nd and so on packet, data error token only */
      set_response_buffer( card, 0x01 ); /* data error token */
      /* Terminate the multiple block read; the card now waits for CMD12 */
      card->multiple_read_active = 0;
      card->command_state = WAITING_FOR_COMMAND;
    } else {
      /* First data packet, override command response */
      /* R1 command response + data error token */
      set_response_buffer2( card, card->r1_status, 0x01 );
    }
    return error;
  }

  /* Last two bytes: CRC */
  memset( &card->response_buffer[ offset+1+512 ], 0x00, 2 );

  card->response_buffer_next = card->response_buffer;
  card->response_buffer_end = card->response_buffer + offset + 1+512+2;

  return LIBSPECTRUM_ERROR_NONE;
}

static void
read_single_block( libspectrum_mmc_card *card )
{
  libspectrum_dword sector_number;
  int error;

  /* Card initialised? */
  if( card->r1_status & IN_IDLE_STATE_MASK ) {
    card->r1_status |= ILLEGAL_COMMAND_MASK;
    set_response_buffer_r1( card );
    return;
  }

  sector_number = libspectrum_read_dword_be( card->current_argument );

  /* Sector out of range */
  if( sector_number >= card->total_sectors ) {
    card->r1_status |= PARAMETER_ERROR_MASK;
    set_response_buffer_r1( card );
    return;
  }

  // Command Response
  card->response_buffer[ 0 ] = card->r1_status;
  /* offset=1 since we already have command response in the buffer */
  error = set_response_buffer_from_sector( card, sector_number, 1 );

  /* A multiple block read streams sectors until the last sector or an
     error; CMD12/STOP_TRANSMISSION terminates it */
  if( !error && READ_MULTIPLE_BLOCK == card->current_command )
    card->multiple_read_active = 1;
}

static void
mmc_read_multiple_blocks( libspectrum_mmc_card *card )
{
#ifdef MMC_SIMULATE_MULTI_BLOCK_DELAY
  if( card->response_buffer_end > (card->response_buffer+256) ) {
    memset( card->response_buffer, 0xff, 16 );
    card->response_buffer_next = card->response_buffer;
    card->response_buffer_end = card->response_buffer + 16;
    return;
  }
#endif

  libspectrum_dword sector_number;
  /* both single and multiple block read start with read_single_block(...) */
  /* thus we need to advance to the next sector first */
  sector_number = libspectrum_read_dword_be( card->current_argument ) + 1;

  /* Sector out of range */
  if( sector_number >= card->total_sectors ) {
    set_response_buffer( card, 0x08 ); /* data error token: out of range */
    /* Terminate the multiple block read; the card now waits for CMD12 */
    card->multiple_read_active = 0;
    card->command_state = WAITING_FOR_COMMAND;
    return;
  }

  set_data_address( card, sector_number );
  /* offset=0 - sector data starts from the beginning of the buffer */
  set_response_buffer_from_sector( card, sector_number, 0 );
}

static void
stop_transmission( libspectrum_mmc_card *card )
{
  int i = 0;
  // typically card may send at least one more byte of the data packet
  if( card->response_buffer_next < card->response_buffer_end ) {
    card->response_buffer[ i++ ] = *(card->response_buffer_next);
    // technically, there may be up to Ncr clock cycles BUSY (0xff)
    // but we always transfer full bytes, so things are naturally
    // Ncr aligned
  } else {
    // if we have nothing to transmit, it still takes one byte cycle
    // for card's state machine to process CMD12, hence, the "stuff" byte
    card->response_buffer[ i++ ] = 0xff;
#ifdef MMC_TRACE_CMD12
    libspectrum_print_error(
        LIBSPECTRUM_ERROR_WARNING,
        "Received STOP_TRANSMISSION when no data in the buffer (cmd %d)\n",
        card->current_command );
#endif /* MMC_TRACE_CMD12 */
  }

  card->response_buffer[ i ] = card->r1_status;
  // for write op the response should have trailing busy (0x00) bytes
  card->response_buffer_next = card->response_buffer;
  card->response_buffer_end = card->response_buffer + i + 1;

  /* A CMD12 terminates an ongoing multiple block read */
  card->multiple_read_active = 0;

  card->command_state = WAITING_FOR_COMMAND;
}

static void
send_csd( libspectrum_mmc_card *card )
{
  /* Card initialised? */
  if( card->r1_status & IN_IDLE_STATE_MASK ) {
    card->r1_status |= ILLEGAL_COMMAND_MASK;
    set_response_buffer_r1( card );
    return;
  }

  card->response_buffer[ 0 ] = card->r1_status; /* R1 command response */
  card->response_buffer[ 1 ] = 0xfe;            /* data token */

  memset( &card->response_buffer[ 2 ], 0x00, 16 );

  /* CSD_STRUCTURE version 2.0 */
  card->response_buffer[ 2 ] = 0x40;

  /* CCC card command classes (spread 8 bits, 4 bits across two bytes) */
  card->response_buffer[ 2 +  4 ] = 0x5b;
  card->response_buffer[ 2 +  5 ] = 0x05;

  /* READ_BL_LEN = 9 => 2 ^ 9 = 512 byte sectors */
  card->response_buffer[ 2 +  5 ] |= 0x09;

  /* C_SIZE (spread 6 bits, 8 bits, 8 bits across three bytes),
     first 6 bits set to zero */
  card->response_buffer[ 2 +  8 ] = ( card->c_size >> 8 ) & 0xff;
  card->response_buffer[ 2 +  9 ] = card->c_size & 0xff;

  /* ERASE_BLK_EN (1 bit ) */
  card->response_buffer[ 2 +  10 ] = 0x40;

  /* SECTOR_SIZE (spread 7 bits, 1 bit across two bytes) */
  card->response_buffer[ 2 +  10 ] |= 0x3f;
  card->response_buffer[ 2 +  11 ]  = 0x80;

  /* WRITE_BL_LEN = 9 => 2 ^ 9 = 512 byte sectors
     (spread 2 bits, 2 bits across two bytes) */
  card->response_buffer[ 2 +  12 ] = 0x10;
  card->response_buffer[ 2 +  13 ] = 0x01;

  /* Bit 0, not used, always 1 */
  card->response_buffer[ 2 +  15 ] = 0x01;

  /* CRC */
  memset( &card->response_buffer[ 18 ], 0x00, 2 );

  card->response_buffer_next = card->response_buffer;
  card->response_buffer_end = card->response_buffer + 20;
}

static void
send_cid( libspectrum_mmc_card *card )
{
  /* Card initialised? */
  if( card->r1_status & IN_IDLE_STATE_MASK ) {
    card->r1_status |= ILLEGAL_COMMAND_MASK;
    set_response_buffer_r1( card );
    return;
  }

  card->response_buffer[ 0 ] = card->r1_status; /* R1 command response */
  card->response_buffer[ 1 ] = 0xfe;            /* data token */

  /* For now, we return a mostly empty CID. This seems to work. */
  memset( &card->response_buffer[ 2 ], 0x00, 16 );

  /* Set blank OID */
  memcpy( &card->response_buffer[ 2 + 1 ], "  ", 2 );

  /* Set product name */
  memcpy( &card->response_buffer[ 2 + 3 ], "Fuse", 4 );

  /* Bit 0, not used, always 1 */
  card->response_buffer[ 2 +  15 ] = 0x01;

  /* CRC */
  memset( &card->response_buffer[ 18 ], 0x00, 2 );

  card->response_buffer_next = card->response_buffer;
  card->response_buffer_end = card->response_buffer + 20;
}

static void
send_write_command_response( libspectrum_mmc_card *card )
{
  libspectrum_dword sector_number;

  /* Card initialised? */
  if( card->r1_status & IN_IDLE_STATE_MASK ) {
    card->r1_status |= ILLEGAL_COMMAND_MASK;
    set_response_buffer_r1( card );
    /* and skip 'waiting for token' state */
    card->command_state = WAITING_FOR_COMMAND;
    return;
  }

  sector_number = libspectrum_read_dword_be( card->current_argument );

  /* Sector out of range */
  if( sector_number >= card->total_sectors ) {
    card->r1_status |= PARAMETER_ERROR_MASK;
    set_response_buffer_r1( card );
    /* and skip 'waiting for token' state */
    card->command_state = WAITING_FOR_COMMAND;
    return;
  }

  set_response_buffer_r1( card );
}

static int
do_application_command( libspectrum_mmc_card *card )
{
  int is_acmd = 1;

  switch( card->current_command ) {
    case SD_STATUS:
    case SEND_NUM_WR_BLOCKS:
    case SET_WR_BLK_ERASE_COUNT:
    case SET_CLR_CARD_DETECT:
    case SEND_SCR:
      /* Application command not implemented, return illegal command for now */
      card->r1_status |= ILLEGAL_COMMAND_MASK;
      set_response_buffer_r1( card );

      libspectrum_print_error(
          LIBSPECTRUM_ERROR_UNKNOWN,
          "Unsupported MMC application command %d received",
          card->current_command );
      break;

    case SD_SEND_OP_COND:
      /* SDHC cards return in_idle_state=1 when:
         1. CMD8 was not issued before ACMD41
         2. ACMD41 is issued with HCS=0 (host only supports SDSC)
      */
      if( card->cmd8_issued && ( card->current_argument[ 0 ] & 0x40 ) ) {
        card->r1_status &= ~IN_IDLE_STATE_MASK;
      }
      set_response_buffer_r1( card );
      break;

    default:
      is_acmd = 0;
      libspectrum_print_error(
          LIBSPECTRUM_ERROR_UNKNOWN,
          "Unknown MMC application command %d received",
          card->current_command );
      break;
  }

  return is_acmd;
}

static void
do_standard_command( libspectrum_mmc_card *card )
{
  switch( card->current_command ) {
    case GO_IDLE_STATE:
      card->r1_status |= IN_IDLE_STATE_MASK;
      card->cmd8_issued = 0;
      set_response_buffer_r1( card );
      break;
    case SEND_IF_COND:
      card->cmd8_issued = 1;

      /* return echo back pattern */
      set_response_buffer_r7( card, 0x00000100 | card->current_argument[ 3 ] );
      break;
    case SEND_CSD:
      send_csd( card );
      break;
    case SEND_CID:
      send_cid( card );
      break;
    case STOP_TRANSMISSION:
      stop_transmission( card );
      break;
    case SET_BLOCKLEN:
      /* SDHC: LOCK_UNLOCK length only (we do not support it yet) */
      /* e.g. card->lock_length = get_data_address( card ); */
      set_response_buffer_r1( card );
      break;
    case READ_SINGLE_BLOCK:
    case READ_MULTIPLE_BLOCK:
      read_single_block( card );
      break;
    case WRITE_BLOCK:
    case WRITE_MULTIPLE_BLOCK:
      send_write_command_response( card );
      break;
    case ERASE_WR_BLK_START:
      erase_wr_blk_start( card );
      break;
    case ERASE_WR_BLK_END:
      erase_wr_blk_end( card );
      break;
    case ERASE:
      erase( card );
      break;
    case APP_CMD:
      card->cmd55_issued = 1;
      set_response_buffer_r1( card );
      break;
    case READ_OCR:
      /* We set only the card capacity status (CCS, bit 30) and card power up
         status bits (bit 31). CCS set indicates an SDHC card. */
      set_response_buffer_r7( card, 0xc0000000 );
      break;
    case CRC_ON_OFF: /* [31:1] stuff bits; [0:0] CRC option '1' - ON, '0' - OFF */
      if( 0 != ( card->current_argument[3] & 0x01 ) ) {
        card->r1_status |= ILLEGAL_COMMAND_MASK;
        libspectrum_print_error(
            LIBSPECTRUM_ERROR_UNKNOWN, "MMC command %d (CRC ON) not implemented",
            card->current_command );
      }
      set_response_buffer_r1( card );
      break;
    default:
      /* Command not implemented or illegal command */
      card->r1_status |= ILLEGAL_COMMAND_MASK;
      set_response_buffer_r1( card );

      libspectrum_print_error(
          LIBSPECTRUM_ERROR_UNKNOWN, "Unknown MMC command %d received",
          card->current_command );
      break;
  }
}

static void
do_command( libspectrum_mmc_card *card )
{
  /* Any new command terminates an ongoing multiple block read */
  card->multiple_read_active = 0;

  /* Previous APP_CMD indicates to the card that the next command is an
     application specific command rather than a standard command */
  if( card->cmd55_issued ) {
    card->cmd55_issued = 0;

    /* If a non-ACMD is sent, then it is respected by the card as a normal
       SD Memory Card command */
    if( do_application_command( card ) )
      return;
  }

  /* Check ongoing erase sequence */
  if( card->erase_sequence != SEQ_ERASE_NONE ) {
    switch( card->current_command ) {
      case ERASE_WR_BLK_START:
      case ERASE_WR_BLK_END:
      case ERASE:
        break;
      default:
        /* Clear erase sequence before executing because an out of erase
           sequence command was received */
        reset_erase_sequence( card );
        card->r1_status |= ERASE_RESET_MASK;
        break;
    }
  }

  do_standard_command( card );
}

static void
write_sector_from_send_buffer( libspectrum_mmc_card *card, libspectrum_dword sector_number )
{
  libspectrum_ide_write_sector_to_hdf( &card->drive, card->cache, sector_number,
   card->send_buffer );

  set_response_buffer2( card, 0x05, 0x01 ); /* data response + busy flag (end of programming) */
}

static void
write_single_block( libspectrum_mmc_card *card )
{
  /* Note: command response sent earlier by send_write_command_response(...) */
  /* we only write the data block and return data response token */
  libspectrum_dword sector_number;

  sector_number = libspectrum_read_dword_be( card->current_argument );
  /* sector_number is validated by the send_write_command_response(...) */
  write_sector_from_send_buffer( card, sector_number );
}

static void
write_multiple_block( libspectrum_mmc_card *card )
{
  /* unlike read op we have two distinct paths for single and multiple blocks */
  /* all sectors of multi-block sequence are handled here */
  libspectrum_dword sector_number = libspectrum_read_dword_be( card->current_argument );

  if( sector_number >= card->total_sectors ) {
    /*                           xxx0SSS1 */
    set_response_buffer( card, 0b00001101U ); /* Data Response token: Data Rejected due to a Write Error */
    /* In case of any error (CRC or Write Error) during Write Multiple Block operation, the host shall stop the
    data transmission using CMD12. In case of a Write Error (response '110'), the host may send CMD13
    (SEND_STATUS) in order to get the cause of the write problem. ACMD22 can be used to find the number
    of well written write blocks. */
    card->command_state = WAITING_FOR_COMMAND;
    return;
  }

  set_data_address( card, sector_number + 1 );  /* keep data address for the next iteration */

  write_sector_from_send_buffer( card, sector_number );
}

static void
do_command_data( libspectrum_mmc_card *card )
{
  switch( card->current_command ) {
    case WRITE_BLOCK:
      write_single_block( card );
      break;
    case WRITE_MULTIPLE_BLOCK:
      write_multiple_block( card );
      break;
    default:
      /* This should never happen as it indicates a failure in our state machine */
      libspectrum_print_error(
          LIBSPECTRUM_ERROR_LOGIC,
          "Attempting to execute unknown MMC data command %d\n",
          card->current_command );
      break;
  }
}

static void
do_data_token( libspectrum_mmc_card *card, libspectrum_byte data )
{
  /* make sure this function handles transition to the next state */
  switch( data ) {
    case TOKEN_START_BLOCK: /* Read, Write single: Start Block token */
      card->command_state = WAITING_FOR_DATA;
      card->data_count = 0;
      break;
    case TOKEN_START_MBWR_BLOCK: /* Write multiple: Start Block token */
      card->command_state = WAITING_FOR_DATA;
      card->data_count = 0;
      break;
    case TOKEN_STOP_TRAN: /* Stop Tran token */
       // TODO: add busy ?
      card->command_state = WAITING_FOR_COMMAND;
      break;
    case 0xff:
      /* we explicitly ignore "idle line", if sent to us */
      break;
    default:
      // Here is a descrepancy in the spec:
      // if host sends anything other than a valid token or a standard high
      // idle line (0xFF), it is operating entirely outside the boundaries of
      // the specification.
      // IIRC, **most** cards ignore anything except valid tokens or idle line.
      // Hence, we ignore 0xff and log everything else.
      libspectrum_print_error(
          LIBSPECTRUM_ERROR_UNKNOWN,
          "Received unexpected MMC data token %d (cmd %d)\n",
          data, card->current_command );
      break;
  }
}

void
libspectrum_mmc_write( libspectrum_mmc_card *card, libspectrum_byte data )
{
  /* No card inserted => no change in state */
  if( !card->drive.disk ) return;

  switch( card->command_state ) {
    case WAITING_FOR_COMMAND:
      if( parse_command( card, data ) )
        card->command_state = WAITING_FOR_DATA0;
      break;
    case WAITING_FOR_DATA0:
    case WAITING_FOR_DATA1:
    case WAITING_FOR_DATA2:
    case WAITING_FOR_DATA3:
      card->current_argument[ card->command_state - 1 ] = data;
      card->command_state++;
      break;
    case WAITING_FOR_CRC:
      /* We ignore the CRC */
      do_command( card );

      /* reset command error flags except in_idle */
      card->r1_status &= IN_IDLE_STATE_MASK;

      /* Let do_command(...) override the next state, if necessary */
      if( card->command_state == WAITING_FOR_CRC ) {
        card->command_state =
          card->current_command == WRITE_BLOCK
              || card->current_command == WRITE_MULTIPLE_BLOCK ?
          WAITING_FOR_DATA_TOKEN :
          WAITING_FOR_COMMAND;
      }
      break;
    case WAITING_FOR_DATA_TOKEN:
      do_data_token( card, data );
      break;
    case WAITING_FOR_DATA:
      card->send_buffer[ card->data_count++ ] = data;
      if( card->data_count == 512 )
        card->command_state = WAITING_FOR_DATA_CRC1;
      break;
    case WAITING_FOR_DATA_CRC1:
      /* We ignore the data CRC as well */
      card->command_state = WAITING_FOR_DATA_CRC2;
      break;
    case WAITING_FOR_DATA_CRC2:
      do_command_data( card );
      /* Let do_command_data(...) override the next state, if necessary */
      if( card->command_state == WAITING_FOR_DATA_CRC2 ) {
        card->command_state = card->current_command == WRITE_MULTIPLE_BLOCK ?
          WAITING_FOR_DATA_TOKEN :
          WAITING_FOR_COMMAND;
      }
      break;
    }
}

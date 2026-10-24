/* mmc.c: unit tests for the MMC/SD card emulation
   Copyright (c) 2026 Philip Kendall
   Copyright (c) 2026 Vic Chwe

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

*/

#include "config.h"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "common.h"
#include "internals.h"
#include "test.h"

/* Data control tokens (see section 7.3.3 of the SD specification) */
#define TEST_TOKEN_START_BLOCK      0xfe /* 11111110 */
#define TEST_TOKEN_START_MBWR_BLOCK 0xfc /* 11111100 */
#define TEST_TOKEN_STOP_TRAN        0xfd /* 11111101 */

/* R1 status bits */
#define TEST_R1_ILLEGAL_COMMAND 0x04
#define TEST_R1_PARAMETER_ERROR 0x40

/* Data response tokens */
#define TEST_DATA_RESPONSE_ACCEPTED     0x05 /* 0101: data accepted */
#define TEST_DATA_RESPONSE_WRITE_ERROR  0x0d /* 1101: data rejected, write error */

/* Data error tokens */
#define TEST_DATA_ERROR_OUT_OF_RANGE 0x08

/* The emulation never computes a real CRC: every data packet ends with
   two zero bytes */
#define TEST_CRC_BYTES 2

/* The fill byte used for sector n by mmc_test_image_create( filename, 1 ) */
#define TEST_SECTOR_FILL( n ) ( (libspectrum_byte)( ( n ) & 0xff ) )

/* The fill byte the multi-block write test uses for sector n; distinct from
   TEST_SECTOR_FILL( n ) so a successful write can be told from the pristine
   image contents */
#define TEST_WRITE_FILL( n ) ( (libspectrum_byte)( ( ( n ) & 0xff ) ^ 0xa5 ) )

enum mmc_spi_command {
  SEND_IF_COND = 8,
  STOP_TRANSMISSION = 12,
  SET_BLOCKLEN = 16,
  READ_SINGLE_BLOCK = 17,
  READ_MULTIPLE_BLOCK = 18,
  WRITE_BLOCK = 24,
  WRITE_MULTIPLE_BLOCK = 25,
  ERASE_WR_BLK_START = 32,
  ERASE_WR_BLK_END = 33,
  APP_CMD = 55,
  CRC_ON_OFF = 59
};

/* The SPI mode command sequence: command byte, 4 argument bytes and a CRC
   byte. Returns the first response byte. */
static libspectrum_byte
mmc_command( libspectrum_mmc_card *card, enum mmc_spi_command command,
             libspectrum_dword argument )
{
  int shift;
  libspectrum_mmc_write( card, 0x40 | command );
  for( shift = 24; shift >= 0; shift -= 8 )
    libspectrum_mmc_write( card, ( argument >> shift ) & 0xff );
  libspectrum_mmc_write( card, 0xff );
  return libspectrum_mmc_read( card );
}

/* The CMD8/ACMD41 initialisation sequence. Returns non-zero on failure. */
static int
mmc_initialise( libspectrum_mmc_card *card )
{
  int i;
  libspectrum_mmc_reset( card );
  /* CMD8 SEND_IF_COND 0x01 - VHS: 2.7V–3.6V; 0xAA - pattern */
  /* expected response R7: 0:R1;1-2:flags;3:Vecho;4:PatternEcho */
  /* 0x01, CmdVer, 0x00, echo VHS (0x01), echo pattern (0xAA) */
  if( mmc_command( card, SEND_IF_COND, 0x1aa ) != 1 ) return 1;
  for( i = 0; i < 4; i++ ) libspectrum_mmc_read( card );
  /* CMD55 APP_CMD; R1 response: 0x01 In Idle State */
  /* ACMD41 SD_SEND_OP_COND; HostCapacitySupport = 1; */
  /*     R1 response: 0x00 Success/Ready (can be: 0x00, 0x01, 0x05) */
  return mmc_command( card, APP_CMD, 0 ) != 1 ||
         mmc_command( card, 41, 0x40000000 ) != 0;
}

/* Create a minimal 1024 sector HDF image for the MMC tests (1024 sectors is
   the smallest card accepted by libspectrum_mmc_insert). If fill_with pattern
   is non-zero, sector n is filled with the byte n & 0xff, otherwise the card
   reads back as zeros. The file is removed again if anything fails. */
static test_return_t
mmc_test_image_create( const char *filename, int fill_with_pattern )
{
  libspectrum_hdf_header header;
  libspectrum_byte sector[512];
  libspectrum_dword i;
  const size_t total_sectors = 1024;
  FILE *file;
  int fd, flags = O_WRONLY | O_CREAT | O_EXCL;

  /* Avoid overwriting an existing file, without relying on C11 fopen modes
     that older Windows runtimes do not support. */
#ifdef O_BINARY
  flags |= O_BINARY;
#endif
  fd = open( filename, flags, 0600 );
  if( fd < 0 ) {
    fprintf( stderr, "%s: unable to create %s: %s\n",
             progname, filename, strerror( errno ) );
    return TEST_INCOMPLETE;
  }
  file = fdopen( fd, "wb" );
  if( !file ) {
    fprintf( stderr, "%s: unable to open stream for %s: %s\n",
             progname, filename, strerror( errno ) );
    close( fd );
    return TEST_INCOMPLETE;
  }
  memset( &header, 0, sizeof( header ) );
  memcpy( header.signature, "RS-IDE", 6 );
  header.id = 0x1a;
  header.datastart_low = sizeof( header );
  /* 1024 cylinders x 1 head x 1 sector per track */
  header.drive_identity[3] = 4;
  header.drive_identity[6] = 1;
  header.drive_identity[12] = 1;
  if( fwrite( &header, 1, sizeof( header ), file ) != sizeof( header ) ) {
    fprintf( stderr, "%s: unable to write %s: %s\n",
             progname, filename, strerror( errno ) );
    fclose( file ); goto fail;
  }

  if( fill_with_pattern ) {
    for( i = 0; i < total_sectors; i++ ) {
      memset( sector, i & 0xff, sizeof( sector ) );
      if( fwrite( sector, 1, sizeof( sector ), file ) != sizeof( sector ) ) {
        fprintf( stderr, "%s: unable to write %s: %s\n",
                 progname, filename, strerror( errno ) );
        fclose( file ); goto fail;
      }
    }
  }

  if( fclose( file ) ) {
    fprintf( stderr, "%s: unable to close %s: %s\n",
             progname, filename, strerror( errno ) );
    goto fail;
  }

  return TEST_PASS;

fail:
  if( remove( filename ) ) {
    fprintf( stderr, "%s: unable to remove %s\n", progname, filename );
  }
  return TEST_INCOMPLETE;
}

/* Read `length` bytes from the card and check they all equal supplied `value` */
static int
mmc_read_matches( libspectrum_mmc_card *card, size_t length,
                  libspectrum_byte value )
{
  size_t i;

  for( i = 0; i < length; i++ ) {
    if( libspectrum_mmc_read( card ) != value ) return 0;
  }

  return 1;
}

/* Read and validate a data packet: a start block token, 512 data bytes and
   two zero CRC bytes */
static int
mmc_read_packet( libspectrum_mmc_card *card, libspectrum_byte token,
                 libspectrum_byte fill )
{
  return libspectrum_mmc_read( card ) == token &&
         mmc_read_matches( card, 512, fill ) &&
         mmc_read_matches( card, TEST_CRC_BYTES, 0x00 );
}

/* Common test set up: create the card image, insert and initialise.
   Returns NULL if anything failed, in which case the caller must not
   continue. If fill_with_pattern is non-zero, sector n of the image is filled
   with the byte n & 0xff, otherwise it reads back as zeros. */
static libspectrum_mmc_card *
mmc_test_setup( const char *filename, const char *test_name, int fill_with_pattern )
{
  libspectrum_mmc_card *card;

  if( mmc_test_image_create( filename, fill_with_pattern ) != TEST_PASS ) return NULL;

  card = libspectrum_mmc_alloc();
  if( !card ) {
    fprintf( stderr, "%s: %s: libspectrum_mmc_alloc failed\n",
             progname, test_name );
    return NULL;
  }

  if( libspectrum_mmc_insert( card, filename ) ) {
    fprintf( stderr, "%s: %s: unable to insert MMC image %s\n",
             progname, test_name, filename );
    libspectrum_mmc_free( card );
    return NULL;
  }

  if( mmc_initialise( card ) ) {
    fprintf( stderr, "%s: %s: unable to initialise MMC card\n",
             progname, test_name );
    libspectrum_mmc_free( card );
    return NULL;
  }

  return card;
}

/* Tear down the test card and remove the test image */
static test_return_t
mmc_test_teardown( libspectrum_mmc_card *card, const char *filename,
                   const char *test_name, int failed )
{
  test_return_t r = failed ? TEST_FAIL : TEST_PASS;

  if( failed )
    fprintf( stderr, "%s: %s failed\n", progname, test_name );

  if( card ) libspectrum_mmc_free( card );

  if( remove( filename ) ) {
    fprintf( stderr, "%s: %s: unable to remove %s\n",
             progname, test_name, filename );
    r = TEST_FAIL;
  }

  return r;
}

/* Out of range sector arguments must be rejected before any disk I/O
   occurs; in particular CMD24 must not enter the data token state */
test_return_t
mmc_high_bit_command_arguments( void )
{
  const char *filename = "test-mmc-dword.hdf";
  libspectrum_mmc_card *card =
    mmc_test_setup( filename, "mmc_high_bit_command_arguments", 0 );
  int failed = 1;
  size_t i, j;
  /* Data Address (32-bit big-endian). Note: our test image has 1024 sectors */
  static const libspectrum_dword arguments[] = { 0x80000001, 0xffffffff };

  if( !card ) return TEST_INCOMPLETE;

  for( j = 0; j < sizeof( arguments ) / sizeof( arguments[0] ); j++ ) {
    libspectrum_dword argument = arguments[j];
    if( mmc_initialise( card ) ||
        mmc_command( card, READ_SINGLE_BLOCK, argument ) != 0x40 ||
        mmc_command( card, ERASE_WR_BLK_START, argument ) != 0x40 ||
        /* multiple BLK_START should be fine */
        mmc_command( card, ERASE_WR_BLK_START, 0 ) != 0 ||
        /* multiple BLK_END should be fine, too */
        mmc_command( card, ERASE_WR_BLK_END, argument ) != 0x40 ||
        /* there is a problem here: CMD24 after CMD33 should set ERASE_RESET
         * status bit (Bit 1 in the R1 response) or ERASE_SEQ_ERROR (Bit 4),
         * so it will get 0x02 or 0x10 in R1 (and the write itself proceeds
         * normally) */
        mmc_command( card, WRITE_BLOCK, argument ) != 0x40 ) goto done;
    libspectrum_mmc_write( card, 0xfe );
    for( i = 0; i < 514; i++ ) libspectrum_mmc_write( card, 0 );
    if( libspectrum_mmc_read( card ) != 0xff ||
        libspectrum_mmc_dirty( card ) ) goto done;
  }

  failed = 0;

done:
  return mmc_test_teardown( card, filename,
                            "mmc_high_bit_command_arguments", failed );
}

/* CMD17 must return exactly one data packet and no more data afterwards */
test_return_t
mmc_read_single_block_no_auto_refill( void )
{
  const char *filename = "test-mmc-single.hdf";
  libspectrum_mmc_card *card = mmc_test_setup(
    filename, "mmc_read_single_block_no_auto_refill", 1 );
  int failed = 1;

  if( !card ) return TEST_INCOMPLETE;

  if( mmc_command( card, READ_SINGLE_BLOCK, 5 ) == 0x00 &&
      mmc_read_packet( card, TEST_TOKEN_START_BLOCK, TEST_SECTOR_FILL( 5 ) ) &&
      /* No further data follows a single block read */
      mmc_read_matches( card, 4, 0xff ) )
    failed = 0;

  return mmc_test_teardown( card, filename,
                            "mmc_read_single_block_no_auto_refill", failed );
}

/* CMD18 streams consecutive sectors after the first packet, with the data
   address advanced by the card itself */
test_return_t
mmc_read_multiple_block_streams_sequential_sectors( void )
{
  const char *filename = "test-mmc-read-multi.hdf";
  libspectrum_mmc_card *card =
    mmc_test_setup( filename,
                    "mmc_read_multiple_block_streams_sequential_sectors", 1 );
  int failed = 1;

  if( !card ) return TEST_INCOMPLETE;

  if( mmc_command( card, READ_MULTIPLE_BLOCK, 100 ) == 0x00 &&
      /* First packet: command response then the sector data */
      mmc_read_packet( card, TEST_TOKEN_START_BLOCK, TEST_SECTOR_FILL( 100 ) ) &&
      /* The read request which refills the buffer returns 0xff */
      libspectrum_mmc_read( card ) == 0xff &&
      /* Subsequent packets carry no command response */
      mmc_read_packet( card, TEST_TOKEN_START_BLOCK, TEST_SECTOR_FILL( 101 ) ) &&
      libspectrum_mmc_read( card ) == 0xff &&
      mmc_read_packet( card, TEST_TOKEN_START_BLOCK, TEST_SECTOR_FILL( 102 ) ) &&
      /* The host terminates the stream with CMD12, gets "stuff" byte first */
      mmc_command( card, STOP_TRANSMISSION, 0 ) == 0xff &&
      /* The actual R1 response follows */
      mmc_read_matches( card, 1, 0x00 ) &&
      /* Finally, the SPI data line is idle */
      mmc_read_matches( card, 2, 0xff ) )
    failed = 0;

  return mmc_test_teardown(
    card, filename, "mmc_read_multiple_block_streams_sequential_sectors",
    failed );
}

/* Reading past the end of the card terminates the stream with a single
   data error token, after which the card waits for CMD12 */
test_return_t
mmc_read_multiple_block_ends_with_data_error_token( void )
{
  const char *filename = "test-mmc-read-end.hdf";
  libspectrum_mmc_card *card =
    mmc_test_setup( filename,
                    "mmc_read_multiple_block_ends_with_data_error_token", 1 );
  int failed = 1;

  if( !card ) return TEST_INCOMPLETE;

  /* Sector 1023 is the last sector of the card */
  if( mmc_command( card, READ_MULTIPLE_BLOCK, 1023 ) == 0x00 &&
      mmc_read_packet( card, TEST_TOKEN_START_BLOCK,
                       TEST_SECTOR_FILL( 1023 ) ) &&
      libspectrum_mmc_read( card ) == 0xff &&
      /* One out of range data error token, not one per read */
      libspectrum_mmc_read( card ) == TEST_DATA_ERROR_OUT_OF_RANGE &&
      mmc_read_matches( card, 8, 0xff ) &&
      /* The card then waits for CMD12 and returns a "stuff" byte */
      mmc_command( card, STOP_TRANSMISSION, 0 ) == 0xff &&
      /* Card sends the actual R1 response */
      mmc_read_matches( card, 1, 0x00 ) &&
      /* Finally, SPI line is in idle state */
      mmc_read_matches( card, 2, 0xff ) &&
      /* And the card carries on normally afterwards */
      mmc_command( card, SET_BLOCKLEN, 0 ) == 0x00 )
    failed = 0;

  return mmc_test_teardown(
    card, filename, "mmc_read_multiple_block_ends_with_data_error_token",
    failed );
}

/* CMD12 during a multiple block read delivers any pending data byte before
   the R1 response */
test_return_t
mmc_read_multiple_block_cmd12_mid_stream( void )
{
  const char *filename = "test-mmc-read-stop.hdf";
  libspectrum_mmc_card *card =
    mmc_test_setup( filename,
                    "mmc_read_multiple_block_cmd12_mid_stream", 1 );
  int failed = 1;

  if( !card ) return TEST_INCOMPLETE;

  /* Stop part way through the first packet: the first unread byte of the
     data packet is delivered before the R1 response */
  if( mmc_command( card, READ_MULTIPLE_BLOCK, 100 ) == 0x00 &&
      libspectrum_mmc_read( card ) == TEST_TOKEN_START_BLOCK &&
      mmc_read_matches( card, 100, TEST_SECTOR_FILL( 100 ) ) &&
      mmc_command( card, STOP_TRANSMISSION, 0 ) == TEST_SECTOR_FILL( 100 ) &&
      libspectrum_mmc_read( card ) == 0x00 &&
      mmc_read_matches( card, 2, 0xff ) &&
      /* With the buffer fully consumed, CMD12 returns just the R1 byte */
      mmc_command( card, READ_MULTIPLE_BLOCK, 100 ) == 0x00 &&
      mmc_read_packet( card, TEST_TOKEN_START_BLOCK,
                       TEST_SECTOR_FILL( 100 ) ) &&
      /* CMD12 returns "stuff" byte */
      mmc_command( card, STOP_TRANSMISSION, 0 ) == 0xff &&
      /* Actual R1 response follows */
      mmc_read_matches( card, 1, 0x00 ) &&
      /* Data line goes into idle state */
      mmc_read_matches( card, 2, 0xff ) )
    failed = 0;

  return mmc_test_teardown( card, filename,
                            "mmc_read_multiple_block_cmd12_mid_stream",
                            failed );
}

/* CMD25 writes consecutive sectors, advancing the data address itself, and
   the data reaches the card image */
test_return_t
mmc_write_multiple_block_writes_sequential_sectors( void )
{
  const char *filename = "test-mmc-write-multi.hdf";
  libspectrum_mmc_card *card =
    mmc_test_setup( filename,
                    "mmc_write_multiple_block_writes_sequential_sectors", 1 );
  const libspectrum_dword first_sector = 200;
  int i, failed = 1;

  if( !card ) return TEST_INCOMPLETE;

  if( mmc_command( card, WRITE_MULTIPLE_BLOCK, first_sector ) != 0x00 ) goto done;

  for( i = 0; i < 3; i++ ) {
    libspectrum_byte fill = TEST_WRITE_FILL( first_sector + i );
    size_t j;

    libspectrum_mmc_write( card, TEST_TOKEN_START_MBWR_BLOCK );
    for( j = 0; j < 512; j++ ) libspectrum_mmc_write( card, fill );
    libspectrum_mmc_write( card, 0 ); /* CRC */
    libspectrum_mmc_write( card, 0 ); /* CRC */

    /* Data response token then the busy flag: the sector was programmed */
    if( libspectrum_mmc_read( card ) != TEST_DATA_RESPONSE_ACCEPTED ) goto done;
    if( libspectrum_mmc_read( card ) != 0x01 ) goto done;
  }

  libspectrum_mmc_write( card, TEST_TOKEN_STOP_TRAN );

  /* The writes must have reached the card image */
  if( !libspectrum_mmc_dirty( card ) ) goto done;

  for( i = 0; i < 3; i++ ) {
    if( mmc_command( card, READ_SINGLE_BLOCK, first_sector + i ) != 0x00 ) goto done;
    if( !mmc_read_packet( card, TEST_TOKEN_START_BLOCK,
                          TEST_WRITE_FILL( first_sector + i ) ) ) goto done;
  }

  /* The card sends a "stuff" byte after CMD12 */
  if( mmc_command( card, STOP_TRANSMISSION, 0 ) != 0xff ) goto done;

  /* Actual R1 response */
  if ( !mmc_read_matches( card, 1, 0x00 ) ) goto done;

  failed = 0;

done:
  return mmc_test_teardown(
    card, filename, "mmc_write_multiple_block_writes_sequential_sectors",
    failed );
}

/* CMD25 arguments beyond the end of the card are rejected without entering
   the data token state */
test_return_t
mmc_write_multiple_block_rejects_out_of_range( void )
{
  const char *filename = "test-mmc-write-range.hdf";
  libspectrum_mmc_card *card =
    mmc_test_setup( filename,
                    "mmc_write_multiple_block_rejects_out_of_range", 1 );
  int failed = 1;

  if( !card ) return TEST_INCOMPLETE;

  /* The first sector past the end of the 1024 sector card */
  if( mmc_command( card, WRITE_MULTIPLE_BLOCK, 1024 ) != TEST_R1_PARAMETER_ERROR ) goto done;
  /* A wild high bit argument */
  if( mmc_command( card, WRITE_MULTIPLE_BLOCK, 0xffffffff ) != TEST_R1_PARAMETER_ERROR ) goto done;
  /* No data token state was entered, so a following command is processed
     normally rather than being swallowed as data */
  if( mmc_command( card, SET_BLOCKLEN, 0 ) != 0x00 ) goto done;

  failed = 0;

done:
  return mmc_test_teardown(
    card, filename, "mmc_write_multiple_block_rejects_out_of_range", failed );
}

/* CMD16 is accepted and CMD59 works with the CRC disabled */
test_return_t
mmc_cmd16_cmd59_stubs( void )
{
  const char *filename = "test-mmc-stubs.hdf";
  libspectrum_mmc_card *card =
    mmc_test_setup( filename, "mmc_cmd16_cmd59_stubs", 1 );
  int failed = 1;

  if( !card ) return TEST_INCOMPLETE;

  if( mmc_command( card, SET_BLOCKLEN, 512 ) == 0x00 &&
      mmc_command( card, CRC_ON_OFF, 0 ) == 0x00 &&
      /* Turning the CRC back on is not implemented and reported as an
         illegal command */
      mmc_command( card, CRC_ON_OFF, 1 ) == TEST_R1_ILLEGAL_COMMAND &&
      /* The illegal command flag does not persist to the next command */
      mmc_command( card, SET_BLOCKLEN, 0 ) == 0x00 )
    failed = 0;

  return mmc_test_teardown( card, filename, "mmc_cmd16_cmd59_stubs", failed );
}

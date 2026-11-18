/* wav_pcm.c: portable integer PCM WAV reader
   Copyright (c) 2026 Fredrick Meunier

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
   GNU General Public License for more details.
*/

#include "config.h"

#include <errno.h>
#include <limits.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "internals.h"
#include "wav_format.h"
#include "wav_internals.h"

struct libspectrum_wav_reader {
  libspectrum_fs_handle *file;
  size_t remaining;
  unsigned channels, bytes;
};

/* Information collected while scanning one RIFF/WAVE file. */
struct wav_info {
  libspectrum_dword rate, data_size;
  unsigned channels, bytes, align;
  int64_t data_offset;
};

static libspectrum_error
wav_error( libspectrum_error error, const char *message )
{
  libspectrum_print_error( error, "WAV: %s", message );
  return error;
}

static libspectrum_error
wav_riff_end( libspectrum_fs_handle *file, uint64_t *end )
{
  libspectrum_byte h[WAV_RIFF_HEADER_SIZE];
  const libspectrum_fs_operations ls_fs = libspectrum_fs;
  int64_t file_size;

  if( ls_fs.length( file, &file_size ) ||
      ls_fs.seek( file, 0, LIBSPECTRUM_FS_START ) ) {
    libspectrum_print_error( LIBSPECTRUM_ERROR_LOGIC,
                            "WAV: failed to query or seek file" );
    return LIBSPECTRUM_ERROR_LOGIC;
  }
  if( ls_fs.read_exact( file, h, sizeof( h ) ) )
    return wav_error( LIBSPECTRUM_ERROR_CORRUPT, "truncated RIFF header" );
  if( memcmp( h, WAV_SIGNATURE_RIFF, WAV_CHUNK_ID_SIZE ) ||
      memcmp( h + WAV_CHUNK_HEADER_SIZE, WAV_SIGNATURE_WAVE,
              WAV_CHUNK_ID_SIZE ) )
    return wav_error( LIBSPECTRUM_ERROR_SIGNATURE, "wrong RIFF/WAVE signature" );
  *end = (uint64_t)libspectrum_read_dword_le( h + WAV_CHUNK_ID_SIZE ) +
         WAV_CHUNK_HEADER_SIZE;
  if( *end < WAV_RIFF_HEADER_SIZE || *end > (uint64_t)file_size )
    return wav_error( LIBSPECTRUM_ERROR_CORRUPT,
                      "RIFF size lies outside file boundaries" );
  return LIBSPECTRUM_ERROR_NONE;
}

static libspectrum_error
wav_extensible_pcm( const libspectrum_byte *format_data, libspectrum_dword size,
                    unsigned bits )
{
  const libspectrum_byte *ptr;
  unsigned extra_size, valid_bits;
  /* KSDATAFORMAT_SUBTYPE_PCM: 00000001-0000-0010-8000-00aa00389b71. */
  static const libspectrum_byte pcm_guid[] = {
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00,
    0x80, 0x00, 0x00, 0xaa, 0x00, 0x38, 0x9b, 0x71
  };

  if( size < WAV_EXTENSIBLE_FORMAT_SIZE )
    return wav_error( LIBSPECTRUM_ERROR_CORRUPT, "short extensible fmt chunk" );
  ptr = format_data + WAV_PCM_FORMAT_SIZE;
  extra_size = libspectrum_read_word( &ptr );
  valid_bits = libspectrum_read_word( &ptr );
  ptr += sizeof( libspectrum_dword ); /* Skip the channel mask. */
  if( extra_size < WAV_EXTENSIBLE_EXTRA_SIZE ||
      extra_size > size - WAV_FORMAT_EX_SIZE || !valid_bits || valid_bits > bits )
    return wav_error( LIBSPECTRUM_ERROR_CORRUPT,
                      "invalid extensible fmt sizes or valid-bit count" );
  /* Only full-width integer PCM; channel masks do not affect averaging. */
  if( valid_bits != bits || memcmp( ptr, pcm_guid, sizeof( pcm_guid ) ) )
    return wav_error( LIBSPECTRUM_ERROR_UNKNOWN,
                      "unsupported extensible encoding or valid-bit count" );
  return LIBSPECTRUM_ERROR_NONE;
}

static libspectrum_error
wav_read_format( libspectrum_fs_handle *file, libspectrum_dword size, struct wav_info *info )
{
  libspectrum_byte h[WAV_EXTENSIBLE_FORMAT_SIZE];
  const libspectrum_byte *ptr = h;
  size_t count = size < sizeof( h ) ? size : sizeof( h );
  unsigned bits, format;
  libspectrum_error error;

  if( size < WAV_PCM_FORMAT_SIZE || libspectrum_fs.read_exact( file, h, count ) )
    return wav_error( LIBSPECTRUM_ERROR_CORRUPT, "short or truncated fmt chunk" );
  format = libspectrum_read_word( &ptr );
  info->channels = libspectrum_read_word( &ptr );
  info->rate = libspectrum_read_dword( &ptr );
  /* Byte rate is redundant; some writers incorrectly store bits per second. */
  ptr += sizeof( libspectrum_dword );
  info->align = libspectrum_read_word( &ptr );
  bits = libspectrum_read_word( &ptr );
  if( format == WAV_FORMAT_EXTENSIBLE ) {
    error = wav_extensible_pcm( h, size, bits );
    if( error ) return error;
  } else if( format != WAV_FORMAT_PCM ) {
    return wav_error( LIBSPECTRUM_ERROR_UNKNOWN, "unsupported audio encoding" );
  }
  if( bits != 8 && bits != 16 && bits != 24 && bits != 32 )
    return wav_error( LIBSPECTRUM_ERROR_UNKNOWN, "unsupported PCM bit depth" );
  info->bytes = bits / 8;
  if( !info->channels || !info->rate ||
      info->align != info->channels * info->bytes )
    return wav_error( LIBSPECTRUM_ERROR_CORRUPT,
                      "invalid channels, sample rate or block alignment" );
  return LIBSPECTRUM_ERROR_NONE;
}

static libspectrum_error
wav_read_chunk_header( libspectrum_fs_handle *file, uint64_t pos, uint64_t end,
                       libspectrum_byte *header, libspectrum_dword *size,
                       uint64_t *next )
{
  const libspectrum_fs_operations ls_fs = libspectrum_fs;
  if( end - pos < WAV_CHUNK_HEADER_SIZE )
    return wav_error( LIBSPECTRUM_ERROR_CORRUPT, "truncated chunk header" );
  if( pos > INT64_MAX || ls_fs.seek( file, (int64_t)pos, LIBSPECTRUM_FS_START ) )
    return wav_error( LIBSPECTRUM_ERROR_LOGIC, "cannot seek to chunk header" );
  if( ls_fs.read_exact( file, header, WAV_CHUNK_HEADER_SIZE ) )
    return wav_error( LIBSPECTRUM_ERROR_CORRUPT, "cannot read chunk header" );
  *size = libspectrum_read_dword_le( header + WAV_CHUNK_ID_SIZE );
  /* RIFF chunks are padded to an even byte boundary. */
  *next = pos + WAV_CHUNK_HEADER_SIZE + (uint64_t)*size + ( *size & 1 );
  if( *next > end )
    return wav_error( LIBSPECTRUM_ERROR_CORRUPT,
                      "chunk size or padding extends beyond RIFF boundary" );
  return LIBSPECTRUM_ERROR_NONE;
}

static libspectrum_error
wav_scan_chunks( libspectrum_fs_handle *file, uint64_t end, struct wav_info *info )
{
  libspectrum_byte h[WAV_CHUNK_HEADER_SIZE];
  libspectrum_dword size;
  libspectrum_error error;
  int have_fmt = 0, have_data = 0;

  for( uint64_t pos = WAV_RIFF_HEADER_SIZE, next; pos < end; pos = next ) {
    error = wav_read_chunk_header( file, pos, end, h, &size, &next );
    if( error ) return error;
    if( !memcmp( h, WAV_CHUNK_FORMAT, WAV_CHUNK_ID_SIZE ) ) {
      if( have_fmt )
        return wav_error( LIBSPECTRUM_ERROR_CORRUPT, "duplicate fmt chunk" );
      error = wav_read_format( file, size, info );
      if( error ) return error;
      have_fmt = 1;
    } else if( !memcmp( h, WAV_CHUNK_DATA, WAV_CHUNK_ID_SIZE ) ) {
      if( have_data )
        return wav_error( LIBSPECTRUM_ERROR_CORRUPT, "duplicate data chunk" );
      if( pos + sizeof( h ) > INT64_MAX )
        return wav_error( LIBSPECTRUM_ERROR_LOGIC, "data offset exceeds seek range" );
      info->data_offset = (int64_t)( pos + sizeof( h ) );
      info->data_size = size;
      have_data = 1;
    }
  }
  if( !have_fmt || !have_data )
    return wav_error( LIBSPECTRUM_ERROR_CORRUPT, "missing fmt or data chunk" );
  return LIBSPECTRUM_ERROR_NONE;
}

static libspectrum_error
wav_prepare_data( libspectrum_fs_handle *file, const struct wav_info *info )
{
  if( info->data_size % info->align )
    return wav_error( LIBSPECTRUM_ERROR_CORRUPT, "data contains a partial frame" );
  /* RIFF sizes are 32-bit; these limits only constrain smaller hosts. */
#if SIZE_MAX <= UINT32_MAX
  if( info->data_size / info->align >
        SIZE_MAX / sizeof( libspectrum_signed_word ) )
    return wav_error( LIBSPECTRUM_ERROR_INVALID,
                      "frame count exceeds host limits" );
#endif
#if SIZE_MAX < UINT32_MAX
  if( info->rate > SIZE_MAX )
    return wav_error( LIBSPECTRUM_ERROR_INVALID,
                      "sample rate exceeds host limits" );
#endif
  if( libspectrum_fs.seek( file, info->data_offset, LIBSPECTRUM_FS_START ) )
    return wav_error( LIBSPECTRUM_ERROR_LOGIC, "cannot seek to audio data" );
  return LIBSPECTRUM_ERROR_NONE;
}

libspectrum_error
libspectrum_wav_reader_open( libspectrum_wav_reader **reader, const char *filename,
                   size_t *sample_rate, size_t *frame_count )
{
  const libspectrum_fs_operations ls_fs = libspectrum_fs;
  libspectrum_fs_handle *file;
  uint64_t end;
  struct wav_info info = { 0 };
  libspectrum_wav_reader *state;
  libspectrum_error error;

  if( !reader ) return LIBSPECTRUM_ERROR_INVALID;
  *reader = NULL;
  if( !filename || !sample_rate || !frame_count )
    return LIBSPECTRUM_ERROR_INVALID;
  error = ls_fs.open( filename, LIBSPECTRUM_FS_READ, &file );
  if( error ) {
    libspectrum_print_error( LIBSPECTRUM_ERROR_LOGIC, "WAV: cannot open %s",
                            filename );
    return LIBSPECTRUM_ERROR_LOGIC;
  }
  error = wav_riff_end( file, &end );
  if( !error ) error = wav_scan_chunks( file, end, &info );
  if( !error ) error = wav_prepare_data( file, &info );
  if( error ) {
    ls_fs.close( file );
    return error;
  }
  state = libspectrum_new( libspectrum_wav_reader, 1 );
  state->file = file;
  state->channels = info.channels;
  state->bytes = info.bytes;
  state->remaining = info.data_size / info.align;
  *sample_rate = info.rate;
  *frame_count = state->remaining;
  *reader = state;
  return LIBSPECTRUM_ERROR_NONE;
}

/* Decode one little-endian sample to signed 16-bit precision. */
static int64_t
wav_decode_sample( const libspectrum_byte *raw, unsigned bytes )
{
  const libspectrum_byte *ptr = raw;
  libspectrum_dword value;
  int64_t sample, divisor;

  /* Centre unsigned 8-bit PCM at zero and scale to signed 16-bit. */
  if( bytes == 1 ) return ( (int)raw[0] - 128 ) * 256;
  if( bytes == 4 ) {
    value = libspectrum_read_dword_le( raw );
  } else {
    value = libspectrum_read_word( &ptr );
    if( bytes == 3 ) value |= (libspectrum_dword)*ptr << 16;
  }
  sample = value;
  /* Sign-extend the most significant byte without signed casts or shifts. */
  if( raw[bytes - 1] & 0x80 ) sample -= (int64_t)1 << ( bytes * 8 );
  if( bytes <= 2 ) return sample;

  /* Arithmetic floor, without implementation-defined signed shifts. */
  divisor = (int64_t)1 << ( ( bytes - 2 ) * 8 );
  return sample >= 0 ? sample / divisor :
         -( ( -sample + divisor - 1 ) / divisor );
}

static libspectrum_error
wav_read_frame( libspectrum_wav_reader *reader,
                libspectrum_signed_word *sample )
{
  libspectrum_byte raw[4];
  int64_t sum = 0;

  for( unsigned c = 0; c < reader->channels; c++ ) {
    if( libspectrum_fs.read_exact( reader->file, raw, reader->bytes ) )
      return wav_error( LIBSPECTRUM_ERROR_CORRUPT, "truncated audio frame" );
    sum += wav_decode_sample( raw, reader->bytes );
  }
  *sample = sum / reader->channels;
  return LIBSPECTRUM_ERROR_NONE;
}

libspectrum_error
libspectrum_wav_reader_read( libspectrum_wav_reader *reader,
                   libspectrum_signed_word *buffer, size_t frames,
                   size_t *frames_read )
{
  libspectrum_error error;

  if( !frames_read ) return LIBSPECTRUM_ERROR_INVALID;
  *frames_read = 0;
  if( !reader || ( frames && !buffer ) ) return LIBSPECTRUM_ERROR_INVALID;
  if( frames > reader->remaining ) frames = reader->remaining;
  for( size_t i = 0; i < frames; i++ ) {
    error = wav_read_frame( reader, buffer + i );
    if( error ) return error;
    reader->remaining--;
    (*frames_read)++;
  }
  return LIBSPECTRUM_ERROR_NONE;
}

libspectrum_error
libspectrum_wav_reader_close( libspectrum_wav_reader *reader )
{
  int failed;
  if( !reader ) return LIBSPECTRUM_ERROR_NONE;
  failed = libspectrum_fs.close( reader->file );
  libspectrum_free( reader );
  return failed ? wav_error( LIBSPECTRUM_ERROR_UNKNOWN,
                             "failed to close input" ) : LIBSPECTRUM_ERROR_NONE;
}

#ifdef WAV_INTERNAL_PCM
libspectrum_error
internal_wav_open( libspectrum_wav_reader **reader, const char *filename,
                   size_t *rate, size_t *count )
{
  return libspectrum_wav_reader_open( reader, filename, rate, count );
}

libspectrum_error
internal_wav_read( libspectrum_wav_reader *reader,
                   libspectrum_signed_word *samples, size_t frames, size_t *got )
{
  return libspectrum_wav_reader_read( reader, samples, frames, got );
}

void
internal_wav_close( libspectrum_wav_reader *reader )
{
  libspectrum_wav_reader_close( reader );
}
#endif

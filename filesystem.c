/* filesystem.c: library-owned synchronous regular-file namespace
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
#include <sys/types.h>
#include <sys/stat.h>
#if defined _WIN32 || defined HAVE_FCNTL_H
#include <fcntl.h>
#endif
#ifdef _WIN32
#include <io.h>
#elif defined HAVE_UNISTD_H
#include <unistd.h>
#endif

#include "internals.h"
#ifdef _WIN32
#include <aclapi.h>
#endif

#define INVALID LIBSPECTRUM_ERROR_INVALID
#define IO_ERROR LIBSPECTRUM_ERROR_UNKNOWN

struct libspectrum_fs_handle {
  void *stream;
};

/* Owner-thread use only. A successful selection or the first filesystem
   operation fixes the implementation for the lifetime of this process. */
static int selected;
static void *context;

static FILE *
native_open_input( const char *name, int update, const char *mode )
{
#ifdef _WIN32
  int fd = _open( name, ( update ? _O_RDWR : _O_RDONLY ) | _O_BINARY );
  FILE *stream = NULL;
  if( fd < 0 ) return NULL;
  {
    HANDLE handle = (HANDLE)_get_osfhandle( fd );
    BY_HANDLE_FILE_INFORMATION info;
    if( GetFileType( handle ) == FILE_TYPE_DISK &&
        GetFileInformationByHandle( handle, &info ) &&
        !( info.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY ) )
      stream = _fdopen( fd, mode );
  }
  if( !stream ) _close( fd );
  return stream;
#elif defined HAVE_UNISTD_H && defined HAVE_FCNTL_H && defined O_NONBLOCK && \
      defined S_ISREG && defined HAVE_OPEN && defined HAVE_FSTAT && \
      defined HAVE_FCNTL && defined HAVE_FDOPEN && defined HAVE_CLOSE
  struct stat info;
  int flags = ( update ? O_RDWR : O_RDONLY ) | O_NONBLOCK;
  int fd;
  FILE *stream = NULL;
#ifdef O_BINARY
  flags |= O_BINARY;
#endif
  fd = open( name, flags );
  if( fd < 0 ) return NULL;

  /* Validate the opened object before enabling blocking I/O. */
  if( !fstat( fd, &info ) && S_ISREG( info.st_mode ) ) {
    flags = fcntl( fd, F_GETFL );
    if( flags >= 0 && !fcntl( fd, F_SETFL, flags & ~O_NONBLOCK ) )
      stream = fdopen( fd, mode );
  }
  if( !stream ) close( fd );
  return stream;
#else
  /* Preserve input support where descriptor validation is unavailable. */
  (void)update;
  return fopen( name, mode );
#endif
}

static libspectrum_error
native_open( void *ctx, const char *name, libspectrum_fs_mode mode, void **out )
{
  static const char *modes[] = { "rb", "wb", "rb+", "ab" };
  (void)ctx;
  if( mode == LIBSPECTRUM_FS_READ || mode == LIBSPECTRUM_FS_READ_WRITE )
    *out = native_open_input( name, mode == LIBSPECTRUM_FS_READ_WRITE, modes[mode] );
  else
    *out = fopen( name, modes[mode] );
  return *out ? LIBSPECTRUM_ERROR_NONE : IO_ERROR;
}

static libspectrum_error
native_read( void *ctx, void *stream, void *buffer, size_t requested,
             size_t *transferred )
{
  (void)ctx;
  *transferred = fread( buffer, 1, requested, stream );
  return ferror( stream ) ? IO_ERROR : LIBSPECTRUM_ERROR_NONE;
}

static libspectrum_error
native_write( void *ctx, void *stream, const void *buffer, size_t requested,
              size_t *transferred )
{
  (void)ctx;
  *transferred = fwrite( buffer, 1, requested, stream );
  return ferror( stream ) ? IO_ERROR : LIBSPECTRUM_ERROR_NONE;
}

static libspectrum_error
native_seek( void *ctx, void *stream, int64_t offset, libspectrum_fs_origin origin )
{
  static const int origins[] = { SEEK_SET, SEEK_CUR, SEEK_END };
  (void)ctx;
#ifdef _WIN32
  return _fseeki64( stream, offset, origins[origin] ) ? IO_ERROR : 0;
#elif defined HAVE_FSEEKO
  if( (int64_t)(off_t)offset != offset ) return INVALID;
  return fseeko( stream, (off_t)offset, origins[origin] ) ? IO_ERROR : 0;
#else
  if( offset < LONG_MIN || offset > LONG_MAX ) return INVALID;
  return fseek( stream, (long)offset, origins[origin] ) ? IO_ERROR : 0;
#endif
}

static libspectrum_error
native_tell( void *ctx, void *stream, int64_t *position )
{
  (void)ctx;
#ifdef _WIN32
  *position = _ftelli64( stream );
#elif defined HAVE_FTELLO
  {
    off_t native_position = ftello( stream );
    if( native_position < 0 || (uintmax_t)native_position > INT64_MAX )
      return IO_ERROR;
    *position = (int64_t)native_position;
  }
#else
  *position = ftell( stream );
#endif
  return *position < 0 ? IO_ERROR : LIBSPECTRUM_ERROR_NONE;
}

static libspectrum_error
native_length( void *ctx, void *stream, int64_t *length )
{
  int64_t saved, end;
  libspectrum_error error, restore_error;

  /* Seek observes buffered output too. Always restore position, including
     after a failed end query. A failed restoration is never success. */
  error = native_tell( ctx, stream, &saved );
  if( error ) return error;
  error = native_seek( ctx, stream, 0, LIBSPECTRUM_FS_END );
  if( !error ) error = native_tell( ctx, stream, &end );
  restore_error = native_seek( ctx, stream, saved, LIBSPECTRUM_FS_START );
  if( error || restore_error ) return error ? error : restore_error;
  *length = end;
  return LIBSPECTRUM_ERROR_NONE;
}

static libspectrum_error
native_flush( void *ctx, void *stream )
{
  (void)ctx;
  return fflush( stream ) ? IO_ERROR : LIBSPECTRUM_ERROR_NONE;
}

static libspectrum_error
native_close( void *ctx, void *stream )
{
  (void)ctx;
  return fclose( stream ) ? IO_ERROR : LIBSPECTRUM_ERROR_NONE;
}

static libspectrum_error
native_exists( void *ctx, const char *name, int *exists )
{
  (void)ctx;
#ifdef _WIN32
  DWORD status;
  /* Avoid CRT stat variants whose size field cannot represent large files. */
  if( GetFileAttributesA( name ) != INVALID_FILE_ATTRIBUTES ) {
    *exists = 1;
    return LIBSPECTRUM_ERROR_NONE;
  }
  status = GetLastError();
  return status == ERROR_FILE_NOT_FOUND || status == ERROR_PATH_NOT_FOUND ?
    LIBSPECTRUM_ERROR_NONE : IO_ERROR;
#else
  struct stat info;
  if( !stat( name, &info ) ) {
    *exists = 1;
    return LIBSPECTRUM_ERROR_NONE;
  }
  return errno == ENOENT || errno == ENOTDIR ? LIBSPECTRUM_ERROR_NONE : IO_ERROR;
#endif
}

static libspectrum_error
native_unlink( void *ctx, const char *name )
{
  (void)ctx;
  return remove( name ) ? IO_ERROR : LIBSPECTRUM_ERROR_NONE;
}

#ifdef _WIN32
static libspectrum_error
windows_writable( const char *name, int *writable )
{
  DWORD attributes, status, desired = GENERIC_WRITE, granted = 0;
  DWORD privilege_size = sizeof( PRIVILEGE_SET );
  HANDLE token = NULL, impersonation = NULL;
  PSECURITY_DESCRIPTOR descriptor = NULL;
  PRIVILEGE_SET local_privileges;
  PPRIVILEGE_SET privileges = &local_privileges;
  GENERIC_MAPPING mapping = {
    FILE_GENERIC_READ, FILE_GENERIC_WRITE, FILE_GENERIC_EXECUTE, FILE_ALL_ACCESS
  };
  BOOL allowed = FALSE, checked;
  libspectrum_error error = IO_ERROR;

  *writable = 0;
  /* Match fopen's narrow Windows pathname convention. This queries metadata,
     not a write-open, and therefore cannot truncate or create the file. */
  attributes = GetFileAttributesA( name );
  if( attributes == INVALID_FILE_ATTRIBUTES ) return IO_ERROR;
  if( attributes & FILE_ATTRIBUTE_DIRECTORY ) return INVALID;
  if( attributes & FILE_ATTRIBUTE_READONLY ) return LIBSPECTRUM_ERROR_NONE;

  status = GetNamedSecurityInfoA( name, SE_FILE_OBJECT,
    OWNER_SECURITY_INFORMATION | GROUP_SECURITY_INFORMATION |
    DACL_SECURITY_INFORMATION, NULL, NULL, NULL, NULL, &descriptor );
  if( status != ERROR_SUCCESS ) {
    SetLastError( status );
    return IO_ERROR;
  }

  /* Respect a caller's impersonation identity. Only fall back to the process
     identity when the thread has no token, never on a token-query failure. */
  if( !OpenThreadToken( GetCurrentThread(), TOKEN_QUERY | TOKEN_DUPLICATE,
                       TRUE, &token ) ) {
    if( GetLastError() != ERROR_NO_TOKEN ) goto cleanup;
    if( !OpenProcessToken( GetCurrentProcess(), TOKEN_QUERY | TOKEN_DUPLICATE,
                          &token ) ) goto cleanup;
  }
  /* AccessCheck requires an impersonation token, not a primary process token. */
  if( !DuplicateToken( token, SecurityImpersonation, &impersonation ) )
    goto cleanup;
  MapGenericMask( &desired, &mapping );
  checked = AccessCheck( descriptor, impersonation, desired, &mapping,
                         privileges, &privilege_size, &granted, &allowed );
  if( !checked && GetLastError() == ERROR_INSUFFICIENT_BUFFER ) {
    privileges = libspectrum_malloc( privilege_size );
    if( !privileges ) {
      SetLastError( ERROR_NOT_ENOUGH_MEMORY );
      error = LIBSPECTRUM_ERROR_MEMORY;
      goto cleanup;
    }
    checked = AccessCheck( descriptor, impersonation, desired, &mapping,
                           privileges, &privilege_size, &granted, &allowed );
  }
  if( !checked ) goto cleanup;
  *writable = allowed != FALSE;
  error = LIBSPECTRUM_ERROR_NONE; /* A completed ACL denial is successful false. */

cleanup:
  status = GetLastError();
  if( privileges != &local_privileges ) libspectrum_free( privileges );
  if( impersonation ) CloseHandle( impersonation );
  if( token ) CloseHandle( token );
  LocalFree( descriptor );
  if( error ) SetLastError( status );
  return error;
}
#endif

static libspectrum_error
native_writable( void *ctx, const char *name, int *writable )
{
  (void)ctx;
#ifdef _WIN32
  return windows_writable( name, writable );
#else
  struct stat info;
  if( stat( name, &info ) ) return IO_ERROR;
#ifdef HAVE_ACCESS
  *writable = !access( name, W_OK );
#else
  (void)writable;
  return INVALID;
#endif
  if( *writable || errno == EACCES || errno == EROFS )
    return LIBSPECTRUM_ERROR_NONE;
  return IO_ERROR;
#endif
}

#define NATIVE_BACKEND { \
  native_open, native_read, native_write, native_length, native_seek, \
  native_tell, native_flush, native_close, native_exists, native_unlink, \
  native_writable \
}
static libspectrum_fs_backend backend = NATIVE_BACKEND;

libspectrum_error
libspectrum_fs_install( const libspectrum_fs_backend *implementation, void *userdata )
{
  if( selected || !implementation || !implementation->open ||
      !implementation->read || !implementation->write || !implementation->length ||
      !implementation->seek || !implementation->tell || !implementation->flush ||
      !implementation->close ) return INVALID;

  backend = *implementation;
  context = userdata;
  selected = 1;
  return LIBSPECTRUM_ERROR_NONE;
}

static libspectrum_error
fs_open( const char *name, libspectrum_fs_mode mode, libspectrum_fs_handle **out )
{
  libspectrum_error error;
  void *stream = NULL;
  libspectrum_fs_handle *handle;

  selected = 1;
  if( !out ) return INVALID;
  *out = NULL;
  if( !name || mode < LIBSPECTRUM_FS_READ || mode > LIBSPECTRUM_FS_APPEND )
    return INVALID;
  handle = libspectrum_new( libspectrum_fs_handle, 1 );
  if( !handle ) return LIBSPECTRUM_ERROR_MEMORY;

  error = backend.open( context, name, mode, &stream );
  if( error || !stream ) {
    if( stream ) backend.close( context, stream );
    libspectrum_free( handle );
    return error ? error : IO_ERROR;
  }
  handle->stream = stream;
  *out = handle;
  return LIBSPECTRUM_ERROR_NONE;
}

static libspectrum_error
fs_close( libspectrum_fs_handle *handle )
{
  libspectrum_error error;
  selected = 1;
  if( !handle ) return INVALID;
  error = backend.close( context, handle->stream );
  libspectrum_free( handle );
  return error;
}

static libspectrum_error
fs_read( libspectrum_fs_handle *handle, void *buffer, size_t requested,
         size_t *transferred )
{
  libspectrum_error error;
  selected = 1;
  if( !transferred ) return INVALID;
  *transferred = 0;
  if( !handle || ( requested && !buffer ) ) return INVALID;
  error = backend.read( context, handle->stream, buffer, requested, transferred );
  if( *transferred > requested ) {
    *transferred = 0;
    return INVALID;
  }
  return error;
}

static libspectrum_error
fs_write( libspectrum_fs_handle *handle, const void *buffer, size_t requested,
          size_t *transferred )
{
  libspectrum_error error;
  selected = 1;
  if( !transferred ) return INVALID;
  *transferred = 0;
  if( !handle || ( requested && !buffer ) ) return INVALID;
  error = backend.write( context, handle->stream, buffer, requested, transferred );
  if( *transferred > requested ) {
    *transferred = 0;
    return INVALID;
  }
  return error;
}

static libspectrum_error
fs_read_exact( libspectrum_fs_handle *handle, void *buffer, size_t length )
{
  unsigned char *ptr = buffer;
  size_t transferred;
  libspectrum_error error;

  selected = 1;
  if( !handle || ( length && !buffer ) ) return INVALID;
  while( length ) {
    error = fs_read( handle, ptr, length, &transferred );
    if( error ) return error;
    if( !transferred ) return IO_ERROR;
    ptr += transferred;
    length -= transferred;
  }
  return LIBSPECTRUM_ERROR_NONE;
}

static libspectrum_error
fs_write_exact( libspectrum_fs_handle *handle, const void *buffer, size_t length )
{
  const unsigned char *ptr = buffer;
  size_t transferred;
  libspectrum_error error;

  selected = 1;
  if( !handle || ( length && !buffer ) ) return INVALID;
  while( length ) {
    error = fs_write( handle, ptr, length, &transferred );
    if( error ) return error;
    if( !transferred ) return IO_ERROR;
    ptr += transferred;
    length -= transferred;
  }
  return LIBSPECTRUM_ERROR_NONE;
}

/* The two numeric queries share failure-output validation. */
#define QUERY(name) \
static libspectrum_error \
fs_##name( libspectrum_fs_handle *handle, int64_t *value ) \
{ \
  libspectrum_error error; \
  selected = 1; \
  if( !value ) return INVALID; \
  *value = -1; \
  if( !handle ) return INVALID; \
  error = backend.name( context, handle->stream, value ); \
  if( error || *value < 0 ) { \
    *value = -1; \
    return error ? error : IO_ERROR; \
  } \
  return LIBSPECTRUM_ERROR_NONE; \
}
QUERY(length)
QUERY(tell)

static libspectrum_error
fs_seek( libspectrum_fs_handle *handle, int64_t offset, libspectrum_fs_origin origin )
{
  libspectrum_error error;
  selected = 1;
  if( !handle || origin < LIBSPECTRUM_FS_START || origin > LIBSPECTRUM_FS_END ||
      ( origin == LIBSPECTRUM_FS_START && offset < 0 ) ) return INVALID;
  error = backend.seek( context, handle->stream, offset, origin );
  return error;
}

static libspectrum_error
fs_flush( libspectrum_fs_handle *handle )
{
  libspectrum_error error;
  selected = 1;
  if( !handle ) return INVALID;
  error = backend.flush( context, handle->stream );
  return error;
}

#define PROBE(name) \
static libspectrum_error \
fs_##name( const char *filename, int *value ) \
{ \
  libspectrum_error error; \
  selected = 1; \
  if( !value ) return INVALID; \
  *value = 0; \
  if( !filename || !backend.name ) return INVALID; \
  error = backend.name( context, filename, value ); \
  *value = error ? 0 : !!*value; \
  return error; \
}
PROBE(exists)
PROBE(writable)

static libspectrum_error
fs_unlink( const char *name )
{
  libspectrum_error error;
  selected = 1;
  if( !name || !backend.unlink ) return INVALID;
  error = backend.unlink( context, name );
  return error;
}

const libspectrum_fs_operations libspectrum_fs = {
  fs_open, fs_read, fs_write, fs_read_exact, fs_write_exact,
  fs_length, fs_seek, fs_tell, fs_flush, fs_close,
  fs_exists, fs_unlink, fs_writable
};

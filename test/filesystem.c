/* Filesystem contract and codec routing regression tests.
   Copyright (c) 2026 Fredrick Meunier

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
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "internals.h"
#ifdef _WIN32
#include <aclapi.h>
#include <direct.h>
#elif defined HAVE_UNISTD_H
#include <unistd.h>
#include <sys/stat.h>
#ifdef HAVE_FCNTL_H
#include <fcntl.h>
#if defined O_NONBLOCK && defined S_ISREG && defined HAVE_OPEN && \
    defined HAVE_FSTAT && defined HAVE_FCNTL && defined HAVE_FDOPEN && \
    defined HAVE_CLOSE
#define TEST_NATIVE_VALIDATION 1
#endif
#endif
#if defined TEST_NATIVE_VALIDATION && defined HAVE_MKFIFO && \
    defined HAVE_FORK && defined HAVE_ALARM && defined HAVE_WAITPID
#include <signal.h>
#include <sys/wait.h>
#define TEST_NATIVE_FIFO 1
#endif
#endif
/* Separate executable: each scenario starts with a fresh filesystem selection. */

/* Deliberately not FILEs. The backend needs no global state. */
typedef struct ram {
  unsigned char bytes[8192];
  size_t length, quantum;
  unsigned opens, closes, reads, writes, flushes;
  int present, writable, stall, invalid_count, transfer_error;
  int close_error, flush_error, open_error, null_open, probe_error, callback_checked;
  int query_error, invalid_query;
  int64_t last_seek;
  libspectrum_fs_mode last_mode;
} ram;
typedef struct ram_handle { ram *owner; int64_t pos; libspectrum_fs_mode mode; } ram_handle;
#define IO LIBSPECTRUM_ERROR_UNKNOWN
#define REQUIRE(x) do { if(!(x)) { fprintf(stderr, "filesystem:%d: %s\n", __LINE__, #x); goto fail; } } while(0)
/* Even an unexpected close result consumes the handle before test cleanup. */
#define CHECK_CLOSE(fn, h, expected_error) do { \
  libspectrum_error close_error = fn(h); h = NULL; \
  REQUIRE(!!close_error == !!(expected_error)); \
} while(0)

static const libspectrum_fs_backend ram_backend;

static libspectrum_error
ram_open(void *ctx, const char *name, libspectrum_fs_mode mode, void **out)
{
  ram *r = ctx;
  ram_handle *h;
  *out = NULL;
  r->callback_checked = libspectrum_fs_install(&ram_backend, r) == LIBSPECTRUM_ERROR_INVALID;
  if(strcmp(name, "ram") || (!r->present &&
     (mode == LIBSPECTRUM_FS_READ || mode == LIBSPECTRUM_FS_READ_WRITE))) return IO;
  if(r->null_open) return 0;
  if(mode == LIBSPECTRUM_FS_WRITE) r->length = 0;
  r->present = 1;
  h = malloc(sizeof(*h));
  if(!h) return LIBSPECTRUM_ERROR_MEMORY;
  h->owner = r; h->mode = mode;
  h->pos = mode == LIBSPECTRUM_FS_APPEND ? (int64_t)r->length : 0;
  *out = h; r->opens++; r->last_mode = mode;
  /* Even erroneous opens returning a handle must be cleaned up. */
  return r->open_error ? IO : 0;
}
static libspectrum_error
ram_read(void *ctx, void *handle, void *p, size_t n, size_t *k)
{
  ram *r = ctx; ram_handle *h = handle;
  r->reads++; *k = 0;
  if(h->owner != r || h->mode == LIBSPECTRUM_FS_WRITE || h->mode == LIBSPECTRUM_FS_APPEND) return IO;
  if(r->invalid_count) { *k = n + 1; return 0; }
  if(r->stall) return 0;
  if(h->pos >= (int64_t)r->length) return 0;
  if(n > r->length - (size_t)h->pos) n = r->length - (size_t)h->pos;
  if(r->quantum && n > r->quantum) n = r->quantum;
  memcpy(p, r->bytes + h->pos, n); h->pos += n; *k = n;
  return r->transfer_error ? IO : 0;
}
static libspectrum_error
ram_write(void *ctx, void *handle, const void *p, size_t n, size_t *k)
{
  ram *r = ctx; ram_handle *h = handle;
  r->writes++; *k = 0;
  if(h->owner != r || h->mode == LIBSPECTRUM_FS_READ) return IO;
  if(r->invalid_count) { *k = n + 1; return IO; }
  if(r->stall) return 0;
  if(h->mode == LIBSPECTRUM_FS_APPEND) h->pos = r->length;
  if(h->pos > (int64_t)sizeof(r->bytes)) return IO;
  if(n > sizeof(r->bytes) - (size_t)h->pos) n = sizeof(r->bytes) - (size_t)h->pos;
  if(r->quantum && n > r->quantum) n = r->quantum;
  memcpy(r->bytes + h->pos, p, n); h->pos += n; *k = n;
  if(h->pos > (int64_t)r->length) r->length = h->pos;
  return r->transfer_error ? IO : 0;
}
static libspectrum_error ram_length(void *ctx, void *h, int64_t *n)
{ ram *r = ctx; (void)h; *n = r->invalid_query ? -1 : (int64_t)r->length; return r->query_error ? IO : 0; }
static libspectrum_error ram_tell(void *ctx, void *h, int64_t *n)
{ ram *r = ctx; *n = r->invalid_query ? -1 : ((ram_handle *)h)->pos; return r->query_error ? IO : 0; }
static libspectrum_error ram_seek(void *ctx, void *handle, int64_t n, libspectrum_fs_origin o)
{
  ram_handle *h = handle;
  int64_t base = o == LIBSPECTRUM_FS_START ? 0 :
                o == LIBSPECTRUM_FS_CURRENT ? h->pos : (int64_t)((ram *)ctx)->length;
  if((n > 0 && base > INT64_MAX - n) || (n < 0 && n < -base)) return IO;
  h->pos = base + n; ((ram *)ctx)->last_seek = h->pos; return 0;
}
static libspectrum_error ram_flush(void *ctx, void *h)
{ ram *r = ctx; (void)h; r->flushes++; return r->flush_error ? IO : 0; }
static libspectrum_error ram_close(void *ctx, void *h)
{ ram *r = ctx; r->closes++; r->callback_checked = libspectrum_fs_install(&ram_backend, r) == LIBSPECTRUM_ERROR_INVALID; free(h); return r->close_error ? IO : 0; }
static libspectrum_error ram_exists(void *ctx, const char *name, int *p)
{ ram *r = ctx; r->callback_checked = libspectrum_fs_install(&ram_backend, r) == LIBSPECTRUM_ERROR_INVALID; *p = !strcmp(name, "ram") && r->present; return r->probe_error ? IO : 0; }
static libspectrum_error ram_unlink(void *ctx, const char *name)
{ ram *r = ctx; if(strcmp(name, "ram") || !r->present) return IO; r->present = 0; return 0; }
static libspectrum_error ram_writable(void *ctx, const char *name, int *p)
{ ram *r = ctx; *p = r->writable; return strcmp(name, "ram") || !r->present ? IO : 0; }
static const libspectrum_fs_backend ram_backend = {
  ram_open, ram_read, ram_write, ram_length, ram_seek, ram_tell,
  ram_flush, ram_close, ram_exists, ram_unlink, ram_writable
};

static int
filesystem_contract(void)
{
  const libspectrum_fs_operations ls_fs = libspectrum_fs;
  ram r = { .quantum = 2, .writable = 1 };
  libspectrum_fs_backend b = ram_backend;
  libspectrum_fs_handle *h = NULL;
  char buf[8]; size_t n; int flag; int64_t pos;
  unsigned calls;

  b.open = NULL;
  REQUIRE(libspectrum_fs_install(&b, &r));
  b = ram_backend;
  REQUIRE(!libspectrum_fs_install(&b, &r));
  REQUIRE(libspectrum_fs_install(&b, &r)); /* no second selection, even before IO */
  memset(&b, 0, sizeof(b)); /* Verify callbacks were copied from the caller's table. */
  REQUIRE(libspectrum_fs_install(&b, NULL));
  REQUIRE(!ls_fs.open("ram", LIBSPECTRUM_FS_WRITE, &h));
  REQUIRE(r.callback_checked && r.last_mode == LIBSPECTRUM_FS_WRITE);
  REQUIRE(libspectrum_fs_install(&ram_backend, &r));
  libspectrum_end();
  REQUIRE(!libspectrum_init());
  REQUIRE(libspectrum_fs_install(&ram_backend, &r)); /* init/end do not change selection */
  REQUIRE(!ls_fs.write_exact(h, "abcdef", 6) && r.writes == 3);
  REQUIRE(!ls_fs.length(h, &pos) && pos == 6);
  REQUIRE(!ls_fs.tell(h, &pos) && pos == 6);
  r.query_error = 1;
  REQUIRE(ls_fs.length(h, &pos) && pos == -1);
  REQUIRE(ls_fs.tell(h, &pos) && pos == -1);
  r.query_error = 0; r.invalid_query = 1;
  REQUIRE(ls_fs.length(h, &pos) && pos == -1);
  REQUIRE(ls_fs.tell(h, &pos) && pos == -1); r.invalid_query = 0;
  REQUIRE(!ls_fs.flush(h) && r.flushes == 1);
  CHECK_CLOSE(ls_fs.close, h, 0);
  REQUIRE(!ls_fs.open("ram", LIBSPECTRUM_FS_READ_WRITE, &h));
  REQUIRE(!ls_fs.read(h, buf, 6, &n) && n == 2);
  REQUIRE(!ls_fs.seek(h, -1, LIBSPECTRUM_FS_CURRENT));
  REQUIRE(!ls_fs.read_exact(h, buf, 5) && !memcmp(buf, "bcdef", 5));
  REQUIRE(!ls_fs.read(h, buf, 1, &n) && !n); /* EOF */
  REQUIRE(ls_fs.read_exact(h, buf, 1));
  REQUIRE(!ls_fs.seek(h, -6, LIBSPECTRUM_FS_END));
  r.stall = 1; REQUIRE(ls_fs.read_exact(h, buf, 1) && ls_fs.write_exact(h, buf, 1));
  r.stall = 0; r.invalid_count = 1;
  n = 99; REQUIRE(ls_fs.read(h, buf, 1, &n) && !n);
  n = 99; REQUIRE(ls_fs.write(h, buf, 1, &n) && !n);
  r.invalid_count = 0; r.transfer_error = 1;
  REQUIRE(ls_fs.read(h, buf, 4, &n) && n == 2);
  REQUIRE(!ls_fs.seek(h, 0, LIBSPECTRUM_FS_START));
  calls = r.reads; REQUIRE(ls_fs.read_exact(h, buf, 4) && r.reads == calls + 1);
  calls = r.writes; REQUIRE(ls_fs.write_exact(h, "1234", 4) && r.writes == calls + 1);
  r.transfer_error = 0;
  REQUIRE(ls_fs.seek(h, -1, LIBSPECTRUM_FS_START));
  REQUIRE(ls_fs.seek(h, 0, (libspectrum_fs_origin)99));
  REQUIRE(!ls_fs.seek(h, INT64_C(1) << 40, LIBSPECTRUM_FS_START));
  REQUIRE(!ls_fs.tell(h, &pos) && pos == (INT64_C(1) << 40));
  r.flush_error = 1; REQUIRE(ls_fs.flush(h)); r.flush_error = 0;
  r.close_error = 1; CHECK_CLOSE(ls_fs.close, h, 1); r.close_error = 0;
  REQUIRE(libspectrum_fs_install(&ram_backend, &r)); /* closing every handle does not unlock it */
  REQUIRE(!ls_fs.open("ram", LIBSPECTRUM_FS_APPEND, &h));
  REQUIRE(!ls_fs.write_exact(h, "!", 1));
  CHECK_CLOSE(ls_fs.close, h, 0);
  REQUIRE(r.length == 7 && r.bytes[6] == '!');
  REQUIRE(!ls_fs.exists("ram", &flag) && flag && r.callback_checked);
  REQUIRE(!ls_fs.writable("ram", &flag) && flag);
  r.writable = 0; REQUIRE(!ls_fs.writable("ram", &flag) && !flag);
  r.probe_error = 1; REQUIRE(ls_fs.exists("ram", &flag) && !flag); r.probe_error = 0;

  REQUIRE(ls_fs.open(SRCDIR "/test/file.hdf", LIBSPECTRUM_FS_READ, &h) && !h);
  REQUIRE(!ls_fs.exists(SRCDIR "/test/file.hdf", &flag) && !flag);
  r.open_error = 1;
  REQUIRE(ls_fs.open("ram", LIBSPECTRUM_FS_READ, &h) && !h);
  r.open_error = 0; REQUIRE(r.callback_checked);
  r.null_open = 1;
  REQUIRE(ls_fs.open("ram", LIBSPECTRUM_FS_READ, &h) && !h); r.null_open = 0;
  REQUIRE(!ls_fs.unlink("ram"));
  REQUIRE(!ls_fs.exists("ram", &flag) && !flag);
  REQUIRE(r.opens == r.closes);
  REQUIRE(libspectrum_fs_install(&ram_backend, &r));
  return 0;
fail:
  if(h) ls_fs.close(h);
  return 1;
}

/* Test setup only: seed RAM without using/selecting library IO. */
static int fixture(ram *r, const char *path)
{
  FILE *file = fopen(path, "rb");
  int error;
  if(!file) return 1;
  r->length = fread(r->bytes, 1, sizeof(r->bytes), file);
  error = ferror(file) || !feof(file);
  if(fclose(file)) error = 1;
  if(!error) r->present = 1;
  return error;
}

static libspectrum_byte
mmc_command( libspectrum_mmc_card *card, libspectrum_byte command,
             libspectrum_dword argument )
{
  int shift;
  libspectrum_mmc_write(card, 0x40 | command);
  for(shift = 24; shift >= 0; shift -= 8)
    libspectrum_mmc_write(card, (argument >> shift) & 0xff);
  libspectrum_mmc_write(card, 0xff);
  return libspectrum_mmc_read(card);
}

static int
filesystem_codecs(void)
{
  ram r = { .quantum = 3 };
  libspectrum_file *file = libspectrum_file_alloc(), *native_file = NULL;
  libspectrum_ide_drive drive = { 0 };
  GHashTable *cache = g_hash_table_new(g_int_hash, g_int_equal);
  libspectrum_mmc_card *mmc = NULL;
  libspectrum_wav_writer *writer = NULL;
  libspectrum_wav_reader *reader = NULL;
  libspectrum_signed_word samples[] = { -32768, -1, 0, 1, 32767 }, output[5];
  unsigned char sector[512], layout[54];
  libspectrum_tape *tape = NULL;
  size_t rate, frames, count, i;
  unsigned before;

#ifdef HAVE_ZLIB_H
  native_file = libspectrum_file_alloc();
  REQUIRE(!fixture(&r, SRCDIR "/test/file.tzx.gz"));
  REQUIRE(!libspectrum_file_read_data(native_file, "ram", r.bytes, r.length));
#endif
  REQUIRE(!libspectrum_fs_install(&ram_backend, &r));
#ifdef HAVE_ZLIB_H
  REQUIRE(!libspectrum_file_open(file, "ram"));
  REQUIRE(libspectrum_file_type(file) == LIBSPECTRUM_ID_TAPE_TZX);
  REQUIRE(!strcmp(libspectrum_file_name(file), "ram"));
  REQUIRE(libspectrum_file_class(file) == libspectrum_file_class(native_file));
  REQUIRE(libspectrum_file_get_storage(file) == libspectrum_file_get_storage(native_file));
  REQUIRE(libspectrum_file_size(file) == libspectrum_file_size(native_file));
  REQUIRE(!memcmp(libspectrum_file_data(file), libspectrum_file_data(native_file),
                  libspectrum_file_size(file)));
  libspectrum_file_free(native_file); native_file = NULL;
  REQUIRE(libspectrum_file_open(file, SRCDIR "/test/file.tzx.gz"));
  REQUIRE(libspectrum_fs_install(&ram_backend, &r));
#endif
  /* Build a small HDF with full header, geometry and sector backing. */
  memset(r.bytes, 0, sizeof(r.bytes));
  memcpy(r.bytes, "RS-IDE\032", 7);
  r.bytes[9] = 128; /* little-endian HDF data start */
  r.bytes[24] = 1; r.bytes[28] = 1; r.bytes[35] = 4; /* 1*1*1024 sectors */
  r.length = sizeof(r.bytes); r.present = 1;
  REQUIRE(!libspectrum_file_open(file, "ram"));
  REQUIRE(libspectrum_file_get_storage(file) == LIBSPECTRUM_FILE_STORAGE_PATH);
  REQUIRE(!libspectrum_ide_insert_into_drive(&drive, "ram"));
  REQUIRE(r.last_mode == LIBSPECTRUM_FS_READ_WRITE);
  REQUIRE(libspectrum_fs_install(&ram_backend, &r));
  memset(sector, 0xa5, sizeof(sector));
  libspectrum_ide_write_sector_to_hdf(&drive, cache, 1, sector);
  memset(sector, 0, sizeof(sector));
  REQUIRE(!libspectrum_ide_read_sector_from_hdf(&drive, cache, 1, sector) && sector[0] == 0xa5);
  r.stall = 1;
  libspectrum_ide_commit_drive(&drive, cache);
  REQUIRE(g_hash_table_size(cache) == 1); /* Failed writes retain their cache entries. */
  r.stall = 0;
  libspectrum_ide_commit_drive(&drive, cache);
  REQUIRE(!g_hash_table_size(cache));
  memset(sector, 0, sizeof(sector));
  REQUIRE(!libspectrum_ide_read_sector_from_hdf(&drive, cache, 1, sector) && sector[511] == 0xa5);
  REQUIRE(libspectrum_ide_read_sector_from_hdf(&drive, cache, UINT32_C(0x80000000), sector));
  REQUIRE(r.last_seek == INT64_C(128) + INT64_C(512) * UINT32_C(0x80000000));
  r.close_error = 1;
  REQUIRE(!libspectrum_ide_eject_from_drive(&drive, cache)); r.close_error = 0;
  REQUIRE(!drive.disk);
  before = r.closes; r.bytes[0] = 0;
  REQUIRE(libspectrum_ide_insert_into_drive(&drive, "ram") && r.closes == before + 1 && !drive.disk);
  r.bytes[0] = 'R'; r.transfer_error = 1; before = r.closes;
  REQUIRE(libspectrum_ide_insert_into_drive(&drive, "ram") && r.closes == before + 1);
  r.transfer_error = 0;
  mmc = libspectrum_mmc_alloc();
  before = r.closes; r.bytes[35] = 0;
  REQUIRE(libspectrum_mmc_insert(mmc, "ram") && r.closes == before + 1);
  r.bytes[35] = 4;
  REQUIRE(!libspectrum_mmc_insert(mmc, "ram"));
  REQUIRE(libspectrum_fs_install(&ram_backend, &r));
  REQUIRE(mmc_command(mmc, 8, 0x1aa) == 1);
  for(i = 0; i < 4; i++) libspectrum_mmc_read(mmc);
  REQUIRE(mmc_command(mmc, 55, 0) == 1);
  REQUIRE(!mmc_command(mmc, 41, 0x40000000));
  REQUIRE(!mmc_command(mmc, 17, 1));
  REQUIRE(libspectrum_mmc_read(mmc) == 0xfe);
  for(i = 0; i < 512; i++) REQUIRE(libspectrum_mmc_read(mmc) == 0xa5);
  libspectrum_mmc_read(mmc); libspectrum_mmc_read(mmc); /* CRC */
  REQUIRE(!mmc_command(mmc, 24, 1));
  libspectrum_mmc_write(mmc, 0xfe);
  for(i = 0; i < 512; i++) libspectrum_mmc_write(mmc, 0x3c);
  libspectrum_mmc_write(mmc, 0); libspectrum_mmc_write(mmc, 0);
  REQUIRE(libspectrum_mmc_read(mmc) == 0x05);
  libspectrum_mmc_read(mmc);
  libspectrum_mmc_commit(mmc);
  REQUIRE(r.bytes[128 + 512] == 0x3c && r.bytes[128 + 1023] == 0x3c);
  libspectrum_mmc_eject(mmc);
  libspectrum_mmc_free(mmc); mmc = NULL;
  /* Packed 256-byte sectors, non-default HDF data offset. */
  r.bytes[8] = 1; r.bytes[9] = 0; r.bytes[10] = 2;
  REQUIRE(!libspectrum_ide_insert_into_drive(&drive, "ram"));
  for(i = 0; i < sizeof(sector); i++) sector[i] = (unsigned char)i;
  libspectrum_ide_write_sector_to_hdf(&drive, cache, 2, sector);
  libspectrum_ide_commit_drive(&drive, cache);
  REQUIRE(r.bytes[512 + 512 + 1] == 2);
  REQUIRE(!libspectrum_ide_read_sector_from_hdf(&drive, cache, 2, sector));
  REQUIRE(sector[0] == 0 && sector[1] == 0xff && sector[2] == 2 && sector[511] == 0xff);
  REQUIRE(!libspectrum_ide_eject_from_drive(&drive, cache));

  REQUIRE(!libspectrum_wav_writer_open(&writer, "ram", 44100,
                                      LIBSPECTRUM_WAV_SAMPLE_SIGNED_16, 5));
  REQUIRE(libspectrum_fs_install(&ram_backend, &r));
  REQUIRE(!libspectrum_wav_writer_write(writer, samples, 5));
  CHECK_CLOSE(libspectrum_wav_writer_close, writer, 0);
  REQUIRE(!libspectrum_wav_reader_open(&reader, "ram", &rate, &frames));
  REQUIRE(rate == 44100 && frames == 5);
  REQUIRE(!libspectrum_wav_reader_read(reader, output, 5, &count) && count == 5);
  REQUIRE(!memcmp(samples, output, sizeof(samples)));
  CHECK_CLOSE(libspectrum_wav_reader_close, reader, 0);
  /* data before fmt: scanner must seek backwards after finding the format. */
  memcpy(layout, r.bytes, sizeof(layout));
  memcpy(r.bytes + 12, layout + 36, 18);
  memcpy(r.bytes + 30, layout + 12, 24);
  REQUIRE(!libspectrum_wav_reader_open(&reader, "ram", &rate, &frames));
  REQUIRE(!libspectrum_wav_reader_read(reader, output, 5, &count) && count == 5);
  REQUIRE(!memcmp(samples, output, sizeof(samples)));
  r.close_error = 1;
  CHECK_CLOSE(libspectrum_wav_reader_close, reader, 1); r.close_error = 0;
  r.transfer_error = 1; before = r.closes;
  REQUIRE(libspectrum_wav_reader_open(&reader, "ram", &rate, &frames));
  REQUIRE(!reader && r.closes == before + 1); r.transfer_error = 0;
  tape = libspectrum_tape_alloc();
  REQUIRE(!libspectrum_wav_read(tape, "ram"));
  libspectrum_tape_free(tape); tape = NULL;
  REQUIRE(!libspectrum_wav_writer_open(&writer, "ram", 8000,
                                      LIBSPECTRUM_WAV_SAMPLE_UNSIGNED_8, 0));
  r.flush_error = 1;
  CHECK_CLOSE(libspectrum_wav_writer_close, writer, 1); r.flush_error = 0;
  REQUIRE(!libspectrum_wav_writer_open(&writer, "ram", 8000,
                                      LIBSPECTRUM_WAV_SAMPLE_UNSIGNED_8, 0));
  r.close_error = 1;
  CHECK_CLOSE(libspectrum_wav_writer_close, writer, 1); r.close_error = 0;
  r.transfer_error = 1; before = r.closes;
  REQUIRE(libspectrum_wav_writer_open(&writer, "ram", 8000,
                                     LIBSPECTRUM_WAV_SAMPLE_UNSIGNED_8, 0));
  REQUIRE(!writer && r.closes == before + 1); r.transfer_error = 0;
  REQUIRE(r.opens == r.closes);
  REQUIRE(!fixture(&r, SRCDIR "/test/wav-mono-threshold.wav"));
  REQUIRE(!libspectrum_wav_reader_open(&reader, "ram", &rate, &frames));
  CHECK_CLOSE(libspectrum_wav_reader_close, reader, 0);
  REQUIRE(libspectrum_fs_install(&ram_backend, &r));
  libspectrum_file_free(file); g_hash_table_destroy(cache);
  return 0;
fail:
  if(native_file) libspectrum_file_free(native_file);
  if(tape) libspectrum_tape_free(tape);
  if(writer) libspectrum_wav_writer_close(writer);
  if(reader) libspectrum_wav_reader_close(reader);
  if(mmc) libspectrum_mmc_free(mmc);
  libspectrum_ide_eject_from_drive(&drive, cache);
  libspectrum_file_free(file); g_hash_table_destroy(cache);
  return 1;
}

static int
filesystem_optional(void)
{
  const libspectrum_fs_operations ls_fs = libspectrum_fs;
  ram r = { 0 };
  libspectrum_fs_backend b = ram_backend;
  int flag;
  b.writable = NULL; b.exists = NULL; b.unlink = NULL;
  REQUIRE(!libspectrum_fs_install(&b, &r));
  REQUIRE(ls_fs.writable("ram", &flag) && !flag);
  REQUIRE(ls_fs.exists("ram", &flag) && !flag);
  REQUIRE(ls_fs.unlink("ram"));
  REQUIRE(ls_fs.writable(SRCDIR "/test/file.hdf", &flag) && !flag);
  REQUIRE(ls_fs.exists(SRCDIR "/test/file.hdf", &flag) && !flag);
  REQUIRE(ls_fs.unlink(SRCDIR "/test/file.hdf"));
  REQUIRE(libspectrum_fs_install(&ram_backend, &r));
  return 0;
fail:
  return 1;
}

static int
filesystem_windows_permissions(void)
{
#ifdef _WIN32
  const libspectrum_fs_operations ls_fs = libspectrum_fs;
  const char *name = "test-filesystem-acl.tmp";
  libspectrum_fs_handle *h = NULL;
  PSECURITY_DESCRIPTOR original = NULL;
  PACL original_dacl = NULL, denied_dacl = NULL;
  SECURITY_DESCRIPTOR_CONTROL control;
  DWORD revision, attributes = FILE_ATTRIBUTE_NORMAL;
  SECURITY_INFORMATION restore_flags = DACL_SECURITY_INFORMATION;
  union { DWORD alignment; BYTE bytes[SECURITY_MAX_SID_SIZE]; } everyone;
  DWORD sid_size = sizeof(everyone.bytes);
  EXPLICIT_ACCESSA entries[2];
  HANDLE token = NULL, impersonation = NULL;
  int flag, created = 0, acl_changed = 0, impersonating = 0, result = 1;

  REQUIRE(!ls_fs.exists(name, &flag) && !flag);
  REQUIRE(!ls_fs.open(name, LIBSPECTRUM_FS_WRITE, &h)); created = 1;
  CHECK_CLOSE(ls_fs.close, h, 0);
  REQUIRE(!ls_fs.writable(name, &flag) && flag);
  attributes = GetFileAttributesA(name);
  REQUIRE(attributes != INVALID_FILE_ATTRIBUTES);
  REQUIRE(SetFileAttributesA(name, attributes | FILE_ATTRIBUTE_READONLY));
  REQUIRE(!ls_fs.writable(name, &flag) && !flag);
  REQUIRE(SetFileAttributesA(name, attributes));
  REQUIRE(GetNamedSecurityInfoA(name, SE_FILE_OBJECT,
    OWNER_SECURITY_INFORMATION | GROUP_SECURITY_INFORMATION | DACL_SECURITY_INFORMATION,
    NULL, NULL, &original_dacl, NULL, &original) == ERROR_SUCCESS);
  REQUIRE(GetSecurityDescriptorControl(original, &control, &revision));
  restore_flags |= control & SE_DACL_PROTECTED ? PROTECTED_DACL_SECURITY_INFORMATION :
                                               UNPROTECTED_DACL_SECURITY_INFORMATION;
  REQUIRE(CreateWellKnownSid(WinWorldSid, NULL, everyone.bytes, &sid_size));
  memset(entries, 0, sizeof(entries));
  entries[0].grfAccessPermissions = FILE_WRITE_DATA;
  entries[0].grfAccessMode = DENY_ACCESS;
  entries[0].Trustee.TrusteeForm = TRUSTEE_IS_SID;
  entries[0].Trustee.TrusteeType = TRUSTEE_IS_WELL_KNOWN_GROUP;
  entries[0].Trustee.ptstrName = (LPSTR)everyone.bytes;
  entries[1] = entries[0];
  entries[1].grfAccessPermissions = FILE_ALL_ACCESS;
  entries[1].grfAccessMode = GRANT_ACCESS;
  REQUIRE(SetEntriesInAclA(2, entries, NULL, &denied_dacl) == ERROR_SUCCESS);
  REQUIRE(SetNamedSecurityInfoA((LPSTR)name, SE_FILE_OBJECT,
    DACL_SECURITY_INFORMATION | PROTECTED_DACL_SECURITY_INFORMATION,
    NULL, NULL, denied_dacl, NULL) == ERROR_SUCCESS);
  acl_changed = 1;
  /* Not read-only: denial must come from the ACL, not the file attribute. */
  REQUIRE(!ls_fs.writable(name, &flag) && !flag);
  REQUIRE(OpenProcessToken(GetCurrentProcess(), TOKEN_QUERY | TOKEN_DUPLICATE, &token));
  REQUIRE(DuplicateToken(token, SecurityImpersonation, &impersonation));
  REQUIRE(SetThreadToken(NULL, impersonation)); impersonating = 1;
  REQUIRE(!ls_fs.writable(name, &flag) && !flag);
  REQUIRE(RevertToSelf()); impersonating = 0;
  REQUIRE(SetNamedSecurityInfoA((LPSTR)name, SE_FILE_OBJECT, restore_flags,
    NULL, NULL, original_dacl, NULL) == ERROR_SUCCESS); acl_changed = 0;
  REQUIRE(!ls_fs.writable(name, &flag) && flag);
  REQUIRE(!ls_fs.unlink(name)); created = 0;
  REQUIRE(ls_fs.writable(name, &flag) && !flag);
  result = 0;
fail:
  if(impersonating && !RevertToSelf()) result = 1;
  if(impersonation) CloseHandle(impersonation);
  if(token) CloseHandle(token);
  if(h) ls_fs.close(h);
  if(acl_changed && SetNamedSecurityInfoA((LPSTR)name, SE_FILE_OBJECT, restore_flags,
      NULL, NULL, original_dacl, NULL) != ERROR_SUCCESS) result = 1;
  if(denied_dacl) LocalFree(denied_dacl);
  if(original) LocalFree(original);
  if(created) {
    if(!SetFileAttributesA(name, attributes)) result = 1;
    if(ls_fs.unlink(name)) result = 1;
  }
  return result;
#else
  puts("Windows permission checks skipped on this platform");
  return 0;
#endif
}

#ifdef TEST_NATIVE_FIFO
static int
fifo_rejected(const char *name, libspectrum_fs_mode mode)
{
  const libspectrum_fs_operations ls_fs = libspectrum_fs;
  pid_t child = fork(), waited;
  int status;

  if(child < 0) return 0;
  if(!child) {
    libspectrum_fs_handle *h = NULL;
    sigset_t signals;
    if(signal(SIGALRM, SIG_DFL) == SIG_ERR || sigemptyset(&signals) ||
       sigaddset(&signals, SIGALRM) || sigprocmask(SIG_UNBLOCK, &signals, NULL))
      _exit(2);
    alarm(3);
    _exit(ls_fs.open(name, mode, &h) && !h ? 0 : 1);
  }
  do {
    waited = waitpid(child, &status, 0);
  } while(waited < 0 && errno == EINTR);
  if(waited == child && WIFSIGNALED(status) && WTERMSIG(status) == SIGALRM)
    fprintf(stderr, "Native FIFO open exceeded its deadline (mode %d)\n", mode);
  return waited == child && WIFEXITED(status) && !WEXITSTATUS(status);
}
#endif

static int
filesystem_native(int failed_first_use)
{
  const libspectrum_fs_operations ls_fs = libspectrum_fs;
  libspectrum_fs_handle *h = NULL;
  ram r = { 0 };
  int flag, created = 0;
  int64_t length, position;
  char bytes[3];
  const char *name = "test-filesystem.tmp";
#if defined _WIN32 || (defined HAVE_UNISTD_H && defined HAVE_MKDIR && defined HAVE_RMDIR)
#define TEST_NATIVE_UNLINK_DIRECTORY 1
  const char *directory = "test-filesystem-unlink-dir.tmp";
  int directory_created = 0;
#endif
#if !defined _WIN32 && defined HAVE_UNISTD_H && defined HAVE_SYMLINK
  const char *link = "test-filesystem-link.tmp";
  int linked = 0;
#endif
#ifdef TEST_NATIVE_FIFO
  const char *fifo = "test-filesystem-fifo.tmp";
  int fifo_created = 0;
#endif
  if(failed_first_use) {
    REQUIRE(ls_fs.open("test-filesystem-missing/no-file", LIBSPECTRUM_FS_READ, &h) && !h);
    REQUIRE(libspectrum_fs_install(&ram_backend, &r));
    return 0;
  }
  /* Querying a name selects native too, without needing an open handle. */
  REQUIRE(!ls_fs.exists(name, &flag) && !flag);
  REQUIRE(libspectrum_fs_install(&ram_backend, &r));
  REQUIRE(ls_fs.open(name, LIBSPECTRUM_FS_READ_WRITE, &h) && !h);
  REQUIRE(!ls_fs.exists(name, &flag) && !flag);
  REQUIRE(!ls_fs.open(name, LIBSPECTRUM_FS_WRITE, &h));
  created = 1;
  REQUIRE(!ls_fs.write_exact(h, "abc", 3));
  REQUIRE(!ls_fs.length(h, &length) && length == 3);
  REQUIRE(!ls_fs.tell(h, &position) && position == 3);
  libspectrum_end();
  REQUIRE(!libspectrum_init());
  CHECK_CLOSE(ls_fs.close, h, 0);
  REQUIRE(libspectrum_fs_install(&ram_backend, &r));
  REQUIRE(!ls_fs.open(name, LIBSPECTRUM_FS_READ, &h));
  REQUIRE(!ls_fs.read_exact(h, bytes, sizeof(bytes)) && !memcmp(bytes, "abc", 3));
  CHECK_CLOSE(ls_fs.close, h, 0);
  REQUIRE(!ls_fs.open(name, LIBSPECTRUM_FS_READ_WRITE, &h));
  REQUIRE(!ls_fs.read_exact(h, bytes, sizeof(bytes)) && !memcmp(bytes, "abc", 3));
  REQUIRE(!ls_fs.seek(h, 1, LIBSPECTRUM_FS_START));
  REQUIRE(!ls_fs.write_exact(h, "Z", 1));
  CHECK_CLOSE(ls_fs.close, h, 0);
  REQUIRE(!ls_fs.exists(".", &flag) && flag);
#if defined _WIN32 || defined TEST_NATIVE_VALIDATION
  REQUIRE(ls_fs.open(".", LIBSPECTRUM_FS_READ, &h) && !h);
  REQUIRE(ls_fs.open(".", LIBSPECTRUM_FS_READ_WRITE, &h) && !h);
#endif
#if !defined _WIN32 && defined HAVE_UNISTD_H && defined HAVE_SYMLINK
  REQUIRE(!symlink(name, link)); linked = 1;
  REQUIRE(!ls_fs.open(link, LIBSPECTRUM_FS_READ, &h));
  REQUIRE(!ls_fs.read_exact(h, bytes, sizeof(bytes)) && !memcmp(bytes, "aZc", 3));
  CHECK_CLOSE(ls_fs.close, h, 0);
  REQUIRE(!ls_fs.open(link, LIBSPECTRUM_FS_READ_WRITE, &h));
  REQUIRE(!ls_fs.read_exact(h, bytes, sizeof(bytes)) && !memcmp(bytes, "aZc", 3));
  CHECK_CLOSE(ls_fs.close, h, 0);
  REQUIRE(!ls_fs.unlink(link)); linked = 0;
  REQUIRE(!ls_fs.exists(link, &flag) && !flag);
  REQUIRE(!ls_fs.open(name, LIBSPECTRUM_FS_READ, &h));
  REQUIRE(!ls_fs.read_exact(h, bytes, sizeof(bytes)) && !memcmp(bytes, "aZc", 3));
  CHECK_CLOSE(ls_fs.close, h, 0);
#endif
#ifdef TEST_NATIVE_FIFO
  REQUIRE(!mkfifo(fifo, 0600)); fifo_created = 1;
  REQUIRE(!ls_fs.exists(fifo, &flag) && flag);
  REQUIRE(fifo_rejected(fifo, LIBSPECTRUM_FS_READ));
  REQUIRE(fifo_rejected(fifo, LIBSPECTRUM_FS_READ_WRITE));
  REQUIRE(!ls_fs.unlink(fifo)); fifo_created = 0;
#endif
#ifdef TEST_NATIVE_UNLINK_DIRECTORY
#ifdef _WIN32
  REQUIRE(!_mkdir(directory));
#else
  REQUIRE(!mkdir(directory, 0700));
#endif
  directory_created = 1;
  REQUIRE(ls_fs.unlink(directory));
  REQUIRE(!ls_fs.exists(directory, &flag) && flag);
#ifdef _WIN32
  REQUIRE(!_rmdir(directory));
#else
  REQUIRE(!rmdir(directory));
#endif
  directory_created = 0;
#endif
  REQUIRE(!ls_fs.unlink(name)); created = 0;
  REQUIRE(!ls_fs.exists(name, &flag) && !flag);
  return 0;
fail:
  if(h) ls_fs.close(h);
#ifdef TEST_NATIVE_UNLINK_DIRECTORY
  if(directory_created) {
#ifdef _WIN32
    _rmdir(directory);
#else
    rmdir(directory);
#endif
  }
#endif
#ifdef TEST_NATIVE_FIFO
  if(fifo_created) ls_fs.unlink(fifo);
#endif
#if !defined _WIN32 && defined HAVE_UNISTD_H && defined HAVE_SYMLINK
  if(linked) ls_fs.unlink(link);
#endif
  if(created) ls_fs.unlink(name);
  return 1;
}

int
main(int argc, char **argv)
{
  int result;
  if(argc != 2) return 2;
  /* No codec init is needed for filesystem selection/use. */
  if(!strcmp(argv[1], "windows-permissions")) return filesystem_windows_permissions();
  if(!strcmp(argv[1], "native")) return filesystem_native(0);
  if(!strcmp(argv[1], "native-failure")) return filesystem_native(1);
  if(!strcmp(argv[1], "optional")) return filesystem_optional();
  if(!strcmp(argv[1], "contract")) return filesystem_contract();
  if(strcmp(argv[1], "codecs")) return 2;
  if(libspectrum_init()) return 2;
  result = filesystem_codecs();
  libspectrum_end();
  return result;
}

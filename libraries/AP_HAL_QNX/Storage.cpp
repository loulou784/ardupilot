#include "Storage.h"

#include <alloca.h>
#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <AP_HAL/AP_HAL.h>

using namespace QNX;

/*
  This stores 'eeprom' data on the SD card, with a 4k size, and a
  in-memory buffer. This keeps the latency down.

  QNX's printf() has no %m, a glibc extension, so errors are spelled out
  with strerror().
 */

// name the storage file after the sketch so you can use the same board
// card for ArduCopter and ArduPlane
#define STORAGE_FILE AP_BUILD_TARGET_NAME ".stg"

extern const AP_HAL::HAL& hal;

static inline int is_dir(const char *path)
{
    struct stat st;

    if (stat(path, &st) < 0) {
        return -errno;
    }

    return S_ISDIR(st.st_mode);
}

static int mkdir_p(const char *path, int len, mode_t mode)
{
    char *start, *end;

    // QNX has no strndupa(), a glibc extension
    start = (char *)alloca(len + 1);
    memcpy(start, path, len);
    start[len] = '\0';
    end = start + len;

    /*
     * scan backwards, replacing '/' with '\0' while the component doesn't
     * exist
     */
    for (;;) {
        int r = is_dir(start);
        if (r > 0) {
            end += strlen(end);

            if (end == start + len) {
                return 0;
            }

            /* end != start, since it would be caught on the first
             * iteration */
            *end = '/';
            break;
        } else if (r == 0) {
            return -ENOTDIR;
        }

        if (end == start) {
            break;
        }

        *end = '\0';

        /* Find the next component, backwards, discarding extra '/'*/
        while (end > start && *end != '/') {
            end--;
        }

        while (end > start && *(end - 1) == '/') {
            end--;
        }
    }

    while (end < start + len) {
        if (mkdir(start, mode) < 0 && errno != EEXIST) {
            return -errno;
        }

        end += strlen(end);
        *end = '/';
    }

    return 0;
}

int Storage::_storage_create(const char *dpath)
{
    int dfd = -1;

    mkdir_p(dpath, strlen(dpath), 0777);
    // unlike Linux, QNX only accepts a directory descriptor in openat()
    // if the directory was opened with O_DIRECTORY
    dfd = open(dpath, O_RDONLY|O_DIRECTORY|O_CLOEXEC);
    if (dfd == -1) {
        fprintf(stderr, "Failed to open storage directory: %s (%s)\n", dpath,
                strerror(errno));
        return -1;
    }

    int fd = openat(dfd, STORAGE_FILE, O_RDWR|O_CREAT|O_CLOEXEC, 0666);

    if (fd == -1) {
        fprintf(stderr, "Failed to create storage file %s/%s\n", dpath,
                STORAGE_FILE);
        goto fail;
    }

    // take up all needed space
    if (ftruncate(fd, sizeof(_buffer)) == -1) {
        fprintf(stderr, "Failed to set file size to %u kB (%s)\n",
                unsigned(sizeof(_buffer) / 1024), strerror(errno));
        close(fd);
        goto fail;
    }

    // ensure the directory is updated with the new size.  QNX's qnx6
    // filesystem fails fsync() on a directory with EINVAL, but its fsync()
    // of the file commits the directory entry too.
    fsync(fd);
    fsync(dfd);

    close(dfd);

    return fd;

fail:
    close(dfd);
    return -1;
}

void Storage::init()
{
    WITH_SEMAPHORE(_sem);
    const char *dpath;

    if (_initialised) {
        return;
    }

    _dirty_mask = 0;

    dpath = hal.util->get_custom_storage_directory();
    if (!dpath) {
        dpath = HAL_BOARD_STORAGE_DIRECTORY;
    }

    int fd = _storage_create(dpath);
    if (fd == -1) {
        AP_HAL::panic("Cannot create storage %s (%s)", dpath, strerror(errno));
    }

    ssize_t ret = read(fd, _buffer, sizeof(_buffer));

    if (ret != sizeof(_buffer)) {
        close(fd);
        AP_HAL::panic("Failed to read %s (%s)", dpath, strerror(errno));
    }

    _fd = fd;
    _initialised = true;
}

/*
  Mark lines dirty while holding _sem, together with the buffer update.
 */
void Storage::_mark_dirty(uint16_t loc, uint16_t length)
{
    if (length == 0) {
        return;
    }
    uint16_t end = loc + length - 1;
    for (uint8_t line=loc>>QNX_STORAGE_LINE_SHIFT;
         line <= end>>QNX_STORAGE_LINE_SHIFT;
         line++) {
        _dirty_mask |= 1U << line;
    }
}

void Storage::read_block(void *dst, uint16_t loc, size_t n)
{
    WITH_SEMAPHORE(_sem);
    if (loc >= sizeof(_buffer)-(n-1)) {
        return;
    }
    init();
    memcpy(dst, &_buffer[loc], n);
}

void Storage::write_block(uint16_t loc, const void *src, size_t n)
{
    WITH_SEMAPHORE(_sem);
    if (loc >= sizeof(_buffer)-(n-1)) {
        return;
    }
    if (memcmp(src, &_buffer[loc], n) != 0) {
        init();
        memcpy(&_buffer[loc], src, n);
        _mark_dirty(loc, n);
    }
}

void Storage::_timer_tick(void)
{
    // Serialize flushes without blocking buffer access during disk IO.
    WITH_SEMAPHORE(_timer_sem);
    uint8_t snapshot[QNX_STORAGE_MAX_WRITE];
    uint8_t i, n;
    uint32_t write_mask;
    {
        WITH_SEMAPHORE(_sem);
        if (!_initialised || _dirty_mask == 0 || _fd == -1) {
            return;
        }

        // Snapshot the first contiguous set of dirty lines.
        for (i=0; i<QNX_STORAGE_NUM_LINES; i++) {
            if (_dirty_mask & (1U<<i)) {
                break;
            }
        }
        if (i == QNX_STORAGE_NUM_LINES) {
            return;
        }
        write_mask = (1U<<i);
        for (n=1; (i+n) < QNX_STORAGE_NUM_LINES &&
                 n < (QNX_STORAGE_MAX_WRITE>>QNX_STORAGE_LINE_SHIFT); n++) {
            if (!(_dirty_mask & (1U<<(n+i)))) {
                break;
            }
            write_mask |= (1U<<(n+i));
        }
        memcpy(snapshot, &_buffer[i<<QNX_STORAGE_LINE_SHIFT], n<<QNX_STORAGE_LINE_SHIFT);
        // Any write after this point must queue the line again, even while
        // the snapshot is still being written to disk.
        _dirty_mask &= ~write_mask;
    }

    if (pwrite(_fd, snapshot, n<<QNX_STORAGE_LINE_SHIFT, i<<QNX_STORAGE_LINE_SHIFT) != n<<QNX_STORAGE_LINE_SHIFT) {
        close(_fd);
        WITH_SEMAPHORE(_sem);
        _dirty_mask |= write_mask;
        _fd = -1;
        return;
    }
    bool clean;
    {
        WITH_SEMAPHORE(_sem);
        clean = (_dirty_mask == 0);
    }
    if (clean && fsync(_fd) != 0) {
        close(_fd);
        WITH_SEMAPHORE(_sem);
        _fd = -1;
    }
}

/*
  get storage size and ptr
 */
bool Storage::get_storage_ptr(void *&ptr, size_t &size)
{
    WITH_SEMAPHORE(_sem);
    if (!_initialised) {
        return false;
    }
    // The caller receives a live buffer, not a snapshot protected by _sem.
    ptr = _buffer;
    size = sizeof(_buffer);
    return true;
}

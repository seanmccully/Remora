/* mbed Microcontroller Library
 * Copyright (c) 2015 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "sdfile.h"
#include <errno.h>

SDFile::SDFile() {}

SDFile::SDFile(const char *path, int flags) : _fs(0), _file(0) {

    bd = new SDMMCBlockDevice(PIN_DETECT);
    _fs = new FATFileSystem("fs", bd);
    int err = open(_fs, path, flags);
    fileOpen = false;
    if (err == 0)
        fileOpen = true;
}

SDFile::SDFile(FATFileSystem *fs, const char *path, int flags)
    : _fs(0), _file(0)
{
    open(fs, path, flags);
}

SDFile::~SDFile()
{
    if (_fs) {
        SDFile::close();
    }
}

bool SDFile::is_open() {
    return fileOpen;
}

int SDFile::open(const char *path, int flags)
{
    if (!_fs)
        return -1;
    int err = _fs->file_open(&_file, path, flags);
    return err;
}

int SDFile::open(FATFileSystem *fs, const char *path, int flags)
{
    if (fs) {
        _fs = fs;
    }
    int err = fs->file_open(&_file, path, flags);
    if (!err) {
        _fs = fs;
    }

    return err;
}

int SDFile::close()
{

    int err = _fs->file_close(_file);
    _fs = 0;
    return err;
}

ssize_t SDFile::read(void *buffer, size_t len)
{
    return _fs->file_read(_file, buffer, len);
}

ssize_t SDFile::write(const void *buffer, size_t len)
{
    return _fs->file_write(_file, buffer, len);
}

int SDFile::sync()
{
    return _fs->file_sync(_file);
}

off_t SDFile::seek(off_t offset, int whence)
{
    return _fs->file_seek(_file, offset, whence);
}

off_t SDFile::tell()
{
    return _fs->file_tell(_file);
}


off_t SDFile::size()
{
    return _fs->file_size(_file);
}

int SDFile::truncate(off_t length)
{
    return _fs->file_truncate(_file, length);
}

#ifndef FS_H
#define FS_H

#include "ff.h"
#include "stdint.h"
#include "string.h"
#include "stdio.h"

inline static void fclose(FIL* file) {
    if (!file) return;
    f_close(file);
    delete file;
}

inline static FIL* fopen(const char* filename, BYTE mode) {
    FIL* res = new FIL();
    if ( f_open(res, filename, mode) == FR_OK ) {
         return res;
    }
    delete res;
    return 0;
}

inline static size_t ftell(FIL* f) {
    return (size_t)f_tell(f);
}

inline static int fseek(FIL* f, size_t pos, int type) {
    if (type == SEEK_SET) {
        return f_lseek(f, pos) == FR_OK ? 0 : 1;
    }
    if (type == SEEK_END) {
        return f_lseek(f, f_size(f) + pos) == FR_OK ? 0 : 1;
    }
    return f_lseek(f, f_tell(f) + pos) == FR_OK ? 0 : 1;
}

inline static size_t fread(void* b, size_t sz1, size_t sz2, FIL* f) {
    size_t res = 0;
    f_read(f, b, sz1 * sz2, &res);
    return res;
}

inline static size_t fwrite(const void* b, size_t sz1, size_t sz2, FIL* f) {
    size_t res = 0;
    f_write(f, b, sz1 * sz2, &res);
    return res;
}

#endif

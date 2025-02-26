/* Copyright (c) 2025, Canaan Bright Sight Co., Ltd
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __UVC_API_H__
#define __UVC_API_H__

#include "mpi_uvc_api.h"

#ifdef __cplusplus
extern "C" {
#endif /* End of #ifdef __cplusplus */

#define UVC_DEBUG (0)
#define USBH_VIDEO_FORMAT_UNCOMPRESSED 0
#define USBH_VIDEO_FORMAT_MJPEG        1
#define BUF_CNT (4)

struct uvc_device {
    int fd;
    bool is_streamon;
    char *frame_buf[BUF_CNT];
};

struct uvc_fmtdesc {
    unsigned int index;
    unsigned char format_type;
    unsigned char description[32];
};

struct uvc_format {
    unsigned int width;
    unsigned int height;
    unsigned char format_type;
};

struct uvc_requestbuffers {
    unsigned int count;
};

struct dfs_mmap2_args
{
    void *addr;
    size_t length;
    int prot;
    int flags;
    off_t pgoffset;

    void *ret;
};

#define VIDIOC_ENUM_FMT     _IOWR('V', 1, struct uvc_fmtdesc)
#define VIDIOC_S_FMT        _IOWR('V', 2, struct uvc_format)
#define VIDIOC_REQBUFS      _IOWR('V', 3, struct uvc_requestbuffers)
#define VIDIOC_QUERYBUF     _IOWR('V',  4, struct uvc_frame)
#define VIDIOC_QBUF         _IOWR('V', 5, struct uvc_frame)
#define VIDIOC_DQBUF        _IOWR('V', 6, struct uvc_frame)
#define VIDIOC_BUFMMAP      _IOW('V', 7, struct dfs_mmap2_args)
#define VIDIOC_STREAMON     _IOW('V', 8, int)
#define VIDIOC_STREAMOFF    _IOW('V', 9, int)

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif

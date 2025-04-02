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

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <sys/mman.h>
#include <errno.h>

#include <sys/vfs.h>
#include <stdbool.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/select.h>

#include "uvc_api.h"

static struct uvc_device uvc_dev;

int uvc_init(int *width, int *height, unsigned char is_jpeg)
{
    int fd;
    int ret = 0;
    bool found =false;
    struct uvc_fmtdesc fmt_desc;
    struct uvc_format format;
    struct uvc_requestbuffers requset_buf;
    struct uvc_frame uvc_frame;
    struct uvc_framedesc frame_desc;
    struct uvc_fpsdesc fps_desc;
    char *frame_buf[BUF_CNT];

    uvc_dev.fd = -1;
    uvc_dev.is_streamon = false;

    fd = open("/dev/video0", O_RDWR);
    if (fd < 0) {
        printf("open dev fail: %s (errno: %d)\n", strerror(errno), errno);
        return -1;
    }

    uvc_dev.fd = fd;

    memset(&fmt_desc, 0, sizeof(fmt_desc));
    fmt_desc.index = 0;

    while (ioctl(fd, VIDIOC_ENUM_FMT, &fmt_desc) == 0) {
#if UVC_DEBUG
        printf("fmt type is %d -> (%s)\n", fmt_desc.format_type, fmt_desc.description);
#endif
        frame_desc.format_type = fmt_desc.format_type;
        frame_desc.index = 0;
        while (ioctl(fd, VIDIOC_ENUM_FRAME, &frame_desc) == 0) {
#if UVC_DEBUG
            printf("wWidth: %4d, wHeight: %4d, DefaultFrameInterval: %d\n",
                   frame_desc.width, frame_desc.height, frame_desc.defaultframeinterval);
#endif

            fps_desc.format_type = fmt_desc.format_type;
            fps_desc.width = frame_desc.width;
            fps_desc.height = frame_desc.height;
            fps_desc.index = 0;
            while (ioctl(fd, VIDIOC_ENUM_INTERVAL, &fps_desc) == 0) {
#if UVC_DEBUG
                printf("FrameInterval[%d]: %d\n", fps_desc.index, fps_desc.frameinterval);
#endif
                fps_desc.index ++;
            }
            frame_desc.index ++;
        }

        if (fmt_desc.format_type == is_jpeg)
            found = true;
        fmt_desc.index ++;
    }

    if (!found) {
        printf("Don't support format\n");
        ret = -1;
        goto err;
    }

    format.format_type = is_jpeg;
    format.width = *width;
    format.height = *height;
    /* we may set the frame interval which device support,but we intentionally
     * set a wrong value, then it will use default frame interval in current resolution */
    format.frameinterval = 0;

#if UVC_DEBUG
    printf("expect resolution: %d X %d\n", *width, *height);
#endif
    if ((ret = ioctl(fd, VIDIOC_S_FMT, &format))) {
        printf("VIDIOC_S_FMT fail: %s (errno: %d)\n", strerror(errno), errno);
        goto err;
    }

    *width = format.width;
    *height = format.height;

#if UVC_DEBUG
    printf("suite resolution: %d X %d, format = %d\n", *width, *height, is_jpeg);
#endif

    requset_buf.count = BUF_CNT;

    if ((ret = ioctl(fd, VIDIOC_REQBUFS, &requset_buf))) {
        printf("VIDIOC_REQBUFS fail: %s (errno: %d)\n", strerror(errno), errno);
        goto err;
    }

    for (int i = 0; i < requset_buf.count; i ++) {
        struct dfs_mmap2_args mmap;

        uvc_frame.index = i;
        if ((ret = ioctl(fd, VIDIOC_QUERYBUF, &uvc_frame))) {
            printf("VIDIOC_QUERYBUF fail: %s (errno: %d)\n", strerror(errno), errno);
            goto err;
        }

        mmap.length = uvc_frame.reserve_1;
        mmap.pgoffset = uvc_frame.reserve_2;
        //mmap.prot = ;
        if ((ret = ioctl(fd, VIDIOC_BUFMMAP, &mmap))) {
            printf("VIDIOC_BUFMMAP fail: %s (errno: %d)\n", strerror(errno), errno);
            goto err;
        }
        uvc_dev.frame_buf[i] = frame_buf[i] = (char *)mmap.addr;
#if UVC_DEBUG
        printf("map addr = %p, len = %ld, offset = %d, vaddr = %p\n",
               mmap.addr, mmap.length, uvc_frame.reserve_2, frame_buf[i]);
#endif

        if ((ret = ioctl(fd, VIDIOC_QBUF, &uvc_frame))) {
            printf("VIDIOC_QBUF fail: %s (errno: %d)\n", strerror(errno), errno);
            goto err;
        }
    }

    if ((ret = ioctl(fd, VIDIOC_STREAMON, NULL))) {
        printf("VIDIOC_STREAMON fail: %s (errno: %d)\n", strerror(errno), errno);
        goto err;
    }

    uvc_dev.is_streamon = true;

    return ret;

err:
    close(fd);

    return ret;
}

void uvc_exit()
{
    int ret;
    int fd = uvc_dev.fd;

    if (uvc_dev.is_streamon == true) {
        if ((ret = ioctl(fd, VIDIOC_STREAMOFF, NULL))) {
            printf("VIDIOC_STREAMOFF fail: %s (errno: %d)\n", strerror(errno), errno);
        }

        if (fd != -1) {
            close(fd);
        }
    }

}

int uvc_get_frame(struct uvc_frame *frame)
{
    int fd = uvc_dev.fd;
    int ret = 0;
    fd_set readset;

    FD_ZERO(&readset);
    FD_SET(fd, &readset);

    if (select(fd + 1, &readset, NULL, NULL, NULL) == 0) {
        printf("do select fail\n");
        ret = -1;
        goto err;
    }

    if ((ret = ioctl(fd, VIDIOC_DQBUF, frame))) {
        printf("VIDIOC_DQBUF fail: %s (errno: %d)\n", strerror(errno), errno);
        goto err;
    }

    frame->userptr = uvc_dev.frame_buf[frame->index];

err:
    return ret;
}

int uvc_put_frame(struct uvc_frame *frame)
{
    int ret = 0;
    int fd = uvc_dev.fd;

    if ((ret = ioctl(fd, VIDIOC_QBUF, frame))) {
        printf("VIDIOC_QBUF fail: %s (errno: %d)\n", strerror(errno), errno);
    }

    return ret;
}

int uvc_get_devinfo(char *info, int len)
{
    int ret = 0;
    int fd = uvc_dev.fd;
    struct usb_string str_manufacturer;
    struct usb_string str_product;
    struct usb_index usb_index;
    int need_len = 0;

    if ((ret = ioctl(fd, VIDIOC_GET_INDEX, &usb_index))) {
        printf("VIDIOC_GET_INDEX fail: %s (errno: %d)\n", strerror(errno), errno);
        goto out;
    }

    str_manufacturer.index = usb_index.iManufacturer;
    if ((ret = ioctl(fd, VIDIOC_GET_STRING, &str_manufacturer))) {
        printf("get iManufacturer fail: %s (errno: %d)\n", strerror(errno), errno);
        goto out;
    }
    need_len += strlen(str_manufacturer.str);

    str_product.index = usb_index.iProduct;
    if ((ret = ioctl(fd, VIDIOC_GET_STRING, &str_product))) {
        printf("get iManufacturer fail: %s (errno: %d)\n", strerror(errno), errno);
        goto out;
    }

    need_len += strlen(str_product.str);
    need_len += 2;

    if (len < need_len) {
        ret = -1;
        goto out;
    }

    snprintf(info, len, "%s %s", str_manufacturer.str, str_product.str);

out:
    return ret;
}

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
#pragma once

#include "sys/ioctl.h"
#include <stdint.h>

#include "mpi_uvc_api.h"

#define IOCTL_USBD_VIDEO_CREATE_POOL _IOW('v', 0x01, void*)
#define IOCTL_USBD_VIDEO_GET_BUFFER  _IOW('v', 0x02, void*)
#define IOCTL_USBD_VIDEO_PUT_BUFFER  _IOW('v', 0x03, void*)
#define IOCTL_USBD_VIDEO_CONFIGURE   _IOW('v', 0x04, void*)
#define IOCTL_USBD_VIDEO_STREAM_ON   _IOW('v', 0x05, void*)
#define IOCTL_USBD_VIDEO_STREAM_OFF  _IOW('v', 0x06, void*)
#define IOCTL_USBD_VIDEO_DEV_STATE   _IOR('v', 0x07, int*)

struct usbd_video_create_pool_cfg_t {
    uint32_t buffer_size;
    uint32_t buffer_count;
};

struct usbd_video_buffer_wrap_t {
    void*    user_buffer;
    uint32_t buffer_size;
};

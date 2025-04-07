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
#include <unistd.h>
#include <string.h>
#include <stdbool.h>
#include <sys/mman.h>
#include <signal.h>

#include "k_module.h"
#include "k_type.h"
#include "k_vb_comm.h"
#include "k_sys_comm.h"
#include "mpi_vb_api.h"
#include "mpi_vvi_api.h"
#include "mpi_sys_api.h"
#include "mpi_vo_api.h"
#include "k_vo_comm.h"

#include "k_connector_comm.h"
#include "mpi_connector_api.h"
#include "mpi_vdec_api.h"

#include "mpi_uvc_api.h"

#define PRIVATE_POLL_SZE                        (1920 * 1080 * 3 / 2) + (4096 * 2)
#define PRIVATE_POLL_NUM                        (4)

#define ALIGN_UP(x, align) (((x) + ((align) - 1)) & ~((align)-1))
#define OUTPUT_BUF_CNT 6

typedef struct
{
    k_u64 layer_phy_addr;
    k_pixel_format format;
    k_vo_point offset;
    k_vo_size act_size;
    k_u32 size;
    k_u32 stride;
    k_u8 global_alptha;

    //only layer0、layer1
    k_u32 func;
    // only layer0
    k_vo_scaler_attr attr;

} layer_info;

void display_hardware_init(void)
{
    // rst display subsystem
    kd_display_reset();
    // set hardware reset;
    kd_display_set_backlight();
}

int vb_init(void)
{
    k_s32 ret = 0;
    k_vb_config config;

    memset(&config, 0, sizeof(config));

    config.max_pool_cnt = 10;
    config.comm_pool[0].blk_cnt = 20;
    config.comm_pool[0].blk_size = PRIVATE_POLL_SZE;
    config.comm_pool[0].mode = VB_REMAP_MODE_NOCACHE;

    ret = kd_mpi_vb_set_config(&config);
    if (ret) {
        printf("kd_mpi_vb_set_config fail, ret = %d\n", ret);
        goto out;
    }

    ret = kd_mpi_vb_init();
    if (ret) {
        printf("kd_mpi_vb_init fail, ret = %d\n", ret);
    }

out:
    return ret;
}

int vb_create_vo_pool(void)
{
    k_s32 pool_id;
    k_vb_pool_config pool_config;

    memset(&pool_config, 0, sizeof(pool_config));
    pool_config.blk_cnt = PRIVATE_POLL_NUM;
    pool_config.blk_size = PRIVATE_POLL_SZE;
    pool_config.mode = VB_REMAP_MODE_NONE;
    pool_id = kd_mpi_vb_create_pool(&pool_config);      // osd0 - 3 argb 320 x 240

    if (VB_INVALID_POOLID == pool_id) {
        printf("create vo pool fail\n");
        return -1;
    }

    return pool_id;
}

k_s32 sample_connector_init(k_connector_type type)
{
    k_u32 ret = 0;
    k_s32 connector_fd;
    k_u32 chip_id = 0x00;
    k_connector_type connector_type = type;
    k_connector_info connector_info;

    memset(&connector_info, 0, sizeof(k_connector_info));

    //connector get sensor info
    ret = kd_mpi_get_connector_info(connector_type, &connector_info);
    if (ret) {
        printf("sample_vicap, the sensor type not supported!\n");
        return ret;
    }

    connector_fd = kd_mpi_connector_open(connector_info.connector_name);
    if (connector_fd < 0) {
        printf("%s, connector open failed.\n", __func__);
        return K_ERR_VO_NOTREADY;
    }

    // set connect power
    ret = kd_mpi_connector_power_set(connector_fd, 1);
    if (ret) {
        goto out;
    }
    // set connect get id
    ret = kd_mpi_connector_id_get(connector_fd, &chip_id);
    if (ret) {
        goto out;
    }
    // connector init
    ret = kd_mpi_connector_init(connector_fd, connector_info);
    if (ret) {
        goto out;
    }

out:
    return ret;
}

int vo_creat_layer_test(k_vo_layer chn_id, layer_info *info)
{
    k_vo_video_layer_attr attr;

    // check layer
    if ((chn_id >= K_MAX_VO_LAYER_NUM) || ((info->func & K_VO_SCALER_ENABLE) && (chn_id != K_VO_LAYER0))
        || ((info->func != 0) && (chn_id == K_VO_LAYER2))) {
        printf("input layer num failed \n");
        return -1 ;
    }

    memset(&attr, 0, sizeof(attr));

    // set offset
    attr.display_rect = info->offset;
    // set act
    attr.img_size = info->act_size;
    // sget size
    info->size = info->act_size.height * info->act_size.width * 3 / 2;
    //set pixel format
    attr.pixel_format = info->format;
    if ((info->format != PIXEL_FORMAT_YUV_SEMIPLANAR_420) &&
        (info->format != PIXEL_FORMAT_YVU_PLANAR_420) &&
        (info->format != PIXEL_FORMAT_YVU_SEMIPLANAR_420)) {
        printf("input pix format failed \n");
        return -1;
    }
    // set stride
    attr.stride = (info->act_size.width / 8 - 1) + ((info->act_size.height - 1) << 16);
    // set function
    attr.func = info->func;
    // set scaler attr
    attr.scaler_attr = info->attr;

    // set video layer atrr
    kd_mpi_vo_set_video_layer_attr(chn_id, &attr);

    // enable layer
    kd_mpi_vo_enable_video_layer(chn_id);

    return 0;
}

k_vb_blk_handle vo_insert_frame(k_video_frame_info *vf_info, void **pic_vaddr)
{
    k_u64 phys_addr = 0;
    k_u32 *virt_addr;
    k_vb_blk_handle handle;
    k_s32 size = 0;

    if (vf_info == NULL)
        return K_FALSE;

    if (vf_info->v_frame.pixel_format == PIXEL_FORMAT_ABGR_8888 || vf_info->v_frame.pixel_format == PIXEL_FORMAT_ARGB_8888)
        size = vf_info->v_frame.height * vf_info->v_frame.width * 4;
    else if (vf_info->v_frame.pixel_format == PIXEL_FORMAT_RGB_565 || vf_info->v_frame.pixel_format == PIXEL_FORMAT_BGR_565)
        size = vf_info->v_frame.height * vf_info->v_frame.width * 2;
    else if (vf_info->v_frame.pixel_format == PIXEL_FORMAT_ABGR_4444 || vf_info->v_frame.pixel_format == PIXEL_FORMAT_ARGB_4444)
        size = vf_info->v_frame.height * vf_info->v_frame.width * 2;
    else if (vf_info->v_frame.pixel_format == PIXEL_FORMAT_RGB_888 || vf_info->v_frame.pixel_format == PIXEL_FORMAT_BGR_888)
        size = vf_info->v_frame.height * vf_info->v_frame.width * 3;
    else if (vf_info->v_frame.pixel_format == PIXEL_FORMAT_ARGB_1555 || vf_info->v_frame.pixel_format == PIXEL_FORMAT_ABGR_1555)
        size = vf_info->v_frame.height * vf_info->v_frame.width * 2;
    else if (vf_info->v_frame.pixel_format == PIXEL_FORMAT_YVU_PLANAR_420 ||
             vf_info->v_frame.pixel_format == PIXEL_FORMAT_YUV_SEMIPLANAR_420 ||
             vf_info->v_frame.pixel_format == PIXEL_FORMAT_YVU_SEMIPLANAR_420)
        size = vf_info->v_frame.height * vf_info->v_frame.width * 3 / 2;

    size = size + 4096;         // 强制4K ，后边得删了

    handle = kd_mpi_vb_get_block(vf_info->pool_id, size, NULL);
    if (handle == VB_INVALID_HANDLE) {
        printf("%s get vb block error\n", __func__);
        return K_FAILED;
    }

    phys_addr = kd_mpi_vb_handle_to_phyaddr(handle);
    if (phys_addr == 0) {
        printf("%s get phys addr error\n", __func__);
        return K_FAILED;
    }

    virt_addr = (k_u32 *)kd_mpi_sys_mmap(phys_addr, size);
    // virt_addr = (k_u32 *)kd_mpi_sys_mmap_cached(phys_addr, size);

    if (virt_addr == NULL) {
        printf("%s mmap error\n", __func__);
        return K_FAILED;
    }

    vf_info->mod_id = K_ID_VO;
    vf_info->v_frame.phys_addr[0] = phys_addr;
    if (vf_info->v_frame.pixel_format == PIXEL_FORMAT_YVU_PLANAR_420) {
        vf_info->v_frame.phys_addr[1] = phys_addr + (vf_info->v_frame.height * vf_info->v_frame.stride[0]);
        vf_info->v_frame.phys_addr[2] = phys_addr + (vf_info->v_frame.height * vf_info->v_frame.stride[0]) +
            (vf_info->v_frame.height * vf_info->v_frame.stride[0] / 4);
    } else if (vf_info->v_frame.pixel_format == PIXEL_FORMAT_YUV_SEMIPLANAR_420 ||
               vf_info->v_frame.pixel_format == PIXEL_FORMAT_YVU_SEMIPLANAR_420) {
        vf_info->v_frame.phys_addr[1] = phys_addr + (vf_info->v_frame.height * vf_info->v_frame.stride[0]);
    }

    *pic_vaddr = virt_addr;

    return handle;
}

k_s32 vo_release_frame(k_vb_blk_handle handle)
{
    return kd_mpi_vb_release_block(handle);
}

static k_s32 vb_create_vdec_pool(int width, int height)
{
    k_vb_pool_config pool_config;

    memset(&pool_config, 0, sizeof(pool_config));
    pool_config.blk_cnt = OUTPUT_BUF_CNT;
    pool_config.blk_size = ALIGN_UP(width * height, 0x1000) * 2;
    pool_config.mode = VB_REMAP_MODE_NOCACHE;

    return kd_mpi_vb_create_pool(&pool_config);
}

static k_s32 vb_destroy_vdec_pool(k_s32 vdec_poolid)
{
    return kd_mpi_vb_destory_pool(vdec_poolid);
}

void yuyv_to_nv12(const char *yuyv, char *nv12, int width, int height) {
    char *y_plane = nv12;               // Y 平面
    char *uv_plane = nv12 + width * height; // UV 交错平面

    for (int j = 0; j < height; j++) {
        for (int i = 0; i < width; i += 2) {
            int index = j * width * 2 + i * 2; // YUYV 格式的索引

            // Y 分量
            y_plane[j * width + i] = yuyv[index];
            y_plane[j * width + i + 1] = yuyv[index + 2];

            // UV 分量（仅偶数行）
            if (j % 2 == 0) {
                uv_plane[(j / 2) * width + i] = yuyv[index + 1];     // U
                uv_plane[(j / 2) * width + i + 1] = yuyv[index + 3]; // V
            }
        }
    }
}

static k_s32 sample_vdec_bind_vo(k_u32 chn_id)
{
    k_mpp_chn vdec_mpp_chn;
    k_mpp_chn vvo_mpp_chn;

    vdec_mpp_chn.mod_id = K_ID_VDEC;
    vdec_mpp_chn.dev_id = 0;
    vdec_mpp_chn.chn_id = 0;
    vvo_mpp_chn.mod_id = K_ID_VO;
    vvo_mpp_chn.dev_id = 0;//VVO_DISPLAY_DEV_ID;
    vvo_mpp_chn.chn_id = chn_id;//VVO_DISPLAY_CHN_ID;

    return kd_mpi_sys_bind(&vdec_mpp_chn, &vvo_mpp_chn);
}

static k_s32 sample_vdec_unbind_vo(k_u32 chn_id)
{
    k_mpp_chn vdec_mpp_chn;
    k_mpp_chn vvo_mpp_chn;

    vdec_mpp_chn.mod_id = K_ID_VDEC;
    vdec_mpp_chn.dev_id = 0;
    vdec_mpp_chn.chn_id = 0;
    vvo_mpp_chn.mod_id = K_ID_VO;
    vvo_mpp_chn.dev_id = 0;//VVO_DISPLAY_DEV_ID;
    vvo_mpp_chn.chn_id = chn_id;//VVO_DISPLAY_CHN_ID;

    return kd_mpi_sys_unbind(&vdec_mpp_chn, &vvo_mpp_chn);
}

static bool exit_flag;

static void sig_handler(int sig_no) {

    exit_flag = true;

    printf("exit sig = %d\n", sig_no);
}

int main(int argc, char **argv)
{
    int ret, width, height, ch = 0;
    int total_frame;
    void *pic_vaddr = NULL;
    k_vo_layer chn_id = K_VO_LAYER1;
    k_connector_type type;
    k_video_frame_info vf_info;
    layer_info info;
    k_vb_blk_handle block;
    bool rotation;
    unsigned char is_jpeg;
    k_s32 vdec_poolid;
    k_s32 vo_poolid;

    if (argc != 7) {
        printf("Usage: ./sample_uvc [connector_type] [rotation] [is_jpeg] [width] [height]"
               "[total_frame]\n");
        return -1;
    }

    type = atoi(argv[1]);
    rotation = atoi(argv[2]);
    is_jpeg = (unsigned char)atoi(argv[3]);
    width = atoi(argv[4]);
    height = atoi(argv[5]);
    total_frame = atoi(argv[6]);
    printf("type = %d, rotation = %d, is_jpeg = %d, (%d X %d) = %d frame\n",
           type, rotation, is_jpeg, width, height, total_frame);

    exit_flag = false;
    signal(SIGINT, sig_handler);
    signal(SIGPIPE, SIG_IGN);

    ret = vb_init();
    if (ret) {
        return -1;
    }

    /* init for uvc part */
    {
        struct uvc_format format = {width, height, is_jpeg, 0};

        ret = uvc_init(&format);
        if (ret) {
            printf("uvc_init fail\n");
            goto err0;
        }

        ret = uvc_start_stream();
        if (ret) {
            printf("uvc start stream fail\n");
            goto err0;
        }

        printf("uvc resolution is (%d X %d)\n", width, height);
    }

    /* init for vo part */
    {
        display_hardware_init();
        ret = sample_connector_init(type);
        if (ret) {
            goto err1;
        }

        vo_poolid = vb_create_vo_pool();
        if (!vo_poolid) {
            goto err1;
        }

        info.format = PIXEL_FORMAT_YUV_SEMIPLANAR_420;
        info.global_alptha = 0xff;
        info.offset.x = 0;
        info.offset.y = 0;

        if (!rotation) {
            info.act_size.width = width;
            info.act_size.height = height;
            info.func = 0;
        } else {
            info.act_size.width = height;
            info.act_size.height = width;
            info.func = K_ROTATION_270;
        }

        vo_creat_layer_test(chn_id, &info);
        memset(&vf_info, 0, sizeof(vf_info));
        vf_info.pool_id = vo_poolid;
        if (!rotation) {
            vf_info.v_frame.width = info.act_size.width;
            vf_info.v_frame.height = info.act_size.height;
            vf_info.v_frame.stride[0] = info.act_size.width;
        } else {
            vf_info.v_frame.width = info.act_size.height;
            vf_info.v_frame.height = info.act_size.width;
            vf_info.v_frame.stride[0] = info.act_size.height;
        }

        vf_info.v_frame.pixel_format = info.format;
        block = vo_insert_frame(&vf_info, &pic_vaddr);
    }

    /* init for vdec part */
    if (is_jpeg) {
        k_vdec_chn_attr attr;

        vdec_poolid = vb_create_vdec_pool(width, height);
        if (vdec_poolid == VB_INVALID_POOLID) {
            printf("fail to create vdec pool\n");
            ret = -1;
            goto err2;
        }

        attr.pic_width = width;
        attr.pic_height = height;
        attr.frame_buf_cnt = OUTPUT_BUF_CNT;
        attr.stream_buf_size = ALIGN_UP(width * height, 0x1000);
        attr.frame_buf_size = attr.stream_buf_size * 2;
        attr.type = K_PT_JPEG;
        attr.frame_buf_pool_id = vdec_poolid;

        ret = kd_mpi_vdec_create_chn(ch, &attr);
        if (ret) {
            printf("kd_mpi_vdec_create_chn fail, ret = %d\n", ret);
            goto err3;
        }

        ret = kd_mpi_vdec_start_chn(ch);
        if (ret) {
            printf("kd_mpi_vdec_start_chn fail, ret = %d\n", ret);
            goto err4;
        }

        ret = sample_vdec_bind_vo(chn_id);
        if (ret) {
            printf("sample_vdec_bind_vo fail, ret = %d\n", ret);
            goto err5;
        }
    }

    {
        int frame_num = 0;

        while (1) {
            struct uvc_frame frame;

            ret = uvc_get_frame(&frame, 5000);
            if (ret) {
                printf("uvc_get_frame fail\n");
                break;
            }

            if (is_jpeg) {
                ret = kd_mpi_vdec_send_stream(ch, &frame.v_stream, -1);
                if (ret) {
                    printf("kd_mpi_vdec_send_stream fail\n");
                    break;
                }
            } else {
                yuyv_to_nv12(frame.userptr, (char *)pic_vaddr, width, height);
                ret = kd_mpi_vo_chn_insert_frame(chn_id, &vf_info);
                if (ret) {
                    printf("kd_mpi_vo_chn_insert_frame fail\n");
                    break;
                }
            }

            ret = uvc_put_frame(&frame);
            if (ret) {
                printf("uvc_put_frame fail\n");
                break;
            }

            if (++frame_num >= total_frame || exit_flag) {
                break;
            }
        }

        if (is_jpeg) {
            sample_vdec_unbind_vo(chn_id);
        }
        kd_mpi_vo_disable_video_layer(chn_id);
        uvc_exit();
    }

    kd_mpi_sys_munmap(pic_vaddr, info.size + 4096);
    vo_release_frame(block);
    kd_mpi_vb_destory_pool(vo_poolid);

    if (is_jpeg) {
        kd_mpi_vdec_stop_chn(ch);
        kd_mpi_vdec_destroy_chn(ch);
        vb_destroy_vdec_pool(vdec_poolid);
    }

    kd_mpi_vb_exit();

    return 0;

err5:
    kd_mpi_vdec_stop_chn(ch);
err4:
    kd_mpi_vdec_destroy_chn(ch);
err3:
    vb_destroy_vdec_pool(vdec_poolid);
err2:
    kd_mpi_vb_destory_pool(vo_poolid);
err1:
    uvc_exit();
err0:
    kd_mpi_vb_exit();

    return ret;
}


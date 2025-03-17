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

#include "connector_dev.h"
#include "io.h"
#include "drv_gpio.h"
#include "k_vo_comm.h"
#include "k_connector_comm.h"

static void ili9881c_800x1280_init(k_u8 test_mode_en)
{
    const k_u8 init_sequence[] = {
        // cmd type, delay, data length, data0 ... dataN
        0x39, 0, 4, 0xff, 0x98, 0x81, 0x03,
        0x15, 0, 2, 0x01, 0x00,
        0x15, 0, 2, 0x02, 0x00,
        0x15, 0, 2, 0x03, 0x53,
        0x15, 0, 2, 0x04, 0x13,
        0x15, 0, 2, 0x05, 0x00,
        0x15, 0, 2, 0x06, 0x04,
        0x15, 0, 2, 0x07, 0x00,
        0x15, 0, 2, 0x08, 0x00,
        0x15, 0, 2, 0x09, 0x22,
        0x15, 0, 2, 0x0a, 0x22,
        0x15, 0, 2, 0x0b, 0x00,
        0x15, 0, 2, 0x0c, 0x01,
        0x15, 0, 2, 0x0d, 0x00,
        0x15, 0, 2, 0x0e, 0x00,
        0x15, 0, 2, 0x0f, 0x25,
        0x15, 0, 2, 0x10, 0x25,
        0x15, 0, 2, 0x11, 0x00,
        0x15, 0, 2, 0x12, 0x00,
        0x15, 0, 2, 0x13, 0x00,
        0x15, 0, 2, 0x14, 0x00,
        0x15, 0, 2, 0x15, 0x00,
        0x15, 0, 2, 0x16, 0x00,
        0x15, 0, 2, 0x17, 0x00,
        0x15, 0, 2, 0x18, 0x00,
        0x15, 0, 2, 0x19, 0x00,
        0x15, 0, 2, 0x1a, 0x00,
        0x15, 0, 2, 0x1b, 0x00,
        0x15, 0, 2, 0x1c, 0x00,
        0x15, 0, 2, 0x1d, 0x00,
        0x15, 0, 2, 0x1e, 0x44,
        0x15, 0, 2, 0x1f, 0x80,
        0x15, 0, 2, 0x20, 0x02,
        0x15, 0, 2, 0x21, 0x03,
        0x15, 0, 2, 0x22, 0x00,
        0x15, 0, 2, 0x23, 0x00,
        0x15, 0, 2, 0x24, 0x00,
        0x15, 0, 2, 0x25, 0x00,
        0x15, 0, 2, 0x26, 0x00,
        0x15, 0, 2, 0x27, 0x00,
        0x15, 0, 2, 0x28, 0x33,
        0x15, 0, 2, 0x29, 0x03,
        0x15, 0, 2, 0x2a, 0x00,
        0x15, 0, 2, 0x2b, 0x00,
        0x15, 0, 2, 0x2c, 0x00,
        0x15, 0, 2, 0x2d, 0x00,
        0x15, 0, 2, 0x2e, 0x00,
        0x15, 0, 2, 0x2f, 0x00,
        0x15, 0, 2, 0x30, 0x00,
        0x15, 0, 2, 0x31, 0x00,
        0x15, 0, 2, 0x32, 0x00,
        0x15, 0, 2, 0x33, 0x00,
        0x15, 0, 2, 0x34, 0x04,
        0x15, 0, 2, 0x35, 0x00,
        0x15, 0, 2, 0x36, 0x00,
        0x15, 0, 2, 0x37, 0x00,
        0x15, 0, 2, 0x38, 0x3c,
        0x15, 0, 2, 0x39, 0x00,
        0x15, 0, 2, 0x3a, 0x40,
        0x15, 0, 2, 0x3b, 0x40,
        0x15, 0, 2, 0x3c, 0x00,
        0x15, 0, 2, 0x3d, 0x00,
        0x15, 0, 2, 0x3e, 0x00,
        0x15, 0, 2, 0x3f, 0x00,
        0x15, 0, 2, 0x40, 0x00,
        0x15, 0, 2, 0x41, 0x00,
        0x15, 0, 2, 0x42, 0x00,
        0x15, 0, 2, 0x43, 0x00,
        0x15, 0, 2, 0x44, 0x00,
        0x15, 0, 2, 0x50, 0x01,
        0x15, 0, 2, 0x51, 0x23,
        0x15, 0, 2, 0x52, 0x45,
        0x15, 0, 2, 0x53, 0x67,
        0x15, 0, 2, 0x54, 0x89,
        0x15, 0, 2, 0x55, 0xab,
        0x15, 0, 2, 0x56, 0x01,
        0x15, 0, 2, 0x57, 0x23,
        0x15, 0, 2, 0x58, 0x45,
        0x15, 0, 2, 0x59, 0x67,
        0x15, 0, 2, 0x5a, 0x89,
        0x15, 0, 2, 0x5b, 0xab,
        0x15, 0, 2, 0x5c, 0xcd,
        0x15, 0, 2, 0x5d, 0xef,
        0x15, 0, 2, 0x5e, 0x11,
        0x15, 0, 2, 0x5f, 0x01,
        0x15, 0, 2, 0x60, 0x00,
        0x15, 0, 2, 0x61, 0x15,
        0x15, 0, 2, 0x62, 0x14,
        0x15, 0, 2, 0x63, 0x0c,
        0x15, 0, 2, 0x64, 0x0d,
        0x15, 0, 2, 0x65, 0x0e,
        0x15, 0, 2, 0x66, 0x0f,
        0x15, 0, 2, 0x67, 0x06,
        0x15, 0, 2, 0x68, 0x02,
        0x15, 0, 2, 0x69, 0x02,
        0x15, 0, 2, 0x6a, 0x02,
        0x15, 0, 2, 0x6b, 0x02,
        0x15, 0, 2, 0x6c, 0x02,
        0x15, 0, 2, 0x6d, 0x02,
        0x15, 0, 2, 0x6e, 0x08,
        0x15, 0, 2, 0x6f, 0x02,
        0x15, 0, 2, 0x70, 0x02,
        0x15, 0, 2, 0x71, 0x02,
        0x15, 0, 2, 0x72, 0x02,
        0x15, 0, 2, 0x73, 0x02,
        0x15, 0, 2, 0x74, 0x02,
        0x15, 0, 2, 0x75, 0x01,
        0x15, 0, 2, 0x76, 0x00,
        0x15, 0, 2, 0x77, 0x15,
        0x15, 0, 2, 0x78, 0x14,
        0x15, 0, 2, 0x79, 0x0c,
        0x15, 0, 2, 0x7a, 0x0d,
        0x15, 0, 2, 0x7b, 0x0e,
        0x15, 0, 2, 0x7c, 0x0f,
        0x15, 0, 2, 0x7d, 0x08,
        0x15, 0, 2, 0x7e, 0x02,
        0x15, 0, 2, 0x7f, 0x02,
        0x15, 0, 2, 0x80, 0x02,
        0x15, 0, 2, 0x81, 0x02,
        0x15, 0, 2, 0x82, 0x02,
        0x15, 0, 2, 0x83, 0x02,
        0x15, 0, 2, 0x84, 0x06,
        0x15, 0, 2, 0x85, 0x02,
        0x15, 0, 2, 0x86, 0x02,
        0x15, 0, 2, 0x87, 0x02,
        0x15, 0, 2, 0x88, 0x02,
        0x15, 0, 2, 0x89, 0x02,
        0x15, 0, 2, 0x8a, 0x02,
        0x39, 0, 4, 0xff, 0x98, 0x81, 0x04,
        0x15, 0, 2, 0x6c, 0x15,
        0x15, 0, 2, 0x6e, 0x3b,
        0x15, 0, 2, 0x6f, 0x55,
        0x15, 0, 2, 0x8d, 0x14,
        0x15, 0, 2, 0x87, 0xba,
        0x15, 0, 2, 0x26, 0x76,
        0x15, 0, 2, 0xb2, 0xd1,
        0x15, 0, 2, 0x3b, 0x98,
        0x15, 0, 2, 0x35, 0x1f,
        0x15, 0, 2, 0x3A, 0x24,
        0x15, 0, 2, 0xb5, 0x27,
        0x15, 0, 2, 0x31, 0x75,
        0x15, 0, 2, 0x30, 0x03,
        0x15, 0, 2, 0x33, 0x14,
        0x15, 0, 2, 0x38, 0x02,
        0x15, 0, 2, 0x39, 0x00,
        0x15, 0, 2, 0x7a, 0x10,
        0x39, 0, 4, 0xff, 0x98, 0x81, 0x01,
        0x15, 0, 2, 0x22, 0x0a,
        0x15, 0, 2, 0x31, 0x0a,
        0x15, 0, 2, 0x50, 0xae,
        0x15, 0, 2, 0x51, 0xa9,
        0x15, 0, 2, 0x60, 0x1f,
        0x15, 0, 2, 0x62, 0x07,
        0x15, 0, 2, 0x63, 0x00,
        0x15, 0, 2, 0x52, 0x00,
        0x15, 0, 2, 0x53, 0x56,
        0x15, 0, 2, 0x54, 0x00,
        0x15, 0, 2, 0x55, 0x59,
        0x15, 0, 2, 0xa0, 0x08,
        0x15, 0, 2, 0xa1, 0x20,
        0x15, 0, 2, 0xa2, 0x2d,
        0x15, 0, 2, 0xa3, 0x13,
        0x15, 0, 2, 0xa4, 0x16,
        0x15, 0, 2, 0xa5, 0x29,
        0x15, 0, 2, 0xa6, 0x1d,
        0x15, 0, 2, 0xa7, 0x1e,
        0x15, 0, 2, 0xa8, 0x77,
        0x15, 0, 2, 0xa9, 0x17,
        0x15, 0, 2, 0xaa, 0x24,
        0x15, 0, 2, 0xab, 0x6a,
        0x15, 0, 2, 0xac, 0x22,
        0x15, 0, 2, 0xad, 0x24,
        0x15, 0, 2, 0xae, 0x5a,
        0x15, 0, 2, 0xaf, 0x2b,
        0x15, 0, 2, 0xb0, 0x2c,
        0x15, 0, 2, 0xb1, 0x4d,
        0x15, 0, 2, 0xb2, 0x6f,
        0x15, 0, 2, 0xb3, 0x3f,
        0x15, 0, 2, 0xc0, 0x08,
        0x15, 0, 2, 0xc1, 0x1e,
        0x15, 0, 2, 0xc2, 0x2b,
        0x15, 0, 2, 0xc3, 0x13,
        0x15, 0, 2, 0xc4, 0x16,
        0x15, 0, 2, 0xc5, 0x28,
        0x15, 0, 2, 0xc6, 0x1a,
        0x15, 0, 2, 0xc7, 0x1d,
        0x15, 0, 2, 0xc8, 0x75,
        0x15, 0, 2, 0xc9, 0x18,
        0x15, 0, 2, 0xca, 0x25,
        0x15, 0, 2, 0xcb, 0x71,
        0x15, 0, 2, 0xcc, 0x23,
        0x15, 0, 2, 0xcd, 0x28,
        0x15, 0, 2, 0xce, 0x59,
        0x15, 0, 2, 0xcf, 0x2c,
        0x15, 0, 2, 0xd0, 0x30,
        0x15, 0, 2, 0xd1, 0x55,
        0x15, 0, 2, 0xd2, 0x6b,
        0x15, 0, 2, 0xd3, 0x3f,
        0x39, 0, 4, 0xff, 0x98, 0x81, 0x00,
        0x05, 0, 1, 0x35,
        0x05,120,1, 0x11,
        0x05, 0, 1, 0x29,
    };

    connector_send_cmd(init_sequence, sizeof(init_sequence), K_FALSE);
}

static void ili9881c_power_reset(k_s32 on)
{
    k_u8 rst_gpio;
    if(0 > (rst_gpio = CONFIG_MPP_DSI_LCD_RESET_PIN)) {
        return;
    }

    kd_pin_mode(rst_gpio, GPIO_DM_OUTPUT);
    if (on) {
        kd_pin_write(rst_gpio, GPIO_PV_HIGH);
    } else {
        kd_pin_write(rst_gpio, GPIO_PV_LOW);
    }
}

static void ili9881c_set_backlight(k_s32 on)
{
    k_u8 backlight_gpio;
    if(0 > (backlight_gpio = CONFIG_MPP_DSI_LCD_BACKLIGHT_PIN)) {
        return;
    }

    kd_pin_mode(backlight_gpio, GPIO_DM_OUTPUT);
    if (on) {
        kd_pin_write(backlight_gpio, GPIO_PV_HIGH);
    } else {
        kd_pin_write(backlight_gpio, GPIO_PV_LOW);
    }
}

static k_s32 ili9881c_power_on(void* ctx, k_s32 on)
{
    k_s32 ret = 0;
    struct connector_driver_dev* dev = ctx;

    if (on) {
        // rst vo;
        k230_display_rst();

        // rst ili9881C
        ili9881c_power_reset(0);
        rt_thread_mdelay(50);
        ili9881c_power_reset(1);
        rt_thread_mdelay(10);

        // enable backlight
        ili9881c_set_backlight(1);
    } else {
        ili9881c_set_backlight(0);
    }

    return ret;
}

static k_s32 ili9881c_set_phy_freq(k_connectori_phy_attr* phy_attr)
{
    k_vo_mipi_phy_attr mipi_phy_attr;

    memset(&mipi_phy_attr, 0, sizeof(k_vo_mipi_phy_attr));

    mipi_phy_attr.m = phy_attr->m;
    mipi_phy_attr.n = phy_attr->n;
    mipi_phy_attr.hs_freq = phy_attr->hs_freq;
    mipi_phy_attr.voc = phy_attr->voc;
    mipi_phy_attr.phy_lan_num = K_DSI_4LAN;
    connector_set_phy_freq(&mipi_phy_attr);

    return 0;
}

static k_s32 ili9881c_dsi_resolution_init(k_connector_info* info)
{
    k_vo_dsi_attr attr;
    k_vo_display_resolution resolution;

    memset(&attr, 0, sizeof(k_vo_dsi_attr));
    attr.lan_num = info->lan_num;
    attr.cmd_mode = info->cmd_mode;
    attr.lp_div = 8;
    attr.work_mode = info->work_mode;
    memcpy(&resolution, &info->resolution, sizeof(k_vo_display_resolution));
    memcpy(&attr.resolution, &resolution, sizeof(k_vo_display_resolution));
    connector_set_dsi_attr(&attr);

    ili9881c_800x1280_init(info->screen_test_mode);

    connector_set_dsi_enable(1);

    if (info->dsi_test_mode == 1) {
        connector_set_dsi_test_mode();
    }

    return 0;
}

static k_s32 ili9881c_vo_resolution_init(k_vo_display_resolution* resolution, k_u32 bg_color, k_u32 intr_line)
{
    k_vo_display_resolution vo_resolution;
    k_vo_pub_attr attr;

    memset(&attr, 0, sizeof(k_vo_pub_attr));
    attr.bg_color = bg_color;
    attr.intf_sync = K_VO_OUT_1080P30;
    attr.intf_type = K_VO_INTF_MIPI;
    attr.sync_info = resolution;

    connector_set_vo_init();
    connector_set_vtth_intr(1, intr_line);
    connector_set_vo_param(&attr);
    connector_set_vo_enable();

    return 0;
}

k_s32 ili9881c_init(void* ctx, k_connector_info* info)
{
    k_s32 ret = 0;
    struct connector_driver_dev* dev = ctx;

    if (info->pixclk_div != 0) {
        connector_set_pixclk(info->pixclk_div);
    }

    ret |= ili9881c_set_phy_freq(&info->phy_attr);
    ret |= ili9881c_dsi_resolution_init(info);
    ret |= ili9881c_vo_resolution_init(&info->resolution, info->bg_color, info->intr_line);

    return ret;
}

static k_s32 ili9881c_get_chip_id(void* ctx, k_u32* chip_id)
{
    k_s32 ret = 0;

    return ret;
}

static k_s32 ili9881c_conn_check(void* ctx, k_s32* conn)
{
    k_s32 ret = 0;

    *conn = 1;

    return ret;
}

static k_s32 ili9881c_set_mirror(void* ctx, k_connector_mirror* mirror)
{
    k_connector_mirror ili9881c_mirror;

    ili9881c_mirror = *mirror;

    switch (ili9881c_mirror) {
    case K_CONNECTOR_MIRROR_HOR:
        break;
    case K_CONNECTOR_MIRROR_VER:
        break;
    case K_CONNECTOR_MIRROR_BOTH:
        break;
    default:
        rt_kprintf("ili9881c_mirror(%d) is not support \n", ili9881c_mirror);
        break;
    }
    return 0;
}

struct connector_driver_dev ili9881c_connector_drv = {
    .connector_name = "ili9881c",
    .connector_func = {
        .connector_power = ili9881c_power_on,
        .connector_init = ili9881c_init,
        .connector_get_chip_id = ili9881c_get_chip_id,
        .connector_conn_check = ili9881c_conn_check,
        .connector_set_mirror = ili9881c_set_mirror,
    },
};

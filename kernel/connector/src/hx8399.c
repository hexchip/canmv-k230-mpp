/* Copyright (c) 2023, Canaan Bright Sight Co., Ltd
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

#define DELAY_MS_BACKLIGHT_DEFAULT     200
#define DELAY_MS_BACKLIGHT_FIRST       1

static k_u8 hx8399_y_mirror = 0;

static k_s32 g_blacklight_delay_ms = DELAY_MS_BACKLIGHT_FIRST;

static void hx8399_v2_init(k_u8 test_mode_en)
{
    const k_u8 init_sequence[] = {
        // cmd type, delay, data length, data0 ... dataN
        0x15, 100,  1,  0x01, // Soft reset
        0x39, 0,    4,  0xB9, 0xFF, 0x83, 0x99,
        0x39, 0,    2,  0xD2, 0xAA,
        0x39, 0,    16, 0xB1, 0x02, 0x04, 0x71, 0x91, 0x01, 0x32, 0x33, 0x11, 0x11, 0xab, 0x4d, 0x56, 0x73, 0x02, 0x02,
        0x39, 0,    16, 0xB2, 0x00, 0x80, 0x80, 0xae, 0x05, 0x07, 0x5a, 0x11, 0x00, 0x00, 0x10, 0x1e, 0x70, 0x03, 0xd4,
        0x39, 0,    45, 0xB4, 0x00, 0xFF, 0x02, 0xC0, 0x02, 0xc0, 0x00, 0x00, 0x08, 0x00, 0x04, 0x06, 0x00, 0x32, 0x04, 0x0a, 0x08, 0x21, 0x03, 0x01, 0x00, 0x0f, 0xb8, 0x8b, 0x02, 0xc0, 0x02, 0xc0, 0x00, 0x00, 0x08, 0x00, 0x04, 0x06, 0x00, 0x32, 0x04, 0x0a, 0x08, 0x01, 0x00, 0x0f, 0xb8, 0x01,
        0x39, 0,    34, 0xD3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x10, 0x04, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x05, 0x05, 0x07, 0x00, 0x00, 0x00, 0x05, 0x40,
        0x39, 0,    33, 0xD5, 0x18, 0x18, 0x19, 0x19, 0x18, 0x18, 0x21, 0x20, 0x01, 0x00, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x2f, 0x2f, 0x30, 0x30, 0x31, 0x31, 0x18, 0x18, 0x18, 0x18,
        0x39, 0,    33, 0xD6, 0x18, 0x18, 0x19, 0x19, 0x40, 0x40, 0x20, 0x21, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x00, 0x01, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x2f, 0x2f, 0x30, 0x30, 0x31, 0x31, 0x40, 0x40, 0x40, 0x40,
        0x39, 0,    17, 0xD8, 0xa2, 0xaa, 0x02, 0xa0, 0xa2, 0xa8, 0x02, 0xa0, 0xb0, 0x00, 0x00, 0x00, 0xb0, 0x00, 0x00, 0x00,
        0x15, 0,    2,  0xBD, 0x01,
        0x39, 0,    17, 0xD8, 0xB0, 0x00, 0x00, 0x00, 0xB0, 0x00, 0x00, 0x00, 0xE2, 0xAA, 0x03, 0xF0, 0xE2, 0xAA, 0x03, 0xF0,
        0x15, 0,    2,  0xBD, 0x02,
        0x39, 0,    9,  0xD8, 0xE2, 0xAA, 0x03, 0xF0, 0xE2, 0xAA, 0x03, 0xF0,
        0x15, 0,    2,  0xBD, 0x00,
        0x39, 0,    3,  0xB6, 0x8D, 0x8D,
        0x15, 0,    2,  0xCC, 0x09,
        0x39, 0,    3,  0xC6, 0xFF, 0xF9,
        0x39, 0,   55,  0xE0, 0x00, 0x12, 0x1f, 0x1a, 0x40, 0x4a, 0x59, 0x55, 0x5e, 0x67, 0x6f, 0x75, 0x7a, 0x82, 0x8b, 0x90, 0x95, 0x9f, 0xa3, 0xad, 0xa2, 0xb2, 0xB6, 0x5e, 0x5a, 0x65, 0x77, 0x00, 0x12, 0x1f, 0x1a, 0x40, 0x4a, 0x59, 0x55, 0x5e, 0x67, 0x6f, 0x75, 0x7a, 0x82, 0x8b, 0x90, 0x95, 0x9f, 0xa3, 0xad, 0xa2, 0xb2, 0xB6, 0x5e, 0x5a, 0x65, 0x77,
        0x15, 100,  1,  0x11,
        0x15, 20,   1,  0x29,
    };

    connector_send_cmd(init_sequence, sizeof(init_sequence), K_FALSE);

    // if (test_mode_en == 1) {
    //     const k_u8 test_sequence[] = {
    //         0x39, 0, 10, 0xB2, 0x0b, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, // Test mode
    //     };
    //     connector_send_cmd(test_sequence, sizeof(test_sequence), K_FALSE);
    // }
}

static void hx8399_power_reset(k_s32 on)
{
    k_u8 rst_gpio;
    if(0 > (rst_gpio = CONFIG_MPP_DSI_LCD_RESET_PIN)) {
        return;
    }

    kd_pin_mode(rst_gpio, GPIO_DM_OUTPUT);
    
    if (on)
        kd_pin_write(rst_gpio, GPIO_PV_HIGH); // GPIO_PV_LOW  GPIO_PV_HIGH
    else
        kd_pin_write(rst_gpio, GPIO_PV_LOW); // GPIO_PV_LOW  GPIO_PV_HIGH

}

static void hx8399_set_backlight(k_s32 on)
{
    k_u8 backlight_gpio;
    if(0 > (backlight_gpio = CONFIG_MPP_DSI_LCD_BACKLIGHT_PIN)) {
        return;
    }

    kd_pin_mode(backlight_gpio, GPIO_DM_OUTPUT);
    if (on)
        kd_pin_write(backlight_gpio, GPIO_PV_HIGH);
    else
        kd_pin_write(backlight_gpio, GPIO_PV_LOW);

}


static k_s32 hx8399_power_on(void* ctx, k_s32 on)
{
    k_s32 ret = 0;
    struct connector_driver_dev* dev = ctx;

    if (on) {
        // rst vo;
        k230_display_rst();
        // rst hx8399 
        hx8399_power_reset(1);
        rt_thread_mdelay(g_blacklight_delay_ms);
        hx8399_power_reset(0);
        rt_thread_mdelay(g_blacklight_delay_ms);
        hx8399_power_reset(1);

        g_blacklight_delay_ms = DELAY_MS_BACKLIGHT_DEFAULT;
        //enable backlight
        hx8399_set_backlight(1);
    } else {
        hx8399_set_backlight(0);
    }
    
    return ret;
}


static k_s32 hx8399_set_phy_freq(k_connectori_phy_attr *phy_attr)
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


static k_s32 hx8399_dsi_resolution_init(k_connector_info *info)
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

    if(info->screen_test_mode)
        hx8399_v2_init(1);
    else
        hx8399_v2_init(0);

    connector_set_dsi_enable(1);

    if(info->dsi_test_mode == 1)
        connector_set_dsi_test_mode();

    return 0;
}


static k_s32 hx8399_vo_resolution_init(k_vo_display_resolution *resolution, k_u32 bg_color, k_u32 intr_line)
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
   

k_s32 hx8399_init(void *ctx, k_connector_info *info)
{
    k_s32 ret = 0;
    struct connector_driver_dev* dev = ctx;

    if(info->pixclk_div != 0)
        connector_set_pixclk(info->pixclk_div);

    ret |= hx8399_set_phy_freq(&info->phy_attr);
    ret |= hx8399_dsi_resolution_init(info);
    ret |= hx8399_vo_resolution_init(&info->resolution, info->bg_color, info->intr_line);

    return ret;
}

static k_s32 hx8399_get_chip_id(void* ctx, k_u32* chip_id)
{
    k_s32 ret = 0;

    return ret;
}


static k_s32 hx8399_conn_check(void* ctx, k_s32* conn)
{
    k_s32 ret = 0;

    *conn = 1;

    return ret;
}


static k_s32 hx8399_set_mirror(void* ctx, k_connector_mirror *mirror)
{
    k_connector_mirror hx8399_mirror;

    hx8399_mirror = *mirror;

    switch(hx8399_mirror)
    {
        case K_CONNECTOR_MIRROR_HOR:
            break;
        case K_CONNECTOR_MIRROR_VER: 
            hx8399_y_mirror = 0x04;
            break;
        case K_CONNECTOR_MIRROR_BOTH: 
            break;
        default :
            rt_kprintf("hx8399_mirror(%d) is not support \n", hx8399_mirror);
            break;
    }
    return 0;
}


struct connector_driver_dev hx8399_connector_drv = {
    .connector_name = "hx8399",
    .connector_func = {
        .connector_power = hx8399_power_on,
        .connector_init = hx8399_init,
        .connector_get_chip_id = hx8399_get_chip_id,
        .connector_conn_check = hx8399_conn_check,
        .connector_set_mirror = hx8399_set_mirror,
    },
};
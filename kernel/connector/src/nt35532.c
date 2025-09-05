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
#include "drv_gpio.h"
#include "io.h"
#include "k_autoconf_comm.h"
#include "k_connector_comm.h"
#include "k_vo_comm.h"

static void lcd_init(void)
{
    const k_u8 init_sequence[] = {
        CONNECTOR_CMD_SEQUENCE(0x15, 50, 0x01, 0x00), // Soft reset
        CONNECTOR_CMD_SEQUENCE(0x15, 0, 0xFF, 0x00), //
        // CONNECTOR_CMD_SEQUENCE(0x15, 30, 0xFB, 0x01), //
        CONNECTOR_CMD_SEQUENCE(0x15, 0, 0xD3, 52), // VBP
        CONNECTOR_CMD_SEQUENCE(0x15, 0, 0xD4, 124), // VFP
        CONNECTOR_CMD_SEQUENCE(0x15, 0, 0xD5, 20), // HBP[7:0]
        CONNECTOR_CMD_SEQUENCE(0x15, 0, 0xD6, 120), // HFP[7:0]
        CONNECTOR_CMD_SEQUENCE(0x15, 0, 0xD7, 0), // b0000'00 HBP[8] HFP[8]
        CONNECTOR_CMD_SEQUENCE(0x15, 120, 0x11, 0x00), //
        CONNECTOR_CMD_SEQUENCE(0x15, 10, 0x29, 0x00), //
    };

    connector_send_cmd(init_sequence, sizeof(init_sequence), K_FALSE);
}

static void nt35532_power_reset(k_s32 on)
{
    k_u8 rst_gpio;

    rst_gpio = CONFIG_MPP_DSI_LCD_RESET_PIN;

    // rt_kprintf("rst_gpio is %d \n",rst_gpio);

    kd_pin_mode(rst_gpio, GPIO_DM_OUTPUT);

    if (on)
        kd_pin_write(rst_gpio, GPIO_PV_HIGH); // GPIO_PV_LOW  GPIO_PV_HIGH
    else
        kd_pin_write(rst_gpio, GPIO_PV_LOW); // GPIO_PV_LOW  GPIO_PV_HIGH
}

static void nt35532_set_backlight(k_s32 on)
{
    k_u8 backlight_gpio;

    if (CONFIG_MPP_DSI_LCD_BACKLIGHT_PIN == 255) // unused
        return;

    backlight_gpio = CONFIG_MPP_DSI_LCD_BACKLIGHT_PIN;

    // rt_kprintf("backlight_gpio is %d \n",backlight_gpio);

    kd_pin_mode(backlight_gpio, GPIO_DM_OUTPUT);
    if (on)
        kd_pin_write(backlight_gpio, GPIO_PV_HIGH);
    else
        kd_pin_write(backlight_gpio, GPIO_PV_LOW);
}

static k_s32 nt35532_power_on(void* ctx, k_s32 on)
{
    k_s32                        ret = 0;
    struct connector_driver_dev* dev = ctx;
    // rst vo;
    k230_display_rst();

    if (on) {
        // rst nt35516
        nt35532_power_reset(1);
        rt_thread_mdelay(50);
        nt35532_power_reset(0);
        rt_thread_mdelay(50);
        nt35532_power_reset(1);

        rt_thread_mdelay(120);
        // enable backlight
        nt35532_set_backlight(1);
    } else {
        nt35532_set_backlight(0);
    }

    return ret;
}

static k_s32 nt35532_set_phy_freq(k_connectori_phy_attr* phy_attr)
{
    k_vo_mipi_phy_attr mipi_phy_attr;

    memset(&mipi_phy_attr, 0, sizeof(k_vo_mipi_phy_attr));

    mipi_phy_attr.m           = phy_attr->m;
    mipi_phy_attr.n           = phy_attr->n;
    mipi_phy_attr.hs_freq     = phy_attr->hs_freq;
    mipi_phy_attr.voc         = phy_attr->voc;
    mipi_phy_attr.phy_lan_num = K_DSI_2LAN;
    connector_set_phy_freq(&mipi_phy_attr);
    // rt_kprintf("config phy success phy_attr->m is %d  phy_attr->n is %d phy_attr->hs_freq is %d phy_attr->voc is %d \n",
    // phy_attr->m,  phy_attr->n, phy_attr->hs_freq, phy_attr->voc);
    return 0;
}

static k_s32 nt35532_dsi_resolution_init(k_connector_info* info)
{
    k_vo_dsi_attr           attr;
    k_vo_display_resolution resolution;

    memset(&attr, 0, sizeof(k_vo_dsi_attr));
    attr.lan_num   = info->lan_num;
    attr.cmd_mode  = info->cmd_mode;
    attr.lp_div    = 8;
    attr.work_mode = info->work_mode;
    memcpy(&resolution, &info->resolution, sizeof(k_vo_display_resolution));
    memcpy(&attr.resolution, &resolution, sizeof(k_vo_display_resolution));
    connector_set_dsi_attr(&attr);

    lcd_init();

    connector_set_dsi_enable(1);

    if (info->dsi_test_mode == 1)
        connector_set_dsi_test_mode();

    return 0;
}

static k_s32 nt35532_vo_resolution_init(k_vo_display_resolution* resolution, k_u32 bg_color, k_u32 intr_line)
{
    k_vo_display_resolution vo_resolution;
    k_vo_pub_attr           attr;

    memset(&attr, 0, sizeof(k_vo_pub_attr));
    attr.bg_color  = bg_color;
    attr.intf_sync = K_VO_OUT_1080P30;
    attr.intf_type = K_VO_INTF_MIPI;
    attr.sync_info = resolution;

    connector_set_vo_init();
    connector_set_vtth_intr(1, intr_line);
    connector_set_vo_param(&attr);
    connector_set_vo_enable();

    return 0;
}

static k_s32 nt35532_init(void* ctx, k_connector_info* info)
{
    k_s32                        ret = 0;
    struct connector_driver_dev* dev = ctx;

    if (info->pixclk_div != 0)
        connector_set_pixclk(info->pixclk_div);

    ret |= nt35532_set_phy_freq(&info->phy_attr);
    ret |= nt35532_dsi_resolution_init(info);
    ret |= nt35532_vo_resolution_init(&info->resolution, info->bg_color, info->intr_line);

    return ret;
}

static k_s32 nt35532_get_chip_id(void* ctx, k_u32* chip_id)
{
    k_s32 ret = 0;

    return ret;
}

static k_s32 nt35532_conn_check(void* ctx, k_s32* conn)
{
    k_s32 ret = 0;

    *conn = 1;

    return ret;
}

struct connector_driver_dev nt35532_connector_drv = {
    .connector_name = "nt35532",
    .connector_func = {
        .connector_power = nt35532_power_on,
        .connector_init = nt35532_init,
        .connector_get_chip_id = nt35532_get_chip_id,
        .connector_conn_check = nt35532_conn_check,
    },
};

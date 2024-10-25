// sensor ae info
static k_sensor_ae_info sensor_csi2_ae_info[] = {
    // 1080P30 
     {
        .frame_length = 0x486,
        .cur_frame_length = 0x486,
        .one_line_exp_time = 0.000014,
        .gain_accuracy = 1024,
        .min_gain = 1,
        .max_gain = 28.547,
        .int_time_delay_frame = 2,
        .gain_delay_frame = 2,
        .color_type = SENSOR_MONO,
        .integration_time_increment = 0.000014,
        .gain_increment = SC132GS_MIN_GAIN_STEP,
        .max_integraion_line = 0x486 - 8,
        .min_integraion_line = 1,
        .max_integraion_time = 0.000014 * (0x486 - 8),
        .min_integraion_time = 0.000014 * 1,
        .cur_integration_time = 0.0,
        .cur_again = 1.0,
        .cur_dgain = 1.0,
        .a_gain = {
            .min = 1.0,
            .max = 15.5,
            .step = (1.0f/128.0f),
        },
        .d_gain = {
            .min = 1.0,
            .max = 15.5,
            .step = (1.0f/1024.0f),
        },
        .cur_fps = 30,
    },
    {
        .frame_length = 0x486,
        .cur_frame_length = 0x486,
        .one_line_exp_time = 0.000014,
        .gain_accuracy = 1024,
        .min_gain = 1,
        .max_gain = 28.547,
        .int_time_delay_frame = 4,
        .gain_delay_frame = 4,
        .color_type = SENSOR_MONO,
        .integration_time_increment = 0.000014,
        .gain_increment = SC132GS_MIN_GAIN_STEP,
        .max_integraion_line = 0x486 - 8,
        .min_integraion_line = 1,
        .max_integraion_time = 0.000014 * (0x486 - 8),
        .min_integraion_time = 0.000014 * 1,
        .cur_integration_time = 0.0,
        .cur_again = 1.0,
        .cur_dgain = 1.0,
        .a_gain = {
            .min = 1.0,
            .max = 15.5,
            .step = (1.0f/128.0f),
        },
        .d_gain = {
            .min = 1.0,
            .max = 15.5,
            .step = (1.0f/1024.0f),
        },
        .cur_fps = 30,
    },
};

static const k_sensor_mode sensor_csi2_mode_list[] = {
    {
        .index = 0,
        .sensor_type = SC132GS_MIPI_CSI2_1080X1200_30FPS_10BIT_LINEAR,
        .size = {
            .bounds_width = 1080,
            .bounds_height = 1280,
            .top = 0,
            .left = 0,
            .width = 1080,
            .height = 1280,
        },
        .fps = 30000,
        .hdr_mode = SENSOR_MODE_LINEAR,
        .bit_width = 10,
        .bayer_pattern = BAYER_PAT_BGGR,//BAYER_PAT_GRBG, //BAYER_PAT_RGGB,
        .mipi_info = {
            .csi_id = 0,
            .mipi_lanes = 4,
            .data_type = 0x2B,
        },
        .reg_list = sc132gs_mipi_2lane_1080x1280_init, //sc132gs_mipi_2lane_640x480_init, // sc132gs_mipi_2lane_1080x1280_init, 
#if defined (MPP_ENABLE_SENSOR_SC132GS_ON_CSI0_USE_CHIP_CLK)
        .mclk_setting = {
            {
                .mclk_setting_en = K_TRUE,
                .setting.id = SENSOR_MCLK2,
                .setting.mclk_sel = SENSOR_PLL1_CLK_DIV4,
                .setting.mclk_div = 25,	// 594/25 = 23.76MHz
            },
            {K_FALSE},
            {K_FALSE},
        },
        .sensor_ae_info = &sensor_csi2_ae_info[0],
#else
        .mclk_setting = {
            {K_FALSE},
            {K_FALSE},
            {K_FALSE},
        },
        .sensor_ae_info = &sensor_csi2_ae_info[0],
#endif
    },

    {
        .index = 1,
        .sensor_type = SC132GS_MIPI_CSI2_640X480_30FPS_10BIT_LINEAR,
        .size = {
            .bounds_width = 640,
            .bounds_height = 480,
            .top = 0,
            .left = 0,
            .width = 640,
            .height = 480,
        },
        .fps = 30000,
        .hdr_mode = SENSOR_MODE_LINEAR,
        .bit_width = 10,
        .bayer_pattern = BAYER_PAT_BGGR,//BAYER_PAT_GRBG, //BAYER_PAT_RGGB,
        .mipi_info = {
            .csi_id = 0,
            .mipi_lanes = 4,
            .data_type = 0x2B,
        },
        .reg_list = sc132gs_mipi_2lane_640x480_init, 
#if defined (MPP_ENABLE_SENSOR_SC132GS_ON_CSI0_USE_CHIP_CLK)
        .mclk_setting = {
            {
                .mclk_setting_en = K_TRUE,
                .setting.id = SENSOR_MCLK2,
                .setting.mclk_sel = SENSOR_PLL1_CLK_DIV4,
                .setting.mclk_div = 24,	// 594/25 = 23.76MHz
            },
            {K_FALSE},
            {K_FALSE},
        },
        .sensor_ae_info = &sensor_csi2_ae_info[1],
#else
        .mclk_setting = {
            {K_FALSE},
            {K_FALSE},
            {K_FALSE},
        },
        .sensor_ae_info = &sensor_csi2_ae_info[1],
#endif
    },
};

#if defined (MPP_ENABLE_SENSOR_SC132GS_ON_CSI2_USE_CHIP_CLK)
_Static_assert(CONFIG_MPP_CSI_DEV2_MCLK_NUM >= 1 && (CONFIG_MPP_CSI_DEV2_MCLK_NUM <= 3), "Invalid CONFIG_MPP_CSI_DEV2_MCLK_NUM");
#endif

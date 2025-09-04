#include "wifi_board.h"
#include "codecs/box_audio_codec.h"
#include "display/lcd_display.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "i2c_device.h"  // IMU需要I2C设备
#include "qmi8658.h"     // IMU传感器
#include "led/single_led.h"  // RGB LED支持
#include "uart_433.h"    // 433MHz UART通信


#include <esp_log.h>
#include <esp_lcd_panel_vendor.h>
#include <driver/i2c_master.h>
#include <driver/spi_common.h>
#include <esp_adc/adc_oneshot.h>
#include <wifi_station.h>
// #include <esp_lcd_touch_ft5x06.h>  // 已移除触摸屏支持
#include <esp_lvgl_port.h>
#include <lvgl.h>


#define TAG "LichuangDevBoard"

// 前向声明SetPowerOffset函数
void SetPowerOffset(bool a);

LV_FONT_DECLARE(font_puhui_20_4);
LV_FONT_DECLARE(font_awesome_20_4);



class CustomAudioCodec : public BoxAudioCodec {
public:
    CustomAudioCodec(i2c_master_bus_handle_t i2c_bus)
        : BoxAudioCodec(i2c_bus,
                       AUDIO_INPUT_SAMPLE_RATE,
                       AUDIO_OUTPUT_SAMPLE_RATE,
                       AUDIO_I2S_GPIO_MCLK,
                       AUDIO_I2S_GPIO_BCLK,
                       AUDIO_I2S_GPIO_WS,
                       AUDIO_I2S_GPIO_DOUT,
                       AUDIO_I2S_GPIO_DIN,
                       GPIO_NUM_NC,
                       AUDIO_CODEC_ES8311_ADDR,
                       AUDIO_CODEC_ES7210_ADDR,
                       AUDIO_INPUT_REFERENCE) {
    }

    virtual void EnableOutput(bool enable) override {
        BoxAudioCodec::EnableOutput(enable);

        // 功放控制逻辑（参考旧版本实现）
#ifdef AMP_ENABLE_GPIO
        if (AMP_ENABLE_GPIO != GPIO_NUM_NC) {
            static bool s_gpio_init = false;
            if (!s_gpio_init) {
                gpio_config_t io_conf = {};
                io_conf.intr_type = GPIO_INTR_DISABLE;
                io_conf.mode = GPIO_MODE_OUTPUT;
                io_conf.pin_bit_mask = (1ULL << AMP_ENABLE_GPIO);
                io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
                io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
                ESP_ERROR_CHECK(gpio_config(&io_conf));
                s_gpio_init = true;
                ESP_LOGI("CustomAudioCodec", "Initialized AMP enable GPIO: %d", AMP_ENABLE_GPIO);
            }
#ifdef AMP_ENABLE_ACTIVE_HIGH
            gpio_set_level(AMP_ENABLE_GPIO, enable ? 1 : 0);
#else
            gpio_set_level(AMP_ENABLE_GPIO, enable ? 0 : 1);
#endif
            ESP_LOGI("CustomAudioCodec", "AMP %s (GPIO %d = %d)",
                     enable ? "enabled" : "disabled",
                     AMP_ENABLE_GPIO,
                     enable ? (AMP_ENABLE_ACTIVE_HIGH ? 1 : 0) : (AMP_ENABLE_ACTIVE_HIGH ? 0 : 1));
        }
#endif
    }
};

class LichuangDevBoard : public WifiBoard {
private:
    i2c_master_bus_handle_t i2c_bus_;
    Button boot_button_;   // GPIO0标准BOOT按键
    Button power_button_;  // GPIO47电源按键 (对应旧项目的BOOT_BUTTON_GPIO)
    LcdDisplay* display_;
    QMI8658* imu_sensor_;
    adc_oneshot_unit_handle_t adc_handle_;  // ADC句柄用于电压检测

    // 三连击检测变量 (与旧项目完全一致)
    int power_click_count_ = 0;
    int64_t last_click_time_ = 0;
    static const int CLICK_TIMEOUT_MS = 1000; // 1秒内需要完成3次点击

    // GPIO47测试任务相关
    TaskHandle_t gpio47_test_task_ = nullptr;

    void InitializeAdc() {
        // 初始化ADC用于电压检测（参考参考项目实现）
        adc_oneshot_unit_init_cfg_t init_config = {};
        init_config.unit_id = ADC_UNIT_1;
        ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle_));

        // 配置ADC通道
        adc_oneshot_chan_cfg_t config = {};
        config.bitwidth = ADC_BITWIDTH_DEFAULT;
        config.atten = ADC_ATTEN_DB_12;  // 支持0-3.3V电压范围

        // 配置ADC_CHANNEL_8和ADC_CHANNEL_9（对应GPIO8和GPIO9）
        adc_oneshot_config_channel(adc_handle_, ADC_CHANNEL_8, &config);
        adc_oneshot_config_channel(adc_handle_, ADC_CHANNEL_9, &config);

        ESP_LOGI(TAG, "ADC initialized for battery voltage detection (CH8=ref, CH9=battery)");
    }

    void InitializeI2c() {
        // Initialize I2C peripheral
        i2c_master_bus_config_t i2c_bus_cfg = {
            .i2c_port = (i2c_port_t)1,
            .sda_io_num = AUDIO_CODEC_I2C_SDA_PIN,
            .scl_io_num = AUDIO_CODEC_I2C_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {
                .enable_internal_pullup = 1,
            },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus_));

        // 初始化IMU传感器
        imu_sensor_ = new QMI8658(i2c_bus_, IMU_I2C_ADDR);
        if (!imu_sensor_->Initialize()) {
            ESP_LOGW(TAG, "Failed to initialize IMU sensor, continuing without IMU");
        }
    }

    void InitializeSpi() {
        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = GPIO_NUM_40;
        buscfg.miso_io_num = GPIO_NUM_NC;
        buscfg.sclk_io_num = GPIO_NUM_41;
        buscfg.quadwp_io_num = GPIO_NUM_NC;
        buscfg.quadhd_io_num = GPIO_NUM_NC;
        buscfg.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }

    void InitializeButtons() {
        // GPIO0 BOOT按键：保持原有功能
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting && !WifiStation::GetInstance().IsConnected()) {
                ResetWifiConfiguration();
            }
            app.ToggleChatState();
        });

        // GPIO47电源按键：完全按照旧项目实现三连击配网功能
        power_button_.OnClick([this]() {
            int64_t current_time = esp_timer_get_time() / 1000;  // 转换为毫秒

            // 如果距离上次点击超过1秒，重置计数器
            if (current_time - last_click_time_ > CLICK_TIMEOUT_MS) {
                power_click_count_ = 0;
            }

            power_click_count_++;
            last_click_time_ = current_time;

            ESP_LOGI(TAG, "Power button click count: %d", power_click_count_);

            // 检查是否在1秒内完成了3次点击
            if (power_click_count_ >= 3) {
                power_click_count_ = 0;     // 重置计数器
                ESP_LOGI(TAG, "Power button triple click detected - entering WiFi config mode");
                ResetWifiConfiguration();  // 直接进入配网模式，不检查设备状态
            }
        });

        // GPIO47电源按键：完全按照旧项目实现长按关机功能
        power_button_.OnLongPress([this]() {
            ESP_LOGI(TAG, "Power button long press detected - powering off");
            SetPowerOffset(false);  // 关机
        });

        // GPIO47电平检测测试任务
        //StartGpio47TestTask();
    }

    void StartGpio47TestTask() {
        ESP_LOGI(TAG, "Starting GPIO47 test task...");

        xTaskCreate([](void* param) {
            LichuangDevBoard* board = static_cast<LichuangDevBoard*>(param);
            board->Gpio47TestTask();
        }, "gpio47_test", 2048, this, 5, &gpio47_test_task_);
    }

    void Gpio47TestTask() {
        ESP_LOGI(TAG, "GPIO47 test task started - monitoring button level");
        ESP_LOGI(TAG, "Hardware: 下拉电阻设计，按键按下=高电平(1)，松开=低电平(0)");

        int last_level = -1;
        int press_count = 0;

        while (true) {
            int current_level = gpio_get_level(GPIO_NUM_47);

            if (current_level != last_level) {
                if (current_level == 1) {
                    // 下拉电阻设计：高电平 = 按键按下
                    press_count++;
                    ESP_LOGI(TAG, "🔴 BUTTON PRESSED (Level=1) - Press count: %d", press_count);
                } else {
                    // 下拉电阻设计：低电平 = 按键松开
                    ESP_LOGI(TAG, "🟢 BUTTON RELEASED (Level=0)");
                }
                last_level = current_level;
            }

            vTaskDelay(pdMS_TO_TICKS(50)); // 50ms检测间隔
        }
    }



    void InitializeSt7789Display() {
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;
        // 液晶屏控制IO初始化
        ESP_LOGD(TAG, "Install panel IO");
        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = GPIO_NUM_NC;
        io_config.dc_gpio_num = GPIO_NUM_39;
        io_config.spi_mode = 2;
        io_config.pclk_hz = 80 * 1000 * 1000;
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI3_HOST, &io_config, &panel_io));

        // 初始化液晶屏驱动芯片ST7789
        ESP_LOGD(TAG, "Install LCD driver");
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = GPIO_NUM_NC;
        panel_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
        panel_config.bits_per_pixel = 16;
        ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(panel_io, &panel_config, &panel));
        
        esp_lcd_panel_reset(panel);
        esp_lcd_panel_init(panel);
        esp_lcd_panel_invert_color(panel, true);
        esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY);
        esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);
        display_ = new SpiLcdDisplay(panel_io, panel,
                                    DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY,
                                    {
                                        .text_font = &font_puhui_20_4,
                                        .icon_font = &font_awesome_20_4,
#if CONFIG_USE_WECHAT_MESSAGE_STYLE
                                        .emoji_font = font_emoji_32_init(),
#else
                                        .emoji_font = font_emoji_64_init(),
#endif
                                    });
    }

    // 已移除触摸屏支持 - InitializeTouch()



public:
    LichuangDevBoard() : boot_button_(BOOT_BUTTON_GPIO),
                        power_button_(POWER_BUTTON_GPIO, true, 3000, 50, false) {
        // GPIO0: 标准BOOT按键，使用默认配置
        // GPIO47: 电源按键，下拉电阻设计，按键按下=高电平，active_high=true
        InitializeAdc();
        InitializeI2c();
        InitializeSpi();
        InitializeSt7789Display();

        // GPIO4电源控制已在Button构造函数中初始化 (与旧项目完全一致)

        InitializeButtons();

        // 初始化433MHz UART通信
#if UART_433_ENABLE
        UART_433_Init();
        ESP_LOGI(TAG, "433MHz UART initialized");
#endif

        GetBacklight()->RestoreBrightness();

        ESP_LOGI(TAG, "LichuangDevBoard initialized with power management (GPIO47 button, GPIO4 power control)");
    }



    virtual AudioCodec* GetAudioCodec() override {
        static CustomAudioCodec audio_codec(i2c_bus_);
        return &audio_codec;
    }

    virtual Display* GetDisplay() override {
        return display_;
    }
    
    virtual Backlight* GetBacklight() override {
        static PwmBacklight backlight(DISPLAY_BACKLIGHT_PIN, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
        return &backlight;
    }

    virtual I2cDevice* GetI2cDevice() override {
        return imu_sensor_;  // QMI8658继承自I2cDevice
    }

    virtual QMI8658* GetIMUSensor() override {
        return imu_sensor_;
    }

    virtual Led* GetLed() override {
        static SingleLed led(RGB_LED_GPIO);  // 使用GPIO 11的RGB LED
        return &led;
    }

    // 电压检测函数（参考参考项目实现）
    double GetBattary() {
        int adc_raw_value = 0;
        int ref = 0;

        // 读取参考电压（ADC_CHANNEL_8对应GPIO8）
        adc_oneshot_read(adc_handle_, ADC_CHANNEL_8, &ref);

        // 读取电池电压（ADC_CHANNEL_9对应GPIO9）
        adc_oneshot_read(adc_handle_, ADC_CHANNEL_9, &adc_raw_value);

        // 计算实际电压值（参考参考项目的计算公式）
        double voltage = 4.98f * adc_raw_value / ref;

        return voltage;
    }

};

DECLARE_BOARD(LichuangDevBoard);

// lichuang-dev电源控制函数 (与旧项目一致但缺少led控制)
void SetPowerOffset(bool a) {
    // 电源控制逻辑:
    // true(1) = 维持供电 (正常运行状态)
    // false(0) = 切断供电 (系统关机，需按键重新开机)
    gpio_set_level(GPIO_NUM_4, a);
    ESP_LOGI("SetPowerOffset", "Power control GPIO4 set to %s", a ? "HIGH (maintain power)" : "LOW (power off)");
}

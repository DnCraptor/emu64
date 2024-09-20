#include <pico/time.h>
#include <pico/multicore.h>
#include <hardware/pwm.h>
#include "hardware/clocks.h"
#include <pico/stdlib.h>
#include <hardware/vreg.h>
#include <pico/stdio.h>

#include "psram_spi.h"
#include "nespad.h"

extern "C" {
#include "vga.h"
#include "ps2.h"
#include "usb.h"
#include "ram_page.h"
}
#if I2S_SOUND
#include "audio.h"
#endif

bool SD_CARD_AVAILABLE = false;
bool runing = true;
static int16_t last_dss_sample = 0;
#if PICO_ON_DEVICE
bool PSRAM_AVAILABLE = false;
uint32_t DIRECT_RAM_BORDER = PSRAM_AVAILABLE ? RAM_SIZE : (SD_CARD_AVAILABLE ? RAM_PAGE_SIZE : RAM_SIZE);
extern "C" uint8_t VIDEORAM[VIDEORAM_SIZE];

#ifdef I2S_SOUND
i2s_config_t i2s_config = i2s_get_default_config();
static int16_t samples[2][441*2] = { 0 };
static int active_buffer = 0;
static int sample_index = 0;
#else
pwm_config config = pwm_get_default_config();
#define PWM_PIN0 (26)
#define PWM_PIN1 (27)
#endif


uint16_t true_covox = 0;

struct semaphore vga_start_semaphore;
/* Renderer loop on Pico's second core */
void __scratch_y("second_core") second_core() {
#ifdef SOUND_ENABLED
#ifdef I2S_SOUND
    i2s_config.sample_freq = SOUND_FREQUENCY;
    i2s_config.dma_trans_count = SOUND_FREQUENCY / 100;
    i2s_volume(&i2s_config, 0);
    i2s_init(&i2s_config);
    sleep_ms(100);
#else
    gpio_set_function(BEEPER_PIN, GPIO_FUNC_PWM);
    pwm_init(pwm_gpio_to_slice_num(BEEPER_PIN), &config, true);

    gpio_set_function(PWM_PIN0, GPIO_FUNC_PWM);
    gpio_set_function(PWM_PIN1, GPIO_FUNC_PWM);

    pwm_config_set_clkdiv(&config, 1.0);
    pwm_config_set_wrap(&config, (1 << 12) - 1); // MAX PWM value

    pwm_init(pwm_gpio_to_slice_num(PWM_PIN0), &config, true);
    pwm_init(pwm_gpio_to_slice_num(PWM_PIN1), &config, true);
#endif
#endif
    graphics_init();
    graphics_set_buffer(VIDEORAM, 320, 200);
    graphics_set_textbuffer(VIDEORAM + 32768);
    memset(VIDEORAM, 0b1010101, VIDEORAM_SIZE);
    graphics_set_bgcolor(0);
    graphics_set_offset(0, 0);
    graphics_set_flashmode(true, true);

    sem_acquire_blocking(&vga_start_semaphore);

    uint64_t tick = time_us_64();
    uint64_t last_timer_tick = tick, last_cursor_blink = tick, last_sound_tick = tick, last_dss_tick = tick;

    while (true) {
        if (tick >= last_cursor_blink + 500000) {
///            cursor_blink_state ^= 1;
            last_cursor_blink = tick;
        }

#ifdef SOUND_ENABLED
        // Sound frequency 44100
        if (tick >= last_sound_tick + (1000000 / SOUND_FREQUENCY)) {
            int16_t sample = 0;
#ifdef I2S_SOUND
            if (speakerenabled) {
                sample += speaker_sample();
            }

            samples[active_buffer][sample_index * 2] = sample;
            samples[active_buffer][sample_index * 2 + 1] = sample;



            if (sample_index++ >= i2s_config.dma_trans_count) {
                sample_index = 0;
                i2s_dma_write(&i2s_config, samples[active_buffer]);
                active_buffer ^= 1;
            }
#else
            int16_t samples[2] = { sample, sample };
            // register uint8_t r_divider = snd_divider + 4; // TODO: tume up divider per channel
            uint16_t corrected_sample_l = (uint16_t)((int32_t)samples[0] + 0x8000L) >> 4;
            uint16_t corrected_sample_r = (uint16_t)((int32_t)samples[1] + 0x8000L) >> 4;
            // register uint8_t l_divider = snd_divider + 4;
            //register uint16_t l_v = (uint16_t)((int32_t)sample + 0x8000L) >> 4;
            pwm_set_gpio_level(PWM_PIN0, corrected_sample_l);
            pwm_set_gpio_level(PWM_PIN1, corrected_sample_r);
#endif
            last_sound_tick = tick;
        }
#endif

        tick = time_us_64();
        tight_loop_contents();
    }
}
#endif

static FATFS fs;

#include "./video_crt_class.h"
#include "./mmu_class.h"
#include "./mos6510_class.h"
#include "./mos6569_class.h"
#include "./mos6581_8085_class.h"
#include "./mos6526_class.h"
#include "./cartridge_class.h"
#include "./reu_class.h"
#include "./georam_class.h"
#include "./floppy1541_class.h"
#include "./tape1530_class.h"
#include "./cpu_info.h"
#include "./vcd_class.h"

#include <functional>

#define C64Takt 985248  // 50,124542Hz (Original C64 PAL)

static MMU _mmu;
static MOS6510 _cpu;
static VICII _vic(VIDEORAM);
static MOS6581_8085 _sid1(0, 44100, 16);
static MOS6581_8085 _sid2(1, 44100, 16);
static MOS6526 _cia1(0);
static MOS6526 _cia2(1);
static CartridgeClass _crt;
static REUClass _reu;
static GEORAMClass _geo;
static TAPE1530 _tape(44100, 16, C64Takt);

int main() {
#if (OVERCLOCKING > 270)
    hw_set_bits(&vreg_and_chip_reset_hw->vreg, VREG_AND_CHIP_RESET_VREG_VSEL_BITS);
    sleep_ms(33);
    set_sys_clock_khz(OVERCLOCKING * 1000, true);
#else
    vreg_set_voltage(VREG_VOLTAGE_1_15);
    sleep_ms(33);
    set_sys_clock_khz(270000, true);
#endif

    //stdio_init_all();

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    for (int i = 0; i < 6; i++) {
        sleep_ms(23);
        gpio_put(PICO_DEFAULT_LED_PIN, true);
        sleep_ms(23);
        gpio_put(PICO_DEFAULT_LED_PIN, false);
    }

    nespad_begin(clock_get_hz(clk_sys) / 1000, NES_GPIO_CLK, NES_GPIO_DATA, NES_GPIO_LAT);
    keyboard_init();

    sem_init(&vga_start_semaphore, 0, 1);
    multicore_launch_core1(second_core);
    sem_release(&vga_start_semaphore);

    sleep_ms(50);

    graphics_set_mode(TEXTMODE_80x30);

    init_psram();

    FRESULT result = f_mount(&fs, "", 1);
    if (FR_OK != result) {
        char tmp[80];
        sprintf(tmp, "Unable to mount SD-card: %d", result);
        logMsg(tmp);
    }
    else {
        SD_CARD_AVAILABLE = true;
    }

    DIRECT_RAM_BORDER = PSRAM_AVAILABLE ? RAM_SIZE : (SD_CARD_AVAILABLE ? RAM_PAGE_SIZE : RAM_SIZE);

    if (!PSRAM_AVAILABLE && !SD_CARD_AVAILABLE) {
        logMsg((char *)"Mo PSRAM or SD CARD available. Only 160Kb RAM will be usable...");
        sleep_ms(3000);
    }

    MMU* mmu = &_mmu;
    MOS6510* cpu = &_cpu;
    VICII* vic = &_vic;
    MOS6581_8085* sid1 = &_sid1;
    MOS6581_8085* sid2 = &_sid2;
    MOS6526* cia1 = &_cia1;
    MOS6526* cia2 = &_cia2;
    CartridgeClass* crt = &_crt;;
    REUClass* reu = &_reu;
    GEORAMClass* geo = &_geo;
    TAPE1530* tape = &_tape;

        logMsg("!!!");
//    reset86();
    while (runing) {
//        if_manager();
//        exec86(2000);
    }
    return 0;
}

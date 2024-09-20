#include <pico/time.h>
#include <pico/multicore.h>
#include <hardware/pwm.h>
#include "hardware/clocks.h"
#include <pico/stdlib.h>
#include <hardware/vreg.h>
#include <pico/stdio.h>

#include "psram_spi.h"
#include "nespad.h"
#include <functional>

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
#define MAX_FLOPPY_NUM 4

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

class C64Class {
    uint8_t c64_iec_wire = 0;       // Leitungen vom C64 zur Floppy Bit 4=ATN 6=CLK 7=DATA
    uint8_t floppy_iec_wire = 0;    // Leitungen von Floppy zur c64 Bit 6=CLK 7=DATA

    bool reset_wire;        // Reset Leitung -> Für Alle Module mit Reset Eingang
    bool rdy_ba_wire;       // Leitung CPU <-- VIC
    bool game_wire;         // Leitung Expansionsport --> MMU;
    bool exrom_wire;        // Leitung Expansionsport --> MMU;
    bool hi_ram_wire;       // Leitung Expansionsport --> MMU;
    bool lo_ram_wire;       // Leitung Expansionsport --> MMU;

    MOS6510_PORT cpu_port;  // Prozessor Port

    bool enable_ext_wires;
    bool ext_rdy_wire;

    /// Temporär ///
    bool        easy_flash_dirty;
    uint8_t     easy_flash_byte;

    PORT cia1_port_a;
    PORT cia1_port_b;
    PORT cia2_port_a;
    PORT cia2_port_b;

    bool        enable_mouse_1351;
    uint16_t    mouse_1351_x_rel;
    uint16_t    mouse_1351_y_rel;

    uint8_t     mouse_port;
    uint8_t     poti_ax;
    uint8_t     poti_ay;
    uint8_t     poti_bx;
    uint8_t     poti_by;

    uint8_t     poti_x;
    uint8_t     poti_y;

    uint8_t     key_matrix_to_port_a[8];
    uint8_t     key_matrix_to_port_b[8];

    bool return_key_is_down;

    bool reu_is_insert;

    uint8_t io_source = 0;
    bool            wait_reset_ready;
    bool enable_stereo_sid = false;
    bool enable_stereo_sid_6channel_mode = false;
    uint16_t stereo_sid_address; /// TODO:

    bool        c64_reset_ready;
    bool        floppy_reset_ready[MAX_FLOPPY_NUM];

    float_t         sid_volume;

    int             c64_frequency;   // Normaler PAL Takt ist 985248 Hz (985248 / (312 Rz * 63 Cycles) = 50,124542 Hz)
                                     // 50 Hz Syncroner Takt ist 982800 Hz

	int				c64_frequency_temp;	// Dient zum zwischenspeichern der C64 Frequenz beim Videorecord.

    int             c64_speed;

    std::function<uint8_t(uint16_t)> *ReadProcTbl;
    std::function<void(uint16_t, uint8_t)> *WriteProcTbl;

    uint32_t    cycle_counter;
	int         limit_cycles_counter;        // Dieser Counter wird wenn er > 0 ist bei jeden Zyklus um 1 runtergezählt
	bool		hold_next_system_cycle;		 // Wird dieses Flag gesetzt wird verhindert das ein C64 Cylce ausgeführt wird

	bool        debug_mode;
    bool        debug_animation;
    float_t     animation_speed_add;
    float_t     animation_speed_counter;
    bool        one_cycle;
    bool        one_opcode;
    int         one_opcode_source;
    bool        cpu_states[5];              // true = Feetch / false = no Feetch ::: Index 0=C64 Cpu, 1-4 Floppy 1-4

    FILE*       debug_logging_file;
    bool        debug_logging;

    bool        warp_mode;

    bool        mouse_is_hidden;
    int         mouse_hide_counter;
    int         mouse_hide_time;

    bool        key_map_is_rec;
    uint8_t     rec_matrix_code;

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
public:
    C64Class(void) {
    return_key_is_down = false;

    enable_mouse_1351 = false;
    mouse_1351_x_rel = mouse_1351_y_rel = 0;

    mouse_port = 0;  // Port1 = 0 ... Port2 = 1
    poti_ax = poti_ay = poti_bx = poti_by = 0xFF;    // HighZ zum Beginn (Keine Paddles / Maus angeschlossen)
    poti_x = poti_y = 0xFF;
    enable_ext_wires = false;

    /// Init Vars ///
    c64_frequency = C64Takt;
    c64_speed = 100;
    io_source = 0;
    cia2->floppy_iec_wire = &floppy_iec_wire;
    cia2->c64_iec_wire = &c64_iec_wire;

    Floppy1541 *floppy[MAX_FLOPPY_NUM];
    for(int i = 0; i < MAX_FLOPPY_NUM; ++i)
    {
        /*** TODO:
        floppy[i] = new Floppy1541(&reset_wire,audio_frequency,16,&floppy_found_breakpoint);
        floppy[i]->SetResetReady(&floppy_reset_ready[i],0xEBFF);
        floppy[i]->SetC64IEC(&c64_iec_wire);
        floppy[i]->SetDeviceNumber(static_cast<uint8_t>(8+i));
        floppy[i]->LoadDosRom(filename);
        //floppy[i]->LoadFloppySounds((char*)"floppy_sounds/motor.raw",(char*)"floppy_sounds/motor_on.raw",(char*)"floppy_sounds/motor_off.raw",(char*)"floppy_sounds/anschlag.raw",(char*)"floppy_sounds/stepper_inc.raw",(char*)"floppy_sounds/stepper_dec.raw");
        floppy[i]->LoadFloppySounds(motor_filename,motor_on_filename,motor_off_filename,bumper_filename,stepper_inc_filename,stepper_dec_filename);
        floppy[i]->SetEnableFloppy(false);
        floppy[i]->SetEnableFloppySound(true);
        */
    }

    /// Callbackroutinen setzen ///
    ReadProcTbl = mmu->CPUReadProcTbl;
    WriteProcTbl = mmu->CPUWriteProcTbl;
    cpu->ReadProcTbl = mmu->CPUReadProcTbl;
    cpu->WriteProcTbl = mmu->CPUWriteProcTbl;
    vic->ReadProcTbl = mmu->VICReadProcTbl;
    vic->RefreshProc = std::bind(&C64Class::VicRefresh,this,std::placeholders::_1);
    reu->ReadProcTbl = mmu->CPUReadProcTbl;
    reu->WriteProcTbl = mmu->CPUWriteProcTbl;


    mmu->VicIOWriteProc = std::bind(&VICII::WriteIO,vic,std::placeholders::_1,std::placeholders::_2);
    mmu->VicIOReadProc = std::bind(&VICII::ReadIO,vic,std::placeholders::_1);
    mmu->SidIOWriteProc = std::bind(&C64Class::WriteSidIO,this,std::placeholders::_1,std::placeholders::_2);
    mmu->SidIOReadProc = std::bind(&C64Class::ReadSidIO,this,std::placeholders::_1);
    mmu->Cia1IOWriteProc = std::bind(&MOS6526::WriteIO,cia1,std::placeholders::_1,std::placeholders::_2);
    mmu->Cia1IOReadProc = std::bind(&MOS6526::ReadIO,cia1,std::placeholders::_1);
    mmu->Cia2IOWriteProc = std::bind(&MOS6526::WriteIO,cia2,std::placeholders::_1,std::placeholders::_2);
    mmu->Cia2IOReadProc = std::bind(&MOS6526::ReadIO,cia2,std::placeholders::_1);

    mmu->CRTRom1WriteProc = std::bind(&CartridgeClass::WriteRom1,crt,std::placeholders::_1,std::placeholders::_2);
    mmu->CRTRom2WriteProc = std::bind(&CartridgeClass::WriteRom2,crt,std::placeholders::_1,std::placeholders::_2);
    mmu->CRTRom3WriteProc = std::bind(&CartridgeClass::WriteRom3,crt,std::placeholders::_1,std::placeholders::_2);
    mmu->CRTRom1ReadProc = std::bind(&CartridgeClass::ReadRom1,crt,std::placeholders::_1);
    mmu->CRTRom2ReadProc = std::bind(&CartridgeClass::ReadRom2,crt,std::placeholders::_1);
    mmu->CRTRom3ReadProc = std::bind(&CartridgeClass::ReadRom3,crt,std::placeholders::_1);
    mmu->IO1ReadProc = std::bind(&C64Class::ReadIO1,this,std::placeholders::_1);
    mmu->IO1WriteProc = std::bind(&C64Class::WriteIO1,this,std::placeholders::_1,std::placeholders::_2);
    mmu->IO2ReadProc = std::bind(&C64Class::ReadIO2,this,std::placeholders::_1);
    mmu->IO2WriteProc = std::bind(&C64Class::WriteIO2,this,std::placeholders::_1,std::placeholders::_2);

    crt->ChangeMemMapProc = std::bind(&MMU::ChangeMemMap,mmu);

    /// Module mit Virtuellen Leitungen verbinden
    mmu->GAME = &game_wire;
    mmu->EXROM = &exrom_wire;
    mmu->RAM_H = &hi_ram_wire;
    mmu->RAM_L = &lo_ram_wire;
    mmu->CPU_PORT = &cpu_port;
    crt->exrom = &exrom_wire;
    crt->game = &game_wire;
    crt->CpuTriggerInterrupt = std::bind(&MOS6510::TriggerInterrupt,cpu,std::placeholders::_1);
    crt->CpuClearInterrupt = std::bind(&MOS6510::ClearInterrupt,cpu,std::placeholders::_1);
    cpu->RDY = &rdy_ba_wire;
    cpu->RESET = &reset_wire;
    cpu->ResetReady = &c64_reset_ready;
    cpu->ResetReadyAdr = 0xE5CD;
    cpu->EnableExtInterrupts = enable_ext_wires;
    cia1->reset_wire = &reset_wire;
    cia1->CpuTriggerInterrupt = std::bind(&MOS6510::TriggerInterrupt,cpu,std::placeholders::_1);
    cia1->CpuClearInterrupt = std::bind(&MOS6510::ClearInterrupt,cpu,std::placeholders::_1);
    cia1->VicTriggerLP = std::bind(&VICII::TriggerLightpen,vic);
    cia1->ChangePOTSwitch = std::bind(&C64Class::ChangePOTSwitch,this);
    cia1->pa = &cia1_port_a;
    cia1->pb = &cia1_port_b;
    cia2->reset_wire = &reset_wire;
    cia2->CpuTriggerInterrupt = std::bind(&MOS6510::TriggerInterrupt,cpu,std::placeholders::_1);
    cia2->CpuClearInterrupt = std::bind(&MOS6510::ClearInterrupt,cpu,std::placeholders::_1);
    cia2->pa = &cia2_port_a;
    cia2->pb = &cia2_port_b;
    vic->ba = &rdy_ba_wire;
    vic->CpuTriggerInterrupt = std::bind(&MOS6510::TriggerInterrupt,cpu,std::placeholders::_1);
    vic->CpuClearInterrupt = std::bind(&MOS6510::ClearInterrupt,cpu,std::placeholders::_1);
    vic->color_ram = mmu->GetFarbramPointer();
    vic->cia2_port_a = cia2_port_a.GetOutputBitsPointer();
    sid1->RESET = &reset_wire;
    sid2->RESET = &reset_wire;
    reu->BA = &rdy_ba_wire;
    reu->CpuTriggerInterrupt = std::bind(&MOS6510::TriggerInterrupt,cpu,std::placeholders::_1);
    reu->CpuClearInterrupt = std::bind(&MOS6510::ClearInterrupt,cpu,std::placeholders::_1);
    reu->RESET = &reset_wire;
    reu->WRITE_FF00 = &cpu->WRITE_FF00;
    reu_is_insert = false;

    /// Tape mit C64 verbinden ///
    tape->CPU_PORT = &cpu_port;
    cia1->flag_pin = &tape->CassRead;

    /// CRT mit MMU verbinden ///
    crt->c64_ram = mmu->GetRAMPointer();
    mmu->EasyFlashDirty1 = crt->GetFlash040Dirty(0);
    mmu->EasyFlashDirty2 = crt->GetFlash040Dirty(1);
    mmu->EasyFlashByte1 = crt->GetFlash040Byte(0);
    mmu->EasyFlashByte2 = crt->GetFlash040Byte(1);

    mmu->EasyFlashDirty1 = &easy_flash_dirty;
    mmu->EasyFlashDirty2 = &easy_flash_dirty;
    mmu->EasyFlashByte1 = &easy_flash_byte;
    mmu->EasyFlashByte2 = &easy_flash_byte;

    rdy_ba_wire = true;
    game_wire = true;
    exrom_wire = true;

    wait_reset_ready = false;

    mmu->Reset();
    cia1->Reset();
    cia2->Reset();

    sid_volume = 1.0f;

    sid1->RESET = &reset_wire;
    sid1->SetC64Zyklen(c64_frequency);     // PAL 63*312*50 = 982800
    sid1->SetChipType(MOS_8580);
    sid1->SoundOutputEnable = true;
    sid1->CycleExact = true;
    sid1->FilterOn = true;
    sid1->Reset();
    sid1->SetPotXY(poti_x, poti_y);

    sid2->RESET = &reset_wire;
    sid2->SetC64Zyklen(c64_frequency);     // PAL 63*312*50 = 982800
    sid2->SetChipType(MOS_8580);
    sid2->SoundOutputEnable = true;
    sid2->CycleExact = true;
    sid2->FilterOn = true;
    sid2->Reset();

    enable_stereo_sid = false;
    enable_stereo_sid_6channel_mode = false;
    stereo_sid_address = 0xD420;

    vic->ba = &rdy_ba_wire;
    vic->cia2_port_a = cia2->pa->GetOutputBitsPointer();

    }
    void VicRefresh(uint8_t *vic_puffer) {}
    void WriteSidIO(uint16_t address, uint8_t value) {
        if(enable_stereo_sid)
        {
            if((address & 0xFFE0) == 0xD400) _sid1.WriteIO(address,value);
            if((address & 0xFFE0) == stereo_sid_address) _sid2.WriteIO(address,value);
        }
        else
        {
            _sid1.WriteIO(address,value);
        }
    }
    uint8_t ReadSidIO(uint16_t address) {
        return _sid1.ReadIO(address);
    }
    void WriteIO1(uint16_t address, uint8_t value) {
    switch(io_source)
    {
    case 0: // NO_MODUL
        break;
    case 1:	// CRT
        crt->WriteIO1(address,value);
        break;
    case 2:	// REU
        reu->WriteIO1(address,value);
        break;
    case 3:	// GEORAM
        geo->WriteIO1(address,value);
        break;
    default:
        break;
    }
    }
    void WriteIO2(uint16_t address, uint8_t value) {
    switch(io_source)
    {
    case 0: // NO_MODUL
        break;
    case 1: // CRT
        crt->WriteIO2(address,value);
        break;
    case 2: // REU
        reu->WriteIO2(address,value);
        break;
    case 3: // GEORAM
        geo->WriteIO2(address,value);
        break;
    default:
        break;
    }
    }
    uint8_t ReadIO1(uint16_t address) {
    switch(io_source)
    {
    case 0: // NO_MODUL
        // return 0;
        return vic->last_read_gp_access;
    case 1: // CRT
        return crt->ReadIO1(address);
    case 2: // REU
        return reu->ReadIO1(address);
    case 3: // GEORAM
        return geo->ReadIO1(address);
    default:
        return 0;
    }
    }
    uint8_t ReadIO2(uint16_t address) {
    switch(io_source)
    {
    case 0: // NO_MODUL
        //return 0;
        return vic->last_read_gp_access;
    case 1: // CRT
        return crt->ReadIO2(address);
    case 2: // REU
        return reu->ReadIO2(address);
    case 3: // GEORAM
        return geo->ReadIO2(address);
    default:
        return 0;
    }
    }
    void ChangePOTSwitch() {
    if(!enable_mouse_1351)
    {
        poti_x = 0xFF;
        poti_y = 0xFF;
    }
    else
    {
        switch(cia1_port_a.GetOutput() >> 6)
        {
        case 0:
            poti_x = poti_y = 0xFF;
            break;
        case 1:
            poti_x = poti_ax;
            poti_y = poti_ay;
            break;
        case 2:
            poti_x = poti_bx;
            poti_y = poti_by;
            break;
        case 3:
            poti_x = poti_ax | poti_bx;
            poti_y = poti_ay | poti_by;
            break;
        }
    }

    sid1->SetPotXY(poti_x, poti_y);
    }
    uint8_t ReadC64Byte(uint16_t address) {
        return ReadProcTbl[(address)>>8](address);
    }
    int Disassemble(FILE* file, uint16_t pc, bool line_draw);
    void NextSystemCycle();
};

static const uint8_t CPU_OPC_INFO[256]={\
0*16+6+0,7*16+5+0,0*16+0+8,7*16+7+8,3*16+2+8,3*16+2+0,3*16+4+0,3*16+4+8,0*16+2+0,1*16+1+0,0*16+1+0,1*16+1+8,2*16+3+8,2*16+3+0,2*16+5+0,2*16+5+8\
,9*16+1+0,8*16+4+0,0*16+0+8,8*16+7+8,6*16+3+8,6*16+3+0,6*16+5+0,6*16+5+8,0*16+1+0,5*16+3+0,0*16+1+8,5*16+6+8,4*16+3+8,4*16+3+0,4*16+6+0,4*16+6+8\
,2*16+5+0,7*16+5+0,0*16+0+8,7*16+7+8,3*16+2+0,3*16+2+0,3*16+4+0,3*16+4+8,0*16+3+0,1*16+1+0,0*16+1+0,1*16+1+8,2*16+3+0,2*16+3+0,2*16+5+0,2*16+5+8\
,9*16+1+0,8*16+4+0,0*16+0+8,8*16+7+8,6*16+3+8,6*16+3+0,6*16+5+0,6*16+5+8,0*16+1+0,5*16+3+0,0*16+1+8,5*16+6+8,4*16+3+8,4*16+3+0,4*16+6+0,4*16+6+8\
,0*16+5+0,7*16+5+0,0*16+0+8,7*16+7+8,3*16+2+8,3*16+2+0,3*16+4+0,3*16+4+8,0*16+2+0,1*16+1+0,0*16+1+0,1*16+1+8,2*16+2+0,2*16+4+0,2*16+5+0,2*16+5+8\
,9*16+1+0,8*16+4+0,0*16+0+8,8*16+7+8,6*16+3+8,6*16+3+0,6*16+5+0,6*16+5+8,0*16+1+0,5*16+3+0,0*16+1+8,5*16+6+8,4*16+3+8,4*16+3+0,4*16+6+0,4*16+6+8\
,0*16+5+0,7*16+5+0,0*16+0+8,7*16+7+8,3*16+2+8,3*16+2+0,3*16+4+0,3*16+4+8,0*16+2+0,1*16+1+0,0*16+1+0,1*16+1+8,10*16+4+0,2*16+3+0,2*16+5+0,2*16+5+8\
,9*16+1+0,8*16+4+0,0*16+0+8,8*16+7+8,6*16+3+8,6*16+3+0,6*16+5+0,6*16+5+8,0*16+1+0,5*16+3+0,0*16+1+8,5*16+6+8,4*16+3+8,4*16+3+0,4*16+6+0,4*16+6+8\
,1*16+1+8,7*16+5+0,0*16+1+8,7*16+5+8,3*16+2+0,3*16+2+0,3*16+2+0,3*16+2+8,0*16+1+0,1*16+1+8,0*16+1+0,1*16+1+8,2*16+3+0,2*16+3+0,2*16+3+0,2*16+3+8\
,9*16+1+0,8*16+5+0,0*16+0+8,4*16+5+8,6*16+1+0,6*16+3+0,11*16+3+0,11*16+3+8,0*16+1+0,5*16+1+0,0*16+1+0,5*16+4+8,5*16+4+8,4*16+4+0,4*16+4+8,5*16+4+8\
,1*16+1+0,7*16+5+0,1*16+1+0,7*16+5+8,3*16+2+0,3*16+2+0,3*16+2+0,3*16+2+8,0*16+1+0,1*16+1+0,0*16+1+0,1*16+1+8,2*16+3+0,2*16+3+0,2*16+3+0,2*16+3+8\
,9*16+1+0,8*16+4+0,0*16+0+8,8*16+4+8,6*16+1+0,6*16+3+0,11*16+3+0,11*16+3+8,0*16+1+0,5*16+3+0,0*16+1+0,5*16+3+8,4*16+3+0,4*16+3+0,5*16+3+0,5*16+3+8\
,1*16+1+0,7*16+5+0,1*16+1+8,7*16+7+8,3*16+2+0,3*16+2+0,3*16+4+0,3*16+4+8,0*16+1+0,1*16+1+0,0*16+1+0,1*16+1+8,2*16+3+0,2*16+3+0,2*16+5+0,2*16+5+8\
,9*16+1+0,8*16+4+0,0*16+0+8,8*16+7+8,6*16+3+8,6*16+3+0,6*16+5+0,6*16+5+8,0*16+1+0,5*16+3+0,0*16+1+8,5*16+6+8,4*16+3+8,4*16+3+0,4*16+6+0,4*16+6+8\
,1*16+1+0,7*16+5+0,1*16+1+8,7*16+7+8,3*16+2+0,3*16+2+0,3*16+4+0,3*16+4+8,0*16+1+0,1*16+1+0,0*16+1+0,1*16+1+8,2*16+3+0,2*16+3+0,2*16+5+0,2*16+5+8\
,9*16+1+0,8*16+4+0,0*16+0+8,8*16+7+8,6*16+3+8,6*16+3+0,6*16+5+0,6*16+5+8,0*16+1+0,5*16+3+0,0*16+1+8,5*16+6+8,4*16+3+8,4*16+3+0,4*16+6+0,4*16+6+8};

static const char* CPU_OPC = {"\
BRKORAJAMSLONOPORAASLSLOPHPORAASLANCNOPORAASLSLO\
BPLORAJAMSLONOPORAASLSLOCLCORANOPSLONOPORAASLSLO\
JSRANDJAMRLABITANDROLRLAPLPANDROLANCBITANDROLRLA\
BMIANDJAMRLANOPANDROLRLASECANDNOPRLANOPANDROLRLA\
RTIEORJAMSRENOPEORLSRSREPHAEORLSRASRJMPEORLSRSRE\
BVCEORJAMSRENOPEORLSRSRECLIEORNOPSRENOPEORLSRSRE\
RTSADCJAMRRANOPADCRORRRAPLAADCRORARRJMPADCRORRRA\
BVSADCJAMRRANOPADCRORRRASEIADCNOPARRNOPADCRORRRA\
NOPSTANOPSAXSTYSTASTXSAXDEYNOPTXAANESTYSTASTXSAX\
BCCSTAJAMSHASTYSTASTXSAXTYASTATXSSHSSHYSTASHXSHA\
LDYLDALDXLAXLDYLDALDXLAXTAYLDATAXLXALDYLDALDXLAX\
BCSLDAJAMLAXLDYLDALDXLAXCLVLDATSXLAELDYLDALDXLAX\
CPYCMPNOPDCPCPYCMPDECDCPINYCMPDEXSBXCPYCMPDECDCP\
BNECMPJAMDCPNOPCMPDECDCPCLDCMPNOPDCPNOPCMPDECDCP\
CPXSBCNOPISBCPXSBCINCISBINXSBCNOPSBCCPXSBCINCISB\
BEQSBCJAMISBNOPSBCINCISBSEDSBCNOPISBNOPSBCINCISB\
RSTIRQNMI"};

void C64Class::NextSystemCycle()
{
///    CheckKeys();

	if(hold_next_system_cycle)
		return;

    cycle_counter++;

    /// Für Externe Erweiterungen ///
    //if(ExtZyklus) ZyklusProcExt();

    floppy_iec_wire = 0;
    for(int i=0; i<MAX_FLOPPY_NUM; i++)
    {
///        cpu_states[i+1] = floppy[i]->OneCycle();
///        floppy_iec_wire |= ~floppy[i]->FloppyIECLocal;
    }
    floppy_iec_wire = ~floppy_iec_wire;

    // PHI0
    if(enable_ext_wires) rdy_ba_wire = ext_rdy_wire;
    cpu_states[0] = cpu->OneZyklus();

    // PHI1
    vic->OneCycle();
    cia1->OneZyklus();
    cia2->OneZyklus();
    sid1->OneZyklus();
    if(enable_stereo_sid) sid2->OneZyklus();
    reu->OneZyklus();
    tape->OneCycle();
    cpu->Phi1();
}

int C64Class::Disassemble(FILE* file, uint16_t pc, bool line_draw)
{
    static char output[50]; output[0] = 0;
    static char address[7]; address[0] = 0;
    static char memory[16]; memory[0] = 0;
    static char opcode[5]; opcode[0] = 0;
    static char addressing[8]; addressing[0] = 0;

    uint16_t TMP;
    uint16_t OPC;

    uint16_t word;
    uint16_t a;
    char b;

    sprintf(address, "$%4.4X ", pc);			// Adresse Ausgeben

    OPC = ReadC64Byte(pc) * 3;					//** Opcodes Ausgeben **//
    sprintf(opcode, "%c%c%c ", CPU_OPC[OPC + 0], CPU_OPC[OPC + 1], CPU_OPC[OPC + 2]);

    TMP = CPU_OPC_INFO[ReadC64Byte(pc)];		//** Memory und Adressierung Ausgeben **//
    TMP = TMP >> 4;
    TMP = TMP & 15;

    switch(TMP)
    {
    case 0:     //** Implizit **//
        sprintf(memory, "$%2.2X          ", ReadC64Byte(pc));
        addressing[0] = 0;
        pc++;
        break;

    case 1:		//** Unmittelbar **//
        sprintf(memory, "$%2.2X $%2.2X      ", ReadC64Byte(pc), ReadC64Byte(pc + 1));
        sprintf(addressing, "#$%2.2X", ReadC64Byte(pc + 1));
        pc += 2;
        break;

    case 2:		//** Absolut **//
        sprintf(memory, "$%2.2X $%2.2X $%2.2X  ", ReadC64Byte(pc), ReadC64Byte(pc + 1), ReadC64Byte(pc + 2));
        word = ReadC64Byte(pc + 1);
        word|=ReadC64Byte(pc + 2)<<8;
        sprintf(addressing, "$%4.4X", word);
        pc += 3;
        break;

    case 3:		//** Zerropage **//
        sprintf(memory, "$%2.2X $%2.2X      ", ReadC64Byte(pc), ReadC64Byte(pc + 1));
        sprintf(addressing, "$%2.2X", ReadC64Byte(pc + 1));
        pc += 2;
        break;

    case 4:		//** Absolut X Indexziert **//
        sprintf(memory, "$%2.2X $%2.2X $%2.2X  ", ReadC64Byte(pc), ReadC64Byte(pc + 1), ReadC64Byte(pc + 2));
        word=ReadC64Byte(pc + 1);
        word|=ReadC64Byte(pc + 2)<<8;
        sprintf(addressing, "$%4.4X,X", word);
        pc += 3;
        break;

    case 5:		//** Absolut Y Indexziert **//
        sprintf(memory, "$%2.2X $%2.2X $%2.2X  ", ReadC64Byte(pc), ReadC64Byte(pc + 1), ReadC64Byte(pc + 2));
        word=ReadC64Byte(pc + 1);
        word|=ReadC64Byte(pc + 2)<<8;
        sprintf(addressing, "$%4.4X,Y", word);
        pc += 3;
        break;

    case 6:		//** Zerropage X Indexziert **//
        sprintf(memory, "$%2.2X $%2.2X      ", ReadC64Byte(pc), ReadC64Byte(pc + 1));
        sprintf(addressing, "$%2.2X,X", ReadC64Byte(pc + 1));
        pc += 2;
        break;

    case 7:		//** Indirekt X Indiziert **//
        sprintf(memory, "$%2.2X $%2.2X      ", ReadC64Byte(pc), ReadC64Byte(pc + 1));
        sprintf(addressing, "($%2.2X,X)", ReadC64Byte(pc + 1));
        pc += 2;
        break;

    case 8:		//** Indirekt Y Indiziert **//
        sprintf(memory, "$%2.2X $%2.2X      ", ReadC64Byte(pc), ReadC64Byte(pc + 1));
        sprintf(addressing, "($%2.2X),Y", ReadC64Byte(pc + 1));
        pc += 2;
        break;

    case 9:
        sprintf(memory, "$%2.2X $%2.2X      ", ReadC64Byte(pc), ReadC64Byte(pc + 1));
        b = ReadC64Byte(pc + 1);
        a = (pc + 2) + b;
        sprintf(addressing, "$%4.4X", a);
        pc += 2;
        break;

    case 10:	//** Indirekt **//
        sprintf(memory, "$%2.2X $%2.2X $%2.2X  ", ReadC64Byte(pc), ReadC64Byte(pc + 1), ReadC64Byte(pc + 2));
        word = ReadC64Byte(pc + 1);
        word |= ReadC64Byte(pc + 2) << 8;
        sprintf(addressing, "($%4.4X)", word);
        pc += 3;
        break;

    case 11:									//** Zerropage Y Indexziert **//
        sprintf(memory, "$%2.2X $%2.2X      ", ReadC64Byte(pc), ReadC64Byte(pc + 1));
        sprintf(addressing, "$%2.2X,Y", ReadC64Byte(pc + 1));
        pc += 2;
        break;
    }

    sprintf(output, "%s%s%s%s", address, memory, opcode, addressing);
    fprintf(file, "%s\n", output);

    OPC /= 3;
    if(((OPC == 0x40) || (OPC == 0x60) || (OPC == 0x4C)) && (line_draw == true))
    {
        fprintf(file, "------------------------------\n");
    }

    return pc;
}

static C64Class _c64;

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

    logMsg("!!!");
    while (runing) {
        _c64.NextSystemCycle();
        logMsg("->");
    }
    return 0;
}

#include "project.h"
#include "cy_syslib.h"
#include "stdio.h"
#include "math.h"

/* ─────────────────────────────────────────────
   CLOCK SETUP
   ───────────────────────────────────────────── */

#define CPU_MHZ_100 

/* ─────────────────────────────────────────────
   NOP TIMING CONSTANTS FOR LED STRIP 
   creates different NOPs for precise delays
   ───────────────────────────────────────────── */
#define NOP1()   __asm volatile("NOP")
#define NOP2()   NOP1(); NOP1()
#define NOP4()   NOP2(); NOP2()
#define NOP8()   NOP4(); NOP4()
#define NOP16()  NOP8(); NOP8()
#define NOP20()  NOP16(); NOP4()
#define NOP40()  NOP20(); NOP20()
#define NOP60()  NOP40(); NOP20()
#define NOP80()  NOP40(); NOP40()

// NOP settings for clock speed of 100 MHz
#if defined(CPU_MHZ_100)
  #define T1H_NOPS  NOP80()
  #define T1L_NOPS  NOP40()
  #define T0H_NOPS  NOP40()
  #define T0L_NOPS  NOP80()
#else
  #error "No CPU speed defined"
#endif

/* ─────────────────────────────────────────────
   LED STRIP CONFIGURATION
   ───────────────────────────────────────────── */
#define NUM_LEDS            70 // total number of LEDs in the strip
#define BYTES_PER_LED       3 // each LED needs 3 bytes (GRB)

// left turn LED half
#define LEFT_TURN_FIRST     0
#define LEFT_TURN_LAST      34
// right turn LED half
#define RIGHT_TURN_FIRST    35
#define RIGHT_TURN_LAST     (NUM_LEDS - 1)

#define BLINK_ON_MS         400
#define BLINK_OFF_MS        400
#define BLINK_CYCLES        6

/* ─────────────────────────────────────────────
   COLOR VALUES  (GRB byte order for WS2812)
   ───────────────────────────────────────────── */
// turn signal yellow
#define YELLOW_G  100
#define YELLOW_R  120
#define YELLOW_B    0

// night light white
#define WHITE_G   100
#define WHITE_R   100
#define WHITE_B   100

// brakelight red
#define RED_G       0
#define RED_R     150
#define RED_B       0

/* ─────────────────────────────────────────────
   SENSOR CONFIGURATION
   ───────────────────────────────────────────── */
#define ACCEL_ADDR       0x18 // I2C address of the accelerometer
#define GYRO_ADDR        0x6B // I2C address of gyroscope

#define G_PER_LSB        0.001f // accelerometer scale factor
#define DPS_PER_LSB      0.00875f   // gyro scale factor 8.75 mdps/LSB for ±245 dps 

#define TURN_THRESH      20.0f      // deg/s 
#define DARK_THRESH      1500       // ADC counts - light sensor threshold
// #define DECEL_THRESH     -0.5f     
// #define DECEL_COUNTS     5   


/* ─────────────────────────────────────────────
   BMS CONFIGURATION
   ───────────────────────────────────────────── */

#define ADC_REF_VOLTAGE   (3.3f) // reference voltage is 3.3 V
#define ADC_MAX_COUNTS    (4095.0f)

#define RELAY_ON()   Cy_GPIO_Clr(Relay_Pin_PORT, Relay_Pin_NUM) // Relay pin is active low
#define RELAY_OFF()  Cy_GPIO_Set(Relay_Pin_PORT, Relay_Pin_NUM)// Relay pin is active low


/* ─────────────────────────────────────────────
   GPIO — WS2812B DATA PIN
   ───────────────────────────────────────────── */
#define LED_ON()   Cy_GPIO_Write(LED_PIN_PORT, LED_PIN_NUM, 1)
#define LED_OFF()  Cy_GPIO_Write(LED_PIN_PORT, LED_PIN_NUM, 0)

/* ─────────────────────────────────────────────
   I2C HELPERS
   ───────────────────────────────────────────── */
static cy_stc_scb_i2c_master_xfer_config_t register_setting;
static uint8_t rbuff[2];
static uint8_t wbuff[2];

static void WaitForOperation(void)
{
    while (0 != (SensorBus3_MasterGetStatus() & CY_SCB_I2C_MASTER_BUSY))
        CyDelayUs(1);
}

static void WriteRegister(uint8_t reg, uint8_t data)
{
    wbuff[0] = reg;
    wbuff[1] = data;
    register_setting.buffer     = wbuff;
    register_setting.bufferSize = 2;
    register_setting.xferPending = false;
    SensorBus3_MasterWrite(&register_setting);
    WaitForOperation();
}

static uint8_t ReadRegister(uint8_t reg)
{
    wbuff[0] = reg;
    register_setting.buffer      = wbuff;
    register_setting.bufferSize  = 1;
    register_setting.xferPending = true;
    SensorBus3_MasterWrite(&register_setting);
    WaitForOperation();

    register_setting.buffer      = rbuff;
    register_setting.xferPending = false;
    SensorBus3_MasterRead(&register_setting);
    WaitForOperation();
    return rbuff[0];
}

/* ─────────────────────────────────────────────
   SENSOR READINGS
   ───────────────────────────────────────────── */
static float measX(void)
{
    register_setting.slaveAddress = ACCEL_ADDR;
    int16_t raw = (int16_t)((ReadRegister(0x29) << 8) | ReadRegister(0x28));
    return (raw >> 4) * G_PER_LSB;
}

static float measY(void)
{
    register_setting.slaveAddress = ACCEL_ADDR;
    int16_t raw = (int16_t)((ReadRegister(0x2B) << 8) | ReadRegister(0x2A));
    return (raw >> 4) * G_PER_LSB;
}

static float measZ(void)
{
    register_setting.slaveAddress = ACCEL_ADDR;
    int16_t raw = (int16_t)((ReadRegister(0x2D) << 8) | ReadRegister(0x2C));
    return (raw >> 4) * G_PER_LSB;
}

static float gyroZ(void)
{
    register_setting.slaveAddress = GYRO_ADDR;
    int16_t raw = (int16_t)((ReadRegister(0x2D) << 8) | ReadRegister(0x2C));
    return raw * DPS_PER_LSB;
}

/* Returns 1 when ambient light is below threshold */
static uint8_t isDark(void)
{
    ADC_StartConvert();
    ADC_IsEndConversion(CY_SAR_WAIT_FOR_RESULT);
    uint16_t adcVal = ADC_GetResult16(0);
    printf("Light = %u\r\n", adcVal);
    return (adcVal < DARK_THRESH) ? 1u : 0u;
}

uint8 isDecel(void)
{
    static float filteredMag = 0.0f;
    static uint8 initialized = 0;

    // --- Configuration Constants ---
    const float ALPHA = 0.5f;      // Smoothing factor (0.0 to 1.0)
    const float THRESHOLD = -0.05f; // Sensitivity (lower = more sensitive)

    float x = measX();
    float y = measY();
    //float z = measZ();
    
    float mag = sqrtf(x*x + y*y);

    if (!initialized) {
        filteredMag = mag;
        initialized = 1;
        return 0;
    }

    // 1. Apply Low-Pass Filter (Exponential Moving Average)
    filteredMag = (ALPHA * mag) + ((1.0f - ALPHA) * filteredMag);

    // 2. Compare raw magnitude to the filtered "baseline"
    float delta = fabs(mag - filteredMag);

    if(delta < THRESHOLD)
    {
        return 1;
    }

    return 0;
}


/* ─────────────────────────────────────────────
   WS2812B BIT-BANG DRIVER
   ───────────────────────────────────────────── */
static inline void send_bit(uint8_t bit) // encodes '1' bit
{
    if (bit) { LED_ON(); T1H_NOPS; LED_OFF(); T1L_NOPS; }
    else      { LED_ON(); T0H_NOPS; LED_OFF(); T0L_NOPS; }
}

static void send_byte(uint8_t byte) // sends 8 bits MSB
{
    for (int8_t i = 7; i >= 0; i--)
        send_bit((byte >> i) & 0x01);
}

static void strip_show(uint8_t colors[][BYTES_PER_LED], uint16_t count)
{
    uint32_t state = Cy_SysLib_EnterCriticalSection(); // disable interrupts before sending
    // send color data to all 70 LEDs
    for (uint16_t i = 0; i < count; i++)
    {
        send_byte(colors[i][0]);   /* Green */
        send_byte(colors[i][1]);   /* Red   */
        send_byte(colors[i][2]);   /* Blue  */
    }
    // reset pulse to latch new colors
    Cy_SysLib_ExitCriticalSection(state);
    LED_OFF();
    Cy_SysLib_DelayUs(300);        /* latch pulse */
}

/* ─────────────────────────────────────────────
   STRIP HELPERS
   ───────────────────────────────────────────── */

// updates color buffer
static void fill_range(uint8_t colors[][BYTES_PER_LED],
                        uint16_t first, uint16_t last,
                        uint8_t g, uint8_t r, uint8_t b)
{
    for (uint16_t i = first; i <= last; i++)
    {
        colors[i][0] = g;
        colors[i][1] = r;
        colors[i][2] = b;
    }
}

// turn off all LEDs
static void all_off(uint8_t colors[][BYTES_PER_LED])
{
    fill_range(colors, 0, NUM_LEDS - 1, 0, 0, 0);
    strip_show(colors, NUM_LEDS);
    Cy_SysLib_DelayUs(500);
}

/* ─────────────────────────────────────────────
   LED STRIP STATES
   ───────────────────────────────────────────── */

/* Single blink step for turn signals — call repeatedly from the loop.
   Returns 1 when the full blink sequence has completed. */

// takes points to cycleCount and Phase to track blink sequence
static uint8_t behavior_turn(uint8_t colors[][BYTES_PER_LED],
                              uint8_t isLeft,
                              uint8_t *cycleCount,
                              uint8_t *phase)
{
    if (*cycleCount >= BLINK_CYCLES) return 1;  /* done */
    
    // decides which helf to turn on or off based on L/R detection
    uint16_t signalFirst = isLeft ? LEFT_TURN_FIRST  : RIGHT_TURN_FIRST;
    uint16_t signalLast  = isLeft ? LEFT_TURN_LAST   : RIGHT_TURN_LAST;
    uint16_t otherFirst  = isLeft ? RIGHT_TURN_FIRST : LEFT_TURN_FIRST;
    uint16_t otherLast   = isLeft ? RIGHT_TURN_LAST  : LEFT_TURN_LAST;

    if (*phase == 0) // ON phase
    {
        fill_range(colors, signalFirst, signalLast, YELLOW_G, YELLOW_R, YELLOW_B);
        fill_range(colors, otherFirst,  otherLast,  0, 0, 0);
        strip_show(colors, NUM_LEDS);
        Cy_SysLib_Delay(BLINK_ON_MS);
        *phase = 1;
    }
    else // OFF phase
    {
        fill_range(colors, signalFirst, signalLast, 0, 0, 0);
        strip_show(colors, NUM_LEDS);
        Cy_SysLib_Delay(BLINK_OFF_MS);
        *phase = 0;
        (*cycleCount)++; // inc blink counter
    }
    return 0;
}

// set all LEDs to white and display on strip
static void behavior_night_light(uint8_t colors[][BYTES_PER_LED])
{
    fill_range(colors, 0, NUM_LEDS - 1, WHITE_G, WHITE_R, WHITE_B);
    strip_show(colors, NUM_LEDS);
}

// set all LEDs to red and display on strip
static void behavior_brake_on(uint8_t colors[][BYTES_PER_LED])
{
    fill_range(colors, 0, NUM_LEDS - 1, RED_G, RED_R, RED_B);
    strip_show(colors, NUM_LEDS);
}

/* ─────────────────────────────────────────────
   MAIN
   ───────────────────────────────────────────── */
int main(void)
{
    __enable_irq(); // enable global interrupts
    
    // start UART and disable input buffering
    UART3_Start();
    setvbuf(stdin, NULL, _IONBF, 0);

    // start I2C and configure accelerometer
    SensorBus3_Start();
    register_setting.slaveAddress = ACCEL_ADDR;
    WriteRegister(0x20, 0x57);   /* accel: ODR 100Hz, all axes enabled */
    WriteRegister(0x23, 0x88);   /* accel: ±2g, high-res */

    // configure gyroscope
    register_setting.slaveAddress = GYRO_ADDR;
    WriteRegister(0x20, 0x0F);   /* gyro: ODR 100Hz, all axes enabled */
    WriteRegister(0x23, 0x00);   /* gyro: ±245 dps */

    // start ADC and trigger first conversion
    ADC_Start();
    ADC_StartConvert();
    Cy_GPIO_Write(LEDON_PORT, LEDON_NUM, 0);

    register_setting.slaveAddress = ACCEL_ADDR;
    uint8_t whoami = ReadRegister(0x0F);
    printf("WHO_AM_I = 0x%02X\r\n", whoami);
    printf("- - - - - System Ready - - - - -\r\n");

    // LED strip 
    uint8_t colors[NUM_LEDS][BYTES_PER_LED] = {0};
    all_off(colors);

    // turn signal fstate variables
    uint8_t turnCycleCount = 0;
    uint8_t turnPhase      = 0;
  

    for (;;)
    {
        // BMS LOGIC
        if(ADC_IsEndConversion(0u)) // reads only if conversion has completed
        {
            int16 result = ADC_GetResult16(0);

            float voltage = ((float)result / ADC_MAX_COUNTS) * ADC_REF_VOLTAGE; // converts raw counts to actual voltage

            if (voltage > 1.5f) {// if input greater than 1.5 V, turn relay ON
                RELAY_ON();
                Cy_GPIO_Write(RELAYON_PORT, RELAYON_NUM, 0);
            }
            else{
                RELAY_OFF();  // if input less than 1.5 V, turn relay OFF
                 Cy_GPIO_Write(RELAYON_PORT, RELAYON_NUM, 1);
            }
        }
        
        // TURN AND BRAKE LOGIC
        float gz = gyroZ();
        printf("gyroZ = %7.3f\r\n", (double)gz);

        uint8_t decel = isDecel();
        uint8_t dark  = isDark();

        if (gz < -TURN_THRESH) // right turn
        {
            if (turnCycleCount >= BLINK_CYCLES)  
            {
                turnCycleCount = 0;
                turnPhase      = 0;
            }
            // ifLeft flag set to 0 -> right turn
            behavior_turn(colors, 0, &turnCycleCount, &turnPhase);
        }
        else if (gz > TURN_THRESH)  // left turn
        {
            if (turnCycleCount >= BLINK_CYCLES)
            {
                turnCycleCount = 0;
                turnPhase      = 0;
            }
            // ifLeft flag set to 1 -> left turn
            behavior_turn(colors, 1, &turnCycleCount, &turnPhase);
        }
        else
        {
            // reset turn counter if no turn
            turnCycleCount = BLINK_CYCLES;

            if (decel) // brake light
            {
                printf("Brake triggered\r\n");
                behavior_brake_on(colors);
            }
            else
            {
                
                if (dark) // night light
                { 
                    printf("Night light ON\r\n");
                    behavior_night_light(colors);
                }
                else
                {
                    all_off(colors);
                }
            }
        }

        CyDelay(100);
    }
}

/* [] END OF FILE */
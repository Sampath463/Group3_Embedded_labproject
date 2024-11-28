#include <string.h>

#include <stddef.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include <math.h>       // new change

#define R 6371000  // Earth's radius in meters

char latDirection[2]; // To store N/S
char longDirection[2]; // To store E/W
char validity; // To store validity
volatile bool validData = false; // To indicate valid GPGLL data

float latitudeValue = 0.0;
float longitudeValue = 0.0;

float xpos_latitude = 15.48732; // admin block
float xpos_longitude = 74.934456;


#define BUFFER_SIZE 128

// Buffer for GPS data
volatile char gpsBuffer[BUFFER_SIZE];
volatile uint8_t bufferIndex = 0;
char latitude[15];
char longitude[15];

int i;
double bearing = 0;
double distance = 0;
// Function Prototypes
void UART1_Init(void);
void UART1_Handler(void);
bool validateChecksum(const char *sentence);
void parseGPGLL(const char *sentence);
float convertToDecimalDegrees(const char *coord, const char *direction);
double toRadians(double degree);
double haversine(double lat1, double lon1, double lat2, double lon2) ;
double calculateBearing(double lat1, double lon1, double lat2, double lon2);
void processing(void);
// LED configuring code`
#define SYS_CLOCK 16000000  // 80 MHz system clock
#define TICKS_PER_US (SYS_CLOCK / 1000000)  // Clock ticks per 탎
#define T0H (TICKS_PER_US * 0.35)  // 0.35 탎 high
#define T0L (TICKS_PER_US * 0.8)   // 0.8 탎 low
#define T1H (TICKS_PER_US * 0.7)   // 0.7 탎 high
#define T1L (TICKS_PER_US * 0.6)   // 0.6 탎 low
#define RESET_DELAY (TICKS_PER_US * 50)  // 50 탎 low for reset
#define NUM_LEDS 16

// Function prototypes
void PWM_Init(void);
void WS2812_SendBit(uint8_t bit);
void WS2812_SendByte(uint8_t byte);
void WS2812_SendColor(uint8_t green, uint8_t red, uint8_t blue);
void WS2812_Reset(void);
void WS2812_TurnOff(uint8_t num_leds);
void WS2812_BlinkLed(uint8_t led_index, uint8_t num_leds);

int main(void) {
    // Initialize UART1
    UART1_Init();
    PWM_Init();

    // Enable global interrupts


    while (1) {
        if (validData) {




            validData = false; // Reset the flag
        }

    }
}
void PWM_Init(void) {
    SYSCTL_RCGCPWM_R |= 0x01;  // Enable PWM0 clock
    SYSCTL_RCGCGPIO_R |= 0x02;  // Enable GPIOB clock
    while (!(SYSCTL_PRPWM_R & 0x01)); // Wait for PWM0 to be ready
    while (!(SYSCTL_PRGPIO_R & 0x02)); // Wait for GPIOB to be ready

    SYSCTL_RCC_R &= ~SYSCTL_RCC_USEPWMDIV;  // Use system clock directly

    GPIO_PORTB_AFSEL_R |= 0x40;  // Enable alternate function on PB6
    GPIO_PORTB_PCTL_R &= ~0x0F000000;
    GPIO_PORTB_PCTL_R |= 0x04000000;  // Configure PB6 as PWM0
    GPIO_PORTB_DEN_R |= 0x40;  // Digital enable PB6

    PWM0_0_CTL_R = 0;  // Disable PWM generator during config
    PWM0_INTEN_R |= PWM_INTEN_INTPWM0; // Enable PWM interrupt for generator 0

    PWM0_0_GENA_R = 0x0000008C;  // Set PWM0 high on load, low on CMPA down
    PWM0_0_LOAD_R = TICKS_PER_US * 1.25 - 1;  // 1.25 탎 period
    PWM0_0_CTL_R |= PWM_0_CTL_MODE; // Enable down-count mode (if not already set)
;  // Enable PWM generator
    PWM0_ENABLE_R |= PWM_ENABLE_PWM0EN;  // Enable PWM output
}

// Send a single bit (1 or 0) to WS2812
void WS2812_SendBit(uint8_t bit) {
    if (bit) {
        // Logic '1': High for 0.7 탎, Low for 0.6 탎 (1.25 탎 total period)
        PWM0_0_LOAD_R = TICKS_PER_US * 1.25 - 1;   // Set total period (1.25 탎)
        PWM0_0_CMPA_R = TICKS_PER_US * 0.7 - 1;   // Set high time (0.7 탎)
    } else {
        // Logic '0': High for 0.35 탎, Low for 0.8 탎 (1.25 탎 total period)
        PWM0_0_LOAD_R = TICKS_PER_US * 1.25 - 1;   // Set total period (1.25 탎)
        PWM0_0_CMPA_R = TICKS_PER_US * 0.35 - 1;  // Set high time (0.35 탎)
    }

    // Wait for the PWM signal to complete one cycle
    while ((PWM0_RIS_R & PWM_RIS_INTPWM0) == 0); // Wait for interrupt flag
    PWM0_ISC_R = PWM_ISC_INTPWM0;
               // Clear the interrupt flag
}


// Send a single byte to WS2812
void WS2812_SendByte(uint8_t byte) {
    int i  = 0 ;
    for (i = 0; i < 8; i++) {
        WS2812_SendBit(byte & 0x80);  // Send MSB first
        byte <<= 1;  // Shift to the next bit
    }
}

// Send a color in GRB format
void WS2812_SendColor(uint8_t green, uint8_t red, uint8_t blue) {
    WS2812_SendByte(green);
    WS2812_SendByte(red);
    WS2812_SendByte(blue);
}

// Send reset signal
void WS2812_Reset(void) {
    PWM0_0_CMPA_R = 0;  // Force low output
    volatile int  i  = 0 ;
    for (i = 0; i < RESET_DELAY; i++);  // Delay for reset
}
void WS2812_TurnOff(uint8_t num_leds) {
    int  i   = 0 ;
    for ( i = 0; i < num_leds; i++) {
        WS2812_SendColor(0x00, 0x00, 0x00);  // Turn off each LED
    }
}

// Function to blink a specific LED
void WS2812_BlinkLed(uint8_t led_index, uint8_t num_leds) {
    WS2812_TurnOff(num_leds);  // Turn off all LEDs
    int i = 0 ;
    for (i = 0; i < num_leds; i++) {
        if (i == led_index) {
            WS2812_SendColor(0xFF, 0xFF, 0xFF);  // White for the selected LED
        } else {
            WS2812_SendColor(0x00, 0x00, 0x00);  // Off for others
        }
    }
}


void UART1_Init(void) {
    // Enable UART1 and GPIOB
    SYSCTL_RCGCUART_R |= ( 1 << 1);

    SYSCTL_RCGC2_R |= (1 << 1);
    GPIO_PORTB_LOCK_R = 0x4C4F434B;
    GPIO_PORTB_CR_R = (1 << 1) | ( 1 << 0);
    // Wait for peripherals to be ready
    while ((SYSCTL_PRUART_R & (1 << 1)) == 0);
    while ((SYSCTL_PRGPIO_R & (1 << 1)) == 0);

    // Configure PB0 and PB1 for UART1
    GPIO_PORTB_AFSEL_R = (1 << 0) | (1 << 1);  // Enable alternate function for PB0 and PB1
    GPIO_PORTB_PCTL_R = (1 << 0) | ( 1 << 4); // Configure PB0, PB1 for UART1
    GPIO_PORTB_DEN_R |= (1 << 0) | (1 << 1);   // Enable digital functionality  // ADD DIR?
    GPIO_PORTB_DIR_R = (1 << 1);
   // GPIO_PORTB_AMSEL_R &= ~((1 << 0) | (1 << 1)); // Disable analog functionality

    // Configure UART1
    UART1_CTL_R &= ~(1 << 0);      // Disable UART1
    UART1_IBRD_R = 104;            // Integer part of baud rate (9600 baud, 16 MHz clock)
    UART1_FBRD_R = 11;             // Fractional part of baud rate
    UART1_LCRH_R = (0x3 << 5);     // 8-bit, no parity, 1 stop bit
    UART1_CC_R = 0x00;             // Use system clock
    UART1_IM_R |= (1 << 4);        // Enable RX interrupt
    UART1_CTL_R |= (1 << 0) | (1 << 8) | (1 << 9); // Enable UART1, TX, and RX

    // Enable UART1 interrupt in NVIC
    NVIC_EN0_R |= (1 << 6);        // UART1 interrupt is IRQ 6
}

void UART1_Handler(void) {
    static bool waitingForDollar = true; // Flag to wait for '$'
    static uint8_t charCounter = 0;     // Counter for the "$GPGLL" check

    if (UART1_MIS_R & (1 << 4)) {       // RX interrupt triggered
        char c = UART1_DR_R & 0xFF;     // Read received character

        if (waitingForDollar) {
            if (c == '$') {             // Start of a new NMEA sentence
                bufferIndex = 0;
                gpsBuffer[bufferIndex++] = c;
                waitingForDollar = false; // No longer waiting for '$'
                charCounter = 0;          // Reset character counter for GPGLL
            }
        } else {
            if (bufferIndex < BUFFER_SIZE - 1) {
                gpsBuffer[bufferIndex++] = c;

                if (charCounter < 5) {
                    // Check if the first 5 characters after '$' form "GPGLL"
                    if (c == "GPGLL"[charCounter]) {
                        charCounter++;
                    } else {
                        // Not "GPGLL", reset and wait for '$' again
                        bufferIndex = 0;
                        waitingForDollar = true;
                    }
                } else if (c == '\n') { // End of the sentence
                    gpsBuffer[bufferIndex] = '\0'; // Null-terminate

                    if (strncmp((const char *)gpsBuffer, "$GPGLL", 6) == 0) {

                            parseGPGLL((const char *)gpsBuffer); // Parse GPGLL sentence
                            processing();
                            int j = 0;
                            for(j=0;j<=70000;j++) {   }

                    }
                    // Reset for the next sentence
                    bufferIndex = 0;
                    waitingForDollar = true;
                }
            } else {
                // Buffer overflow, reset and wait for '$'
                bufferIndex = 0;
                waitingForDollar = true;
            }
        }

        UART1_ICR_R |= (1 << 4); // Clear the interrupt
    }
}



void parseGPGLL(const char *sentence) {
    char tempBuffer[BUFFER_SIZE];
    strncpy(tempBuffer, sentence, BUFFER_SIZE);

    char *token = strtok(tempBuffer, ",");
    uint8_t field = 0;

    while (token != NULL) {
        field++;
        switch (field) {
            case 2: // Latitude
                strncpy(latitude, token, sizeof(latitude) - 1);
                latitude[sizeof(latitude) - 1] = '\0'; // Ensure null-termination
                break;
            case 3: // Latitude direction (N/S)
                strncpy(latDirection, token, sizeof(latDirection) - 1);
                latDirection[sizeof(latDirection) - 1] = '\0';
                break;
            case 4: // Longitude
                strncpy(longitude, token, sizeof(longitude) - 1);
                longitude[sizeof(longitude) - 1] = '\0'; // Ensure null-termination
                break;
            case 5: // Longitude direction (E/W)
                strncpy(longDirection, token, sizeof(longDirection) - 1);
                longDirection[sizeof(longDirection) - 1] = '\0';
                break;
            case 6: // UTC time (optional, can be ignored)
                break;
            case 7: // Validity
                validity = token[0]; // Validity is 'A' or 'V'
                break;
        }
        token = strtok(NULL, ",");
    }

    if (validity == 'A') { // If data is valid
        latitudeValue = convertToDecimalDegrees(latitude, latDirection);
        longitudeValue = convertToDecimalDegrees(longitude, longDirection);
        validData = true;
    }


}



float convertToDecimalDegrees(const char *coord, const char *direction) {
    // Convert string to float
    float value = atof(coord);

    // Extract degrees and minutes
    int degrees = (int)(value / 100);  // Integer division gives degrees
    float minutes = value - (degrees * 100); // Remaining part is minutes

    // Convert to decimal degrees
    float decimalDegrees = degrees + (minutes / 60.0);

    // Apply negative sign for S (South) or W (West)
    if (direction[0] == 'S' || direction[0] == 'W') {
        decimalDegrees = -decimalDegrees;
    }

    return decimalDegrees;
}

double calculateBearing(double lat1, double lon1, double lat2, double lon2) {
    double phi1 = toRadians(lat1);
    double phi2 = toRadians(lat2);
    double delta_lambda = toRadians(lon2 - lon1);

    double x = sin(delta_lambda) * cos(phi2);
    double y = cos(phi1) * sin(phi2) - sin(phi1) * cos(phi2) * cos(delta_lambda);

    double initial_bearing = atan2(x, y);
    // Convert bearing from radians to degrees and normalize to 0-360
    double compass_bearing = fmod((initial_bearing * 180.0 / M_PI) + 360.0, 360.0);

    return compass_bearing;
}

double haversine(double lat1, double lon1, double lat2, double lon2) {
    double phi1 = toRadians(lat1);
    double phi2 = toRadians(lat2);
    double delta_phi = toRadians(lat2 - lat1);
    double delta_lambda = toRadians(lon2 - lon1);

    double a = sin(delta_phi / 2) * sin(delta_phi / 2) +
               cos(phi1) * cos(phi2) * sin(delta_lambda / 2) * sin(delta_lambda / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));

    return R * c;  // Distance in meters
}

double toRadians(double degree) {
    return degree * M_PI / 180.0;
}

void processing(void){

    bearing = calculateBearing( latitudeValue,  longitudeValue,  xpos_latitude ,  xpos_longitude);
                distance = haversine( latitudeValue,  longitudeValue,  xpos_latitude ,  xpos_longitude);
                if (bearing >= 0 && bearing < 22.5) {
                            WS2812_BlinkLed(0, NUM_LEDS);  // Blink first LED
                        } else if (bearing >= 22.5 && bearing < 45) {
                            WS2812_BlinkLed(1, NUM_LEDS);  // Blink second LED
                        } else if (bearing >= 45 && bearing < 67.5) {
                            WS2812_BlinkLed(2, NUM_LEDS);  // Blink third LED
                        } else if (bearing >= 67.5 && bearing < 90) {
                            WS2812_BlinkLed(3, NUM_LEDS);  // Blink fourth LED
                        } else if (bearing >= 90 && bearing < 112.5) {
                            WS2812_BlinkLed(4, NUM_LEDS);  // Blink fifth LED
                        } else if (bearing >= 112.5 && bearing < 135) {
                            WS2812_BlinkLed(5, NUM_LEDS);  // Blink sixth LED
                        } else if (bearing >= 135 && bearing < 157.5) {
                            WS2812_BlinkLed(6, NUM_LEDS);  // Blink seventh LED
                        } else if (bearing >= 157.5 && bearing < 180) {
                            WS2812_BlinkLed(7, NUM_LEDS);  // Blink eighth LED
                        } else if (bearing >= 180 && bearing < 202.5) {
                            WS2812_BlinkLed(8, NUM_LEDS);  // Blink ninth LED
                        } else if (bearing >= 202.5 && bearing < 225) {
                            WS2812_BlinkLed(9, NUM_LEDS);  // Blink tenth LED
                        } else if (bearing >= 225 && bearing < 247.5) {
                            WS2812_BlinkLed(10, NUM_LEDS);  // Blink eleventh LED
                        } else if (bearing >= 247.5 && bearing < 270) {
                            WS2812_BlinkLed(11, NUM_LEDS);  // Blink twelfth LED
                        } else if (bearing >= 270 && bearing < 292.5) {
                            WS2812_BlinkLed(12, NUM_LEDS);  // Blink thirteenth LED
                        }
                        else if (bearing >= 292.5 && bearing < 315) {
                                    WS2812_BlinkLed(13, NUM_LEDS);  // Blink fourteenth LED
                                } else if (bearing >= 315 && bearing < 337.5) {
                                    WS2812_BlinkLed(14, NUM_LEDS);  // Blink fifteenth LED
                                } else if (bearing >= 337.5 && bearing < 360) {
                                    WS2812_BlinkLed(15, NUM_LEDS);  // Blink sixteenth LED
                                }


}

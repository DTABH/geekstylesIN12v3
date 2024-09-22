#include "stm8s.h"

/* v.1.01 added the ability to set RGB LED parameters 22-09-2024
 *
 * v.1.00 initial release 14-09-2024
 *
 *
 * NixieClock IN12 V3 board (frontside board)
 * JM Nixieclock IN12 V4.2 20190514 (backside board)
 *
 *   STM8S003F3  TSSOP20 8K FLASH, 1K RAM, 128 byte eeprom.
 *
 *   Pin 1   PD4 (HS) UART1_CK/TIM2_CH1/BEEP          input <= SW UP  & 10k R28 pullup, GND when pressed
 *   Pin 2   PD5 (HS) UART1_TX/AIN5                   input <= SW DOWN & 10k R29 pullup, GND when pressed
 *   Pin 3   PD6 (HS) UART1_RX                        output => 1k R11 => Q2b for colon neons DS1 & DS2
 *   Pin 4   NRST                                     debug connector pin 3
 *   Pin 5   PA1 OSCIN                           
 *   Pin 6   PA2 OSCOUT
 *   Pin 7   Vss                                      GND       
 *   Pin 8   Vcap     
 *   Pin 9   Vdd                                      +3.3V     
 *   Pin 10  PA3 (HS) SPI_NSS/TIM2_CH3
 *   Pin 11  PB5 (T)  I2C_SDA/TIM1_BKIN               inout <=> DS3231 SDA & 10k R21 pullup
 *   Pin 12  PB4 (T)  I2C_SCL/ADC_ETR                 output => DS3231 scl & 10k R20 pullup 
 *   Pin 13  PC3 (HS) TIM1_CH3/[TLI]/[TIM1_CH1N]      output => 1k R25 => Q4b for LEDs red
 *   Pin 14  PC4 (HS) TIM1_CH4/CLK_CCO/AIN2/TIM1_CH2N output => 1k R26 => Q3b for LEDs blue
 *   Pin 15  PC5 (HS) SPI_SCK/TIM2_CH1                output => 595 U4 sr data in & 10k R19 pullup
 *   Pin 16  PC6 (HS) SPI_MOSI/TIM1_CH1               output => 1k R27 => Q5b for LEDs green
 *   Pin 17  PC7 (HS) SPI_MISO/TIM1_CH2               output => all 595 RCLK (latch) & 10k R18 pullup
 *   Pin 18  PD1 (HS) SWIM                            debug connector pin 2
 *   Pin 19  PD2 (HS) AIN3/TIM2_CH3                   output => all 595 SRCLK & 10k R17 pullup
 *   Pin 20  PD3 (HS) AIN4/TIM2_CH2/ADC_ETR           output => 595 U4 /OE (all other 595 /OE grounded)
 *
 *
 *    PC5
 *     |
 *     V
 *   595 U4
 *     Q0 -> 2003A U5  I1, O1-> N4 0
 *     Q1 -> 2003A U5  I2, O2-> N4 1
 *     Q2 -> 2003A U5  I3, O3-> N4 2
 *     Q3 -> 2003A U5  I4, O4-> N4 3
 *     Q4 -> 2003A U5  I5, O5-> N4 4
 *     Q5 -> 2003A U5  I6, O6-> N4 5
 *     Q6 -> 2003A U5  I7, O7-> N4 6
 *     Q7 -> 2003A U6  I1, O1-> N4 7
 *     |
 *     V
 *   595 U7
 *     Q0 -> 2003A U6  I2, O2-> N4 8
 *     Q1 -> 2003A U6  I3, O3-> N4 9
 *     Q2 -> 2003A U6  I4, O4-> N3 0
 *     Q3 -> 2003A U6  I5, O5-> N3 1
 *     Q4 -> 2003A U6  I6, O6-> N3 2
 *     Q5 -> 2003A U6  I7, O7-> N3 3
 *     Q6 -> 2003A U8  I1, O1-> N3 4
 *     Q7 -> 2003A U8  I2, O2-> N3 5
 *     |
 *     V
 *   595 U9
 *     Q0 -> 2003A U8  I3, O3-> N3 6
 *     Q1 -> 2003A U8  I4, O4-> N3 7
 *     Q2 -> 2003A U8  I5, O5-> N3 8
 *     Q3 -> 2003A U8  I6, O6-> N3 9
 *     Q4 -> 2003A U8  I7, O7-> N2 0
 *     Q5 -> 2003A U10 I1, O1-> N2 1
 *     Q6 -> 2003A U10 I2, O2-> N2 2
 *     Q7 -> 2003A U10 I3, O3-> N2 3
 *     |
 *     V
 *   595 U11
 *     Q0 -> 2003A U10 I4, O4-> N2 4
 *     Q1 -> 2003A U10 I5, O5-> N2 5
 *     Q2 -> 2003A U10 I6, O6-> N2 6
 *     Q3 -> 2003A U10 I7, O7-> N2 7
 *     Q4 -> 2003A U12 I1, O1-> N2 8
 *     Q5 -> 2003A U12 I2, O2-> N2 9
 *     Q6 -> 2003A U12 I3, O3-> N1 0
 *     Q7 -> 2003A U12 I4, O4-> N1 1
 *     |
 *     V
 *   595 U13
 *     Q0 -> 2003A U12 I5, O5-> N1 2
 *     Q1 -> 2003A U12 I6, O6-> N1 3
 *     Q2 -> 2003A U12 I7, O7-> N1 4
 *     Q3 -> 2003A U14 I1, O1-> N1 5
 *     Q4 -> 2003A U14 I2, O2-> N1 6
 *     Q5 -> 2003A U14 I3, O3-> N1 7
 *     Q6 -> 2003A U14 I4, O4-> N1 8
 *     Q7 -> 2003A U14 I5, O5-> N1 9
 *
 *
 *   debug connector
 *    Pin 1   3.3V
 *    Pin 2   SWIM
 *    Pin 3   NRST
 *    Pin 4   GND
 *   
 *
*/


// DS3231 I2C address
#define DS3231_ADDRESS	0x58			
#define SDA_PIN 			GPIO_PIN_5
#define SCL_PIN 			GPIO_PIN_4

// HC595 pin definitions
#define CLOCK_PIN   	GPIO_PIN_2   // Pin 19 (SRCLK)
#define LATCH_PIN   	GPIO_PIN_7   // Pin 17 (RCLK)
#define OE_PIN      	GPIO_PIN_3   // Pin 20 (OE)
#define DATA_PIN    	GPIO_PIN_5   // Pin 15 (PC5, DATA connected to DS of the first 74HC595)

// Pin definitions for buttons
#define BUTTON_UP    GPIO_PIN_4
#define BUTTON_DOWN  GPIO_PIN_5

// Software debounce delay
#define DEBOUNCE_DELAY_MS 2
#define LONG_PRESS_THRESHOLD_MS 100
#define TIMEOUT_DELAY_MS 10000

// Enum for adjustment parameters
typedef enum {
    PARAM_MINUTES,
    PARAM_HOURS,
    PARAM_DAY,
    PARAM_MONTH,
    PARAM_YEAR,
    PARAM_WEEKDAY,
    PARAM_MAX
} ParamType;



// Function prototypes
/* void I2C_Init1(void);
void DS3231_SetTime(uint8_t hour, uint8_t minute, uint8_t second);
void DS3231_ReadTime(uint8_t *hour, uint8_t *minute, uint8_t *second);
*/ 
uint8_t BCD_To_Decimal(uint8_t bcd);
uint8_t Decimal_To_BCD(uint8_t decimal);
void latch_data(void);
void enable_output(void);
void disable_output(void);
void adjust_time(void);
void gpio_init1(void);
void delay (uint16_t ms);
uint8_t edit(uint8_t x, uint8_t y, uint8_t parameter);
uint8_t changeRGB (uint8_t i, uint8_t parameter);
uint8_t editRGB(uint8_t i, uint8_t y, uint8_t parameter);





// initialize GPIO's 
void gpio_init1(void) {
	
	// init ports
		GPIO_DeInit(GPIOB); //prepare Port B for working
		GPIO_DeInit(GPIOC); // prepare Port C for working
		GPIO_DeInit(GPIOD); // prepare Port D for working


		//Declare PD6 as push pull Output pin (used for semicolon)
    GPIO_Init (GPIOD, GPIO_PIN_6, GPIO_MODE_OUT_PP_HIGH_FAST);
	  
	 
	 // Enable clock for GPIOC
  //   CLK_PeripheralClockConfig(CLK_PERIPHERAL_GPIOC, ENABLE);
	 
	 // Configure PC3, PC5, and PC6 as output for TIM1 (PWM)
   // PC3 (Red) -> TIM1_CH3, PC5 (Blue) -> TIM1_CH1, PC6 (Green) -> TIM1_CH2
   // Set PC3, PC5, and PC6 as alternate function push-pull outputs
    
		GPIO_Init(GPIOC, GPIO_PIN_3, GPIO_MODE_OUT_PP_HIGH_FAST);
		TIM1_OC3Init(TIM1_OCMODE_PWM2, TIM1_OUTPUTSTATE_ENABLE, TIM1_OUTPUTNSTATE_ENABLE,
                (uint16_t)4047, TIM1_OCPOLARITY_LOW, TIM1_OCNPOLARITY_HIGH, TIM1_OCIDLESTATE_SET,
                TIM1_OCNIDLESTATE_RESET);
		
		GPIO_Init(GPIOC, GPIO_PIN_4, GPIO_MODE_OUT_PP_HIGH_FAST);
		TIM1_OC4Init(TIM1_OCMODE_PWM2, TIM1_OUTPUTSTATE_ENABLE, (uint16_t) 4047, TIM1_OCPOLARITY_LOW,
                             TIM1_OCIDLESTATE_SET);
		
		
		GPIO_Init(GPIOC, GPIO_PIN_6, GPIO_MODE_OUT_PP_HIGH_FAST);
		TIM1_OC1Init(TIM1_OCMODE_PWM2, TIM1_OUTPUTSTATE_ENABLE, TIM1_OUTPUTNSTATE_ENABLE,
                (uint16_t) 4047, TIM1_OCPOLARITY_LOW, TIM1_OCNPOLARITY_HIGH, TIM1_OCIDLESTATE_SET,
                TIM1_OCNIDLESTATE_RESET); 
		
		
		TIM1_TimeBaseInit(0,TIM1_COUNTERMODE_UP, 4095,0);
		TIM1_Cmd(ENABLE);
		TIM1_CtrlPWMOutputs(ENABLE);
		
		

    // Initialize GPIOs for the pins used with the 74HC595
		// GPIO_Init (GPIOD, GPIO_PIN_6, GPIO_MODE_OUT_PP_HIGH_FAST);
    GPIO_Init(GPIOD, CLOCK_PIN, GPIO_MODE_OUT_PP_HIGH_FAST);  // SRCLK (Shift Clock)
    GPIO_Init(GPIOC, LATCH_PIN, GPIO_MODE_OUT_PP_HIGH_FAST);   // RCLK (Latch Clock)
    GPIO_Init(GPIOD, OE_PIN, GPIO_MODE_OUT_PP_HIGH_FAST);     // OE (Output Enable)
    GPIO_Init(GPIOC, DATA_PIN, GPIO_MODE_OUT_PP_HIGH_FAST);   // DATA (connected to DS)
    
    // Disable output (OE is active low)
    GPIO_WriteHigh(GPIOB, OE_PIN);


		// Initialize GPIOs for the pins used for the push buttons
		GPIO_Init(GPIOD, BUTTON_UP, GPIO_MODE_IN_PU_NO_IT);   // Pull-up input for UP button
    GPIO_Init(GPIOD, BUTTON_DOWN, GPIO_MODE_IN_PU_NO_IT); // Pull-up input for DOWN button



}



//  
int TenBit_2_Dec(uint32_t bits) {
    // Mask to extract the lower 10 bits
    uint32_t mask = 0x3FF;  // 10 bits mask (binary: 0000001111111111)
    
    // Extract the 10-bit number
    uint32_t value = bits & mask;

    // Check if the number is negative (if the 10th bit is 1, it is negative in 2's complement)
    if (value & (1 << 9)) {
        // If negative, convert to signed by extending the sign bit
        value |= ~mask;  // Set upper bits to 1
    }
    return (value);
    // return (int)value;  // Return as a signed integer
}


// Initialize the I2C (Bit-bang)
void I2C_init(void) {
    GPIO_Init(GPIOB, SDA_PIN, GPIO_MODE_OUT_OD_HIZ_SLOW);
    GPIO_Init(GPIOB, SCL_PIN, GPIO_MODE_OUT_OD_HIZ_SLOW);
    
    GPIO_WriteHigh(GPIOB, SDA_PIN);  // SDA high
    GPIO_WriteHigh(GPIOB, SCL_PIN);  // SCL high
}

// Small delay function to control the timing of I2C
void I2C_delay(void) {
uint16_t i;
for (i = 0; i < 50; i++);  // Adjust the delay as necessary
}

// Generate I2C start condition
void I2C_start(void) {
    GPIO_WriteHigh(GPIOB, SDA_PIN);  // SDA high
    GPIO_WriteHigh(GPIOB, SCL_PIN);  // SCL high
    I2C_delay();
    GPIO_WriteLow(GPIOB, SDA_PIN);   // SDA low
    I2C_delay();
    GPIO_WriteLow(GPIOB, SCL_PIN);   // SCL low
}

// Generate I2C stop condition
void I2C_stop(void) {
    GPIO_WriteLow(GPIOB, SDA_PIN);   // SDA low
    GPIO_WriteHigh(GPIOB, SCL_PIN);  // SCL high
    I2C_delay();
    GPIO_WriteHigh(GPIOB, SDA_PIN);  // SDA high
    I2C_delay();
}

// Write a single bit on the I2C bus
void I2C_write_bit(uint8_t bit) {
    if (bit) {
        GPIO_WriteHigh(GPIOB, SDA_PIN);  // SDA high
    } else {
        GPIO_WriteLow(GPIOB, SDA_PIN);   // SDA low
    }
    I2C_delay();
    GPIO_WriteHigh(GPIOB, SCL_PIN);  // SCL high
    I2C_delay();
    GPIO_WriteLow(GPIOB, SCL_PIN);   // SCL low
}

// Read a single bit from the I2C bus
uint8_t I2C_read_bit(void) {
    uint8_t bit;
    GPIO_WriteHigh(GPIOB, SDA_PIN);  // Release SDA (input mode)
    I2C_delay();
    GPIO_WriteHigh(GPIOB, SCL_PIN);  // SCL high
    I2C_delay();
    bit = GPIO_ReadInputPin(GPIOB, SDA_PIN) ? 1 : 0;
    GPIO_WriteLow(GPIOB, SCL_PIN);   // SCL low
    return bit;
}

// Write a byte to the I2C bus
uint8_t I2C_write_byte(uint8_t byte) {
		uint8_t i;
		for (i = 0; i < 8; i++) {
        I2C_write_bit((byte & 0x80) != 0);  // Write MSB first
        byte <<= 1;
    }
    return I2C_read_bit();  // Return ACK/NACK
}

// Read a byte from the I2C bus
uint8_t I2C_read_byte(uint8_t ack) {
    uint8_t byte = 0;
		uint8_t i;
    for (i = 0; i < 8; i++) {
        byte <<= 1;
        byte |= I2C_read_bit();
    }
    I2C_write_bit(ack);  // Send ACK/NACK
    return byte;
}



void set_ds3231_time(uint8_t hours, uint8_t minutes, uint8_t seconds, uint8_t day, uint8_t month, uint8_t year, uint8_t weekday) {
    I2C_start();
    I2C_write_byte(0xD0);  // DS3231 address with write bit
    I2C_write_byte(0x00);  // Set register pointer to 00h (seconds)
    I2C_write_byte(Decimal_To_BCD(seconds));  // Set seconds
    I2C_write_byte(Decimal_To_BCD(minutes));  // Set minutes
    I2C_write_byte(Decimal_To_BCD(hours));    // Set hours
		I2C_write_byte(Decimal_To_BCD(weekday));  // Set day
    I2C_write_byte(Decimal_To_BCD(day));  // Set month
    I2C_write_byte(Decimal_To_BCD(month));    // Set year
    I2C_write_byte(Decimal_To_BCD(year));    // Set weekday
    I2C_stop();
}

// Get the current time and date from the DS3231
void get_ds3231_time(uint8_t *hours, uint8_t *minutes, uint8_t *seconds, uint8_t *day, uint8_t *month, uint8_t *year, uint8_t *weekday) {
    I2C_start();
    I2C_write_byte( 0xD0 );  // DS3231 address + write bit
		I2C_write_byte(0x00);  // Start from seconds register
    I2C_start();
    I2C_write_byte( 0xD1);  // DS3231 address + read bit
		
    // Read time
    *seconds = BCD_To_Decimal(I2C_read_byte(0)); 
    *minutes = BCD_To_Decimal(I2C_read_byte(0));
    *hours = BCD_To_Decimal(I2C_read_byte(0));
    *weekday = BCD_To_Decimal(I2C_read_byte(0));
    *day = BCD_To_Decimal(I2C_read_byte(0));
    *month = BCD_To_Decimal(I2C_read_byte(0));
    *year = BCD_To_Decimal(I2C_read_byte(1));

    I2C_stop();
}




void DS3231_read_temperature(int16_t *temperature) {
    int8_t temp_msb;
    uint8_t temp_lsb;
    int16_t temp;
		
    I2C_start();
    I2C_write_byte(0xD0);  // DS3231 address with write bit
    I2C_write_byte(0x11);  // Set register pointer to 11h (temperature MSB)
    I2C_start();           // Repeated start
    I2C_write_byte(0xD1);  // DS3231 address with read bit
    temp_msb = I2C_read_byte(0);   // Read temperature MSB
    temp_lsb = I2C_read_byte(1);   // Read temperature LSB, send NACK
    *temperature = (temp_msb * 100) + ((temp_lsb >> 6)*25);
		I2C_stop();
    
    // Combine MSB and LSB to form the temperature value (0.25°C per LSB)
    
	}



void delay (uint16_t ms) //Function Definition
 {
                 uint16_t i =0 ;
                 int j=0;
                 for (i=0; i<=ms; i++)
                 {
                                 for (j=0; j<120; j++) // Nop = Fosc/4
                                 _asm("nop"); //Perform no operation //assembly code              
                 }
 }



// Shift 10 bits of data into the 595 shift registers
void shift_out(uint16_t data) {
		uint8_t	i;
    for (i = 0; i < 10; i++) {
        // Write the MSB (bit 7) to the DATA pin
        if (data & 0x1) {
            GPIO_WriteHigh(GPIOC, DATA_PIN);  // Set data high if bit is 1
        } else {
            GPIO_WriteLow(GPIOC, DATA_PIN);   // Set data low if bit is 0
        }
				// Pulse the SRCLK (Shift Clock)
        GPIO_WriteHigh(GPIOD, CLOCK_PIN);
        GPIO_WriteLow(GPIOD, CLOCK_PIN);
				// Shift to the next bit
        data >>= 1;
    }
}

uint16_t get10BitPattern(uint8_t digit) {
    // Ensure digit is within the valid range (0-9)
    if (digit > 9) {
        return 0; // Invalid digit, return 0
    }

    // Shift 1 to the left by (9 - digit) positions to create the 10-bit pattern
    return (1 << (9 - digit));
		
}


// Read button state with software debouncing
uint8_t read_button(GPIO_Pin_TypeDef button) {
    if (GPIO_ReadInputPin(GPIOD, button) == RESET) {
        delay (DEBOUNCE_DELAY_MS); // Debounce delay
        if (GPIO_ReadInputPin(GPIOD, button) == RESET) {
            while (GPIO_ReadInputPin(GPIOD, button) == RESET); // Wait for button release
            return 1; // Button pressed
        }
    }
    return 0; // Button not pressed
}




void display_nixxies(uint8_t hours, uint8_t minutes){

		if (hours < 99){
		shift_out(get10BitPattern(((hours / 10) % 10)));
		shift_out(get10BitPattern((hours % 10)));
		}
		else {
		shift_out(0);
		shift_out(0);
		}
		if (minutes < 99){
		shift_out(get10BitPattern(((minutes /10)% 10)));
		shift_out(get10BitPattern((minutes % 10)));
		}
		else {
		shift_out(0);
		shift_out(0);
		}
		latch_data();      // Latch the shifted data to the output registers
		enable_output();
}






int main(void) {
		uint8_t hour, minute, second;
		uint8_t hours, minutes, seconds, oldseconds;
		uint8_t year, month, day, weekday;
		uint8_t red, green, blue;
		uint8_t i;
		int16_t temperature;
		uint8_t nixie_digits[]={0x0,0x1,0x2,0x3};
		uint16_t teller;
		
		// declare I2c ports
		gpio_init1();

		I2C_init();
		
		
		/* Define FLASH programming time */
    FLASH_SetProgrammingTime(FLASH_PROGRAMTIME_STANDARD);

    /* Unlock Data memory */
    FLASH_Unlock(FLASH_MEMTYPE_DATA);
			
		/* retrieve RGB parameters from flash memory */
		red = FLASH_ReadByte(0x40A5);
		green = FLASH_ReadByte(0x40A6);
		blue = FLASH_ReadByte(0x40A7);
		
		/* set RGB LED values */
		TIM1_SetCompare3 (red *16 ) ; // red
		TIM1_SetCompare1 (green * 16) ; // green
		TIM1_SetCompare4 (blue * 16 ) ; // blue
		

   
    // Infinite loop to read the time continuously
		while (1) {
  
			// Read the time
			get_ds3231_time(&hours, &minutes, &seconds, &day, &month, &year, &weekday);
			
			// Gget DS3231 internal temperature
			DS3231_read_temperature(&temperature);    // Simple delay to slow down the loop
		
		
			// TIM1_SetCompare1(0); // green
			// TIM1_SetCompare3((4096 / 60) * seconds ); // red
			// TIM1_SetCompare4(0);	// blue
	
			
			if (GPIO_ReadInputPin(GPIOD, BUTTON_DOWN) == RESET){
			 i= editRGB (0,0,244);	
			 i= editRGB (1,0,244);	
			 i= editRGB (2,0,244);	
			}
			
			
			if (GPIO_ReadInputPin(GPIOD, BUTTON_UP) == RESET){
				i=0;
				hours = edit (0,0, hours);
				minutes = edit (1,0, minutes);
				day = edit (2,0, day);
				month = edit (3, 0, month);
				year = edit (4,0, year);
			}
		
		
			if ( seconds > 45 && seconds < 50 ) //show temperature (int. part)
				{
				GPIO_WriteLow(GPIOD,GPIO_PIN_6);
				display_nixxies( 0xAA , (temperature / 100));
				} 
			else 
			{
				if ( seconds	> 30 && seconds < 35) // show date, alternating display (day + month) & ( 20 + year )
				{
					if ( seconds & 1){ // 
						GPIO_WriteLow(GPIOD,GPIO_PIN_6);
						display_nixxies( day, month);
						}
					else {
						GPIO_WriteLow(GPIOD,GPIO_PIN_6);
						display_nixxies( 20, year);	
						}
				}	
				else // normal show hours + minutes
				{
						if (oldseconds != seconds ){
							oldseconds = seconds;	
							GPIO_WriteReverse(GPIOD,GPIO_PIN_6); // toggle the semicolon every second
						} 
				display_nixxies( hours, minutes);
				}
		}
				
		}
}






uint8_t BCD_To_Decimal(uint8_t bcd) {
    return ((bcd >> 4) * 10) + (bcd & 0x0F);
}

uint8_t Decimal_To_BCD(uint8_t decimal) {
    return ((decimal / 10) << 4) | (decimal % 10);
}



// Latch the shifted data into the output registers
void latch_data(void) {
    // Pulse the RCLK (Register Clock)
    GPIO_WriteHigh(GPIOC, LATCH_PIN);
    GPIO_WriteLow(GPIOC, LATCH_PIN);
}

// Enable the output (OE is active low)
void enable_output(void) {
    GPIO_WriteLow(GPIOD, OE_PIN);  // Set OE low to enable output
}

// Disable the output (OE is disabled = high)
void disable_output(void) {
    GPIO_WriteHigh(GPIOD, OE_PIN);  // Set OE high to disable output
}


uint8_t edit(uint8_t i, uint8_t y, uint8_t parameter){  // char text[3];
  	uint8_t hour, minute, second;
		uint8_t hours, minutes, seconds, oldseconds;
		uint8_t year, month, day, weekday;


get_ds3231_time(&hours, &minutes, &seconds, &day, &month, &year, &weekday);

switch (i){
										case 0:
											parameter = hours;
											break;
										case 1:
											parameter = minutes;
											break;
										case 2:
											parameter = day;
											break;
										case 3:
											parameter = month;
											break;
										case 4:
											parameter = year;
											break;
										case 5:
											parameter = weekday;
										default:
											break;
										}
										

while(GPIO_ReadInputPin(GPIOD, BUTTON_UP) == RESET){
}                        // Wait until button (pin #8) is released

	
	while(TRUE){
				
				//				while((GPIO_ReadInputPin(GPIOD, BUTTON_DOWN) == RESET)) 
				
								while(read_button (BUTTON_DOWN)) {                      // If button (pin #9) is pressed
									
									parameter++;
									
									if (i == 0 && parameter > 23){               // If hours > 23 ==> hours = 0
										parameter = 0;}
										
									if (i == 1 && parameter > 59){               // If minutes > 59 ==> minutes = 0
									parameter = 0;}
											// display_nixxies( hours, minutes);
									if (i == 2 && parameter > 31){               // If date > 31 ==> date = 1
									parameter = 1;}
									
									if (i == 3 && parameter > 12){               // If month > 12 ==> month = 1
									parameter = 1;}
									
									if (i == 4 && parameter > 99){               // If year > 99 ==> year = 0
									parameter = 0;}
							 
									if (i == 5 && parameter > 7){               // If year > 99 ==> year = 0
									parameter = 1;}
							 
}								
									
									switch (i){
										case 0:
											GPIO_WriteHigh(GPIOD,GPIO_PIN_6);
											display_nixxies( 255, minutes);
											break;
										case 1:
											GPIO_WriteHigh(GPIOD,GPIO_PIN_6);
											display_nixxies( hours, 255);
											break;
										case 2:
											GPIO_WriteLow(GPIOD,GPIO_PIN_6);
											display_nixxies( 255, month);
											break;
										case 3:
											GPIO_WriteLow(GPIOD,GPIO_PIN_6);
											display_nixxies( day, 255);
											break;
										case 4:
											GPIO_WriteLow(GPIOD,GPIO_PIN_6);
											display_nixxies( 20, 255);
											break;											
										case 5:
											GPIO_WriteLow(GPIOD,GPIO_PIN_6);
											display_nixxies( 255, 255);
											break;											
										default:
											GPIO_WriteHigh(GPIOD,GPIO_PIN_6);
											display_nixxies( 255, 255);
											break;
											
										}
							 
									delay (200);                              // Wait 200ms
									
								
    
								switch (i){
										case 0:
											display_nixxies( parameter, minutes);
											break;
										case 1:
											display_nixxies( hours, parameter);
											break;
										case 2:
											display_nixxies( parameter, month);
											break;
										case 3:
											display_nixxies( day, parameter);
											break;
										case 4:
											display_nixxies( 20, parameter);
											break;
										case 5:
											display_nixxies( parameter, 255);
											break;					
										default:
											display_nixxies( 255, 255);
											break;
										}
									
									delay (200);

								if (read_button(BUTTON_UP)) {                         // If button (pin #8) is pressed
								
									switch (i){
										case 0:
											hours = parameter;
											break;
										case 1:
											minutes = parameter;
											break;
										case 2:
											day = parameter;
											break;
										case 3:
											month = parameter;
											break;
										case 4:
											year = parameter;
											break;
										case 5:
											weekday = parameter;
										default:
											break;	
										}
										seconds = 0;
								set_ds3231_time(hours, minutes, seconds, day, month, year, weekday);  // Update the time with any changes
							//	i++;                                       // Increament 'i' for the next parameter
								return parameter;                          // Return parameter value and exit
								}
								
				
		}
	
	
}





uint8_t editRGB(uint8_t i, uint8_t y, uint8_t parameter){  // char text[3];
  	uint8_t red, green, blue;
		uint32_t mem = 0x00;
		
		
		red = FLASH_ReadByte(0x40A5);
		green = FLASH_ReadByte(0x40A6);
		blue = FLASH_ReadByte(0x40A7);		

switch (i){
										case 0:  // red
											parameter = red;
											break;
										case 1:  // green
											parameter = green;
											break;
										case 2:  // blue
											parameter = blue;
											break;
										default:
											break;
										}
										

while(GPIO_ReadInputPin(GPIOD, BUTTON_UP) == RESET){
}                        // Wait until button is released

	
	while(TRUE){
				
				//				while((GPIO_ReadInputPin(GPIOD, BUTTON_DOWN) == RESET)) 
				
								while(read_button (BUTTON_UP)) {                      // If button (pin #9) is pressed
									
									parameter++;
									
									
}								
									
									switch (i){
										case 0:
											TIM1_SetCompare3(( parameter * 16)); // red
											display_nixxies( 1, 255);
											break;
										case 1:
											TIM1_SetCompare1(( parameter * 16)); // green
											display_nixxies( 2, 255);
											break;
										case 2:
											TIM1_SetCompare4(( parameter * 16));	// blue
											display_nixxies( 3, 255);
											break;
										default:
											GPIO_WriteHigh(GPIOD,GPIO_PIN_6);
											display_nixxies( 255, 255);
											break;
											
										}
							 
									delay (200);                              // Wait 200ms
									
								
    
								switch (i){
										case 0: // edit hours
											
											display_nixxies( 1, parameter);
											break;
										case 1: // edit minutes
											display_nixxies( 2, parameter);
											break;
										case 2: // edit day
											display_nixxies( 3, parameter);
											break;
										default:
											display_nixxies( 255, 255);
											break;
										}
									
									delay (200);

								if (read_button(BUTTON_DOWN)) {                         // If button (pin #8) is pressed
								
									switch (i){
										case 0: // exit hours edit
											red = parameter;
										//		FLASH_ProgramByte(0x40A5,red);
											break;
										case 1: // exit minutes edit
										//	FLASH_ProgramByte(0x40A6,green);
											green = parameter;
											break; 
										case 2: // exit day edit
											blue = parameter;
										//	FLASH_ProgramByte(0x40A7,blue);
											break;
										default:
											break;	
										}
										// write new parameters to flash memory
										FLASH_ProgramByte(0x40A5,red);
										FLASH_ProgramByte(0x40A6,green);
										FLASH_ProgramByte(0x40A7,blue);
										// set RGB LED 
										TIM1_SetCompare1(( green * 16)); // green
										TIM1_SetCompare3(( red * 16)); // red
										TIM1_SetCompare4(( blue * 16));	// blue
	

										// set_ds3231_time(hours, minutes, seconds, day, month, year, weekday);  // Update the time with any changes
										// i++;
								return parameter;                          // Return parameter value and exit
								}
						
		
				
		}
	
	
}




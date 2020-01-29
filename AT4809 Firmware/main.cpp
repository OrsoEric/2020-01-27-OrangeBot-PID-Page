/****************************************************************
**	OrangeBot Project
*****************************************************************
**        /
**       /
**      /
** ______ \
**         \
**          \
*****************************************************************
**	OrangeBot AT4809 Firmware
*****************************************************************
**  This firmware is meant to run on the OrangeBot robot due to
**	participate in the PiWars 2020
**
**	Compiler flags: -std=c++11 -fno-threadsafe-statics -fkeep-inline-functions
**		-Os							|
**		-std=c++11					|
**		-fno-threadsafe-statics		| disable mutex around static local variables
**		-fkeep-inline-functions		| allow use of inline methods outside of .h for PidS16
****************************************************************/

/****************************************************************
**	DESCRIPTION
****************************************************************
**	OrangeBot MVP
**	UART interface with RPI
**	Uniparser V4
**	Set PWM from serial interface
****************************************************************/

/****************************************************************
**	HISTORY VERSION
****************************************************************
**		2019-11-01
**	Add UART3 uc <--> RPI
**	Add Uniparser V4
**	Add platform messages
**	Test OrangeBot Remote Control MVP
**		2019-11-05
**	Clone 2019-10-02 OrangeBot MVP Remote control
**	Add Encoder ISR
**		2019-11-09
**	Added Pid S16 class
**		2020-01-01
**	set_platform_speed_handler update to two motor layout
**		2020-01-02
**	A difficult to find bug about following PWM references. might be something deep
**	Prune out everything and slowly add modules back in
**		2020-01-03
**	Reworked VNH7040 HAL API to hide direction, 8bit PWM and reverse layout.
**		2020-01-23
**	Added PID Class
**	Added speed PID. 
**	ROBOT_PWM message. ROBOT_SPD message
**		2020-01-25
**	Main loop rate counter
**	UART bandwidth counters
**	AT4809 performance message
**		2020-01-26
**	BUG: Found a bug with the s16_to_str library!!! Rework library since I learned a lot since then.
**	BUGFIX: Updated from at_string.h to String_uc.h. It's a class based on templates and static methods with class enum definitions
**		2020-01-27
**	BUG: Sometimes PWM commands are not processed correctly
**	BUGFIX: The problem was that with larger commands, the UART RX buffer of 16B was not enough and lost char along the way
**	Updated Uniparser V5 class library to test for runtime argument digit overflow. Some minor overflow on last digit might still happen
**	Added answer to SET_SPD_PID message
**	Updated pid_s16 version 2020-01-23 without saturation handler callback
****************************************************************/

/****************************************************************
**	USED PINS
****************************************************************
**	VNH7040
**				|	DRV0	|	DRV1	|	DRV2	|	DRV3	|	VNH7040
**	-------------------------------------------------------------------------
**	uC_SEN		|	PF0		|	PF0		|	PF0		|	PF0		|	SENSE ENABLE
**	uC_DIAG		|	PF1		|	PF1		|	PF1		|	PF1		|	SEL1
**	uC_PWM		|	PA2,B20	|	PA3,B21	|	PB4,B22	|	PB5,B23	|	PWM
**	uC_CTRLA	|	PA4		|	PA6		|	PB2		|	PD6		|	INA, SEL0
**	uC_CTRLB	|	PA5		|	PA7		|	PB3		|	PD7		|	INB
****************************************************************/

/****************************************************************
**	KNOWN BUGS
****************************************************************
**		2020-01-26a
**	BUG: Found a bug with the s16_to_str library!!! Rework library since I learned a lot since then.
**	BUGFIX: Updated from at_string.h to String_uc.h. It's a class based on templates and static methods with class enum definitions
**		2020-01-27a
**	BUG: Sometimes PWM commands are not processed correctly
**	BUGFIX: The problem was that with larger commands, the UART RX buffer of 16B was not enough and lost char along the way
****************************************************************/

/****************************************************************
**	DEFINES
****************************************************************/

#define EVER (;;)

/****************************************************************
**	INCLUDES
****************************************************************/

//type definition using the bit width and signedness
#include <stdint.h>
//define the ISR routune, ISR vector, and the sei() cli() function
#include <avr/interrupt.h>
//name all the register and bit
#include <avr/io.h>
//General purpose macros
#include "at_utils.h"
//AT4809 PORT macros definitions
#include "at4809_port.h"
//from number to string
//#include "string_uc.h"
//Universal Parser V4
#include "uniparser.h"

#include "global.h"
//hard delay
#include <util/delay.h>

/****************************************************************
** FUNCTION PROTOTYPES
****************************************************************/

extern bool init_parser_commands( Orangebot::Uniparser &parser_tmp );
	
	///----------------------------------------------------------------------
	///	PERIPHERALS
	///----------------------------------------------------------------------

	///----------------------------------------------------------------------
	///	PID
	///----------------------------------------------------------------------


/****************************************************************
** GLOBAL VARIABLES
****************************************************************/

volatile Isr_flags g_isr_flags;

	///----------------------------------------------------------------------
	///	BUFFERS
	///----------------------------------------------------------------------
	//	Buffers structure and data vectors

//Safe circular buffer for UART input data
volatile At_buf8_safe rpi_rx_buf;
//Safe circular buffer for uart tx data
At_buf8 rpi_tx_buf;
//allocate the working vector for the buffer
uint8_t v0[ RPI_RX_BUF_SIZE ];
//allocate the working vector for the buffer
uint8_t v1[ RPI_TX_BUF_SIZE ];
//Raspberry PI UART RX Parser
Orangebot::Uniparser rpi_rx_parser;

	///--------------------------------------------------------------------------
	///	CONTROL SYSTEM
	///--------------------------------------------------------------------------

	///--------------------------------------------------------------------------
	///	MOTORS
	///--------------------------------------------------------------------------

	///--------------------------------------------------------------------------
	///	ENCODERS
	///--------------------------------------------------------------------------

/****************************************************************************
**  Function
**  main |
****************************************************************************/
//! @return bool |
//! @brief dummy method to copy the code
//! @details test the declaration of a lambda method
/***************************************************************************/

int main(void)
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//system tick prescaler
	uint8_t pre = 0;
	//activity LED prescaler
	uint8_t pre_led = 0;
	//Blink speed of the LED. Start slow
	uint8_t blink_speed = 99;
	//Prescaler for slow code
	uint16_t pre_slow = 0;
	
	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

		///UART RX BUFFER INIT
	//I init the rx and tx buffers
	//attach vector to buffer
	AT_BUF_ATTACH( rpi_rx_buf, v0, RPI_RX_BUF_SIZE);
	//attach vector to buffer
	AT_BUF_ATTACH( rpi_tx_buf, v1, RPI_TX_BUF_SIZE);

	//! Initialize AT4809 internal peripherals
	init();

		//!	Initialize VNH7040
	//Enable sense output
	SET_BIT_VALUE( PORTF.OUT, 0, true );
	//Diagnostic mode OFF
	SET_BIT_VALUE( PORTF.OUT, 1, false );
	
		///----------------------------------------------------------------------
		///	REGISTER PARSER COMMANDS
		///----------------------------------------------------------------------
	
	//Initialize UART parser from RPI to OrangeBot Motor Board
	init_parser_commands( rpi_rx_parser );
	//Initialize all control systems
	init_ctrl_system();
	
	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	//Main loop
	for EVER
	{
		//----------------------------------------------------------------
		//	CPU LOAD
		//----------------------------------------------------------------
		//	Count how many main loop scans per seconds happen
		//	The more work ISR and modules have to do, the less scans per seconds happen
		
		//If: performance profiling is enabled by RPI message
		if (g_b_enable_performance == true)
		{
			//Main loop has been executed. Clip number of counts to max counter
			g_board_performance = AT_TOP_INC( g_board_performance, 65535 );	
		}
		
		//----------------------------------------------------------------
		//	MAIN SYSTEM TICK
		//----------------------------------------------------------------
		//	1ms from RTC counter
		
		//If: System Tick
		if (g_isr_flags.system_tick == 1)
		{
			//Clear system tick
			g_isr_flags.system_tick = 0;
															
			//----------------------------------------------------------------
			//	SLOW CODE
			//----------------------------------------------------------------
							
			//If: performance profiling is enabled by RPI message
			if (g_b_enable_performance == true)
			{
				//If prescaler rollover
				if (pre_slow == 0)
				{
					//Send performance message and reset performance counters
					send_message_performance();
				}
				//Increment with top
				pre_slow = AT_TOP_INC( pre_slow, PERFORMANCE_PRESCALER );
			}
				
			//If control system prescaler has reset
			if (pre == 0)
			{
				//Execute speed PID
				g_isr_flags.ctrl_updt = true;
								
				//----------------------------------------------------------------
				//	LED BLINK
				//----------------------------------------------------------------
				//		Two speeds
				//	slow: not in timeout and commands can be executed
				//	fast: in timeout, motor stopped
				
				if (pre_led == 0)
				{
					//Toggle PF5.
					SET_BIT( PORTF.OUTTGL, 5 );	
				}
				//Increment with top
				pre_led = AT_TOP_INC( pre_led, blink_speed );
				
				//----------------------------------------------------------------
				//	PARSER TIMEOUT
				//----------------------------------------------------------------
				
				//If: timeout not detected
				if (g_f_timeout_detected == false)
				{
					//Update communication timeout counter
					g_uart_timeout_cnt++;
					//If: timeout 
					if (g_uart_timeout_cnt >= RPI_COM_TIMEOUT)
					{
						//If: it's the first time the communication timeout is detected
						if (g_f_timeout_detected == false)
						{
							//LED is blinking faster
							blink_speed = 9;
						}
						//raise the timeout flag
						g_f_timeout_detected = true;
						//Set motors to full stop
						g_control_mode_target = CONTROL_STOP;
						//If, communication timeout was detected
						report_error( ERR_CODE_COMMUNICATION_TIMEOUT );
					}
				}
				//If: currently in timeout
				else
				{
					//If timeout counter has been reset
					if (g_uart_timeout_cnt < RPI_COM_TIMEOUT)
					{
						//This is the only code allowed to reset the timeout flag
						g_f_timeout_detected = false;
						//LED is blinking slower
						blink_speed = 99;
					}
				}
			}	//If: Control System Prescaler
			//Increment prescaler and reset if it exceeds the TOP.
			pre = AT_TOP_INC( pre, PRE_CTRL_SYS);
			
		}	//End If: System Tick
		
		//----------------------------------------------------------------
		//	VNH7040 MOTOR CONTROL SYSTEM
		//----------------------------------------------------------------		
		
		//If: Authorized to execute one step the PID speed controllers
		if (g_isr_flags.ctrl_updt == 1)
		{
			//Clear flag
			g_isr_flags.ctrl_updt = 0;
			//Execute the control system for all VNH7040 motor controllers
			control_system();
		}	//End If: Authorized to execute one step the PID speed controllers
		
		//----------------------------------------------------------------
		//	AT4809 --> RPI USART TX
		//----------------------------------------------------------------
		
		//if: RPI TX buffer is not empty and the RPI TX HW buffer is ready to transmit
		if ( (AT_BUF_NUMELEM( rpi_tx_buf ) > 0) && (IS_BIT_ONE(USART3.STATUS, USART_DREIF_bp)))
		{
			//temp var
			uint8_t tx_tmp;
			//Get the byte to be filtered out
			tx_tmp = AT_BUF_PEEK( rpi_tx_buf );
			AT_BUF_KICK( rpi_tx_buf );
			//Send data through the UART3
			USART3.TXDATAL = tx_tmp;
			//If: performance profiling is enabled by RPI message
			if (g_b_enable_performance == true)
			{
				//A byte has been transmitted, eating up bandwidth
				g_board_rpi_txo_bandwidth++;
			}
		}	//End If: RPI TX
		
		//----------------------------------------------------------------
		//	RPI --> AT4809 USART RX
		//----------------------------------------------------------------
		
		//if: RX buffer is not empty	
		if (AT_BUF_NUMELEM( rpi_rx_buf ) > 0)
		{
			//temp var
			uint8_t rx_tmp;
				
				///Get data
			//Get the byte from the RX buffer (ISR put it there)
			rx_tmp = AT_BUF_PEEK( rpi_rx_buf );
			AT_BUF_KICK_SAFER( rpi_rx_buf );

				///Loop back
			//Push into tx buffer
			//AT_BUF_PUSH( rpi_tx_buf, rx_tmp );

				///Command parser
			//feed the input RX byte to the parser and listen for errors
			if (rpi_rx_parser.parse( rx_tmp ) == true)
			{
				report_error( ERR_UNIPARSER_RUNTIME );
			}
			
			//If: performance profiling is enabled by RPI message
			if (g_b_enable_performance == true)
			{
				//A byte has been received and processed, eating up bandwidth
				g_board_rpi_rxi_bandwidth++;
			}
			
		} //endif: RPI RX buffer is not empty

	}	//End: Main loop

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------

	return 0;
}	//end: main

/***************************************************************************/
//!	@brief function
//!	function_template
/***************************************************************************/
//! @param x |
//! @return void |
//! @details
/***************************************************************************/

void function_template( void )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------
	
	return;
}	//End function: 



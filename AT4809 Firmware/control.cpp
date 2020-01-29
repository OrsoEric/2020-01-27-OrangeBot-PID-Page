/****************************************************************************
**	INCLUDE
****************************************************************************/

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
//Program wide definitions
#include "global.h"
//This class handles slope on multiple PWM channels
#include "ctrl_pwm.h"
//Generic S16 PID Class
#include "pid_s16.h"

/****************************************************************************
**	NAMESPACES
****************************************************************************/

/****************************************************************************
**	GLOBAL VARS
****************************************************************************/

//Actual and target control system mode
Control_mode g_control_mode					= CONTROL_STOP;
Control_mode g_control_mode_target			= CONTROL_STOP;
//Slew rate limiter controller for the PWM channels
Orangebot::Ctrl_pwm g_vnh7040_pwm_ctrl;
//Each encoder has an associated PID controller
Orangebot::Pid_s16 g_vnh7040_spd_pid[ NUM_ENC ];
//PID speed reference
int16_t g_pid_spd_target[NUM_ENC];

/****************************************************************************
**	FUNCTION
****************************************************************************/

/***************************************************************************/
//!	@brief function
//!	init_ctrl_system | void
/***************************************************************************/
//! @param x |
//! @return void |
//! @details
/***************************************************************************/

bool init_ctrl_system( void )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//Counter
	uint8_t t;

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	//Initialize PWM Slew Rate Limiter
	g_vnh7040_pwm_ctrl = Orangebot::Ctrl_pwm( MAX_VNH7040_PWM, MAX_VNH7040_PWM_SLOPE );
	//For each encoder
	for (t = 0;t < NUM_ENC;t++)
	{
		//Initialize reference
		g_pid_spd_target[ t ] = (int16_t)0;
		//Initialize the control system associated with that encoder
		g_vnh7040_spd_pid[ t ] = Orangebot::Pid_s16();
		//Initialize Unlock handler
		g_vnh7040_spd_pid[ t ].limit_sat_th() = PID_SAT_TH;
		//Set command limiter
		g_vnh7040_spd_pid[ t ].limit_cmd_max() = MAX_VNH7040_PWM;
		g_vnh7040_spd_pid[ t ].limit_cmd_max() = -MAX_VNH7040_PWM;
	}

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------
	
	return false; //OK
}	//End function: init_ctrl_system | void

/***************************************************************************/
//!	@brief function
//!	control_system
/***************************************************************************/
//! @return bool | true: failed to execute control system
//! @brief select and execute correct control system
//! @details
/***************************************************************************/

bool control_system( void )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//Counter
	uint8_t t;

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	//----------------------------------------------------------------
	//	CONTROL MODE
	//----------------------------------------------------------------
	
	//If: switch of control mode
	if (g_control_mode != g_control_mode_target)
	{
		//Control mode is the one desired by the user
		g_control_mode = g_control_mode_target;
		//Signal switch to main
		send_msg_ctrl_mode();
		//For: Each VNH7040 motor driver
		for (t= 0;t < NUM_VNH7040;t++)
		{
			//Reset the PWM controllers
			g_vnh7040_pwm_ctrl.reset();	
		}
		//For: Each encoder
		for (t= 0;t < NUM_ENC;t++)
		{
			//Reset the Speed PID controller
			g_vnh7040_spd_pid[t].reset();
		}
		
		
		
	}
	//otherwise control mode is already correct
	else
	{
		//Do nothing
	}
	
		//----------------------------------------------------------------
		//	UPDATE ENCODER POSITION AND SPEED
		//----------------------------------------------------------------

	//Update position and speed encoder readings
	if (process_enc() == true)
	{
		return true;  //FAIL
	}
	
		//----------------------------------------------------------------
		//	EXECUTE CONTROL SYSTEM
		//----------------------------------------------------------------
	
	//! Execute a step in the right control mode
	//If: control system is in STOP state
	if (g_control_mode == CONTROL_STOP)
	{
		//Forcefully reset the PWM controller
		g_vnh7040_pwm_ctrl.reset();
		//For: every VNH7040 motor controller
		for (t = 0;t < NUM_VNH7040;t++)
		{
			//Set driver to stopped
			set_vnh7040_pwm( t, 0 );
		} //End for: every VNH7040 motor controller
	}	//End If: control system is in STOP state
	//If: control system is open loop PWM
	else if (g_control_mode == CONTROL_PWM)
	{
		//Execute slew rate limiter
		g_vnh7040_pwm_ctrl.update();
		//For: every VNH7040 motor controller
		for (t = 0;t < NUM_VNH7040;t++)
		{
			//Apply computed PWM settings to the motors
			set_vnh7040_pwm( t, g_vnh7040_pwm_ctrl.pwm(t) );
		} //End for: every VNH7040 motor controller
	}	//End If: control system is open loop PWM
	//If: control system is closed loop speed
	else if (g_control_mode == CONTROL_SPD)
	{
		//For: every encoder
		for (t = 0;t < NUM_ENC;t++)
		{
			//Execute control system and Feed command to the PWM slew rate limiter
			g_vnh7040_pwm_ctrl.target( t ) = g_vnh7040_spd_pid[t].exe( g_pid_spd_target[t], g_enc_spd[t] );
		}
		//Execute slew rate limiter
		g_vnh7040_pwm_ctrl.update();
		//For: every VNH7040 motor controller
		for (t = 0;t < NUM_VNH7040;t++)
		{
			//Apply computed PWM settings to the motors
			set_vnh7040_pwm( t, g_vnh7040_pwm_ctrl.pwm(t) );
		} //End for: every VNH7040 motor controller
	}	//End If: control system is closed loop speed
	//if: undefined control system
	else
	{
		//Signal error
		report_error( ERR_CODE_UNDEFINED_CONTROL_SYSTEM );
		//Reset control mode
		g_control_mode_target = CONTROL_STOP;
	}	//End if: undefined control system

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------
	
	return false; //OK
}	//End function: control_system | void

/***************************************************************************/
//!	@brief function
//!	set_platform_pwm | int16_t | int16_t
/***************************************************************************/
//! @param right | int16_t | pwm to apply to the right side wheel(s)
//! @param left | int16_t | pwm to apply to the left side wheel(s)
//! @return bool | false = OK | true = a wrong VNH7040 index was given
//! @brief Control PWM of the platform to move according to the layout
//! @details
//!	HAL Hardware Abstraction Layer API for the VNH7040 driver
//! Control PWM of the platform to move according to the layout
/***************************************************************************/

bool set_platform_pwm( int16_t right, int16_t left )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------
	
	//int16_t data[2];
	
	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	//The PWM references are set as target for the PWM slope controller
	g_vnh7040_pwm_ctrl.target( 0 ) = right;
	g_vnh7040_pwm_ctrl.target( 1 ) = left;
	//Debug: send actual PWM back as response to PWM message
	//data[0] = g_vnh7040_pwm_ctrl.pwm( 0 );
	//data[1] = g_vnh7040_pwm_ctrl.pwm( 1 );
	//Debug send message
	//send_data( 2, data );
	
	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------
	
	return false; //OK
}	//End function:

/***************************************************************************/
//!	@brief function
//!	set_platform_spd | int16_t | int16_t
/***************************************************************************/
//! @param right | int16_t | spd to apply to the right side wheel(s)
//! @param left | int16_t | spd to apply to the left side wheel(s)
//! @return bool | false = OK | true = a wrong encoder index was given
//! @brief Control PWM of the platform to move according to the layout
//! @details
//!	HAL Hardware Abstraction Layer API
//! Control SPD of the platform to move according to the layout
/***************************************************************************/

bool set_platform_spd( int16_t right, int16_t left )
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

	//The PWM references are set as target for the PWM slope controller
	g_pid_spd_target[0] = right;
	g_pid_spd_target[1] = -left;
	
	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------
	
	return false; //OK
}	//End function: set_platform_spd | int16_t | int16_t

/***************************************************************************/
//!	@brief function
//!	set_spd_pid | int16_t, int16_t, int16_t
/***************************************************************************/
//! @param gain_kp | proportional gain
//! @param gain_ki | integral gain
//! @param gain_kd | derivative gain
//! @return bool | always succeed
//! @details
//! Set the SPD PID gain for all encoder channels
/***************************************************************************/

bool set_spd_pid( int16_t gain_kp, int16_t gain_ki, int16_t gain_kd )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//Counter
	uint8_t t;

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	//For: each encoder channel
	for (t = 0;t < NUM_ENC;t++)
	{
		//Set given encoder channel speed PID parameter
		g_vnh7040_spd_pid[t].gain_kp() = gain_kp;
		g_vnh7040_spd_pid[t].gain_ki() = gain_ki;
		g_vnh7040_spd_pid[t].gain_kd() = gain_kd;
	}

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------
	
	return false; //OK
}	//End function: set_spd_pid | int16_t, int16_t, int16_t


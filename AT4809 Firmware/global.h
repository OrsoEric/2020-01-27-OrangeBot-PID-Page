#ifndef GLOBAL_H
	//header environment variable, is used to detect multiple inclusion
	//of the same header, and can be used in the c file to detect the
	//included library
	#define GLOBAL_H

	/****************************************************************************
	**	ENVROIMENT VARIABILE
	****************************************************************************/

	#define F_CPU 20000000

	/****************************************************************************
	**	GLOBAL INCLUDE
	**	TIPS: you can put here the library common to all source file
	****************************************************************************/

	/****************************************************************************
	**	DEFINE
	****************************************************************************/

		///----------------------------------------------------------------------
		///	BUFFERS
		///----------------------------------------------------------------------

	#define RPI_RX_BUF_SIZE		32
	#define RPI_TX_BUF_SIZE		64
	
		///----------------------------------------------------------------------
		///	PARSER
		///----------------------------------------------------------------------
	
	//Communication timeout in communication system ticks		
	#define RPI_COM_TIMEOUT			200
	//Maximum size of a signature string
	#define MAX_SIGNATURE_LENGTH	32
	//How often send AT4809 performance counters to the RPI if enabled
	#define PERFORMANCE_PRESCALER	1000
	//How much divide the UART bandwidth performance counter
	#define UART_BANDWIDTH_SCALE	1
	
		///----------------------------------------------------------------------
		///	VNH7040 DC MOTOR CONTROLLER
		///----------------------------------------------------------------------
	
	//Number of VNH7040 DC Motor drivers installed
	#define NUM_VNH7040				4
	//PWM limits
	#define MIN_VNH7040_PWM			16		//Too low PWM settings do not result in motion
	#define MAX_VNH7040_PWM			127		//Limit maximum PWM setting
	#define MAX_VNH7040_PWM_SLOPE	10		//Maximum PWM change in PWM unit per control system tick. In change of direction is forced at least one pass from zero
	//Set which motors need to be controlled in reverse to achieve forward motion with positive PWM
	#define LAYOUT_VNH7040_REVERSE	(0)

		///----------------------------------------------------------------------
		///	ENCODERS
		///----------------------------------------------------------------------
	
	//Number of quadrature encoders
	#define NUM_ENC				2
	//Threshold upon which local counters are synced with global counters
	#define ENC_UPDATE_TH		100
	//Number of times allowed to retry an update of global encoder vars before failing
	#define ENC_UPDATE_RETRY	3
	
		///----------------------------------------------------------------------
		///	CONTROL SYSTEM
		///----------------------------------------------------------------------

	//Prescaler of the control system from the main system tick
	#define PRE_CTRL_SYS		32
	
		///----------------------------------------------------------------------
		///	PID
		///----------------------------------------------------------------------
		
	//Number of a PID gain parameters
	#define NUM_PID_GAINS		3
	//If PID is saturated for this number of update, the PID is unlocked in error
	#define PID_SAT_TH			100
	
	/****************************************************************************
	**	ENUM
	****************************************************************************/
		
	//Control modes
	typedef enum _Control_mode
	{
		CONTROL_STOP,	//Emergency stop mode
		CONTROL_PWM,		//PWM control mode. Just a slew rate limiter between user and drivers
		CONTROL_SPD,		//Speed control mode
		CONTROL_POS			//Position control mode
	} Control_mode;

	//Error codes that can be experienced by the program
	typedef enum _Error_code
	{
		ERR_CODE_UNDEFINED_CONTROL_SYSTEM,
		ERR_CODE_COMMUNICATION_TIMEOUT,
		ERR_BAD_PARSER_DICTIONARY,
		ERR_UNIPARSER_RUNTIME,
		ERR_BAD_BOARD_SIGN,
		ERR_BAD_PARSER_RUNTIME_ARGUMENT,
		ERR_ENC_RETRY							//The encoder failed to update global vars within its allowed retries
	} Error_code;

	/****************************************************************************
	**	MACRO
	****************************************************************************/

		///----------------------------------------------------------------------
		///	LEDS
		///----------------------------------------------------------------------

	#define LED0_TOGGLE()	\
		TOGGLE_BIT( PORTB, PB6 )

	/****************************************************************************
	**	TYPEDEF
	****************************************************************************/

	//Global flags raised by ISR functions
	typedef struct _Isr_flags Isr_flags;

	/****************************************************************************
	**	STRUCTURE
	****************************************************************************/

	//Global flags raised by ISR functions
	struct _Isr_flags
	{
		//First byte
		uint8_t system_tick		: 1;	//System Tick
		uint8_t ctrl_updt		: 1;	//Control System
		uint8_t enc_sem			: 1;	//true = Encoder ISR is forbidden to write into the 32b encoder counters
		uint8_t enc_updt		: 1;	//true = Encoder ISR is forced to update the 32b counters and clear this flag if possible.
		uint8_t					: 4;	//unused bits
	};

	/****************************************************************************
	**	PROTOTYPE: INITIALISATION
	****************************************************************************/

	//port configuration and call the peripherals initialization
	extern void init( void );

	/****************************************************************************
	**	PROTOTYPE: FUNCTION
	****************************************************************************/
		
		///----------------------------------------------------------------------
		///	COMMUNICATION
		///----------------------------------------------------------------------

	//Signal error
	void report_error( Error_code err_code );
	//Error code handler function
	void send_msg_ctrl_mode( void );
	
	void send_data( uint8_t data_len, int16_t *data );
	//Send current PWM setting
	void send_message_pwm_dual( void );
	//Send board performance message to the RPI. Reset performance counters
	void send_message_performance( void );
	
	
		///----------------------------------------------------------------------
		///	PARSER
		///----------------------------------------------------------------------
		//	Handlers are meant to be called automatically when a command is decoded
		//	User should not call them directly
	
	//Handle Ping message coming from the RPI 3B+
	extern void ping_handler( void );
	//Handle Request for signature coming from the RPI 3B+
	extern void send_signature_handler( void );
	//Send board current performance
	extern void send_performance_handler( uint8_t enable );
	//Robot independent messages
	extern void set_platform_pwm_handler( int16_t right, int16_t left );
	extern void set_platform_spd_handler( int16_t right, int16_t left );
	//Handle request for absolute encoder position
	extern void send_enc_pos_handler( uint8_t index );
	//Handle request for all encoder speed
	extern void send_enc_spd_handler( void );
	//Master asks for current speed PID settings. Fourth parameter is fixed point position
	extern void get_spd_pid_handler( void );
	//Handle master set of speed PID gain
	extern void set_spd_pid_handler( int16_t gain_kp, int16_t gain_ki, int16_t gain_kd );

		///----------------------------------------------------------------------
		///	VNH7040 MOTORS
		///----------------------------------------------------------------------
		// HAL API to control the VNH7040 drivers
		
	//Set direction and pwm setting of a VNH7040 controlled motor
	extern bool set_vnh7040_pwm( uint8_t index, int16_t pwm );
	
		///----------------------------------------------------------------------
		///	PLATFORM HAL API
		///----------------------------------------------------------------------
		// HAL API. Abstract motor layout
	
	//Control PWM of the platform to move according to the layout
	extern bool set_platform_pwm( int16_t right, int16_t left );
	//Control SPD of the platform to move according to the layout
	extern bool set_platform_spd( int16_t right, int16_t left );
	
		///----------------------------------------------------------------------
		///	ENCODERS
		///----------------------------------------------------------------------

	//Decode four quadrature encoder channels
	extern void quad_encoder_decoder( uint8_t enc_in );
	//Force an update and save the 32b encoder counters in an input vector
	extern bool get_enc_cnt( int32_t *enc_cnt );
	//Force update of the global 32b counters and compute position and speed
	extern bool process_enc( void );
	
		///----------------------------------------------------------------------
		///	CONTROL SYSTEM
		///----------------------------------------------------------------------

	//Initialize the control systems
	extern bool init_ctrl_system( void );
	//Handle switch between control systems and execute the correct control system. Handles the stop mode.
	extern bool control_system( void );
	//Set the SPD PID gain for all encoder channels
	extern bool set_spd_pid( int16_t gain_kp, int16_t gain_ki, int16_t gain_kd );
	//Called if a PID is unable to keep a lock
	extern void pid_unlock_handler( void );
	
	/****************************************************************************
	**	PROTOTYPE: GLOBAL VARIABILE
	****************************************************************************/

		///----------------------------------------------------------------------
		///	STATUS FLAGS
		///----------------------------------------------------------------------

	//Volatile flags used by ISRs
	extern volatile	Isr_flags g_isr_flags;

		///--------------------------------------------------------------------------
		///	PERFORMANCE
		///--------------------------------------------------------------------------

	//Enable board performance profiling
	extern bool g_b_enable_performance;
	//Number of main loop execution per system tick
	extern uint32_t g_board_performance;
	//Number of bytes received from UART
	extern uint32_t g_board_rpi_rxi_bandwidth;
	//Number of bytes transmitted from UART
	extern uint32_t g_board_rpi_txo_bandwidth;
	
		///----------------------------------------------------------------------
		///	BUFFERS
		///----------------------------------------------------------------------
		//	Buffers structure and data vectors

	//Safe circular buffer for UART input data
	extern volatile At_buf8_safe rpi_rx_buf;
	//Safe circular buffer for uart tx data
	extern At_buf8 rpi_tx_buf;
	//allocate the working vector for the buffer
	extern uint8_t v0[ RPI_RX_BUF_SIZE ];
	//allocate the working vector for the buffer
	extern uint8_t v1[ RPI_TX_BUF_SIZE ];
	
		///--------------------------------------------------------------------------
		///	PARSER
		///--------------------------------------------------------------------------

	//Board Signature
	extern uint8_t *g_board_sign;
	//communication timeout counter
	extern uint8_t g_uart_timeout_cnt;
	//Communication timeout has been detected
	extern bool g_f_timeout_detected;
	
		///--------------------------------------------------------------------------
		///	VNH7040 DC MOTOR CONTROLLER
		///--------------------------------------------------------------------------
	
	
	
		///--------------------------------------------------------------------------
		///	ENCODERS
		///--------------------------------------------------------------------------
	
	//Global 32b encoder counters
	extern volatile int32_t g_enc_cnt[NUM_ENC];
	//Encoder absolute position
	extern int32_t g_enc_pos[NUM_ENC];
	//Encoder speed
	extern int16_t g_enc_spd[NUM_ENC];
		
		///--------------------------------------------------------------------------
		///	CONTROL SYSTEM
		///--------------------------------------------------------------------------
			
	//Actual and target control system mode
	extern Control_mode g_control_mode;
	extern Control_mode g_control_mode_target;
	//PID speed reference
	extern int16_t g_pid_spd_target[NUM_ENC];
	
#else
	#warning "multiple inclusion of the header file global.h"
#endif



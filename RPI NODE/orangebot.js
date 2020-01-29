//-----------------------------------------------------------------------------------
//	HISTORY
//-----------------------------------------------------------------------------------
//		2020-01-01
//	Tested remote control application on OrangeBot via Wi-Wi
//		2020-01-04
//	Include native C++ module
//		2020-01-18
//	Added C++ library version 2020-01-18
//		2020-01-19
//	Added Encoder position and speed request and feedback data
//		2020-01-23
//	Moved to standard platform messages and firmware revision 2020-01-22

//-----------------------------------------------------------------------------------
//	BUGS
//-----------------------------------------------------------------------------------
//		2020-01-20-Zero-Speed
//	Symptoms: Sometimes, when key remains pressed, motors stop for a moment before resumnig
//	Bug Squashing: Problem has been insulated before the serial transmission. PWMR%SL%S message becomes zero
//	Bug Squashing: Problem has been insulated after key detection script. Up/down keys are correctly processed

//-----------------------------------------------------------------------------------
//	INCLUDE
//-----------------------------------------------------------------------------------

//operating system library. Used to get local IP address
const os = require("os");
//file system library. Used to load file stored inside back end server (https://nodejs.org/api/fs.html)
const fs = require("fs");
//http system library. Handles basic html requests
const http = require("http").createServer(http_handler);
//url library. Used to process html url requests
const url = require("url");
//Websocket
const io = require("socket.io")(http);
//Websocket used to stream video
const websocket = require("ws");
//Communicate with the RPI hardware serial port on the GPIO
const SerialPort = require("serialport");
//Include native C++ module
const orangebot_platform_cpp_module = require('./build/Release/OrangebotNodeCpp.node');
console.log('My custom c++ module',orangebot_platform_cpp_module);

//orangebot_platform_cpp_module.parse("PWM:+100:+100");
//console.log("pwm0:", orangebot_platform_cpp_module.getPwm(0));

//TEST: Get robot status class Orangebot::Panopticon from the C++ library
test_parser();

function test_parser()
{
	const test_message = "F4\0ERR\0";
	console.log( "Parse ERR signature, ", test_message );
	orangebot_platform_cpp_module.parse( test_message );
	var robot_status = orangebot_platform_cpp_module.get_status();
	console.log("Parsed Signature: ",robot_status.signature );
	if (robot_status.signature != "ERR")
	{
		console.log("Parser test: SUCCESS");
	}
	else
	{
		console.log("ERR: Failed to parse test signature\n");
	}
}

module.exports = orangebot_platform_cpp_module;

//-----------------------------------------------------------------------------------
//	CONFIGURATION
//-----------------------------------------------------------------------------------

//Port the server will listen to
var server_port = 8080;
var websocket_stream_port = 8082;
//Path of the http and css files for the http server
var file_index_name = "index.html";
var file_script_key_name = "orangebot_key.js";
var file_jsplayer_name = "jsmpeg.min.js";
//Http and css files loaded into memory for fast access
var file_index;
var file_script_key;
var file_jsplayer;
//Name of the local video stream
var stream_name = "mystream";
//Time between emission of data to browser
const time_send_robot_data_to_browser = 333
//Time between emission of serial messages to the robot electronics
const time_send_serial_messages = 500

//-----------------------------------------------------------------------------------
//	MOTOR VARS
//-----------------------------------------------------------------------------------

//Speed of the right and left wheels. Arbitrary unit. range from -max_velocity to +max_velocity integer
var g_right = 0;
var g_left = 0;
//Minimum and maximum speed allowed
var min_velocity = 0;
var max_velocity = 150;
var velocity = 109;
//Ratio of forward to sideways during turn. 0 full turn, 1 full forward
var steering_ratio = 0.7;
//Current direction of the platform
var direction = { forward : 0, right : 0 };
//Setting of the speed PID
var g_spd_pid =
{
	speed_kp : 0,
	speed_ki : 0,
	speed_kd : 0
}

//-----------------------------------------------------------------------------------
//	ENCODER VARS
//-----------------------------------------------------------------------------------

//Number of encoder installed on the platform
const num_enc = 2;
//Ask one encoder at a time 
var scan_encoder_index = 0

//-----------------------------------------------------------------------------------
//	DETECT SERVER OWN IP
//-----------------------------------------------------------------------------------

//If just one interface, store the server IP Here
var server_ip;
//Get local IP address of the server
//https://stackoverflow.com/questions/3653065/get-local-ip-address-in-node-js
var ifaces = os.networkInterfaces();

Object.keys(ifaces).forEach
(
	function (ifname)
	{
		var alias = 0;

		ifaces[ifname].forEach
		(
			function (iface)
			{
				if ('IPv4' !== iface.family || iface.internal !== false)
				{
				  // skip over internal (i.e. 127.0.0.1) and non-ipv4 addresses
				  return;
				}

				if (alias >= 1)
				{
					// this single interface has multiple ipv4 addresses
					console.log('INFO: Server interface ' +alias +' - ' + ifname + ':' + alias, iface.address);
				}
				else
				{
					server_ip = iface.address;
					// this interface has only one ipv4 adress
					console.log('INFO: Server interface - ' +ifname, iface.address);
				}
				++alias;
			}
		);
	}
);

//-----------------------------------------------------------------------------------
//	HTTP SERVER
//-----------------------------------------------------------------------------------
//	Fetch and serves local files to client

//Create http server and listen to the given port
http.listen
(
	server_port,
	function( )
	{
		console.log('INFO: ' +server_ip +' listening to html requests on port ' +server_port);
		//Pre-load http, css and js files into memory to improve http request latency
		file_index = load_file( file_index_name );
		file_script_key = load_file( file_script_key_name );
		file_jsplayer = load_file( file_jsplayer_name );
	}
);

//-----------------------------------------------------------------------------------
//	HTTP REQUESTS HANDLER
//-----------------------------------------------------------------------------------
//	Answer to client http requests. Serve http, css and js files

function http_handler(req, res)
{
	//If client asks for root
	if (req.url == '/')
	{
		//Request main page
		res.writeHead( 200, {"Content-Type": detect_content(file_index_name),"Content-Length":file_index.length} );
		res.write(file_index);
		res.end();

		console.log("INFO: Serving file: " +req.url);
	}
	//If client asks for css file
	else if (req.url == ("/" +file_script_key_name))
	{
		//Request main page
		res.writeHead( 200, {"Content-Type": detect_content(file_script_key_name),"Content-Length" :file_script_key.length} );
		res.write(file_script_key);
		res.end();

		console.log("INFO: Serving file: " +req.url);
	}
	//If client asks for css file
	else if (req.url == ("/" +file_jsplayer_name))
	{
		//Request main page
		res.writeHead( 200, {"Content-Type": detect_content(file_jsplayer_name),"Content-Length" :file_jsplayer.length} );
		res.write(file_jsplayer);
		res.end();

		console.log("INFO: Serving file: " +req.url);
	}
	//Listening to the port the stream from ffmpeg will flow into
	else if (req.url = "/mystream")
	{
		res.connection.setTimeout(0);

		console.log( "Stream Connected: " +req.socket.remoteAddress + ":" +req.socket.remotePort );

		req.on
		(
			"data",
			function(data)
			{
				streaming_websocket.broadcast(data);
			}
		);

		req.on
		(
			"end",
			function()
			{
				console.log("local stream has ended");
				if (req.socket.recording)
				{
					req.socket.recording.close();
				}
			}
		);

	}
	//If client asks for an unhandled path
	else
	{
		res.end();
		console.log("ERR: Invalid file request" +req.url);
	}
}

//-----------------------------------------------------------------------------------
//	WEBSOCKET SERVER: CONTROL/FEEDBACK REQUESTS
//-----------------------------------------------------------------------------------
//	Handle websocket connection to the client

io.on
(
	"connection",
	function (socket)
	{
		console.log("connecting...");

		socket.emit("welcome", { payload: "Server says hello" });

		//Periodically send the current server time to the client in string form
		setInterval
		(
			function()
			{
				//Fetch the object containing the current robot status vars
				var robot_status = orangebot_platform_cpp_module.get_status();
				//Send the object directly to the browser
				socket.emit("robot_status", robot_status );	
			},
			//Send periodically
			time_send_robot_data_to_browser
		);

		socket.on
		(
			"myclick",
			function (data)
			{
				timestamp_ms = get_timestamp_ms();
				socket.emit("profile_ping", { timestamp: timestamp_ms });
				console.log("button event: " +" client says: " +data.payload);
			}
		);

		//Keys are processed by the html running on the browser
		socket.on
		(
			"direction",
			function (data)
			{
				timestamp_ms = get_timestamp_ms();
				//socket.emit("profile_ping", { timestamp: timestamp_ms });
				//Process the direction coming from the platform
				[g_right, g_left] = process_direction( data.direction );
				//Instantly forward the platform message to minimize latency
				//send_message_set_pwm_dual( g_right, g_left );
				send_robot_direction_message( g_spd_pid, g_right, g_left );
				console.log("direction: ", data.direction, " | PWM R: ", g_right, " | PWM L: ", g_left);
			}
		);
		
		//PID Set parameter message
		socket.on
		(
			"pid",
			function (data)
			{
				console.log("Speed PID: ", data);
				//Save new PID setting
				g_spd_pid = data;
				//Send PID setting
				send_message_set_speed_pid( g_spd_pid );
			}
		);

		//profile packets from the client are answer that allows to compute roundway trip time
		socket.on
		(
			"profile_pong",
			function (data)
			{
				timestamp_ms_pong = get_timestamp_ms();
				timestamp_ms_ping = data.timestamp;
				console.log("Pong received. Round trip time[ms]: " +(timestamp_ms_pong -timestamp_ms_ping));
			}
		);
	}
);

//-----------------------------------------------------------------------------------
//	WEBSOCKET SERVER: STREAMING VIDEO
//-----------------------------------------------------------------------------------
//	Current toolchain is
//	v4l2 -> ffmpeg -(mpeg1 over TS on localhost)-> node -(websocket)-> client -> javascript -> canvas

// Websocket Server
var streaming_websocket = new websocket.Server({port: websocket_stream_port, perMessageDeflate: false});

streaming_websocket.connectionCount = 0;

streaming_websocket.on
(
	"connection",
	function(socket, upgradeReq)
	{
		streaming_websocket.connectionCount++;
		console.log
		(
			'New websocket Connection: ',
			(upgradeReq || socket.upgradeReq).socket.remoteAddress,
			(upgradeReq || socket.upgradeReq).headers['user-agent'],
			'('+streaming_websocket.connectionCount+" total)"
		);

		socket.on
		(
			'close',
			function(code, message)
			{
				streaming_websocket.connectionCount--;
				console.log('Disconnected websocket ('+streaming_websocket.connectionCount+' total)');
			}
		);
	}
);

streaming_websocket.broadcast = function(data)
{
	streaming_websocket.clients.forEach
	(
		function each(client)
		{
			if (client.readyState === websocket.OPEN)
			{
				client.send(data);
			}
		}
	);
};

//-----------------------------------------------------------------------------------
//	SERIAL PORT
//-----------------------------------------------------------------------------------
//	Communication with the HotBlack Hat through GPIO

//Connect to the serial port on th GPIO
var my_uart = new SerialPort
(
	"/dev/ttyS0",
	{
		baudRate: 256000,
		openImmediately: true
	},
	false
);

//-----------------------------------------------------------------------------------
//	SERIAL PORT HANDLER
//-----------------------------------------------------------------------------------

//Detect port open
my_uart.on
(
	"open",
	function()
	{
		console.log("Port is open!");
		//Initialize robot communication
		robot_communication_init();
	}
);

//Data from hat. Currently does not work neither in HW nor in SW.
my_uart.on
(
	"data",
	function(data)
	{
		console.log( "RX: ", data.toString() );
		//Feed data to C++ module that takes care of decoding the serial stream
		orangebot_platform_cpp_module.parse( data.toString() );
	}
);

//-----------------------------------------------------------------------------------
//	FUNCTIONS
//-----------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------
//	INIT RPI3B+ <-> Serial Robot Board Communication
//-----------------------------------------------------------------------------------

function robot_communication_init()
{
	//Ask robot for firmware signature
	send_message_signature_request();
	//Ask for periodic update about AT8090 board performance
	send_message_cpu_load_request();
	
	//Periodically send the current server time to the client in string form
	setInterval
	(
		function()
		{
			//Ask for the next encoder position
			send_message_abs_enc_request( scan_encoder_index );
			//Scan next encoder
			scan_encoder_index++;
			if (scan_encoder_index >= num_enc)
			{
				scan_encoder_index = 0;
			}
			//Ask for the next encoder speed
			send_message_enc_spd_request();
			//Show decoded
			console.log( orangebot_platform_cpp_module.get_status() );
			//Control the plaform
			send_robot_direction_message( g_spd_pid, g_right, g_left );
		},
		//Send periodically speed to the motors
		time_send_serial_messages
	);
}

//-----------------------------------------------------------------------------------
//	SERVER DATE&TIME
//-----------------------------------------------------------------------------------
//	Get server time in string form

function get_server_time()
{
	my_date = new Date();

	return my_date.toUTCString();
}

//-----------------------------------------------------------------------------------
//	TIMESTAMP
//-----------------------------------------------------------------------------------
//	Profile performance in ms

function get_timestamp_ms()
{
	my_date = new Date();
	return 1000.0* my_date.getSeconds() +my_date.getMilliseconds()
}

//-----------------------------------------------------------------------------------
//	FILE LOADER
//-----------------------------------------------------------------------------------
//	Load files into memory for improved latency

function load_file( file_name )
{
	var file_tmp;
	var file_path =  __dirname +"/" +file_name;

	//HTML index file
	try
	{
		file_tmp = fs.readFileSync( file_path );
	}
	catch (err)
	{
		console.log("ERR: " +err.code +" failed to load: " +file_path);
		throw err;
	}

	console.log("INFO: " +file_path +" has been loaded into memory");

	return file_tmp;
}

//-----------------------------------------------------------------------------------
//	CONTENT TYPE DETECTOR
//-----------------------------------------------------------------------------------
//	Return the right content type to give correct information to the client browser

function detect_content( file_name )
{
	if (file_name.includes(".html"))
	{
        return "text/html";
	}
	else if (file_name.includes(".css"))
	{
		return "text/css";
	}
	else if (file_name.includes(".js"))
	{
		return "application/javascript";
	}
	else
	{
		throw "invalid extension";

	}
}

//-----------------------------------------------------------------------------------
//	DIRECTION
//-----------------------------------------------------------------------------------

//Compute pwm from direction
function process_direction( direction )
{
	//console.log( "arguments: ", direction.forward, direction.right );
	
	var forward = direction.forward;
	var right = direction.right;
	
	
	//compute PWM
	var pwm_right = forward *velocity -right*velocity *steering_ratio;
	var pwm_left = forward *velocity +right *velocity *steering_ratio;
	
	//If right wheel exceed the overal maximum speed
	if (pwm_right > velocity)
	{
		//Push the difference on the other wheel in opposite direction
		pwm_right = velocity;
		pwm_left = pwm_left -pwm_right +velocity;
		
	}
	else if (pwm_right < -velocity)
	{
		//Push the difference on the other wheel in opposite direction
		pwm_right = -velocity;
		pwm_left = pwm_left -pwm_right -velocity;
		
	}
	
	//If right wheel exceed the overal maximum speed
	if (pwm_left > velocity)
	{
		//Push the difference on the other wheel in opposite direction
		pwm_left = velocity;
		pwm_right = pwm_right -pwm_left +velocity;
		
	}
	else if (pwm_left < -velocity)
	{
		//Push the difference on the other wheel in opposite direction
		pwm_left = -velocity;
		pwm_right = pwm_right -pwm_left -velocity;
		
	}
	//Return processed PWM
	return [pwm_right, pwm_left]
}

//Send the appropriate robot control target message
function send_robot_direction_message( pid, target_right, target_left )
{
	if (pid.ctrl_mode == "off")
	{
		//Do nothing
	}
	else if (pid.ctrl_mode == "pwm")
	{
		//Send PWM message. Automatically switch to PWM mode
		send_message_set_pwm_dual( target_right, target_left );
	}
	else if (pid.ctrl_mode == "spd")
	{
		//Send SPD message. Automatically switch to SPD mode
		send_message_set_spd_dual( target_right, target_left );
	}	
	else
	{
		console.log( "ERR: Unknown control mode:", pid.ctrl_mode );
	}
}

//-----------------------------------------------------------------------------------
//	SEND SERIAL PORT MESSAGES
//-----------------------------------------------------------------------------------

//Send ping message to keep the connection alive
function message_ping( )
{
	my_uart.write
	(
		"P\0",
		function(err, res)
		{
			if (err)
			{
				console.log("err ", err);
			}
		}
	);
}

//Ask for signature 
function send_message_signature_request( )
{
	//Construct message
	msg = "F\0";
	//Send message through UART
	my_uart.write
	(
		msg,
		function(err, res)
		{
			if (err)
			{
				console.log("err ", err);
			}
			else
			{
				console.log("TX: ", msg);
			}
		}
	);
}

//Ask for microcontroller CPU Load
function send_message_cpu_load_request( )
{
	//Construct encoder absolute position request
	var msg = "GET_PERFORMANCE" +"1" +"\0";
	//UART Send message
	my_uart.write
	(
		msg,
		function(err, res)
		{
			if (err)
			{
				console.log("err ", err);
			}
			else
			{
				console.log("TX: ", msg);
			}
		}
	);
}


//Ask for encoder absolute position
function send_message_abs_enc_request( scan_encoder_index )
{
	//Construct encoder absolute position request
	var msg = "ENC_ABS" + scan_encoder_index + "\0";
	//UART Send message
	my_uart.write
	(
		msg,
		function(err, res)
		{
			if (err)
			{
				console.log("err ", err);
			}
			else
			{
				console.log("TX: ", msg);
			}
		}
	);
}

//Ask for encoder speed
function send_message_enc_spd_request()
{
	//Construct encoder absolute position request
	var msg = "ENC_SPD\0";
	//UART Send message
	my_uart.write
	(
		msg,
		function(err, res)
		{
			if (err)
			{
				console.log("err ", err);
			}
			else
			{
				console.log("TX: ", msg);
			}
		}
	);
}

//Compute the PWM set message to send the Robot Board
function send_message_set_pwm_dual( pwm_r, pwm_l )
{
	var msg;
	//Add plus signs when needed
	msg = "ROBOT_PWM" + pwm_r + ":" + pwm_l + "\0";
	/*
	if ((pwm_r<0) && (pwm_l<0))
	{
		msg = "ROBOT_PWM" + pwm_r + ":" + pwm_l + "\0";	
	}
	else if ((pwm_r<0) && (pwm_l>=0))
	{
		msg = "ROBOT_PWM" + pwm_r + ":" +"+" + pwm_l + "\0";	
	}
	else if ((pwm_r>=0) && (pwm_l<0))
	{
		msg = "ROBOT_PWM"  +"+" + pwm_r + ":" + pwm_l + "\0";	
	}
	else
	{
		msg = "ROBOT_PWM"  + "+" + pwm_r + ":"  + "+" + pwm_l + "\0";	
	}
	*/
	//
	my_uart.write
	(
		msg,
		function(err, res)
		{
			if (err)
			{
				console.log("err ", err);
			}
			else
			{
				console.log("TX: ", msg);
			}
		}
	);
}

//Compute the SPD set message to send the Robot Board
function send_message_set_spd_dual( spd_r, spd_l )
{
	var msg;
	//Add plus signs when needed
	msg = "ROBOT_SPD" + spd_r + ":" + spd_l + "\0";
	//
	my_uart.write
	(
		msg,
		function(err, res)
		{
			if (err)
			{
				console.log("err ", err);
			}
			else
			{
				console.log("TX: ", msg);
			}
		}
	);
}

//The RPI wants to change the speed PID parameters
function send_message_set_speed_pid( speed_pid )
{
	var msg;
	//Add plus signs when needed
	msg = "SET_SPD_PID" +speed_pid.speed_kp +":" + speed_pid.speed_ki +":" +speed_pid.speed_kp +"\0";
	//Send data through UART
	my_uart.write
	(
		msg,
		function(err, res)
		{
			if (err)
			{
				console.log("err ", err);
			}
			else
			{
				console.log("TX: ", msg);
			}
		}
	);
}


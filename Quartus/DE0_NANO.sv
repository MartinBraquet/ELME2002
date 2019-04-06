//=======================================================
//  This code is generated by Terasic System Builder
//=======================================================

module DE0_NANO(

	//////////// CLOC0K //////////
	CLOCK_50,

	//////////// LED //////////
	LED,

	//////////// KEY //////////
	KEY,

	//////////// SW //////////
	SW,

	//////////// SDRAM //////////
	DRAM_ADDR,
	DRAM_BA,
	DRAM_CAS_N,
	DRAM_CKE,
	DRAM_CLK,
	DRAM_CS_N,
	DRAM_DQ,
	DRAM_DQM,
	DRAM_RAS_N,
	DRAM_WE_N,

	//////////// EPCS //////////
	EPCS_ASDO,
	EPCS_DATA0,
	EPCS_DCLK,
	EPCS_NCSO,

	//////////// Accelerometer and EEPROM //////////
	G_SENSOR_CS_N,
	G_SENSOR_INT,
	I2C_SCLK,
	I2C_SDAT,

	//////////// ADC //////////
	ADC_CS_N,
	ADC_SADDR,
	ADC_SCLK,
	ADC_SDAT,

	//////////// 2x13 GPIO Header //////////
	GPIO_2,
	GPIO_2_IN,

	//////////// GPIO_0, GPIO_0 connect to GPIO Default //////////
	GPIO_0_PI,
	GPIO_0_PI_IN,

	//////////// GPIO_1, GPIO_1 connect to GPIO Default //////////
	GPIO_1,
	GPIO_1_IN 
);

//=======================================================
//  PARAMETER declarations
//=======================================================


//=======================================================
//  PORT declarations
//=======================================================

//////////// CLOCK //////////
input 		          		CLOCK_50;

//////////// LED //////////
output		     [7:0]		LED;

//////////// KEY //////////
input 		     [1:0]		KEY;

//////////// SW //////////
input 		     [3:0]		SW;

//////////// SDRAM //////////
output		    [12:0]		DRAM_ADDR;
output		     [1:0]		DRAM_BA;
output		          		DRAM_CAS_N;
output		          		DRAM_CKE;
output		          		DRAM_CLK;
output		          		DRAM_CS_N;
inout 		    [15:0]		DRAM_DQ;
output		     [1:0]		DRAM_DQM;
output		          		DRAM_RAS_N;
output		          		DRAM_WE_N;

//////////// EPCS //////////
output		          		EPCS_ASDO;
input 		          		EPCS_DATA0;
output		          		EPCS_DCLK;
output		          		EPCS_NCSO;

//////////// Accelerometer and EEPROM //////////
output		          		G_SENSOR_CS_N;
input 		          		G_SENSOR_INT;
output		          		I2C_SCLK;
inout 		          		I2C_SDAT;

//////////// ADC //////////
output		          		ADC_CS_N;
output		          		ADC_SADDR;
output		          		ADC_SCLK;
input 		          		ADC_SDAT;

//////////// 2x13 GPIO Header //////////
inout 		    [12:0]		GPIO_2;
input 		     [2:0]		GPIO_2_IN;

//////////// GPIO_0, GPIO_0 connect to GPIO Default //////////
inout 		    [33:0]		GPIO_0_PI;
input 		     [1:0]		GPIO_0_PI_IN;

//////////// GPIO_1, GPIO_1 connect to GPIO Default //////////
inout 		    [33:0]		GPIO_1;
input 		     [1:0]		GPIO_1_IN;


	
//=======================================================
//  Link between GPIO_ 1 and the components
//=======================================================

//This pin is used to reset directly the FPGA from the raspberryPI
assign reset = GPIO_0_PI[0];


//=======================================================
//  SPI
//=======================================================

logic	spi_clk, spi_cs, spi_mosi, spi_miso;
logic MemWriteM;
logic [31:0] DataAdrM, WriteDataM, spi_data;
logic  signalrasp;

assign MemWriteM = 1; //This is used to always update the value from the output register.

spi_slave spi_slave_instance(
	.SPI_CLK    (spi_clk),
	.SPI_CS     (spi_cs),
	.SPI_MOSI   (spi_mosi),
	.SPI_MISO   (spi_miso),
	.Data_WE    (MemWriteM & cs_spi),
	.Data_Addr  (DataAdrM),
	.Data_Write (WriteDataM),
	.Data_Read  (spi_data),
	.Clk        (clk)
);

assign spi_clk  		= GPIO_0_PI[11];	// SCLK = pin 16 = GPIO_11
assign spi_cs   		= GPIO_0_PI[9];	// CE0  = pin 14 = GPIO_9
assign spi_mosi     	= GPIO_0_PI[15];	// MOSI = pin 20 = GPIO_15, send data from PI

assign GPIO_0_PI[13] = spi_cs ? 1'bz : spi_miso;  // MISO = pin 18 = GPIO_13, received data from PI

//Usefull signals
logic reset_enc_LEFT_WHEEL_SPI;
logic reset_enc_RIGHT_WHEEL_SPI;
logic resetUltrasonicSPI;
logic resetPneumaticSPI;
logic [31:0] commandPneumaticSPI;

//=======================================================
//  ENABLE VOLTAGE TRANSLATOR
//=======================================================
assign GPIO_1[32] = 1;

//=======================================================
//  DYNAMIXEL
//=======================================================

logic RPi_UART_TX, RPi_UART_RX, RPi_UART_DIR;

// connections
assign RPi_UART_TX 	= GPIO_0_PI[28];
assign GPIO_1[6] = RPi_UART_TX;

assign RPi_UART_RX 	= GPIO_1[4];
assign GPIO_0_PI[26] 	= RPi_UART_RX;

assign RPi_UART_DIR 	= GPIO_0_PI[21];
assign GPIO_1[2] 	= RPi_UART_DIR;
//=======================================================
//Pneumatic computation
//=======================================================

logic valve1,valve2,piston1,piston2,piston3; 
logic resetPneumatic;

//assign GPIO_1[14] = 1'b0;//valve1 ;
//assign GPIO_1[20] = 1'b0;//valve2 ;
//assign GPIO_1[16] = 1'b0;//piston1;
//assign GPIO_1[18] = 1'b0;//piston2;
//assign GPIO_1[22] = 1'b0;// piston3;
assign GPIO_1[14] = commandPneumaticSPI[0]; //valve1 ;
assign GPIO_1[20] = commandPneumaticSPI[1]; //valve2 ;
assign GPIO_1[16] = commandPneumaticSPI[2]; //piston1;
assign GPIO_1[18] = commandPneumaticSPI[3]; //piston2;
assign GPIO_1[22] = commandPneumaticSPI[4]; //piston3;

//logic [31:0] counter1;
//counterUltrasonic my_count(.clk(clk),
//			   .reset(reset),
//			   .counter(counter1));



//pneumatic pneuma (.clk(clk),
//					.reset(resetPneumatic),
//					.signalrasp(commandPneumaticSPI[3:0]),
//					//.signalrasp(counter1[27:24]),
//					.valve1(valve1), 
//					.valve2(valve2), 
//					.piston1(piston1), 
//					.piston2(piston2), 
//					.piston3(piston3)
//					);

//=======================================================
//Ultrasonic sensor
//=======================================================
logic trig1,trig2,trig3,trig4;
logic echo1,echo2,echo3,echo4;
logic [31:0] count_max_ultrasonic_sensor,count1,deltaT_ultrasonic1,deltaT_ultrasonic2,deltaT_ultrasonic3,deltaT_ultrasonic4;
logic resetUltrasonic;
logic [3:0]stateFSM1,stateFSM2,stateFSM3,stateFSM4; 


assign count_max_ultrasonic_sensor = 32'd50000000;



assign echo1 = GPIO_1[17];
assign echo2 = GPIO_1[13];	
assign echo3 = GPIO_1[7];  
assign echo4 = GPIO_1[3];

					

//logic [31:0] TesTregister;
//counterU testCounter(clk, reset,  TesTregister); 
 
assign GPIO_1[15] = trig1;
assign GPIO_1[11] = trig2;	
assign GPIO_1[5] =  trig3;
assign GPIO_1[1] =  trig4;

ultrasonic_sensor ultraSonic1(CLOCK_50,resetUltrasonic,count_max_ultrasonic_sensor,echo1,trig1,deltaT_ultrasonic1);
ultrasonic_sensor ultraSonic2(CLOCK_50,resetUltrasonic,count_max_ultrasonic_sensor,echo2,trig2,deltaT_ultrasonic2);
ultrasonic_sensor ultraSonic3(CLOCK_50,resetUltrasonic,count_max_ultrasonic_sensor,echo3,trig3,deltaT_ultrasonic3);
ultrasonic_sensor ultraSonic4(CLOCK_50,resetUltrasonic,count_max_ultrasonic_sensor,echo4,trig4,deltaT_ultrasonic4);



//=======================================================
//  Encoder declaration
//=======================================================

logic [31:0]	enc_counter_LEFT_WHEEL_M, enc_counter_RIGHT_WHEEL_M,enc_counter_LEFT_WHEEL_O, enc_counter_RIGHT_WHEEL_O;
logic 			reset_enc_LEFT_WHEEL, reset_enc_RIGHT_WHEEL;
logic 			RaE,RbE,LaE,LbE,RaO,Rbo,LaO,LbO;

// signals a and b for the encorders
assign LaE = GPIO_1[27];
assign LbE = GPIO_1[25];
assign RaE = GPIO_1[21];
assign RbE = GPIO_1[19];
assign LaO = GPIO_1[33];
assign LbO = GPIO_1[31];
assign RaO = GPIO_1[28];
assign RbO = GPIO_1[26];

//encodeur right position
quadrature_decoder encoder_decoderLEFT_WHEEL_M(CLOCK_50, reset_enc_LEFT_WHEEL, LaE, LbE, enc_counter_LEFT_WHEEL_M);
// encodeur left position
quadrature_decoder encoder_decoderRIGHT_WHEEL_M(CLOCK_50, reset_enc_RIGHT_WHEEL, RaE, RbE, enc_counter_RIGHT_WHEEL_M);
// Odometer right position
quadrature_decoder encoder_decoderRIGHT_WHEEL_O(CLOCK_50, reset_enc_RIGHT_WHEEL, RaO, RbO, enc_counter_RIGHT_WHEEL_O);
// Odometer left position
quadrature_decoder encoder_decoderLEFT_WHEEL_O(CLOCK_50, reset_enc_LEFT_WHEEL, LaO, LbO, enc_counter_LEFT_WHEEL_O);



//=======================================================
//  Speed computations
//=======================================================

logic [31:0]	speed_LEFT_WHEEL_M, speed_RIGHT_WHEEL_M,speed_LEFT_WHEEL_O, speed_RIGHT_WHEEL_O;
logic 			reset_speed_LEFT_WHEEL, reset_speed_RIGHT_WHEEL;

//encodeur right speed
rotation_speed rotation_speedRIGHT_WHEEL_M(CLOCK_50, reset_speed_RIGHT_WHEEL, enc_counter_RIGHT_WHEEL_M, speed_RIGHT_WHEEL_M);
//encodeur left speed
rotation_speed rotation_speedLEFT_WHEEL_M(CLOCK_50, reset_speed_LEFT_WHEEL, enc_counter_LEFT_WHEEL_M, speed_LEFT_WHEEL_M);
//odometer right speed 
rotation_speed rotation_speedRIGHT_WHEEL_O(CLOCK_50, reset_speed_RIGHT_WHEEL, enc_counter_RIGHT_WHEEL_O, speed_RIGHT_WHEEL_O);
//odometer left speed
rotation_speed rotation_speedLEFT_WHEEL_O(CLOCK_50, reset_speed_LEFT_WHEEL, enc_counter_LEFT_WHEEL_O, speed_LEFT_WHEEL_O);

//=======================================================
//  Actions to be done
//=======================================================

logic clk,reset;
logic [31:0] ReadDataM;
logic [7:0] led_reg;


assign clk = CLOCK_50;
assign MemWriteM = 1'b1; 


// Chip Select logic
// For the moment we only use SPI, need a logic if we want to read a value from an input register
assign cs_spi    = 1'b1;  

// Read Data
always_comb // Not very useful yet, can become useful is we want to readData from multiple sources
	if (cs_spi) ReadDataM = spi_data;
	else ReadDataM = 32'b0;

// Adress for the SPIRegister
// The count goes up to 16 to link the 16 register, can be increased if needed
assign DataAdrM = {26'd0, registerCount, 2'd0}; 

// Update output register from the SPI
logic [3:0] registerCount;
counter SpiCounter(clk, reset, registerCount); 

// This code updates the data on the registers in misoRAM with counter
always_comb
	case (registerCount) 
		4'd0:  WriteDataM = enc_counter_LEFT_WHEEL_M; 	// motor left wheel position R1
		4'd1:  WriteDataM = enc_counter_RIGHT_WHEEL_M;	// motor right wheel position R2
		4'd2:  WriteDataM = enc_counter_LEFT_WHEEL_O;	
		4'd3:  WriteDataM = enc_counter_RIGHT_WHEEL_O;
		4'd4:  WriteDataM = speed_LEFT_WHEEL_M;
		4'd5:  WriteDataM = speed_RIGHT_WHEEL_M;
		4'd6:	 WriteDataM = speed_LEFT_WHEEL_O;
		4'd7:  WriteDataM = speed_RIGHT_WHEEL_O;
//		4'd8:  WriteDataM = deltaT_ultrasonic1;
//		4'd7:  WriteDataM = deltaT_ultrasonic2;
//		4'd10:  WriteDataM = deltaT_ultrasonic3;
//		4'd11:  WriteDataM = deltaT_ultrasonic4;
		default: WriteDataM = 32'h00000000;
	endcase

	
	// This code updates the data from the input register in mosiRAM with counter
always_ff @(posedge clk)
	case (registerCount)
		4'd0:  reset_enc_LEFT_WHEEL_SPI = ReadDataM[0]; // reset signal for the LEFT wheel R1      
		4'd1:  reset_enc_RIGHT_WHEEL_SPI = ReadDataM[0]; // reset signal for the RIGHT wheel R2
		4'd2:  resetUltrasonicSPI = ReadDataM[0]; // reset signal for the RIGHT wheel R2
		4'd3:  resetPneumaticSPI = ReadDataM[0] ;
		4'd4:  commandPneumaticSPI = ReadDataM;
	endcase	


//all the signal that needs to be reset should be declared here	
always_ff @ (posedge clk, posedge reset) 
	if (reset)
		begin // Reset the counter and light on all the led to see that it is well reset
			LED = 8'hff;
			reset_enc_LEFT_WHEEL = 1'b1;
			reset_enc_RIGHT_WHEEL = 1'b1;
			reset_speed_LEFT_WHEEL = 1'b1;
			reset_speed_RIGHT_WHEEL = 1'b1;
			resetUltrasonic = 1'b1;
			resetPneumatic = 1'b1;
			
		end		
	else
		begin 
			//LED = led_reg;
			LED = commandPneumaticSPI[7:0];
			reset_enc_LEFT_WHEEL = reset_enc_LEFT_WHEEL_SPI;
			reset_enc_RIGHT_WHEEL = reset_enc_RIGHT_WHEEL_SPI;
			reset_speed_LEFT_WHEEL = 1'b0;
			reset_speed_RIGHT_WHEEL = 1'b0;
			resetUltrasonic = resetUltrasonicSPI;
			resetPneumatic = resetPneumaticSPI;
		end
	

// To use the LED	
always_ff@(posedge clk)
begin
	led_reg[3:0] = enc_counter_LEFT_WHEEL_M[3:0];
	led_reg[7:4] = enc_counter_RIGHT_WHEEL_M[3:0];
	//led_reg[3:0] = counter1[30:27];
	
end
endmodule


// This counter is used to update each register (n = 16): thus it requires n clock cycles to update each reg
module counter(input clk,
					input reset,
					output[3:0] counter);
					
always_ff @(posedge clk, posedge reset)
	if(reset) 
		counter <= 4'b0;
	else 
		counter <= counter + 1;
	
endmodule


		


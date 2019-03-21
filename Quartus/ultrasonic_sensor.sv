module ultrasonic_sensor(
	input logic 		clk,
	input logic		reset,
	input logic[31:0]    	countMax,
	input logic 		echo,
	output logic 		trigger,
	output logic[31:0]  	deltaT);

	
logic [31:0] count;
logic [31:0] echoCount; 
logic resetCount;

counterUltrasonic my_count(.clk(clk),
			   .reset(resetCount),
			   .counter(count));

// FSM  to handel the ultrasonic sensors
// 1) TRIG_ST send q signal to the sensor to tells it to take a measure
// 2) ECHO_ST wait for the signal to be back, and count to get the time
// 3) WAIT_ST can be tuned to choose how often we wants to take a measure
typedef enum logic[1:0]{RESET_ST,TRIG_ST,ECHO_ST,WAIT_ST} statetype;
statetype state, nextstate;  
    
always_ff @(posedge clk,posedge reset) 
	if(reset) state <= RESET_ST;
	else state <= nextstate;
	 
always_comb
case(state)
		 RESET_ST : if(reset) 
							begin
							resetCount = 1'b1;
							trigger = 1'b0;
							nextstate =  RESET_ST;
							end
						else
							begin
							resetCount = 1'b0;
							trigger = 1'b1;
							nextstate =  TRIG_ST;
							end
							
						
		 TRIG_ST : if (echo == 1'b1 || count >= 31'd7500) //7500 represent 15 microseconds
							begin //If started switch to receive echo
							resetCount = 1'b1;
							trigger = 1'b0;
							nextstate = ECHO_ST;
							end
						else
							begin
							resetCount = 1'b0;
							trigger = 1'b1;
							nextstate = TRIG_ST; // start the ultrasonic sensor
							end

		 ECHO_ST :  if (echo == 1'b1)
							begin
							resetCount = 1'b0;
							trigger = 1'b0;
							nextstate = ECHO_ST; // wait until we get the line going low again
							end 
						else 	
							begin
							resetCount = 1'b0;
							trigger = 1'b0;
							nextstate =  WAIT_ST;  	
							end 
		

		 WAIT_ST :  if (count >=  countMax) 
							begin //wait before reusing the sensor again
							resetCount = 1'b1;
							trigger = 1'b0;
							nextstate = TRIG_ST;   
							end 
						else 
							begin 
							resetCount = 1'b0;
							trigger = 1'b0;
							nextstate =  WAIT_ST;
							end 
							
		default : 	begin 
							resetCount = 1'b1;
							trigger = 1'b0;
							nextstate = RESET_ST;	
						end
		endcase
	
// To save the value of the counter just when the counter is send back
always_ff @( posedge reset, negedge echo && (state == ECHO_ST ||  state == WAIT_ST))
	if (reset) echoCount <= 0;
	else echoCount <=  count;

//output logic 
assign deltaT = echoCount ; 
	
endmodule


// This counter is used to update each register (n = 16): thus it requires n clock cycles to update each reg
module counterUltrasonic(input logic clk,
			 input logic reset,
			 output logic [31:0] counter);
					
always_ff @(posedge clk, posedge reset)
	if(reset) 
		counter <= 31'b0;
	else 
		counter <= counter + 1;
	
endmodule





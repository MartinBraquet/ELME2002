module ultrasonic_sensor(
	input logic 		clk,
	input logic		reset,
	input logic[31:0] countMax,
	input logic 		echo,
	output logic 		trigger,
	output logic 		stateFSM,
	output logic[31:0]  	deltaT);
	
logic [31:0] countFSM;
logic [31:0] echoCount; 
logic resetCount;



counterUltrasonic my_count(.clk(clk),
			   .reset(resetCount),
			   .counter(countFSM));		
				
				
 //FSM  to handel the ultrasonic sensors
 //1) RESET_ST initialize all the parameter when reset is 1
 //2)	TRIG_ST send q signal to the sensor to tells it to take a measure
 //3) ECHO_ST wait for the signal to be back, and count to get the time
 //4) WAIT_ST can be tuned to choose how often we wants to take a measure
 
typedef enum logic[1:0]{RESET_ST,TRIG_ST,ECHO_ST,WAIT_ST} statetype;
statetype state, nextstate;  
    
always_ff @(posedge clk,posedge reset) 
	if(reset) state <= RESET_ST;
	else state <= nextstate;
	 
always_comb
case(state)
		 RESET_ST : 
						if(reset) 
							begin
							resetCount = 1'b1;
							trigger = 1'b0;
							nextstate =  RESET_ST;
							end
						else
							begin
							resetCount = 1'b1;
							trigger = 1'b0;
							nextstate =  TRIG_ST;
							end
						
		 TRIG_ST : if (echo == 1'b0)
							begin 
								resetCount = 1'b0;	
								nextstate = TRIG_ST; 
								if (countFSM >= 32'd7500) 
									trigger = 1'b0;    //Wait for the echo
								else	
									trigger = 1'b1;	//Trig			
							end
						else
							begin
								resetCount = 1'b1;
								trigger = 1'b0;
								nextstate = ECHO_ST; // swtich to echo
							end

		 ECHO_ST : 		
		 
							if (echo == 1'b1)
								begin
								resetCount = 1'b0;
								trigger = 1'b0;
								nextstate = ECHO_ST; // wait until we get the line going low again
								end
							else 	
								begin
								resetCount = 1'b0;
								trigger = 1'b0;
								nextstate =   WAIT_ST ;  
								end
							
		

		 WAIT_ST :  if (countFSM >=  countMax) 
							begin //wait before reusing the sensor again
							trigger = 1'b0;
							resetCount = 1'b1;
							nextstate = RESET_ST;  
							end 
						else 
							begin 
							trigger = 1'b0;
							resetCount = 1'b0;
							nextstate =  WAIT_ST;
							end 
							
		default : 	begin 
							resetCount = 1'b1;
							trigger = 1'b0;
							nextstate = RESET_ST;	
						end
		endcase
	
//// To save the value of the counter just when the counter is send back
always_ff @( posedge reset, negedge echo && (state == ECHO_ST ||  state == WAIT_ST))
	if (reset) echoCount <= 0;
	else echoCount <= countFSM;

//output logic 
assign deltaT = echoCount*170/50000000 ; // to convert in meter with the speed of sound
assign stateFSM = {state == WAIT_ST,state == ECHO_ST,state == TRIG_ST,state == RESET_ST};
assign countO = countFSM;

endmodule




// This counter is used to update each register (n = 16): thus it requires n clock cycles to update each reg
module counterUltrasonic(input logic clk,
								 input logic reset,
								 output logic [31:0] counter);
					
always_ff @(posedge clk, posedge reset)
	if(reset) 
		counter <= 32'b0;
	else 
		counter <= counter + 1;
	
endmodule




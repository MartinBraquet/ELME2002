module ultrasonic_sensor(
	input logic 			clk,
	input logic				reset,
	input logic		      trigger,
	input logic[15:0]    countMax,
	output logic  			echo);

logic [15:0] count;




always_ff @ (posedge clk)
	if (reset | count< countMax) 
		count <= 16'd0;
	else 
		count <= count + 1;
	
endmodule

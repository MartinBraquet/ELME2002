module rotation_speed(
	input logic 			clk,
	input logic				reset,
	input logic	 [31:0]	angle,
	output logic [31:0]	omega);

logic [24:0] count;
logic [31:0] angle_before, angle_after;

assign omega = angle_after - angle_before;
	
always_ff @ (posedge clk)
	if (reset) count <= 25'd0;
	else count <= count + 1;
	
always_ff @ (posedge clk)
	if (count == 25'd0) 
		begin
			angle_after  <= angle;
			angle_before <= angle_after;
		end
	
endmodule

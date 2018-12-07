module quadrature_decoder(
	input logic 			clk,
	input logic				reset,
	input logic				channel_A, channel_B,
	output logic [31:0]	counter);

	
logic [2:0] channel_A_delayed, channel_B_delayed;

always_ff @(posedge clk) channel_A_delayed <= {channel_A_delayed[1:0], channel_A};
always_ff @(posedge clk) channel_B_delayed <= {channel_B_delayed[1:0], channel_B};

logic count_enable, count_direction;
assign count_enable		= channel_A_delayed[1] ^ channel_A_delayed[2] ^ channel_B_delayed[1] ^ channel_B_delayed[2];
assign count_direction	= channel_A_delayed[1] ^ channel_B_delayed[2];

always_ff @ (posedge clk)
	if (reset) counter <= 0;
	else if (count_enable)
		if(count_direction)	counter <= counter + 1; 
		else 						counter <= counter - 1;
	
endmodule

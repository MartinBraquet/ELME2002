`timescale 1 ps / 1 ps

module MyTestbench();

  logic        clk;
  logic        reset;
  
  wire [33:0]	GPIO_0;
  wire [12:0]	GPIO_2;

  integer f;
	 
 

  assign GPIO_0[1] = reset;
  
  // initialize test
  initial
    begin
      reset <= 1; # 22; reset <= 0;
    end

  // generate clock to sequence tests
  always
    begin
      clk <= 1; # 1; clk <= 0; # 1;
    end
	 
	 
logic	[31:0] angle;
logic [31:0] omega;

assign angle = 32'd10;
	 
rotation_speed test(clk, reset, angle, omega);

endmodule

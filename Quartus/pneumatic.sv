module pneumatic (input clk,
					input reset,
					input signalrasp
					output signal1,
					output signal2,
					output signal3
					output [2:0] signal4
					);
					
				assign signal4 ={ 1'b signal1,1'b signal2,1'b signal3};
					
	always_ff @(posedge clk, posedge reset)
	if(signalrasp == 1'b0 ) 
		begin
		signal1= 1'b0;
		signal2= 1'b0;
		signal3= 1'b0;
		end
	else 
	begin
	   signal1= 1'b1;
		signal2= 1'b1;
		signal3= 1'b1;	
	end	

endmodule
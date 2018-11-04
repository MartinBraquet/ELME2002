//----------------------------------------------------------------------
// Module making the interface between the NIOS and the module UART_TXT
// It reads and writes in registers exported from the NIOS
//-----------------------------------------------------------------------
module UART_Dynamixel (
	// NIOS signals
	input				clk,
	input 			reset,
	input				write_en,
	input				read_en,
	input  reg[2:0]	rw_ad,
	input  reg[31:0]	write_data,
	output reg[31:0]	read_data,
	// exported signals
	input				RXD,
	output			TXD, 
	output 			UART_DIR,
	output			debug
);

logic 			baud_clk, TXD_done, RXD_enable, RXD_done;
logic [31:0] 	TXD_data_1, TXD_data_2, RXD_data_1, RXD_data_2;
logic communication_fail, start_RX;
reg TXD_enable_prev, TXD_enable, start_communication;

always_ff @(posedge clk)	begin
	// TXD
	if 	  ((read_en) & (rw_ad == 3'b100)) read_data	<= {31'b0, TXD_done};
	if ((write_en) & (rw_ad == 3'b100)) TXD_enable			<= write_data[0];
	else if ((write_en) & (rw_ad == 3'b101)) TXD_data_1 			<= write_data;
	else if ((write_en) & (rw_ad == 3'b110)) TXD_data_2 			<= write_data;
	//RXD
	else if ((read_en) & (rw_ad == 3'b001)) read_data 	<= RXD_data_1;
	else if ((read_en) & (rw_ad == 3'b010)) read_data 	<= RXD_data_2;
	else if ((read_en) & (rw_ad == 3'b000)) read_data 	<= {30'b0, communication_fail, RXD_done};
end

Baudrate_Generator baudgen(clk, reset, baud_clk);
UART_Dynamixel_TXD txd(clk, baud_clk, reset, start_RX, TXD_data_1, TXD_data_2, TXD, UART_DIR, TXD_done);
UART_Dynamixel_RXD rxd(baud_clk, reset, start_RX, RXD, RXD_done, communication_fail, RXD_data_1, RXD_data_2);
assign debug = start_communication;



//assign start_communication = TXD_enable;

always @(posedge TXD_enable)	
	start_communication <= 1'b1;
	
always @(posedge baud_clk, posedge reset)
	if(reset) TXD_enable_prev <= 1'b0;
	else TXD_enable_prev <= TXD_enable;
	
assign start_RX = ~TXD_enable_prev && TXD_enable;

endmodule 

//------------------------------------------------------------
// Module used to send instruction packets
//------------------------------------------------------------
module UART_Dynamixel_TXD(
	input logic 			clk, baud_clk, reset, start_communication,
	input logic [31:0] 	data1, data2,
	output logic 			TXD, UART_DIR,
	output reg packet_sent
);

typedef enum logic [2:0] {Wait,Init,Prepare, Transmit,Sent, End} statetype;
statetype	state, nextstate;

logic [7:0] length, id, instr, checksum, P0, P1, P2;
reg [63:0] frame;
reg [9:0] byte_cnt;
reg [7:0] byte2send;
logic byte_ready;

reg [9:0] cnt_prep;

always_comb
	begin
		id = data1[7:0];
		length = data1[15:8];
		instr = data1[23:16];
		checksum = data1[31:24];
		P0 = data2[7:0];
		P1	= data2[15:8];
		P2	= data2[23:16];
	end

always_ff @(posedge clk, posedge reset) begin
	if(reset) state <= Wait;
	else		 state <= nextstate;
end

logic byte_sent;	

	
always_comb
	case(state)
	Wait: if(start_communication) nextstate = Init;
			else nextstate = Wait;
	Init: nextstate = Prepare;
	Prepare: if(byte_cnt > 8'd3 + length) nextstate = End;
		else nextstate = Transmit;
	Transmit: if(byte_sent) nextstate = Sent;
			else nextstate = Transmit;
	Sent: nextstate = Prepare;
	End:	nextstate = Wait;
	default: nextstate = Wait;
	endcase
	
always_ff @(posedge clk, posedge reset)
	begin
		if(reset) begin
			byte_cnt <= 10'd0;
			byte2send <= 8'b0;
			end
		else
			if(state == Init)
				frame <= {P2, P1, P0, instr, length, id, 8'hFF, 8'hFF}; 
			else if(state == Prepare)
				begin
				if (byte_cnt == 8'd3 + length) byte2send <= checksum; 
				else byte2send <= frame[7:0];
				end
			else if(state == Sent)
				begin
				frame <= frame >> 8;
				byte_cnt <= byte_cnt + 10'd1;
				end
			else if(state == End)
				byte_cnt <= 10'd0;
	end

assign byte_ready = state == Transmit;
assign UART_DIR = ~(state == Wait);

always_ff @(posedge clk, posedge reset)
	if(reset) packet_sent <= 1'b0;
	else if (state == End) packet_sent <= 1'b1;

UART_TX_BYTE ubyte(clk, baud_clk, reset, byte_ready, byte2send, byte_sent, TXD);

endmodule

/*------------------------------------------------------------
							UART_TX_BYTE
--------------------------------------------------------------*/
module UART_TX_BYTE(input clk, input baud_clk, reset, TxD_start,
		    input [7:0] TxD_data,
		    output byte_sent, TxD);

typedef enum logic [2:0] {idle, init, transmit, sent} statetype;
statetype	state;
reg [3:0] cnt;
reg [7:0] buffer;

always @(posedge clk)
case(state)
  idle: if(TxD_start) state <= init;
  init: if(baud_clk) state <= transmit;
  transmit: if(cnt == 10'd10) state <= sent;
  sent: state <= idle;
  default: state <= idle;
endcase

always_ff @(posedge clk, posedge reset)
if(reset) buffer <= 8'b0;
else if(state == init) buffer <= TxD_data;
	

always_ff @(posedge baud_clk, posedge reset)
	if(reset) cnt <= 4'b0;
	else if(state == transmit) cnt <= cnt +4'b1;
	else if(cnt == 10'd10) cnt <= 4'b0;

logic out;
always_comb
case(cnt)
  0: out = 1'b0; // start bit
  1: out = buffer[0];
  2: out = buffer[1];
  3: out = buffer[2];
  4: out = buffer[3];
  5: out = buffer[4];
  6: out = buffer[5];
  7: out = buffer[6];
  8: out = buffer[7];
  9: out = 1'b1; // stop bit
  default out = 1'b1;
endcase

assign TxD = (state != transmit) | ((state == transmit) & out);
assign byte_sent = (state == sent);

endmodule


//--------------------------------------------------------------------------
// UART_Dynamixel_RXD
//--------------------------------------------------------------------------
module UART_Dynamixel_RXD (
	input logic					clk, reset, start_communication, RXD,
	output logic				data_ready, fail_reg,
	output logic [31:0] 		data1, data2
);

typedef enum logic [2:0] {S0,S1,S2,S3} statetype;
statetype	state, nextstate;

logic 		RXD_prev, start, start_byte, end_of_packet, communication_fail;
logic[3:0] 	cnt_byte, cnt_bits;
logic[7:0] 	data, ID, Length, Error, P1, P2, Checksum, cnt_fail;
logic[11:0] RXD_reg;

assign start_byte = (~RXD) && RXD_prev && ((cnt_bits > 4'd7) || (cnt_byte==4'b0));
assign start = (cnt_byte == 4'd3);
assign end_of_packet	= (RXD_reg == 12'hFFF);
assign communication_fail = cnt_fail > 8'd200;

always_ff @(posedge clk, posedge reset) begin
	if(reset) state <= S0;
	else if(communication_fail) state <= S0;
	else 		 state <= nextstate;
end

always_ff @(posedge clk) begin
	if(communication_fail) 	fail_reg <= 1'b1;
	else if(state == S1) 	fail_reg	<= 1'b0;
end

always_ff @(posedge clk) begin
	RXD_prev <= RXD;
	RXD_reg <= {RXD_reg[10:0],RXD};
end

always_ff @(posedge clk) begin
	if(state == S0) cnt_byte <= 4'b0;
	else if(start_byte)  begin
		cnt_byte <= cnt_byte + 4'b1;
		cnt_bits <= 4'b0;
	end
	else cnt_bits <= cnt_bits + 4'b1;
end

always_comb begin
  case(state)
    S0: if(start_communication) nextstate = S1; // Waiting for activation
		  else nextstate = S0;
    S1: if(start) nextstate = S2; 					// Waiting for receiving something
		  else nextstate = S1;
    S2: if(end_of_packet) nextstate = S3; 		// Recording data send by Dynamixel into registers
		  else nextstate = S2;
	 S3: nextstate = S0;									// Creation of the registers
    default: nextstate = S0;
  endcase
end

always_ff @(posedge clk) begin
	if(state == S0) begin
		data_ready	<= 1'b1;
		cnt_fail		<= 8'b0;
	end
	else if(state == S1) begin
		cnt_fail		<= cnt_fail + 8'b1;
		data_ready	<= 1'b0;
		data1 		<= 32'b0;
		data2 		<= 32'b0;
		data 			<= {RXD,data[7:1]};
	end
	else if(state == S2) begin
		data 	<= {RXD,data[7:1]};
	end
	else if(state == S3) begin
		data1 		<= {Checksum, Error, Length, ID};
		data2 		<= {P2,P1};
	end
end

always_ff @(posedge clk) begin
	if(state == S1) begin
		ID 			<= 8'b0;
		Length 		<= 8'b0;
		Error 		<= 8'b0;
		P1 			<= 8'b0;
		P2 			<= 8'b0;
		Checksum 	<= 8'b0;
	end
	else if(cnt_bits == 4'd8) begin
		if(cnt_byte == 4'd3) ID 		 <= data;
		else if(cnt_byte == 4'd4) Length 	 <= data;
		else if(cnt_byte == 4'd5) Error 	 <= data;
		else if(cnt_byte == 4'd6) begin
			if(Length == 8'd2) Checksum <= data;
			else					 P1 <= data;
		end
		else if(cnt_byte == 4'd7) begin
			if(Length == 8'd3) Checksum <= data;
			else				    P2 <= data;
		end
		else if((cnt_byte == 4'd8) && (Length == 8'd4)) Checksum <= data;	
	end
end


endmodule

//--------------------------------------------------------------------------
// Module used to generate a clock of 57.142 [kHz] from a clock of 50 [MHz]
//--------------------------------------------------------------------------
module Baudrate_Generator(input clk, input reset, output baud_clk);

reg[9:0] cnt;

always @(posedge clk or posedge reset) begin
	if (reset) cnt <= 10'b0;
	else begin
		cnt <= cnt + 10'b1;
		if(cnt == 10'd875) begin				
			cnt <= 10'b0;
		end
	end 
end

assign baud_clk = cnt == 10'd0;

endmodule 
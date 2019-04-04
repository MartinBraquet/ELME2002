module pneumatic (input clk,
					input reset,
					input [3:0] signalrasp,
					output valve1, 
					output valve2, 
					output piston1, 
					output piston2, 
					output piston3
					);
					


always_comb

case (signalrasp)

4'd0 : begin				//Rentrer tous les pistons + éteindre ventouses
        piston1 = 0;
		  piston2 = 0;
		  piston3 = 0;
		  valve1 = 	0;
		  valve2	=	0;
		  end

 4'd1 : begin				//Sortir les pistons de l etage 1 + ventouses
        piston1 = 1;
		  piston2 = 1;
		  piston3 = 0;
		  valve1 = 	1;
		  valve2	=	0;
		  end
					
 4'd2 : begin				//Rentrer le plus long piston de l etage 1 et eteindre ventouse --> 3 palets charges
        piston1 = 0;
		  piston2 = 1;
		  valve1 = 	0;
		  valve2=	0;
		  piston3 = 0;
		  end
		  
 4'd3 :  begin				// Chercher les 3 palets restant
        piston1 = 1;
		  piston2 = 1;
		  valve1 =	1;
		  valve2=	0;
		  piston3 = 0;
		  end		  
		  
 4'd4 : begin				// Se deplacer avec les 3 palets rentres et accroches aux ventouses
		  piston1 = 0;
		  piston2 = 0;
		  valve1 = 	1;
		  valve2=	0;
		  piston3 = 0;
		  end
			
 4'd5 :	begin				// Garder les 3 palets accroches aux ventouses du bas et sortir piston du haut pour decharger les 3 palets du bac
        piston1 = 0;
		  piston2 = 0;
		  valve1  = 1;
		  valve2  = 1;
		  piston3 = 1;
		  end
 
  4'd6 :	begin				// Rentrer piston du haut 
        piston1 = 0;
		  piston2 = 0;
		  valve1 =  1;
		  valve2=   0;
		  piston3 = 0;
		  end
 
  4'd7 :	begin				// Lacher les 3 derniers palets dans le bac
        piston1 = 0;
		  piston2 = 1;
		  valve1  = 0;
		  valve2  = 0;
		  piston3 = 0;
		  end
		  
 4'd8 :	begin				// Decharger les 3 palets du bac
        piston1 = 0;
		  piston2 = 0;
		  valve1 =  0;
		  valve2=   1;
		  piston3 = 1;
		  end
	
	
	default : 
	begin    valve1 =  0; 
				valve2 =  0;
				piston1 = 0; 
				piston2 = 0; 
	         piston3 = 0;
			end
	
	
	endcase

					
endmodule



//logic valve1_mod1, valve1_mod2, piston1_mod1,piston1_mod2,piston2_mod1,piston2_mod2,piston2_mod3;
//logic valve2_mod1,valve2_mod2,piston3_mod1,piston3_mod2;
//
//			
// seq1 seq1(.clk(clk),
//					.reset(reset),.count(count),
//					 .valve1(valve1_mod1),  
//					 .piston1(piston_mod1), 
//					 .piston2(piston2_mod1)
//					);
//					
//seq2 seq2(.clk(clk),
//					.reset(reset),.count(count),
//					 .piston1(piston1_mod2), 
//					 .piston2(piston2_mod2)
//					);
//					
//seq2 seq2_bis(.clk(clk),
//					.reset(reset),.count(count),
//					 .piston2(piston1_mod2), 
//					 .valve1(valve1_mod2)
//					);					
//					
//			
//seq3 seq3(.clk(clk),
//					.reset(reset),.count(count),
//					 .piston3(piston3_mod1),
//					 .valve2(valve2_mod1),
//					);
//					
// seq4 seq4(.clk(clk),
//					.reset(reset),.count(count),
//					 .piston3(piston3_mod2),
//					 .valve2(valve2_mod2),
//					);
//	
//			
			
			
//
//case (signalrasp)
//
// 1'd1 : begin
//        piston1 = piston1_mod1;
//		  piston2 = piston1_mod2;
//		  piston3 = 0;
//		  valve1 = valve1_mod1;
//		  valve2=0;
//		  end
//					
// 1'd2 : begin
//        piston1 = piston1_mod2;
//		  piston2 = piston2_mod2;
//		  valve1 = valve1_mod2;
//		  valve2=0;
//		  piston3 = 0;
//		  end
//		  
// 1'd3 :  begin
//       
//		  piston2 = piston2_mod3;
//		  valve1 = valve1_mod2;
//		  valve2=0;
//		  piston3 = 0;
//		  end		  
//		  
// 1'd4 : begin
//        piston3 = piston3_mod1;
//		  valve2 = valve2_mod1;
//		  end
//			
// 1'd5 :	begin
//        piston3 = piston3_mod2;
//		  valve2 = valve2_mod2;
//		  end
// 
//
//					
//
//	
//	
//	default : 
//	begin    valve1 =0; 
//				valve2 = 0;
//				piston1 = 0; 
//				piston2 = 0; 
//	         piston3 = 0;
//				end
//	
//	
//	endcase
//
//					
//endmodule

////Piston1 go out
////Piston2  go out
////Valve1 go active
//
//module seq1 (input clk,
//					input reset,input [31:0] count,
//					output valve1,  
//					output piston1, 
//					output piston2
//					);
//
//typedef enum logic [1:0] {S0 , S1, S2, S3 } statetype;
//statetype state, nextstate;
//
//
//// state register
//always_ff @(posedge clk, posedge reset)
//if (reset) state <= S0;
//else state <= nextstate;
//
//
//// next state logic
//always_comb
//
//case (state)
//
//S0: if (count >= 10000) nextstate = S1;
//else nextstate = S0;
//
//
//S1: if ( count >=50000000) nextstate = S2;
//else nextstate = S1;
//
//
//S2: if ( count >=60000000) 
//		nextstate = S3;
//	else
//		nextstate = S2;
// 
// S3: if ( count >=60000000) 
//		nextstate = S3;
//	else
//		nextstate = S3;
//
//
//
//default: nextstate = S0;
//
//endcase
//
////output logic
//assign piston1 = ((state==S1)||(state==S2)||(state==S3));
//assign piston2 = ((state==S2)||(state==S3));
//assign valve1 = (state==S3);
//endmodule
//
//
////Piston1 go in
////Piston2 go in
////Vavle1 still active
//
//module seq2 (input clk,
//					input reset,input [31:0] count,  
//					output piston1, 
//					output piston2
//					);
//
//typedef enum logic [1:0] {S0 , S1, S2} statetype;
//statetype state, nextstate;
//
//
//// state register
//always_ff @(posedge clk, posedge reset)
//if (reset) state <= S0;
//else state <= nextstate;
//
//
//// next state logic
//always_comb
//
//case (state)
//
//S0: if (count >= 70000000) nextstate = S1;
//else nextstate = S0;
//
//
//S1: if ( count >=80000000) nextstate = S2;
//
//else nextstate = S1;
//
//
//S2: if ( count >=90000000) 
//
// nextstate = S2;
//
//
//
//
//default: nextstate = S0;
//
//endcase
//
////output logic
//
//assign piston2 = ~((state==S1)||(state==S2));
//assign piston1 = ~(state==S2);
//
//
//endmodule
//
//
////
////Piston2 go in
//// valve1 go inactive
////Vavle1 still active
//module seq2_bis (input clk,
//					input reset,input [31:0] count,  
//					output piston2, 
//					output valve1
//					);
//
//typedef enum logic [1:0] {S0 , S1, S2} statetype;
//statetype state, nextstate;
//
//
//// state register
//always_ff @(posedge clk, posedge reset)
//if (reset) state <= S0;
//else state <= nextstate;
//
//
//// next state logic
//always_comb
//
//case (state)
//
//S0: if (count >= 70000000) nextstate = S1;
//else nextstate = S0;
//
//
//S1: if ( count >=80000000) nextstate = S2;
//
//else nextstate = S1;
//
//
//S2: if ( count >=90000000) 
//
// nextstate = S2;
//
//
//
//
//default: nextstate = S0;
//
//endcase
//
////output logic
//
//assign piston2 = ~((state==S1)||(state==S2));
//assign valve1 = ~(state==S2);
//
//
//endmodule
//
//
////Piston3 go out
////valve2 go active
//
//module seq3 (input clk,
//					input reset,input [31:0] count,  
//					output piston3, 
//					output valve2
//					);
//
//typedef enum logic [1:0] {S0 , S1, S2} statetype;
//statetype state, nextstate;
//
//
//// state register
//always_ff @(posedge clk, posedge reset)
//if (reset) state <= S0;
//else state <= nextstate;
//
//
//// next state logic
//always_comb
//
//case (state)
//
//S0: if (count >= 120000000) nextstate = S1;
//else nextstate = S0;
//
//
//S1: if ( count >=130000000) nextstate = S2;
//
//else nextstate = S1;
//
//
//S2: if ( count >=140000000) 
//
// nextstate = S2;
//
//
//
//
//default: nextstate = S0;
//
//endcase
//
////output logic
//
//assign piston3 = ((state==S1)||(state==S2));
//assign valve2 = (state==S2);
//
//
//endmodule
//
//
//
//
////Piston3 go in
////valve2 go inactive
//
//module seq4 (input clk,
//					input reset,input [31:0] count,  
//					output piston3, 
//					output valve2
//					);
//
//typedef enum logic [1:0] {S0 , S1, S2} statetype;
//statetype state, nextstate;
//
//
//// state register
//always_ff @(posedge clk, posedge reset)
//if (reset) state <= S0;
//else state <= nextstate;
//
//
//// next state logic
//always_comb
//
//case (state)
//
//S0: if (count >= 150000000) nextstate = S1;
//else nextstate = S0;
//
//
//S1: if (piston3 == 1'b0 && count >=160000000) nextstate = S2;
//
//else nextstate = S1;
//
//
//S2: if (valve2 == 1'b0 && count >=170000000) 
//
// nextstate = S2;
//
//
//
//
//default: nextstate = S0;
//
//endcase
//
////output logic
//
//assign piston3 = ~((state==S1)||(state==S2));
//assign valve2 = ~(state==S2);





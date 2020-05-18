
module control (input [5:0] instruction,
					output reg_dst, jump, branch, mem_read, mem_to_reg, alu_op, mem_write, alu_src, reg_write);

	/*
	instruction    reg_dst    jmp    branch    mem_read     mem_to_reg    alu_op    mem_write    alu_src    reg_write		
	  000000          1        0        0         0             0            1          0           0           1	
	  000001          0        0        1         0             0            1          0           0           0	
	  000010          0        1        0         0             0            0          0           0           0	
	  000011          1        1        0         0             1            0          0           0           1	
	  000100          0        0        1         0             0            1          0           0           0	
	  000101          0        0        1         0             0            1          0           0           0	
	  000110          0        0        1         0             0            1          0           0           0	
	  001001          0        0        0         0             0            1          0           1           1	
	  001010          0        0        0         0             0            1          0           1           1
	  001011          0        0        0         0             0            1          0           1           1
	  001100          0        0        0         0             0            1          0           1           1	
	  001101          0        0        0         0             0            1          0           1           1	
	  001110          0        0        0         0             0            1          0           1           1          	
	  001111		  0		   0        0		  0             0            1          0           1           1
	  100000		  0	       0        0         1             1          	 0          0           1           1
	  100011		  0	       0        0         1             1          	 0          0           1           1 	
	  101000		  0        0        0         0             0            0          1           1           0          	
	  101011		  0        0        0         0             0            0          1           1           0
	*/
	
	assign reg_dst 		= (instruction[5:2] == 4'b0) ? 
							(((instruction[1:0] == 2'b00) || (instruction[1:0] == 2'b11)) ? 1'b1 : 1'b0) : 1'b0;
	assign jmp 			= (instruction[5:2] == 4'b0) ? 
							(((instruction[1:0] == 2'b10) || (instruction[1:0] == 2'b11)) ? 1'b1 : 1'b0) : 1'b0;
	assign branch 		= (instruction[5:2] == 4'b0001) ? 1'b1 : 
							((instruction == 5'b000001) ? 1'b1 : 1'b0);
	assign mem_read		= ({instruction[5], instruction[3]} == 2'b10) ? 1'b1 : 1'b0;
	
	assign mem_to_reg 	= ({instruction[5], instruction[3]} == 2'b10) ? 1'b1 : 
							((instruction == 5'b000011) ? 1'b1 : 1'b0);
	assign alu_op 		= (instruction[5:1] == 5'b1) ? 1'b0 : 
							((instruction == 5'b00001) ? 1'b0 : 1'b1);
	assign mem_write 	= (instruction[5:3] == 3'b101) ? 1'b1 : 1'b0;
	
	assign alu_src		= (instruction[5:3] == 3'b0) ? 1'b0 : 1'b1;
	
	assign reg_write	= ((instruction[5:2] == 4'b0001) || (instruction[5:3] == 3'b101)) ? 1'b0 :
							(((instruction == 5'b000001) || (instruction == 5'b000010)) ? 1'b0 : 1'b1);

endmodule
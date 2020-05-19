
module control (input [5:0] instruction,
					output reg_dst, jump, branch, mem_read, mem_to_reg, alu_op, mem_write, alu_src, reg_write);

	/*
	instruction    reg_dst    jump    branch    mem_read     mem_to_reg    alu_op    mem_write    alu_src    reg_write		
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
	assign jump 		= (instruction[5:2] == 4'b0) ? 
							(((instruction[1:0] == 2'b10) || (instruction[1:0] == 2'b11)) ? 1'b1 : 1'b0) : 1'b0;
	assign branch 		= (instruction[5:2] == 4'b0001) ? 1'b1 : 
							((instruction == 5'b000001) ? 1'b1 : 1'b0);
	assign mem_read		= ({instruction[5], instruction[3]} == 2'b10) ? 1'b1 : 1'b0;
	
	assign mem_to_reg 	= ({instruction[5], instruction[3]} == 2'b10) ? 1'b1 : 
							((instruction == 5'b000011) ? 1'b1 : 1'b0);
	assign alu_op 		= (instruction[5] == 1'b1) ? 1'b0 : 
							((instruction == 5'b000010 || instruction == 5'b000011) ? 1'b0 : 1'b1);
	assign mem_write 	= (instruction[5:3] == 3'b101) ? 1'b1 : 1'b0;
	
	assign alu_src		= (instruction[5:3] == 3'b0) ? 1'b0 : 1'b1;
	
	assign reg_write	= ((instruction[5:2] == 4'b0001) || (instruction[5:3] == 3'b101)) ? 1'b0 :
							(((instruction == 5'b000001) || (instruction == 5'b000010)) ? 1'b0 : 1'b1);

endmodule

module register (input clk, reg_write, [4:0] reg_1, [4:0] reg_2, [4:0] write_reg, [31:0] write_data, 
					output [31:0] read_data_1,  [31:0] read_data_2);

	reg [31:0] regs [31:0];
	
	integer i;
	initial begin
		for (i=0; i<32; i=i+1)
			regs[i] = 32'b0;
	end
	
	always @ (posedge clk) begin
		if (reg_write) begin
			regs[write_reg] = write_data;
		end
	end
	
	assign read_data_1 = regs[reg_1];
	assign read_data_2 = regs[reg_2];

endmodule

module arithmetic_logic_unit (input [5:0] alu_control, [31:0] A, [31:0] B,
								output zero, reg [31:0] alu_result);

	wire two_s_complement_of_b;
	wire [31:0] addition_res, subtraction_res, multiplication_res;
						
	assign two_s_complement_of_b = ~B + 1'b1;
						
	adder_31_bit adder (A, B, addition_res);
	adder_31_bit for_sub (A, two_s_complement_of_b, subtraction_res);
	
	multiplier_31_bit (A, B[15:0], multiplication_res);
					
	always @ (*) begin
		case (alu_control)
			6'b100000: 
				// addition
				alu_result = addition_res;
			6'b100001:
				// unsigned addition
				alu_result = addition_res;
			6'b100100:
				// bitwise AND
				alu_result = A & B;
			6'b011010:
				// division
				alu_result = A / B;
			6'b011011:
				// unsigned division
				alu_result = A / B;
			6'b011000:
				// multiplication
				alu_result = multiplication_res;
			6'b011001:
				// unsigned multiplication
				alu_result = multiplication_res;
			6'b100010:
				// subtraction
				alu_result = subtraction_res;
			6'b100011:
				// unsigned subtraction
				alu_result = subtraction_res;
			6'b100110:
				// bitwise XOR
				alu_result = A ^ B;
			6'b100101:
				// bitwise OR
				alu_result = A | B;
			6'b101010:
				// slt signed
				begin
					if (A[31] == 1'b1 && B[31] == 1'b1) begin
						if (A[30:0] > B[30:0])
							alu_result = 1'b1;
						else
							alu_result = 1'b0;
					end 
					else if (A[31] == 1'b1)
						alu_result = 1'b1;
					else if (B[31] == 1'b1)
						alu_result = 1'b0;
					else begin
						if (A[30:0] > B[30:0])
							alu_result = 1'b1;
						else
							alu_result = 1'b0;
					end
				end
			6'b101011:
				// slt unsigned
				begin
					if (A < B)
						alu_result = 1'b1;
					else
						alu_result = 1'b0;
				end
			6'b000011:
				// sra
				alu_result = 6'b0; // later
			6'b000010:
				// srl
				alu_result = 6'b0; // later
			6'b000110:
				// srlv
				alu_result = B >> A;
			6'b000100:
				// sllv
				alu_result = B << A;
			default:
				alu_result = 32'b0;
		endcase
	end

endmodule

module adder_31_bit (input [31:0] A, [31:0] B, 
						output [31:0] sum);

	genvar i;
	wire [30:0] c;
	wire c_out;

	for (i=0; i<32; i=i+1) begin
		if (i == 0)
			full_adder fa (A[i], B[i], 1'b0, sum[i], c[i]);
		else if (i == 31) begin
			full_adder fa (A[i], B[i], 1'b0, sum[i], c_out);
		end else
			full_adder fa (A[i], B[i], c[i-1], sum[i], c[i]);
	end
	
	assign sum[31] = (c_out) ? 1'b1 : sum[31];

endmodule

module full_adder(x, y, c_in, sum, c_out);

	input x, y, c_in;
	output sum, c_out;

	assign {c_out, sum} = x + y + c_in;

endmodule

module multiplier_31_bit (input [31:0] x, [15:0] y,
							output reg [31:0] res);

	reg [15:0] i = 0;
	reg [31:0] t_x;

	always @ (x or y) begin
		res = 0;
		t_x = x;
		for (i=0; i<16; i=i+1) begin
			if (y[i] == 1)
				res = res + t_x;
			t_x = t_x << 1;
		end
	end

endmodule

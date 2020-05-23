
module mips_processor;
	
	reg clk;
	
	initial begin
				clk = 1'b0;
		#160 	$stop;
	end
	
	always #5 clk = ~clk;
	
	reg [31:0] program_counter = -1;
	reg [31:0] instruction_memory [31:0];
	reg [31:0] instruction = 32'b0;
	
	initial begin
		$readmemh("test_program_1_mem_file.dat", instruction_memory);
	end
	
	wire zero;
	wire [2:0] alu_op;
	wire [3:0] alu_code;
	wire [31:0] alu_result;
	wire [31:0] read_data, read_data_1, read_data_2;
	wire reg_dst, jump, branch, mem_read, mem_to_reg, mem_write, alu_src, reg_write;
	
	wire [31:0] alu_in_reg_2;
	wire [31:0] sign_extended_15_0;
	wire [4:0]	write_register;
	wire [31:0] reg_write_data;
	wire branch_enable;
	
	assign sign_extended_15_0 = {{16{instruction[15]}}, instruction[15:0]};
	assign alu_in_reg_2 = (alu_src == 1'b0) ? read_data_2 : sign_extended_15_0;
	assign write_register = (reg_dst == 1'b0) ? instruction[20:16] : instruction[15:11];
	assign reg_write_data = (mem_to_reg == 1'b1) ? read_data : alu_result;
	assign branch_enable = branch & zero;
	
	control cntrl_unit (instruction[31:26], reg_dst, jump, branch, mem_read, 
							mem_to_reg, mem_write, alu_src, reg_write, alu_op);
							
	alu_control alu_cntrl (alu_op, instruction[5:0], alu_code);
	
	register reg_unit (clk, reg_write, instruction[25:21], instruction[20:16], write_register, reg_write_data, read_data_1, read_data_2);
	
	arithmetic_logic_unit alu (alu_code, read_data_1, alu_in_reg_2, zero, alu_result);
								
	data_memory memory (clk, mem_read, mem_write, alu_result, read_data_2, read_data);
	
	always @ (posedge clk) begin
		if (jump == 1'b0 && branch_enable == 1'b0)
			program_counter = program_counter + 1;
		else if (jump == 1'b0 && branch_enable == 1'b1) begin
			program_counter = program_counter + 1;
			program_counter = program_counter + (sign_extended_15_0 << 2);
		end else if (jump == 1'b1) begin
			program_counter = program_counter + 1;
			program_counter = {program_counter[31:28], instruction[25:0] << 2};
		end
		
		instruction = instruction_memory[program_counter];
		$display("%t INSTR %b", $time, instruction);
	end
	
endmodule

module control (input [5:0] instruction,
					output reg_dst, jump, branch, mem_read, mem_to_reg, mem_write, alu_src, reg_write, [2:0] alu_op);

	/*
	instruction    reg_dst    jump    branch    mem_read     mem_to_reg    alu_op    mem_write    alu_src    reg_write		
	  000000          1        0        0         0             0           111         0           0           1
	  000010          0        1        0         0             0           000         0           0           0
	  000011          0        1        0         0             1           000         0           0           1
	  000100          0        0        1         0             0           001         0           0           0
	  000101          0        0        1         0             0           101         0           0           0
	  001000          0		   0		0		  0				0			000			0			1			1
	  001001          0        0        0         0             0           000         0           1           1
	  001010          0        0        0         0             0           001         0           1           1
	  001011          0        0        0         0             0           001         0           1           1
	  001100          0        0        0         0             0           010         0           1           1
	  001101          0        0        0         0             0           011         0           1           1
	  001110          0        0        0         0             0           100         0           1           1
	  100000		  0	       0        0         1             1          	000         0           1           1
	  100011		  0	       0        0         1             1          	000         0           1           1
	  101000		  0        0        0         0             0          	000         1           1           0
	  101011		  0        0        0         0             0           000         1           1           0
	*/
	
	reg [10:0] out;
	
	always @ (instruction) begin																				
		case (instruction)
			6'b000000:
				out = 11'b10000111001;
			6'b000010:
				out = 11'b01000000000;
			6'b000011:
				out = 11'b01001000001;
			6'b000100:
				out = 11'b00100001000;
			6'b000101:
				out = 11'b00100101000;
			6'b001000:
				out = 11'b00000000011;
			6'b001001:
				out = 11'b00000000011;
			6'b001010:
				out = 11'b00000001011;
			6'b001011:
				out = 11'b00000001011;
			6'b001100:
				out = 11'b00000010011;
			6'b001101:
				out = 11'b00000011011;
			6'b001110:
				out = 11'b00000100011;
			6'b100000:
				out = 11'b00011000011;
			6'b100011:
				out = 11'b00011000011;
			6'b101000:
				out = 11'b00000000110;
			6'b101011:
				out = 11'b00000000110;
			default:
				out = 11'b0;
		endcase
		
		$display("%t CNTRL %b %b", $time, instruction, out);
	end

	assign reg_dst 		= out[10];
	assign jump 		= out[9];
	assign branch 		= out[8];
	assign mem_read 	= out[7];
	assign mem_to_reg 	= out[6];
	assign alu_op 		= out[5:3];
	assign mem_write 	= out[2];
	assign alu_src 		= out[1];
	assign reg_write 	= out[0];

endmodule

module alu_control (input [2:0] alu_op, [5:0] func,
					output reg [3:0] alu_code);
						
	// addition 		4'b0000
	// subtraction 		4'b0001
	// multiplication 	4'b0010
	// division			4'b0011
	
	// and				4'b0100
	// or				4'b0101
	// nor				4'b0110
	// xor				4'b0111
	
	// shift_right		4'b1000
	// shift_left		4'b1001
	// sub_not			4'b1010
					
	always @ (alu_op or func) begin		
		case (alu_op)
			3'b000:
				alu_code = 4'b0000;
			3'b001:
				alu_code = 4'b1010;
			3'b010:
				alu_code = 4'b0100;
			3'b011:
				alu_code = 4'b0101;
			3'b100:
				alu_code = 4'b0111;
			3'b101:
				alu_code = 4'b1010;
			3'b111:
				begin
					case (func)
						6'b100000:		// addition
							alu_code = 4'b0000;
						6'b100001:		// addition u
							alu_code = 4'b0000;
						6'b100100:		// and
							alu_code = 4'b0100;
						6'b100111:		// nor
							alu_code = 4'b0110;
						6'b100101:		// or
							alu_code = 4'b0101;
						6'b101010:		// slt
							alu_code = 4'b1010;
						6'b101011:		// slt u
							alu_code = 4'b1010;
						6'b100010:		// sub
							alu_code = 4'b0001;
						6'b100011:		// sub u
							alu_code = 4'b0001;
						6'b100110:		// xor
							alu_code = 4'b0111;
						6'b000000:		// shift left
							alu_code = 4'b1001;
						6'b000100:		// shift left
							alu_code = 4'b1001;
						6'b000011:		// shift right
							alu_code = 4'b1000;
						6'b000111:		// shift right
							alu_code = 4'b1000;
						6'b000010:		// shift right
							alu_code = 4'b1000;
						6'b000110:		// shift right
							alu_code = 4'b1000;
					endcase
				end
		endcase
		
		$display("%t ALU_CNTRL %b %b %b", $time, alu_op, func, alu_code);
	end
	
endmodule

module register (input clk, reg_write, [4:0] reg_1, [4:0] reg_2, [4:0] write_reg, [31:0] write_data, 
					output [31:0] read_data_1,  [31:0] read_data_2);

	reg [31:0] regs [31:0];
	
	integer i;
	initial begin
		for (i=0; i<255; i=i+1) begin
			regs[i] <= 32'b0;
		end
	end
	
	always @ (posedge clk) begin
		if (reg_write) begin
			regs[write_reg] <= write_data;
		end
		
		$writememh("reg_file.dat", regs);
		$display("%t REG %b %b %b %b %b %b %b", $time, reg_write, reg_1, reg_2, write_reg, write_data, 
																				read_data_1, read_data_2);
	end
	
	assign read_data_1 = regs[reg_1];
	assign read_data_2 = regs[reg_2];
		
	/*
	initial begin
		#70 begin
			for (i=0; i<32; i=i+1) begin
				$display("%t %b -> %b", $time, i, regs[i]);
			end
		end
	end
	*/

endmodule

module arithmetic_logic_unit (input [3:0] alu_control, [31:0] A, [31:0] B,
								output reg zero, [31:0] alu_result);
	
	reg sub = 0;
	wire [31:0] add_sub_result;
	
	adder_subtracter_32_bit adder (sub, A, B, add_sub_result);
													
	always @ (alu_control or A or B or add_sub_result) begin		
		case (alu_control)
			4'b0000: 
				alu_result = add_sub_result;
			4'b0001:
				begin
					sub = 1;
					alu_result = add_sub_result;
					if (alu_result == 32'b0)
						zero = 1'b1;
					else
						zero = 1'b0;
				end
			/*4'b1010:
				begin
					sub = 1;
					alu_result = add_sub_result;
					if (alu_result != 32'b0)
						zero = 1'b1;
					else
						zero = 1'b0;
				end
			*/
			4'b0010:
				alu_result = A * B;
			4'b0011:
				alu_result = A / B;
			4'b0100:
				alu_result = A & B;
			4'b0101:
				alu_result = A | B;
			4'b0110:
				alu_result = ~(A | B);
			4'b0111:
				alu_result = A ^ B;
			4'b1000:
				alu_result = B >> A;
			4'b1001:
				alu_result = B << A;
			4'b1010:
				begin
					if (A < B)
						alu_result = 32'b1;
					else
						alu_result = 32'b0;
				end
			default:
				alu_result = 32'b0;
		endcase
		
		$display("%t ALU %b %b %b %b %b", $time, alu_control, A, B, zero, alu_result);
	end
		
endmodule

module data_memory(input clk, mem_read, mem_write, [31:0] address, [31:0] write_data, 
					output reg [31:0] read_data);

	reg [31:0] mem [7:0];
	
	integer i;
	initial begin
		for (i=0; i<255; i=i+1)
			mem[i] = 32'b0;
	end
	
	always @ (posedge clk) begin
		if (mem_write)
			mem[address] = write_data;
		if (mem_read)
			read_data = mem[address];
		else
			read_data = 32'b0;
			
		$display("%t DATA_MEM %b %b %b %b %b", $time, mem_read, mem_write, address, write_data, read_data);
	end

endmodule

module full_adder(input x, y, c_in, 
					output reg sum, c_out);
		
	always @ (*) begin
		{c_out, sum} <= x + y + c_in;
	end

endmodule

module adder_subtracter_32_bit (input sub, [31:0] A, B,
									output [31:0] sum);

	genvar i;
	wire [31:0] c;
	wire [31:0] FIN_B;
	
	assign FIN_B = (sub == 1'b1) ? (~B + 1) : B;
 	
	generate 
		for (i=0; i<32; i=i+1) begin
			if (i == 0)
				full_adder fa (A[i], FIN_B[i], 1'b0, sum[i], c[i]);
			else
				full_adder fa (A[i], FIN_B[i], c[i-1], sum[i], c[i]);
		end 
	endgenerate
	
endmodule

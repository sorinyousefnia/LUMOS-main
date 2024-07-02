`include "Defines.vh"

module Fixed_Point_Unit 
#(
    parameter WIDTH = 32,
    parameter FBITS = 10
)
(
    input wire clk,
    input wire reset,
    
    input wire [WIDTH - 1 : 0] operand_1,
    input wire [WIDTH - 1 : 0] operand_2,
    
    input wire [ 1 : 0] operation,

    output reg [WIDTH - 1 : 0] result,
    output reg ready

    
);

    always @(*)
    begin
        case (operation)
            `FPU_ADD    : begin result <= operand_1 + operand_2; ready <= 1; end
            `FPU_SUB    : begin result <= operand_1 - operand_2; ready <= 1; end
            `FPU_MUL    : begin result <= product[WIDTH + FBITS - 1 : FBITS]; ready <= product_ready; end
            `FPU_SQRT   : begin result <= root; ready <= root_ready; end
            default     : begin result <= 'bz; ready <= 0; end
        endcase
    end

    always @(posedge reset)
    begin
        if (reset)  ready = 0;
        else        ready = 'bz;
    end
    // ------------------- //
    // Square Root Circuit //
    // ------------------- //
    reg [WIDTH - 1 : 0] root;
    reg root_ready;

        /*
         *  Describe Your Square Root Calculator Circuit Here.
         */

    // Internal signals for square root
    reg [WIDTH - 1 : 0] radicand;
    reg [WIDTH - 1 : 0] temp_root;
    reg [WIDTH - 1 : 0] temp_result;
    reg [WIDTH - 1 : 0] final_result;
    reg [WIDTH - 1 : 0] iteration;
    reg [WIDTH - 1 : 0] operand_1_temp;

    // State machine for square root
    localparam INITIAL = 2'd0;  // 2'd0 -> zero in decimal 
    localparam SQRT = 2'd1;     // 2'd1 -> one in decimal
    reg state1;
    reg [1 : 0] two_bit;

    always @(posedge clk) begin
        if (reset)
        begin
            state1 <= INITIAL;
            root <= 0;
            root_ready <= 0;
            operand_1_temp <= operand_1;
        end
        else 
        begin
            case (state1)
                INITIAL: begin
                    if (operation == `FPU_SQRT) begin
                        // Initialize for square root calculation
                        radicand <= operand_1[WIDTH - 1: WIDTH - 2];
                        temp_root <= 2'b01;
                        iteration <= (WIDTH + FBITS) / 2; // Calculate iterations for fixed-point
                        state1 <= SQRT;
                        final_result <= 0;
                    end
                end
                SQRT: begin
                    if (iteration > 0) 
                    begin
                        //radicand minus temp_root to find next digit for final_result
                        temp_result <= radicand - temp_root;
                        //Checking if temp_result is negative or not
                        if(temp_result < 0 ) 
                        begin
                            //If result is negative next digit is 0 and a shift will work
                            final_result <= (final_result << 1);
                        end 
                        else 
                        begin
                            //If result is not negative next digit will be 1. A shift and a plus 1 
                            final_result <= (final_result << 1) + 1;
                        end

                        //Shifting operan_1_temp to bring next two MSB bits
                        operand_1_temp <= operand_1_temp << 2;
                        //Extract two MSB bits of operand_1e_temp
                        two_bit <= operand_1_temp[WIDTH - 1 : WIDTH - 2];
                        //Append 01 to radicand for next iteration
                        radicand <= (radicand << 2) + two_bit;
                        //Append 01 to result for next iteration
                        temp_root <= (final_result << 2) + 1 ;
                        
                        iteration <= iteration - 1;
                    end else begin
                        root <= final_result;
                        root_ready <= 1;
                        state1 <= INITIAL;
                    end
                end
            endcase
        end
    end

        


    // ------------------ //
    // Multiplier Circuit //
    // ------------------ //   
    reg [64 - 1 : 0] product;
    reg product_ready;

    reg     [15 : 0] multiplierCircuitInput1;
    reg     [15 : 0] multiplierCircuitInput2;
    wire    [31 : 0] multiplierCircuitResult;

    Multiplier multiplier_circuit
    (
        .operand_1(multiplierCircuitInput1),
        .operand_2(multiplierCircuitInput2),
        .product(multiplierCircuitResult)
    );

    reg     [31 : 0] partialProduct1;
    reg     [31 : 0] partialProduct2;
    reg     [31 : 0] partialProduct3;
    reg     [31 : 0] partialProduct4;
    
        /*
         *  Describe Your 32-bit Multiplier Circuit Here.
         */

    //Three bit register to store 32-bit multiplication state
    reg [2:0] state;

    //Define parameters for each state
    localparam INITIALSTATE = 3'b000;   // 3'b000 -> 0
    localparam FIRSTMUL = 3'b001;       //3'b001 -> 1
    localparam SECONDMUL = 3'b010;      //3'b010 -> 2
    localparam THIRDMUL = 3'b011;       //3'b011 -> 3
    localparam FOURTHMUL = 3'b100;      //3'b100 -> 4
    localparam ADD = 3'b101;            //3'b101 -> 5

    always @(posedge clk) begin
        if (reset)                  // if reset all state parms should be 0 
        begin
            state <= INITIALSTATE;  // set state to 0 
            product <= 0;
            product_ready <= 0;
        end
         else 
         begin
            case (state)
                //Initial state to initialize registers
                INITIALSTATE:
                begin
                    if (operation == `FPU_MUL) begin   // check if our Fpu multiply function is called 
                        // Al * Bl
                        multiplierCircuitInput1 <= operand_1[15 : 0];
                        multiplierCircuitInput2 <= operand_2[15 : 0];
                        //Partial products should be 0 at start
                        partialProduct1 <= 0;
                        partialProduct2 <= 0;
                        partialProduct3 <= 0;
                        partialProduct4 <= 0;
                        // Change state to FIRSTMUL at the next cycle
                        state <= FIRSTMUL;
                    end
                end
                FIRSTMUL: begin
                    // Store partial product result
                    partialProduct1 <= multiplierCircuitResult;
                    // Ah * Bl
                    multiplierCircuitInput1 <= operand_1[31 : 16];
                    multiplierCircuitInput2 <= operand_2[15 : 0];
                    // Change state to be SECONDMUL at next cycle
                    state <= SECONDMUL;
                end
                SECONDMUL: begin
                    // Store partial product result with 16-bit shift to the left
                    partialProduct2 <= multiplierCircuitResult << 16;
                    // Al * Bh
                    multiplierCircuitInput1 <= operand_1[15 : 0];
                    multiplierCircuitInput2 <= operand_2[31 : 16];
                    // Change state to be THIRDMUL at next cycle
                    state <= THIRDMUL;
                end
                THIRDMUL: begin
                    // Store partial product with 16-bit shift to the left
                    partialProduct3 <= multiplierCircuitResult << 16;
                    // Ah * Bh
                    multiplierCircuitInput1 <= operand_1[31 : 16];
                    multiplierCircuitInput2 <= operand_2[31 : 16];
                    // Change state to be FOURTHMUL at next cycle
                    state <= FOURTHMUL;
                end
                FOURTHMUL: begin
                    // Store partial product with 32-bit shift to the left
                    partialProduct4 <= multiplierCircuitResult << 32;
                    //  Change state to be ADD at next cycle
                    state <= ADD;
                end
                ADD: begin
                    // Store partial product
                    product <= partialProduct1 + partialProduct2 + partialProduct3 + partialProduct4;
                    // Move to the beggining
                    state <= INITIALSTATE;
                    product_ready <= 1; 
                end
                default: begin
                    state <= INITIALSTATE;
                end
            endcase
        end
    end
endmodule




module Multiplier
(
    input wire [15 : 0] operand_1,
    input wire [15 : 0] operand_2,

    output reg [31 : 0] product
);

    always @(*)
    begin
        product <= operand_1 * operand_2;
    end
endmodule

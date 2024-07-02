# IRAN UNIVERSITY OF SCIENCE AND TECHNOLOGY
# SORIN YOUSEFNIA    99413289
# SABA PIRAHMADIAN   99411209

<div align="justify">

# Fixed Point Unit Verilog Code Report

## Introduction
The provided Verilog code defines a Fixed Point Unit (FPU) designed to perform basic arithmetic operations, including addition, subtraction, multiplication, and square root calculations, on fixed-point numbers. This report provides an in-depth explanation of the module's functionality, parameters, input/output ports, and the internal workings of the square root and multiplication calculation circuits.

## Module Declaration and Parameters
### Module Declaration
```verilog
module Fixed_Point_Unit 
#(
    parameter WIDTH = 32,
    parameter FBITS = 10
)
```
- **WIDTH**: This parameter defines the bit-width of the operands and the result. The default value is 32 bits.
- **FBITS**: This parameter specifies the number of fractional bits in the fixed-point representation. The default value is 10 bits.

### Input and Output Ports
```verilog
(
    input wire clk,
    input wire reset,
    
    input wire [WIDTH - 1 : 0] operand_1,
    input wire [WIDTH - 1 : 0] operand_2,
    
    input wire [ 1 : 0] operation,

    output reg [WIDTH - 1 : 0] result,
    output reg ready
);
```
- **clk**: Clock signal input.
- **reset**: Reset signal input, active high.
- **operand_1**: First operand for the arithmetic operations.
- **operand_2**: Second operand for the arithmetic operations.
- **operation**: Specifies the operation to be performed (addition, subtraction, multiplication, square root).
- **result**: The result of the arithmetic operation.
- **ready**: Indicates if the result is ready.

## Operation Handling
The module uses a combinational always block to handle different operations based on the value of the `operation` input.
```verilog
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
```
- **`FPU_ADD`**: Performs addition of `operand_1` and `operand_2`, sets `ready` to 1.
- **`FPU_SUB`**: Performs subtraction of `operand_2` from `operand_1`, sets `ready` to 1.
- **`FPU_MUL`**: Multiplies `operand_1` and `operand_2`, and selects the appropriate bits for the result, sets `ready` to `product_ready`.
- **`FPU_SQRT`**: Sets `result` to the computed square root value and `ready` to `root_ready`.
- **default**: Sets `result` to high-impedance and `ready` to 0.

## Reset Handling
The module uses an always block triggered by the reset signal to initialize the `ready` signal.
```verilog
always @(posedge reset)
begin
    if (reset)  ready = 0;
    else        ready = 'bz;
end
```
- When `reset` is high, `ready` is set to 0.
- When `reset` is low, `ready` is set to high-impedance.

## Square Root Calculation
### Registers and Internal Signals
```verilog
reg [WIDTH - 1 : 0] root;
reg root_ready;

reg [WIDTH - 1 : 0] radicand;
reg [WIDTH - 1 : 0] temp_root;
reg [WIDTH - 1 : 0] temp_result;
reg [WIDTH - 1 : 0] final_result;
reg [WIDTH - 1 : 0] iteration;
reg [WIDTH - 1 : 0] operand_1_temp;

localparam INITIAL = 2'd0;
localparam SQRT = 2'd1;
reg state1;
reg [1 : 0] two_bit;
```
- **root**: Stores the final computed square root value.
- **root_ready**: Indicates if the square root calculation is complete.
- **radicand**: Stores the current value being processed in the square root calculation.
- **temp_root**: Temporary register for storing intermediate root values.
- **temp_result**: Temporary register for storing intermediate subtraction results.
- **final_result**: Stores the final square root value before assigning to `root`.
- **iteration**: Counter for the number of iterations required for the square root calculation.
- **operand_1_temp**: Temporary storage for `operand_1` during the calculation.
- **state1**: State variable for the state machine controlling the square root calculation.
- **two_bit**: Temporary storage for the two most significant bits extracted during the calculation.

### State Machine for Square Root Calculation
The square root calculation uses a state machine with two states: INITIAL and SQRT.
```verilog
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
                    radicand <= operand_1[WIDTH - 1: WIDTH - 2];
                    temp_root <= 2'b01;
                    iteration <= (WIDTH + FBITS) / 2;
                    state1 <= SQRT;
                    final_result <= 0;
                end
            end
            SQRT: begin
                if (iteration > 0) 
                begin
                    temp_result <= radicand - temp_root;
                    if(temp_result < 0 ) 
                    begin
                        final_result <= (final_result << 1);
                    end 
                    else 
                    begin
                        final_result <= (final_result << 1) + 1;
                    end

                    operand_1_temp <= operand_1_temp << 2;
                    two_bit <= operand_1_temp[WIDTH - 1 : WIDTH - 2];
                    radicand <= (radicand << 2) + two_bit;
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
```
- **INITIAL state**: Initializes variables for the square root calculation when the operation is `FPU_SQRT`.
- **SQRT state**: Iteratively computes the square root using bitwise operations and updates the state machine and registers accordingly. Once the required number of iterations is complete, the result is assigned to `root` and `root_ready` is set to 1.

### Square Root Algorithm Explanation
The square root algorithm implemented here is an iterative method based on the digit-by-digit calculation. It processes two bits of the operand at a time, updating the intermediate results (`radicand` and `final_result`) accordingly. The algorithm performs the following steps:
1. Initialize the radicand with the two most significant bits of `operand_1` and `temp_root` with binary '01'.
2. Iterate for `(WIDTH + FBITS) / 2` cycles:
   - Subtract `temp_root` from the current `radicand`.
   - If the result is negative, shift `final_result` left by 1 and do not change `radicand`.
   - If the result is non-negative, shift `final_result` left by 1 and add 1, and update `radicand`.
   - Shift `operand_1_temp` left by 2 bits to extract the next two bits and append them to `radicand`.
   - Update `temp_root` with the new value for the next iteration.
3. Assign the final value of `final_result` to `root` and set `root_ready` to 1.


## Multiplier Circuit

### Overview
The multiplier circuit in this Verilog code is designed to perform 32-bit fixed-point multiplication by breaking the operands into 16-bit segments and using a series of partial products to achieve the final result. This approach is efficient for hardware implementation and allows for the use of smaller, simpler multiplier circuits.

### Internal Signals and Registers
The following internal signals and registers are used in the multiplier circuit:

- **product**: A 64-bit register that stores the final computed product value.
- **product_ready**: A flag that indicates whether the multiplication calculation is complete.
- **multiplierCircuitInput1**: A 16-bit register for the first operand segment.
- **multiplierCircuitInput2**: A 16-bit register for the second operand segment.
- **multiplierCircuitResult**: A 32-bit wire that carries the result of the multiplication performed by the `Multiplier` module.
- **partialProduct1**: A 32-bit register that stores the first partial product.
- **partialProduct2**: A 32-bit register that stores the second partial product, shifted left by 16 bits.
- **partialProduct3**: A 32-bit register that stores the third partial product, shifted left by 16 bits.
- **partialProduct4**: A 32-bit register that stores the fourth partial product, shifted left by 32 bits.
- **state**: A 3-bit register that keeps track of the state machine's current state.

### Multiplier Module
The multiplier circuit utilizes a separate `Multiplier` module to perform the actual multiplication of 16-bit segments. This module is instantiated as follows:
```verilog
Multiplier multiplier_circuit
(
    .operand_1(multiplierCircuitInput1),
    .operand_2(multiplierCircuitInput2),
    .product(multiplierCircuitResult)
);
```
The `Multiplier` module multiplies two 16-bit operands and outputs a 32-bit product:
```verilog
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
```
- **operand_1**: The first 16-bit operand.
- **operand_2**: The second 16-bit operand.
- **product**: The 32-bit product of the multiplication.

### State Machine for Multiplication
The multiplier circuit uses a state machine to control the sequence of operations required to compute the final product. The states are defined as follows:
```verilog
localparam INITIALSTATE = 3'b000;
localparam FIRSTMUL = 3'b001;
localparam SECONDMUL = 3'b010;
localparam THIRDMUL = 3'b011;
localparam FOURTHMUL = 3'b100;
localparam ADD = 3'b101;
```
- **INITIALSTATE**: Initialize registers and prepare for the first multiplication.
- **FIRSTMUL**: Compute the first partial product (`Al * Bl`).
- **SECONDMUL**: Compute the second partial product (`Ah * Bl`), shifted left by 16 bits.
- **THIRDMUL**: Compute the third partial product (`Al * Bh`), shifted left by 16 bits.
- **FOURTHMUL**: Compute the fourth partial product (`Ah * Bh`), shifted left by 32 bits.
- **ADD**: Sum all partial products to obtain the final product.

### State Machine Logic
The state machine logic is implemented as follows:
```verilog
always @(posedge clk) begin
    if (reset) begin
        state <= INITIALSTATE;
        product <= 0;
        product_ready <= 0;
    end else begin
        case (state)
            INITIALSTATE: begin
                if (operation == `FPU_MUL) begin
                    multiplierCircuitInput1 <= operand_1[15 : 0];
                    multiplierCircuitInput2 <= operand_2[15 : 0];
                    partialProduct1 <= 0;
                    partialProduct2 <= 0;
                    partialProduct3 <= 0;
                    partialProduct4 <= 0;
                    state <= FIRSTMUL;
                end
            end
            FIRSTMUL: begin
                partialProduct1 <= multiplierCircuitResult;
                multiplierCircuitInput1 <= operand_1[31 : 16];
                multiplierCircuitInput2 <= operand_2[15 : 0];
                state <= SECONDMUL;
            end
            SECONDMUL: begin
                partialProduct2 <= multiplierCircuitResult << 16;
                multiplierCircuitInput1 <= operand_1[15 : 0];
                multiplierCircuitInput2 <= operand_2[31 : 16];
                state <= THIRDMUL;
            end
            THIRDMUL: begin
                partialProduct3 <= multiplierCircuitResult << 16;
                multiplierCircuitInput1 <= operand_1[31 : 16];
                multiplierCircuitInput2 <= operand_2[31 : 16];
                state <= FOURTHMUL;
            end
            FOURTHMUL: begin
                partialProduct4 <= multiplierCircuitResult << 32;
                state <= ADD;
            end
            ADD: begin
                product <= partialProduct1 + partialProduct2 + partialProduct3 + partialProduct4;
                state <= INITIALSTATE;
                product_ready <= 1;
            end
            default: begin
                state <= INITIALSTATE;
            end
        endcase
    end
end
```
- **INITIALSTATE**: If the `FPU_MUL` operation is selected, it initializes the input registers for the first partial product and clears the partial products.
- **FIRSTMUL**: Computes the first partial product (`Al * Bl`) and prepares the inputs for the second partial product.
- **SECONDMUL**: Computes the second partial product (`Ah * Bl`), shifts it left by 16 bits, and prepares the inputs for the third partial product.
- **THIRDMUL**: Computes the third partial product (`Al * Bh`), shifts it left by 16 bits, and prepares the inputs for the fourth partial product.
- **FOURTHMUL**: Computes the fourth partial product (`Ah * Bh`), shifts it left by 32 bits.
- **ADD**: Adds all partial products to form the final product and sets `product_ready` to 1.

### Multiplication Algorithm Explanation
The multiplication algorithm breaks down the 32-bit multiplication into four 16-bit multiplications and then combines the results. This process can be described as follows:

1. **First Partial Product (`Al * Bl`)**:
   - Multiplies the lower 16 bits of both operands.
   - Stores the result directly as `partialProduct1`.

2. **Second Partial Product (`Ah * Bl`)**:
   - Multiplies the upper 16 bits of `operand_1` with the lower 16 bits of `operand_2`.
   - Shifts the result left by 16 bits and stores it as `partialProduct2`.

3. **Third Partial Product (`Al * Bh`)**:
   - Multiplies the lower 16 bits of `operand_1` with the upper 16 bits of `operand_2`.
   - Shifts the result left by 16 bits and stores it as `partialProduct3`.

4. **Fourth Partial Product (`Ah * Bh`)**:
   - Multiplies the upper 16 bits of both operands.
   - Shifts the result left by 32 bits and stores it as `partialProduct4`.

5. **Summing Partial Products**:
   - Adds all four partial products to obtain the final 64-bit product.
   - Sets `product_ready` to 1 to indicate the multiplication is complete.

### Advantages of the Approach
- **Efficiency**: By breaking down the multiplication into smaller segments, the circuit can use simpler, faster 16-bit multipliers.
- **Modularity**: The use of a separate `Multiplier` module allows for easy modification or replacement if a different multiplication algorithm is desired.
- **Scalability**: The approach can be extended to larger bit-widths by further breaking down the operands and increasing the number of partial products.



### Assembly file Report

## Code Breakdown

The code consists of two main sections: initialization and the loop. Below is a detailed breakdown of each instruction.

### Initialization Section

1. Setting Stack Pointer
   ``` Assembly
   li sp, 0x3C00
   ```
   - li (load immediate) instruction loads the immediate value 0x3C00 into the stack pointer (sp).
   - This sets the initial address of the stack to 0x3C00.

2. Setting Global Pointer
    ``` Assembly
   addi gp, sp, 392
   ```
   - addi (add immediate) instruction adds the immediate value 392 to the current value of sp and stores the result in the global pointer (gp).
   - This sets the global pointer to 0x3D88 (0x3C00 + 392).

### Loop Section

The loop begins at the label loop and includes several floating-point operations as well as loop control instructions.

3. Load Floating-Point Values
    ``` Assembly
   flw f1, 0(sp)
   flw f2, 4(sp)
   ```
   - flw (floating-point load word) instructions load a 32-bit floating-point value from the memory address pointed to by sp and sp + 4 into the registers f1 and f2 respectively.
   - This means the code is loading two consecutive floating-point values from the stack into f1 and f2.

4. Floating-Point Multiplications
   ``` Assembly
   fmul.s f10, f1, f1
   fmul.s f20, f2, f2
   ```

   - fmul.s (floating-point multiply single-precision) instructions perform single-precision floating-point multiplication.
   - The first instruction squares the value in f1 and stores the result in f10.
   - The second instruction squares the value in f2 and stores the result in f20.

5. Floating-Point Addition
   ``` Assembly
   fadd.s f30, f10, f20
   ```

   - fadd.s (floating-point add single-precision) instruction adds the values in f10 and f20 and stores the result in f30.
   - This effectively sums the squares of f1 and f2.

6. Square Root Calculation
    ``` Assembly
   fsqrt.s x3, f30
   ```

   - fsqrt.s (floating-point square root single-precision) instruction calculates the square root of the value in f30 and stores the result in x3.

7. Accumulating Result
    ``` Assembly
   fadd.s f0, f0, f3
   ```

   - fadd.s instruction adds the value in f3 to f0 and stores the result back in f0.
   - This accumulates the results of the square root operations in f0.

8. Updating Stack Pointer
    ``` Assembly
   addi sp, sp, 8
   ```

   - addi instruction increments the stack pointer (sp) by 8.
   - This moves the stack pointer to the next pair of floating-point values in memory.

9. Loop Control
    ``` Assembly
   blt sp, gp, loop
   ```

   - blt (branch if less than) instruction compares sp and gp.
   - If sp is less than gp, the code branches back to the label loop.
   - This creates a loop that continues as long as sp is less than gp.

10. Breakpoint
    ``` Assembly
    ebreak
    ```
    - ebreak (environment break) instruction causes the program to halt, often used for debugging purposes.

## Summary

This Verilog assembly code initializes the stack pointer and global pointer, then enters a loop where it processes pairs of floating-point values by calculating the Euclidean distance for each pair and accumulating the results. The loop iterates through a block of memory, updating the stack pointer each time, and halts execution when the stack pointer reaches the global pointer.

### pictures
![plot](/Signal%20Images/first.png)
![plot](/Signal%20Images/second.png)
![plot](/Signal%20Images/third.png)
![plot](/Signal%20Images/forth.png)

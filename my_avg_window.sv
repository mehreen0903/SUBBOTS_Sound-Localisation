`timescale 1ns/1ps
`default_nettype none

/**
 * Average Value Shift Register module
 * sliding window of width N and outputs average
 * This module accepts a stream input that is piped into N shift registers. The output is the
 * average of all values contained in the shift registers.
 *
 * Note: N must be a power of two for this module to calculate the average value correctly.
 *          Module will probably work best with N = 4 or 8 due to NO pipelining in addition stages
 */
module avg_window #(
	parameter int N          = 8, // Number of shift registers to average
   parameter int DATA_WIDTH = 16 // Input/output data bitwidth
) (
    input  wire logic                  clk,
    input  wire logic                  sresetn, //synchronous reset
    input  wire logic [DATA_WIDTH-1:0] data_in,
    input  wire logic                  data_valid, //1 or 0 depending on if its valid?
    output logic [DATA_WIDTH-1:0] average,
    output logic                  average_valid
);


    ////////////////////////////////////////////////////////////////////////////////////////////////
    // SECTION: Types and Constants Declarations


    localparam DIVISION_SHIFT = $clog2(N); // to avg by dividing by N, uses right arithmetic shift, so if N =8, shifts by 3 so divides by 8.


    ////////////////////////////////////////////////////////////////////////////////////////////////
    // SECTION: Signal Declarations


    logic [DATA_WIDTH-1:0] shift_register [N-1:0]; //array of N registers, each 16bits
    logic [N-1:0]          valid_shift_register; //array of N flipflops to track validity


    ////////////////////////////////////////////////////////////////////////////////////////////////
    // SECTION: Output Assignments


    assign average_valid = valid_shift_register[N-1];


    ////////////////////////////////////////////////////////////////////////////////////////////////
    // SECTION: Logic Implementation


    // Shift Register Control Logic
    always_ff @(posedge clk or negedge sresetn) begin
        // Asynchronous Active-Low Reset
        if (!sresetn) begin
            // Set all registers to zero on reset
            shift_register       <= '{default: '0};
            valid_shift_register <= '0;
        end else begin
            // Shift register logic, no backpressure, advance all registers on valid input
            if (data_valid) begin
                shift_register[0]       <= data_in; //new data enters first register
                valid_shift_register[0] <= data_valid; //valid flag set high if data_valid is high

                for (int i = 1; i < N; i++) begin
                    shift_register[i]       <= shift_register[i-1]; //all data shifted right by 1, oldest sample discarded
                    valid_shift_register[i] <= valid_shift_register[i-1]; //same here
                end
            end
            //when data_valid = 0, no more new values are copied into the shift registers
        end
    end

    // Averaging Combinational Logic: calculates average, each value is /N then summed.
    always_comb begin
        average = '0;
        for (int i = 0; i < N; i++) begin
            average += shift_register[i] >>> DIVISION_SHIFT;
            //same as: average = (SR[0] >> DIVISION_SHIFT) + (SR[1] >> DIVISION_SHIFT) + ... + (SR[N-1] >> DIVISION_SHIFT)
            //happens asynchronously
        end
    end
endmodule

`default_nettype wire

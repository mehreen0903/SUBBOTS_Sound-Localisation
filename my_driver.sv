`timescale 1ns/1ps
`default_nettype none

/**
 * ADS8528 ADC Controller
 *
 * This module configures an ADS8528 ADC to run in the 'parallel' mode with an external clk then
 * collects data on a periodic basis, outputting the data using a data_valid signal.
 *
 * Note: this module does not support backpressure on the output.
 *
 * TODO:
 *      - rename module to adc_ads8528_ctrl without breaking Quartus project
 *      - Remove bloat/refactor module
 *      - Confirm module is working in hardware-testing
 *         - Create ADS8528 interface to use as port to this module (ADS8528.Master will contain all adc control signals)
 *      - Standardize the output interface (AXI-Stream?)
 *        - Backpressure support
 *        - Generalize module to adc_ads85x8_ctrl (add parameter for datawidth to support 12, 14, and 16 chip versions)
 */
module driver (
    input  wire         clk,
    input  wire         sresetn, //synch reset, active low
    input  wire         busy,          // Indicates a conversion is taking place on ADS8528 (Active-high)
    
    inout  wire      [15:0] data_adc,  // input/output databits from ADS8528

    // ADS8528 Control Signals
    output logic        read_n,        // Tells ADS8528 its data output has been read (Active-low)
    output logic        write_n,       // Tells ADS8528 a write operation taking place (Active-low)
    output logic        chipselect_n,  // Chip-select bit, must be deasserted for operation (Active-low), always 0 so chips is alway selected
    output logic        software_mode, // Controls whether ADS8528 is in software or hardware mode- set to 1 for software control
    output logic        serial_mode,   // Controls whether ADS8528 is in serial or parallel mode - set to 0 for parallel
    output logic        standby_n,     // Powers entire device down when deasserted and in hardware mode- set to 1 to power chip

    // ADS8528 starts conversion of channel 'x' on rising-edge of conv_start_x
    output logic        conv_start_a, //starts converswion on all channels
    output logic        conv_start_b,
    output logic        conv_start_c,
    output logic        conv_start_d,

    // Driver handshake
    output logic [15:0] data_out,      // ADC data sample out
    output logic        data_valid     // Indicates that data_out is valid
);

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // SECTION: Types and Constants Declarations

    typedef enum {
        WRITE_ON,
        WRITE_OFF,
        START_CONV,
        WAIT_BUSY,
        READ_ON,
        DATA_OUT
    } state_t;

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // SECTION: Signal Declarations

    // Driver FSM state register
    state_t state_ff;

    // Controls all 4 conv_start bits with one bit
    logic conv_start;

    // Controls number of transactions of reads/writes throughout ADC Driver FSM
    logic [3:0] num_transactions;

    // Raw data to drive to ADC databits
    logic [15:0] data_adc_out;

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // SECTION: Output Assignments

//tri-state logicL drives data_adc_out onto bus when NOT reading AND writing is active AND chip is selected
//in WRITE_ON: chip is selected and write_n is low so data is driven
//in READ_ON: read_n is low, release data, let ADC drive
//otherwise, no one drives and its just z
    assign data_adc = (!read_n || (write_n && chipselect_n)) ? 16'bz : data_adc_out;

    assign software_mode = 1'b1;
    assign serial_mode   = 1'b0;
    assign standby_n     = 1'b1;
    assign chipselect_n  = 1'b0;

    // All channel conversions are started simultaneously
    assign conv_start_a = conv_start;
    assign conv_start_b = conv_start;
    assign conv_start_c = conv_start;
    assign conv_start_d = conv_start;


    ////////////////////////////////////////////////////////////////////////////////////////////////
    // SECTION: Logic Implementation

    // ADS8528 Driver State Machine Transition Control
    always_ff @(posedge clk) begin
        if (!sresetn) begin
            state_ff         <= WRITE_ON;
            num_transactions <= '0;
            data_out         <= 'X;
            data_valid       <= 1'b0;

        end else begin
            case (state_ff)
                // Drive write_n with databits with config register info
                WRITE_ON: begin
                    state_ff         <= WRITE_OFF;
                    num_transactions <= num_transactions + 1'b1; //counter for number of transactions occuring
                end

                // TODO ? write_n not deasserted: Deassert write_n to allow the config register to then latch the next set of bits
                //falling edge of write_n tells chip to latch data to data_adc in its config register
               //2 writes to split config across 2 16bit writes
                WRITE_OFF: begin
                    if (num_transactions == 4'd2) begin //if two config writes done, move on, if not go back to WRITE_ON
                        state_ff <= START_CONV;
                    end else begin
                        state_ff <= WRITE_ON;
                    end
                end

                // Initiate a conversion and wait for busy to be asserted
                //triggers all 4 channels to begin conversion
                START_CONV: begin
                    num_transactions <= '0; //resets num_transacs to 0 for future reads
                    if (busy) begin //moves on once chip is busy converting, from top module
                        state_ff     <= WAIT_BUSY;
                    end
                end

                // Wait for busy to be deasserted, waits for analog->digital conversion to happen
                WAIT_BUSY: begin
                    if (~busy) begin //moves o when conversions are all complete
                        state_ff <= READ_ON;
                    end
                end

                // Read an ADC data sample, read converted data from adc
                READ_ON: begin
                    state_ff         <= DATA_OUT;
                    num_transactions <= num_transactions + 1'b1; //says weve read 1 of 4 channels, dont we need 4 read cycles to read all of them one at a time?
                    data_out         <= data_adc; //one of our outputs
                    data_valid       <= 1'b1;
                end

                // Send the ADC sample to output register (no backpressure, must be read immediately)
                //deliver readdata to output and prepare for next read, deassert read_n?
                //dataout holds for two cycles, read_on latches and strobes data_vaild, data_out holds this for one more cycle so logic can get it
                DATA_OUT: begin
                    data_out   <= 'X;
                    data_valid <= 1'b0;
                    if (num_transactions == 4'd8) begin //twice as each readon and dataout pair increments twice, if 8, start new conv cycle
                        state_ff  <= START_CONV;
                    end else begin //if not 8, go back to read_on
                        state_ff  <= READ_ON;
                    end
                end

                default: begin
                    state_ff         <= WRITE_ON;
                    num_transactions <= '0;
                end
            endcase
        end
    end

    // ADS8528 Driver State Machine Output Control
    always_comb begin
        // Default deasserted combinational values
        conv_start   = 1'b0; 
        write_n      = 1'b1;
        read_n       = 1'b1;
        data_adc_out = 'X;

        case (state_ff)
            // Assert write with LSB/MSB of config register
            WRITE_ON: begin
                write_n = 1'b0; //asserts to tell chip im writing
                if (num_transactions == '0) begin
                    data_adc_out = 16'h8054; // First write

                end else begin 
                    data_adc_out = 16'h43FF; // Second write
                end
            end

            // Assert conversion start bits
            START_CONV: begin
                conv_start = 1'b1; //all conv_start a/b/c/d go high
            end

            // Assert read
            READ_ON: begin
                read_n = 1'b0; //tells chip im reading
            end
        endcase

        // Hold write deasserted during reset
        if (!sresetn) begin
            write_n = 1'b1;
        end
    end
endmodule

`default_nettype wire

/*Timing Sequence Example
Clock 0: Reset completes, state_ff = WRITE_ON, num_transactions = 0
Clock 1: STATE: WRITE_ON → write_n=0, data_adc_out=0x8054, increment counter to 1, next state WRITE_OFF
Clock 2: STATE: WRITE_OFF → write_n=1 (falling edge latches config), check counter (1 < 2), next state WRITE_ON
Clock 3: STATE: WRITE_ON → write_n=0, data_adc_out=0x43FF, increment counter to 2, next state WRITE_OFF
Clock 4: STATE: WRITE_OFF → write_n=1, check counter (2 == 2), next state START_CONV
Clock 5: STATE: START_CONV → conv_start=1, num_transactions=0, wait for busy to rise
Clock 6: busy rises (ADC is converting) → next state WAIT_BUSY
Clock N: busy falls (conversion done) → next state READ_ON
Clock N+1: STATE: READ_ON → read_n=0, capture data_adc→data_out, data_valid=1, increment to num_trans=1, next DATA_OUT
Clock N+2: STATE: DATA_OUT → read_n=1, check counter (1 < 8), next state READ_ON
Clock N+3: STATE: READ_ON → read_n=0 again for second channel, increment to 2, next DATA_OUT
... repeat until num_transactions == 8 (all 4 channels read, each counted twice)
Clock N+10: STATE: DATA_OUT → num_transactions==8, next state START_CONV (new cycle begins)
Data Flow Summary

Initialize: Write 2 config registers to set up the ADC
Trigger: Start conversion on all 4 channels simultaneously
Wait: Let ADC complete (busy signal guides us)
Read: Read each of the 4 converted values, one per cycle
Output: Strobe each value with data_valid
Repeat: Loop back to step 2

Key Design Notes
No Backpressure: If downstream logic can't accept data when data_valid=1, the data is lost. There's no way to pause the state machine.
Fixed Configuration: The config values (0x8054, 0x43FF) are hardcoded—if you need different ADC settings, you must modify the Verilog.
Parallel Channels: All 4 channels convert at once, but you get the data sequentially (one per clock cycle).
Tri-State Bus: The module carefully manages the 16-bit data bus so only one driver is active at a time (you during writes, ADC during reads). */
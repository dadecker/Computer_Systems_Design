`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    15:47:16 01/30/2015 
// Design Name: 
// Module Name:    loopback 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: 
//
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////

module audio_message_recorder( //( switches, leds, rs232_tx, rs232_rx, reset, clk );

	// Top-level Inputs and Outputs
	// These connect directly to FPGA pins via the pin map
	//
	// Control - clk, rst, etc
	input			reset,			// Remember: ACTIVE LOW!!!
	input			reset_pll,
	input			clk,			// 100 MHz
//	input			ETH_REFCLK,	//	100 MHz
//	input 		pb_clk,		// 100 MHz
	// GPIO
	input	[7:0]	switches,
	output	[7:0]	leds,
	// RS232 Lines
	input			rs232_rx,
	output			rs232_tx,
	input [3:0] button,
	
	//sockit_top I/O
//	 input  clk,

    inout  AUD_ADCLRCK,
    input  AUD_ADCDAT,

    inout  AUD_DACLRCK,
    output AUD_DACDAT,

    output AUD_XCK,
    inout  AUD_BCLK,

    output AUD_I2C_SCLK,
    inout  AUD_I2C_SDAT,

    output AUD_MUTE,
	 output PLL_LOCKED,
//	 output PLL_LOCKED2,
	 
 //   input  [3:0] KEY,
//    input  [3:0] SW,
    output [2:0] LED,
	 
//	input [25:0] address, 
	 
//	 output [25:0] max_ram_address,
	 
//	input [DATA_BIT_WIDTH-1:0] data_in,
//	input write_enable,
	
//	output rdy, rd_data_pres,
	
//	input read_request,
//	input read_ack,
//	output [DATA_BIT_WIDTH-1:0] data_out,
	 
//	input reset, clk, sys_clk,
//	input sysclk,
//	output clkout,
	
	output 	hw_ram_rasn,
	output 	hw_ram_casn,
	output 	hw_ram_wen,
	output[2:0] hw_ram_ba,
	inout 	hw_ram_udqs_p,
	inout 	hw_ram_udqs_n,
	inout 	hw_ram_ldqs_p,
	inout 	hw_ram_ldqs_n,
	output 	hw_ram_udm,
	output 	hw_ram_ldm,
	output 	hw_ram_ck,
	output 	hw_ram_ckn,
	output 	hw_ram_cke,
	output 	hw_ram_odt,
	output[12:0] hw_ram_ad,
	inout [15:0] hw_ram_dq,
	inout 	hw_rzq_pin,
	inout 	hw_zio_pin,
	output ledRAM,
	output freqcount
//	input hw_sys_rst_i,
//	input hw_ram_clk
	
	);
		

	// Wires and Register Declarations
	//
	// PicoBlaze Data Lines
	wire	[7:0]	pb_port_id;
	wire	[7:0]	pb_out_port;
	reg		[7:0]	pb_in_port;
	wire			pb_read_strobe;
	wire			pb_write_strobe;
	// PicoBlaze CPU Control Wires
	wire			pb_reset;
	wire			pb_interrupt;
	wire			pb_int_ack;
	
	// UART wires
	wire			write_to_uart;
	wire			uart_buffer_full;
	wire			uart_data_present;
	reg				read_from_uart;
	wire			uart_reset;
	// UART Data Lines
	// TX does not need a wire, as it is fed directly by pb_out_port
	wire	[7:0]	uart_rx_data;
	
	// LED wires
	wire write_to_leds;
	wire led_reset;

	wire main_clk;
	wire ram_clk;
wire audio_clk;
//wire clk_100MHz;
wire [1:0] sample_end;
wire [1:0] sample_req;
//wire [15:0] audio_output;
wire [15:0] audio_input;

//Extra registers
reg [1:0] recordmode = 2'b00;
reg audio_mute = 1'b0;

//reg [1:0] sample_end;
//reg [1:0] sample_req;
reg [15:0] audio_output;
//reg [15:0] audio_input;

reg [25:0] DDR2_address = 26'h0000000; 
reg [25:0] DDR2_max_address;
	 
reg [15:0] DDR2_data_in;
reg DDR2_write_enable = 1'b1;
		wire 			systemCLK;
wire DDR2_rdy;
wire DDR2_rd_data_pres;
	
reg DDR2_read_request;
reg DDR2_read_ack;
wire [15:0] DDR2_data_out;

reg [7:0] DDR2_address_low;
reg [7:0] DDR2_address_mid;
reg [7:0] DDR2_address_high;
reg [2:0] DDR2_address_banksel;

//reg [3:0] LED;
//reg [7:0] leds;

//assign audio_output = audio_output_reg;
/*
always @(posedge main_clk) begin
		if(reset) begin
			DDR2_address <= 0;
			DDR2_write_enable <= 0;
			recordmode <= 2'b00;
			audio_mute <= 1'b0;
		end
		else if (recordmode == 2'b01 & DDR2_rdy & sample_end) begin
			DDR2_data_in <= audio_input;
			DDR2_write_enable <= 1'b1;
			audio_mute <= 1'b1;
		end
		else if (recordmode == 2'b10 & DDR2_rd_data_pres & sample_req) begin
			audio_output_reg <= DDR2_data_out;
			DDR2_write_enable <= 1'b0;
			audio_mute <= 1'b1;
		end
		else if ((recordmode != 2'b00) & DDR2_address < DDR2_max_address) begin
			DDR2_address <= DDR2_address + 1'b1;
		end
		else begin
			DDR2_address <= DDR2_address;
			audio_mute <= 1'b1;
		end
end */
/*
always @(*) begin
	if(reset) begin
			DDR2_address <= 0;
			DDR2_write_enable <= 0;
			recordmode <= 2'b00;
			audio_mute <= 1'b0;
		end
		else 
		if (DDR2_address == 26'h0FFFFFF) begin
			DDR2_address <= 0;
			DDR2_write_enable <= 1'b1;
			audio_mute <= 1'b1;
		end
		else begin
			DDR2_write_enable <= 1'b1;			
			DDR2_address <= DDR2_address + 1'b1;
		end
end	*/
		//	input reset, clk, sys_clk,
//	input sysclk,
//	output clkout,
//assign reset = ~reset;
// Clock PLL that synthesizes two frequencies: 50 MHz and 11.2896 MHz
// Input 100 MHz 
//Debouncer module instances for money and soda_db selection inputs
//reset_debounce resetdb(.clk(main_clk),		// INPUT: Master clock (100 MHz)
//			.reset(reset),						// INPUT: quarter_db signal
//			.reset1(reset_1));					// OUTPUT: Debounced quarter_db signal
//			.reset2(reset_2));					// OUTPUT: Debounced quarter_db signal
/*
clock_generator pll (
	 .CLK_IN1 (ETH_REFCLK),
	 .CLK_OUT1 (main_clk),   // 50 MHz
    .CLK_OUT2 (audio_clk),  // 11.2896 MHz
	 .RESET (reset_pll),
	 .LOCKED (PLL_LOCKED)
);
*/

clk_wiz_v3_6 pll (
	 .CLK_IN1 (clk_100MHz),
	 .CLK_OUT1 (main_clk),   // 50 MHz
    .CLK_OUT2 (audio_clk),  // 11.2896 MHz
	 .RESET (reset_pll),
	 .LOCKED (PLL_LOCKED)
);


//wire CLKFB;
clkgen clock_generator (
	 .CLK_IN1 (systemCLK),
//	 .CLKFB_IN(CLKFB),
	 .CLK_OUT1 (clk_100MHz)    // 100 MHz	 unbuf
//	 .CLK_OUT2 (main_clk),   // 50 MHz
//    .CLK_OUT3 (audio_clk),  // 11.2896 MHz
//	 .CLKFB_OUT(CLKFB),
//	 .RESET (reset2),
//	 .LOCKED (PLL_LOCKED)
);

//Stuff from the DDR2 test project

//	input 	CLK, reset;
//	output 	status;
	
//	input  [7:0]	switches; 		// address
//	input	 [7:0]	dip_switches; 	// data to be written into RAM
//	output [7:0]	leds;			// data read out of RAM
	
	reg 	[25:0] address = 0;
	reg 	[15:0]	RAMin;
	wire 	[15:0]	RAMout;
	reg	[7:0] dataOut; 
	reg 			reqRead;
	reg 			enableWrite;
	reg 			ackRead = 0;

	reg    [7:0] audio_ramhi;
	reg    [7:0] audio_ramlo;
	reg freqcount;
	wire rdy, 	dataPresent;
	wire [25:0]	max_ram_address;
	
	reg [3:0]	state=4'b0000;
	
	parameter stInit = 4'b0000;
	parameter stReadFromPorts = 3'b001;
	parameter stMemWrite = 3'b0010;
	parameter stMemReadReq  = 3'b0011;
	parameter stMemReadData = 3'b0100;
	
//reg [15:0] last_sample;
reg ram_write_request;
reg ram_read_request;
reg ram_write_ack;
reg ram_read_ack;
reg [15:0] audio_input_sample;
reg [15:0] audio_output_sample;
reg [15:0] debug_sample;
	
/*	// FSM to read and write to DDR2 RAM
	always @(posedge systemCLK)
	begin
		if (reset_1) begin 
			address <= 0;
			state <= stInit;
		end
		else begin
			if(rdy) begin // Only if RAM is rdy for read/write
				
				case (state)

				  // Initialization state
				  stInit: begin 
				   ackRead <= 1'b0;
					ram_read_ack <= 1'b0;
					ram_write_ack <= 1'b0;
					state <= stReadFromPorts;
				  end
				  
				  // Read from the ports
				  // switches are read and used as address
				  // dip_switches are read and used as data to be written into RAM
				  stReadFromPorts: begin
//				   address <= {26'b00_0000_0000_0000_0000_0000_0000};
					if (ram_write_request) begin
						RAMin <= audio_input_sample;
//						debug_sample <= audio_input_sample;						
						ram_write_ack <= 1'b1;
						state <= stMemWrite;
					end
					else begin
						state <= stReadFromPorts;
					end
				  end
				  
				  // Write cycle, raise write enable
				  stMemWrite: begin
					ram_write_ack <= 1'b0;
				   enableWrite <= 1'b1;
					state <= stMemReadReq;
				  end
				  
				  // Read cycle 1, pull down write enable, raise read request
				  stMemReadReq: begin
					enableWrite <= 1'b0;
					if(ram_read_request) begin
					reqRead <= 1'b1;
					state <= stMemReadData;
					end
					else begin
						state <= stMemReadReq;
					end
				  end
				  
				  // Read cycle 2
				  // Waite until data is valid i.e., when dataPresent is 1
				  stMemReadData: begin
					reqRead <= 1'b0;
					if(dataPresent & !ram_read_ack) begin // data is present, read to dataOut register
//						audio_ramlo <= RAMout[7:0];
//						audio_ramhi <= RAMout[15:8];
//						leds <= RAMout[7:0];
						audio_output_sample <= RAMout;
//						audio_output_sample <= debug_sample;
						ram_read_ack <= 1'b1;
						ackRead <= 1'b1;	 // acknowledge the read
					end
					else begin				  // stay in the same state until data is valid
						state <= stMemReadData;
					end
					if(!ram_read_request) begin	
						if (address == max_ram_address) begin
							address <= 0;
						end
						else begin
							address <= address + 1'b1;
						end
						state <= stInit;
//					end
//					end

					end

				  end
			 
			 endcase 
			end // rdy
		end
end
*/

	// FSM to read and write to DDR2 RAM
	always @(posedge systemCLK)
	begin
		if (reset) begin 
			address <= 0;
			state <= stInit;
		end
		else
			if(rdy) begin // Only if RAM is rdy for read/write
				
				case (state)

				  // Initialization state
				  stInit: begin 
				   ackRead <= 1'b0;
					enableWrite <= 1'b0;
					if(switches[2]) begin  //reset address to 0
						address <= 0;
					end
					else begin
						address <= address;
					end
					
					if(!switches[0] & switches[1] & !switches[2]) begin //record
						if (address == max_ram_address) begin
							freqcount <= 1'b1;
						end
						else begin
							address <= address + 1'b1;
							freqcount <= 1'b0;
						end
						state <= stReadFromPorts;
					end
					else if (switches[0] & switches[1] & !switches[2]) begin //playback
						if (address == max_ram_address) begin
							freqcount <= 1'b1;
						end
						else begin
							address <= address + 1'b1;
							freqcount <= 1'b0;
						end
						state <= stMemReadReq;
					end
					else begin
						state <= stInit;
					end
					end
				  
				  // Read from the ports
				  // switches are read and used as address
				  // dip_switches are read and used as data to be written into RAM
				  stReadFromPorts: begin
					if(sample_end) begin
//				   address <= {18'b00_0000_0000_0000_0000, switches};
					RAMin <= audio_input_sample;
					state <= stMemWrite;
					end
					else begin
						state <= stReadFromPorts;
					end
				  end
				  
				  // Write cycle, raise write enable
				  stMemWrite: begin
				   enableWrite <= 1'b1;
//					state <= stMemReadReq;
					state <= stInit;
				  end
				  
				  // Read cycle 1, pull down write enable, raise read request
				  stMemReadReq: begin
				  if(sample_end) begin
//				  if(sample_req) begin
					enableWrite <= 1'b0;
					reqRead <= 1'b1;
					state <= stMemReadData;
				  end
				  else begin
						state <= stMemReadReq;
					end
				end
				  
				  // Read cycle 2
				  // Waite until data is valid i.e., when dataPresent is 1
				  stMemReadData: begin
					reqRead <= 1'b0;
					if(dataPresent) begin // data is present, read to dataOut register
						audio_output_sample = RAMout;
						ackRead <= 1'b1;	 // acknowledge the read
//						if (address == max_ram_address) begin
//							address <= 0;
//						end
//						else begin
//							address <= address + 1'b1;
//						end
						state <= stInit;
					end
					else begin				  // stay in the same state until data is valid
						state <= stMemReadData;
					end
				  end
			 
			 endcase 
			end // rdy
		end

always @(posedge audio_clk) begin
    if (sample_end) begin
        audio_input_sample <= audio_input;
    end
 //   if (sample_req) begin
 //       if (control[FEEDBACK])
 //           audio_output <= audio_output_sample;
 //       else if (control[SINE]) begin
 //           dat <= romdata[index];
 //           if (index == 7'd99)
 //               index <= 7'd00;
 //           else
 //              index <= index + 1'b1;
 //       end else
 //           dat <= 16'd0;
 //   end
end

/*
always @(posedge audio_clk) begin
    if (sample_end) begin
	 	ram_write_request <= 1'b1;
//		ram_read_request <= 1'b1; //get started early?
      audio_input_sample <= audio_input;
//		debug_sample <= audio_input;
//		LED <= 4'b0000;
    end
	else begin
		ram_write_request <= ram_write_request;
//		LED <= 4'b0000;
	end
	if (ram_write_ack) begin
			 ram_write_request <= 1'b0;
	end
	else begin
          audio_input_sample <= audio_input_sample;
    end
   if (sample_req) begin
		ram_read_request <= 1'b1;
	end
	else begin
		ram_read_request <= ram_read_request;
//			 audio_output <= audio_output_sample; //seemed to break the silence with garbled noise once, sync issue?
	end
	 if (ram_read_ack) begin
			 audio_output <= audio_output_sample;
//          audio_output <= debug_sample;
			 ram_read_request <= 1'b0;
//		LED <= 4'b0000;
		end
	else begin
          audio_output_sample <= audio_output_sample;
//			 LED <= 4'b1111;
			 
    end
end
*/
//	ClkDivider clk_div1(
//	.clk(systemCLK),
//		.clk_div(freqcount));
	// ram interface instantiation
	ram_interface_wrapper RAMRapper (
					.address(address),				// input 
					.data_in(RAMin), 					// input
					.write_enable(enableWrite), 	//	input
					.read_request(reqRead), 		//	input
					.read_ack(ackRead), 
					.data_out(RAMout), 				// output from ram to wire
					.reset(reset_1), 
					.hw_ram_clk(clk), //clk_100MHz
					.hw_ram_rasn(hw_ram_rasn), 
					.hw_ram_casn(hw_ram_casn),
					.hw_ram_wen(hw_ram_wen), 
					.hw_ram_ba(hw_ram_ba), 
					.hw_ram_udqs_p(hw_ram_udqs_p), 
					.hw_ram_udqs_n(hw_ram_udqs_n), 
					.hw_ram_ldqs_p(hw_ram_ldqs_p), 
					.hw_ram_ldqs_n(hw_ram_ldqs_n), 
					.hw_ram_udm(hw_ram_udm), 
					.hw_ram_ldm(hw_ram_ldm), 
					.hw_ram_ck(hw_ram_ck), 
					.hw_ram_ckn(hw_ram_ckn), 
					.hw_ram_cke(hw_ram_cke), 
					.hw_ram_odt(hw_ram_odt),
					.hw_ram_ad(hw_ram_ad), 
					.hw_ram_dq(hw_ram_dq), 
					.hw_rzq_pin(hw_rzq_pin), 
					.hw_zio_pin(hw_zio_pin), 
					.clkout(systemCLK), 
			//		.clkout_50MHz(clk_50MHz), 
					.sys_clk(systemCLK), 
					.rdy(rdy), 
					.rd_data_pres(dataPresent),
					.max_ram_address(max_ram_address),
					.ledRAM(ledRAM)
					);		
					
//assign status = rdy; // ready signal from ram
assign leds = address[23:16]; // output data read to leds
assign LED = state;
//assign audio_output = RAMout;
//assign status = rdy;
/*
ram_interface_wrapper DDR2_RAM (
//	.DATA_BYTE_WIDTH(1'b1),
	.address(DDR2_address), 
	.data_in(audio_input), 
	.write_enable(DDR2_write_enable), 
	.read_request(sample_req),
	.read_ack(DDR2_read_ack), 
//	.data_out(DDR2_data_out), 
	.data_out(audio_output), 
	.hw_sys_rst_i(hw_sys_rst_i), 
	.hw_sys_clk(hw_sys_clk), 
	.hw_ram_rasn(hw_ram_rasn), 
	.hw_ram_casn(hw_ram_casn),
	.hw_ram_wen(hw_ram_wen), 
	.hw_ram_ba(hw_ram_ba), 
	.hw_ram_udqs_p(hw_ram_udqs_p), 
	.hw_ram_udqs_n(hw_ram_udqs_n), 
	.hw_ram_ldqs_p(hw_ram_ldqs_p), 
	.hw_ram_ldqs_n(hw_ram_ldqs_n), 
	.hw_ram_udm(hw_ram_udm), 
	.hw_ram_ldm(hw_ram_ldm), 
	.hw_ram_ck(hw_ram_ck), 
	.hw_ram_ckn(hw_ram_ckn), 
	.hw_ram_cke(hw_ram_cke), 
	.hw_ram_odt(hw_ram_odt),
	.hw_ram_ad(hw_ram_ad), 
	.hw_ram_dq(hw_ram_dq), 
	.hw_rzq_pin(hw_rzq_pin), 
	.hw_zio_pin(hw_zio_pin), 
	.clkout(), 
	.sys_clk(main_clk), 
	.rdy(DDR2_rdy), 
	.rd_data_pres(DDR2_rd_data_pres),
	.max_ram_address(DDR2_max_ram_address)
);

*/
// I2C Protocol - FPGA is Master, Codec is Slave
i2c_av_config av_config (
    .clk (main_clk),
    .reset (reset_1),
    .i2c_sclk (AUD_I2C_SCLK),
    .i2c_sdat (AUD_I2C_SDAT)//,
 //   .status (LED)
);

assign AUD_XCK = audio_clk;
//assign system_state = switches[0];

assign AUD_MUTE = switches[0];  // active low, so set to 1 and disable mute
//assign AUD_MUTE = 1'b1;  // active low, so set to 1 and disable mute

// Serial to parallel conversion 
audio_codec ac (
    .clk (audio_clk),
    .reset (reset_1),
    .sample_end (sample_end),
    .sample_req (sample_req),
 //   .audio_output (audio_output),
    .audio_output (RAMout),
    .audio_input (audio_input),
    .channel_sel (2'b10),

    .AUD_ADCLRCK (AUD_ADCLRCK),
    .AUD_ADCDAT (AUD_ADCDAT),
    .AUD_DACLRCK (AUD_DACLRCK),
    .AUD_DACDAT (AUD_DACDAT),
    .AUD_BCLK (AUD_BCLK)
);

assign reset_1 = reset_pll;
// Audio source
// (1) Sine wave mode (tone)  SW7 UP
// (2) Playback mode (feedback from LINE IN to LINE OUT) SW6 UP
/*
audio_effects ae (
	  .clk (audio_clk),
    .sample_end (sample_end[1]),
    .sample_req (sample_req[1]),
    .audio_output (audio_output),
    .audio_input  (audio_input),
    .control (SW)
);
*/
	// LED Driver and control logic
	//
	// LED driver expects ACTIVE-HIGH reset
	/*
//	reg [7:0] dummy_leds;
	assign led_reset = reset_1;
	// LED driver instantiation
	led_driver_wrapper led_driver (
		.led_value(pb_out_port),
		.leds(dummy_leds), //leds
		.write_to_leds(write_to_leds),
		.reset(led_reset),
		.clk(main_clk) //main_clk
	);
*/
	
	// UART and control logic
	//
	// UART expects ACTIVE-HIGH reset	
	assign uart_reset =  reset_1;
	// UART instantiation
	//
	// Within the UART Module (rs232_uart.v), make sure you fill in the
	// appropriate sections.
	rs232_uart UART (
		.tx_data_in(pb_out_port), // The UART only accepts data from PB, so we just tie the PB output to the UART input.
		.write_tx_data(write_to_uart), // Goes high when PB sends write strobe and PORT_ID is the UART write port number
		.tx_buffer_full(uart_buffer_full),
		.rx_data_out(uart_rx_data),
		.read_rx_data_ack(read_from_uart),
		.rx_data_present(uart_data_present),
		.rs232_tx(rs232_tx),
		.rs232_rx(rs232_rx),
		.reset(uart_reset),
		.clk(clk_100MHz) //main_clk
	);	
	
	// PicoBlaze and control logic
	//
	// PB expects ACTIVE-HIGH reset (changed from ~reset)
	assign pb_reset = reset_1;
	// Disable interrupt by assigning 0 to interrupt
	assign pb_interrupt = 1'b0;
	// PB CPU instantiation
	//
	// Within the PicoBlaze Module (picoblaze.v), make sure you fill in the
	// appropriate sections.
	picoblaze CPU (
		.port_id(pb_port_id),
		.read_strobe(pb_read_strobe),
		.in_port(pb_in_port),
		.write_strobe(pb_write_strobe),
		.out_port(pb_out_port),
		.interrupt(pb_interrupt),
		.interrupt_ack(),
		.reset(pb_reset),
		.clk(clk_100MHz) //main clk
	);	
	// PB I/O selection/routing
	//
	// Handle PicoBlaze Output Port Logic
	// Output Ports:
	// * leds_out : port 01
	// * uart_data_tx : port 03
	//
	// These signals are effectively "write enable" lines for the UART and LED
	// Driver modules. They must be asserted when PB is outputting to the
	// corresponding port
	assign write_to_leds = pb_write_strobe & (pb_port_id == 8'h01);
	assign write_to_uart = pb_write_strobe & (pb_port_id == 8'h03);
	//
	// Handle PicoBlaze Input Port Logic
	// Input Ports:
	// * switches_in : port 00
	// * uart_data_rx : port 02
	// * uart_data_present : port 04 (1-bit, assigned to LSB)
	// * uart_buffer_full: port 05 (1-bit, assigned to LSB)
	//
	// This process block gets the value of the requested input port device
	// and passes it to PBs in_port. When PB is not requestng data from
	// a valid input port, set the input to static 0.
	always @(posedge clk_100MHz or posedge pb_reset)
	begin
		if(pb_reset) begin
			pb_in_port <= 0;
			read_from_uart <= 0;
		end else begin
			// Set pb input port to appropriate value
			case(pb_port_id)
				8'h00: pb_in_port <= switches;
				8'h02: pb_in_port <= uart_rx_data;
				8'h04: pb_in_port <= {7'b0000000,uart_data_present};
				8'h05: pb_in_port <= {7'b0000000,uart_buffer_full};
				8'h06: pb_in_port <= audio_ramlo;
				8'h06: pb_in_port <= audio_ramhi;
				default: pb_in_port <= 0;
			endcase
			// Set up acknowledge/enable signals.
			//
			// Some modules, such as the UART, need confirmation that the data
			// has been read, since it needs to push it off the queue and make
			// the next byte available. This logic will set the 'read_from'
			// signal high for corresponding ports, as needed. Most input
			// ports will not need this.
			read_from_uart <= pb_read_strobe & (pb_port_id == 8'h02);
		end
	end

endmodule

/*
module reset_debounce(clk, reset, reset1);//, reset2);
     
localparam constantNumber = 20000000; 	//20Hz, FOR THE BOARD
//localparam constantNumber = 1; 		//FOR THE SIMULATION
input clk, reset;
output reset1;
//output reset2;
reg [31:0] count = 0;
reg reset1 = 0;
//reg reset2 = 0;
reg reset_triggered = 1'b0;
 
always @ (posedge(clk)) begin
   if (~reset_triggered & reset) begin
        reset_triggered <= 1'b1;
	end
	else if (count == constantNumber - 1) begin
        count <= 32'b0;
		  reset_triggered <= 1'b0;
	end
   else if (reset_triggered & ~reset) begin
        count <= count + 1;
	end
	else begin
		count <= count;
	end	
end

always @ (posedge(clk)) begin
	if (count > (constantNumber - 10000000)) begin
        reset1 <= 1'b1;
	end
//	else if ((count > (constantNumber - 80000000)) & (count < (constantNumber - 40000000))) begin
//        reset2 <= 1'b1;
//	end
   else begin
        reset1 <= 1'b0;
//        reset2 <= 1'b0;
	end
end
endmodule */
/*
module ClkDivider (clk, clk_div);
     
localparam constantNumber = 37500000; 	//10Hz, FOR THE BOARD
//localparam constantNumber = 2; 		//FOR THE SIMULATION
input clk;
output clk_div;
reg [31:0] count = 0;
reg clk_div = 0;
 
always @ (posedge(clk)) begin
	 if (count == constantNumber - 1)
        count <= 32'b0;
    else
        count <= count + 1;
end

always @ (posedge(clk)) begin 
	 if (count == constantNumber - 1)
        clk_div <= ~clk_div;
    else
        clk_div <= clk_div;
end
endmodule
*/
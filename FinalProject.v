
module FinalProject (
	// Inputs
	CLOCK_50,
	KEY,

	AUD_ADCDAT,

	// Bidirectionals
	AUD_BCLK,
	AUD_ADCLRCK,
	AUD_DACLRCK,

	FPGA_I2C_SDAT,

	// Outputs
	AUD_XCK,
	AUD_DACDAT,

	FPGA_I2C_SCLK,
	SW
);

/*****************************************************************************
 *                           Parameter Declarations                          *
 *****************************************************************************/
localparam  C        = 7'b1000000,
				Cs       = 7'b1100000,
				D			= 7'b0100000,
				Ds       = 7'b0110000,
				E        = 7'b0010000,
				F        = 7'b0001000,
				Fs       = 7'b0001100,
				G        = 7'b0000100,
				Gs       = 7'b0000110,
				A        = 7'b0000010,
				As       = 7'b0000011,
				B        = 7'b0000001;

/*****************************************************************************
 *                             Port Declarations                             *
 *****************************************************************************/
// Inputs
input				CLOCK_50;
input		[3:0]	KEY;
input		[9:0]	SW;

input				AUD_ADCDAT;

// Bidirectionals
inout				AUD_BCLK;
inout				AUD_ADCLRCK;
inout				AUD_DACLRCK;

inout				FPGA_I2C_SDAT;

// Outputs
output				AUD_XCK;
output				AUD_DACDAT;

output				FPGA_I2C_SCLK;

/*****************************************************************************
 *                 Internal Wires and Registers Declarations                 *
 *****************************************************************************/
// Internal Wires
wire				audio_in_available;
wire		[31:0]	left_channel_audio_in;
wire		[31:0]	right_channel_audio_in;
wire				read_audio_in;

wire				audio_out_allowed;
wire		[31:0]	left_channel_audio_out;
wire		[31:0]	right_channel_audio_out;
wire				write_audio_out;

// Internal Registers

reg [18:0] delay_cnt;
wire [18:0] delay;

reg [18:0] delay_cnt2;
wire [18:0] delay2;

reg snd;
reg snd2;

reg [15:0] freq;
reg [18:0] octaveFreq;

reg [15:0] freq2;
// State Machine Registers

/*****************************************************************************
 *                         Finite State Machine(s)                           *
 *****************************************************************************/


/*****************************************************************************
 *                             Sequential Logic                              *
 *****************************************************************************/

always @(posedge CLOCK_50)
	if(delay_cnt == delay) begin
		delay_cnt <= 0;
		snd <= !snd;
	end else delay_cnt <= delay_cnt + 1;

always @(posedge CLOCK_50)
	if(delay_cnt2 == delay2) begin
		delay_cnt2 <= 0;
		snd2 <= !snd2;
	end else delay_cnt2 <= delay_cnt2 + 1;
/*****************************************************************************
 *                            Combinational Logic                            *
 *****************************************************************************/
always @(*)
	case(SW[6:0]) 
		C: freq = 16'd23889; //1046.5Hz
		Cs: freq = 16'd22548; //1108.73Hz
		D: freq = 16'd21283; //1174.66Hz
		Ds: freq = 16'd20088; //1244.51Hz 
		E: freq = 16'd18961; //1318.51Hz
		F: freq = 16'd17897; //1396.91Hz
		Fs: freq = 16'd16892; //1479.98Hz
		G: freq = 16'd15944; //1567.98Hz
		Gs: freq = 16'd15049; //1661.22Hz
		A: freq = 16'd14205; //1760hz
		As: freq = 16'd13407; //1864.66Hz
		B: freq = 16'd12655; //1975.53Hz
		default: freq = 16'd65536; //default frequency, largest 16 bit decimal value
	endcase

always @(*)
	case(SW[9]) 
		1: freq2 = 16'd7103; 
		default: freq2 = 16'd23889; 
	endcase

	
//	octaves
always@(*)
	case(SW[8:7])
		//highest octave
		2'd0: octaveFreq = freq; 
		//high octave, frequency / 2
		2'd1: octaveFreq = freq + freq; 
		//middle octave, frequency / 4 
		2'd2: octaveFreq = freq + freq + freq + freq; 
		//low octave, frequency / 8
		2'd3: octaveFreq = freq + freq + freq + freq + freq + freq + freq + freq; 
	endcase
	
assign delay = octaveFreq;
//assign delay = {SW[3:0], 15'd3000};

assign delay2 = freq2;

wire [31:0] sound = (SW == 0) ? 0 : snd ? 32'd10000000 : -32'd10000000;
wire [31:0] sound2 = (SW[9] == 0) ? 0 : snd2 ? 32'd10000000 : -32'd10000000;

assign read_audio_in			= audio_in_available & audio_out_allowed;

assign left_channel_audio_out	= left_channel_audio_in+sound;
assign right_channel_audio_out	= right_channel_audio_in+sound2;
assign write_audio_out			= audio_in_available & audio_out_allowed;

/*****************************************************************************
 *                              Internal Modules                             *
 *****************************************************************************/

Audio_Controller Audio_Controller (
	// Inputs
	.CLOCK_50						(CLOCK_50),
	.reset						(~KEY[0]),

	.clear_audio_in_memory		(),
	.read_audio_in				(read_audio_in),
	
	.clear_audio_out_memory		(),
	.left_channel_audio_out		(left_channel_audio_out),
	.right_channel_audio_out	(right_channel_audio_out),
	.write_audio_out			(write_audio_out),

	.AUD_ADCDAT					(AUD_ADCDAT),

	// Bidirectionals
	.AUD_BCLK					(AUD_BCLK),
	.AUD_ADCLRCK				(AUD_ADCLRCK),
	.AUD_DACLRCK				(AUD_DACLRCK),


	// Outputs
	.audio_in_available			(audio_in_available),
	.left_channel_audio_in		(left_channel_audio_in),
	.right_channel_audio_in		(right_channel_audio_in),

	.audio_out_allowed			(audio_out_allowed),

	.AUD_XCK					(AUD_XCK),
	.AUD_DACDAT					(AUD_DACDAT)

);

avconf #(.USE_MIC_INPUT(1)) avc (
	.FPGA_I2C_SCLK					(FPGA_I2C_SCLK),
	.FPGA_I2C_SDAT					(FPGA_I2C_SDAT),
	.CLOCK_50					(CLOCK_50),
	.reset						(~KEY[0])
);

endmodule


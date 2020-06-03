`timescale 1ns/10ps
module cpu5_testbench();

reg  [31:0] instrbus;
reg  [31:0] instrbusin[0:35];
wire [31:0] iaddrbus, daddrbus;
reg  [31:0] iaddrbusout[0:35], daddrbusout[0:35];
wire [31:0] databus;
reg  [31:0] databusk, databusin[0:35], databusout[0:35];
reg         clk, reset;
reg         clkd;

reg [31:0] dontcare;
reg [24*8:1] iname[0:35];
integer error, k, ntests;

	parameter Rformat	= 6'b000000;
	parameter ADDI		= 6'b000011;
	parameter SUBI		= 6'b000010;
	parameter XORI		= 6'b000001;
	parameter ANDI		= 6'b001111;
	parameter ORI		= 6'b001100;
	parameter LW		= 6'b011110;
	parameter SW		= 6'b011111;
	parameter BEQ		= 6'b110000;
	parameter BNE		= 6'b110001;
	parameter ADD		= 6'b000011;
	parameter SUB		= 6'b000010;
	parameter XOR		= 6'b000001;
	parameter AND		= 6'b000111;
	parameter OR		= 6'b000100;
	parameter SLT		= 6'b110110;
	parameter SLE		= 6'b110111;

cpu5 dut(.reset(reset),.clk(clk),.iaddrbus(iaddrbus),.ibus(instrbus),.daddrbus(daddrbus),.databus(databus));

initial begin
// This test file runs the following program.

iname[0] = "ADDI  R20, R0, #-1";
iname[1] = "ADDI  R21, R0, #1";
iname[2] = "ADDI  R22, R0, #2";
iname[3] = "LW    R24, 0(R20)";
iname[4] = "LW    R25, 0(R21)";
iname[5] = "SW    1000(R22), R20";
iname[6] = "SW    2(R0), R21";
iname[7] = "ADD   R26, R24, R25";
iname[8] = "SUBI  R17, R24, 6420";
iname[9] = "SUB   R27, R24, R25";
iname[10] = "ANDI R18, R24, #0";     
iname[11] = "AND  R28, R24, R0";     
iname[12] = "XORI R19, R24, 6420";
iname[13] = "XOR  R29, R24, R25";
iname[14] = "ORI  R20, R24, 6420";
iname[15] = "OR   R30, R24, R25";
iname[16] = "SW   0(R26),  R26";
iname[17] = "SW   0(R17),  R27";
iname[18] = "SW   1000(R18),  R28"; 
iname[19] = "SW   0(R19),  R29";
iname[20] = "SW   0(R20),  R30";
iname[21] = "SLT  R1,  R0,  R21";  // Setting R1 to 32'h00000001 (since, R0 < R21).
iname[22] = "ADDI R5,  R0, #1";
iname[23] = "ADDI R6,  R0, #1";
iname[24] = "BNE  R0,  R1, #10";   // Branching to (32'h00000060 + 32'h00000004 + 32'h00000028 = 32'h0000008C) since, R0 != R1.
iname[25] = "ADDI R8,  R0, #1";    // Delay Slot
//Branched Location - 32'h0000008C //
iname[26] = "SLE  R2,  R0, R0";    // Setting R2 to 32'h00000001 (since, R0 = R0).
iname[27] = "NOP";
iname[28] = "NOP";
iname[29] = "BEQ  R0,  R2, #25";   // NOT Branching since, R2 != R0. 
iname[30] = "NOP";                 // Delay Slot
iname[31] = "BEQ  R2,  R2, #10";   // Branching to (32h'0000000A0 + 32'h00000004 + 32'h00000028 = 32'h000000CC)
iname[32] = "ADDI R20, R0, #1";    // Delay Slot
//Branched Location - 32'h000000CC //
iname[33] = "NOP";
iname[34] = "NOP";
iname[35] = "NOP";

// 20 more testbenches


dontcare = 32'hx;

//* ADDI  R20, R0, #-1
iaddrbusout[0] = 32'h00000000;
//            opcode source1   dest      Immediate...
instrbusin[0]={ADDI, 5'b00000, 5'b10100, 16'hFFFF};

daddrbusout[0] = dontcare;
databusin[0] = 32'bzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz;
databusout[0] = dontcare;

//* ADDI  R21, R0, #1
iaddrbusout[1] = 32'h00000004;
//            opcode source1   dest      Immediate...
instrbusin[1]={ADDI, 5'b00000, 5'b10101, 16'h0001};

daddrbusout[1] = dontcare;
databusin[1] = 32'bzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz;
databusout[1] = dontcare;

//* ADDI  R22, R0, #2
iaddrbusout[2] = 32'h00000008;
//            opcode source1   dest      Immediate...
instrbusin[2]={ADDI, 5'b00000, 5'b10110, 16'h0002};

daddrbusout[2] = dontcare;
databusin[2] = 32'bzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz;
databusout[2] = dontcare;

//* LW     R24, 0(R20)
iaddrbusout[3] = 32'h0000000C;
//            opcode source1   dest      Immediate...
instrbusin[3]={LW, 5'b10100, 5'b11000, 16'h0000};

daddrbusout[3] = 32'hFFFFFFFF;
databusin[3] = 32'hCCCCCCCC;
databusout[3] = dontcare;

//* LW     R25, 0(R21)
iaddrbusout[4] = 32'h00000010;
//            opcode source1   dest      Immediate...
instrbusin[4]={LW, 5'b10101, 5'b11001, 16'h0000};

daddrbusout[4] = 32'h00000001;
databusin[4] = 32'hAAAAAAAA;
databusout[4] = dontcare;

//* SW     1000(R22), R20
iaddrbusout[5] = 32'h00000014;
//            opcode source1   dest      Immediate...
instrbusin[5]={SW, 5'b10110, 5'b10100, 16'h1000};

daddrbusout[5] = 32'h00001002;
databusin[5] = 32'bzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz;
databusout[5] = 32'hFFFFFFFF;

//* SW     2(R0), R21
iaddrbusout[6] = 32'h00000018;
//            opcode source1   dest      Immediate...
instrbusin[6]={SW, 5'b00000, 5'b10101, 16'h0002};

daddrbusout[6] = 32'h00000002;
databusin[6] = 32'bzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz;
databusout[6] = 32'h00000001;

//* ADD   R26, R24, R25
iaddrbusout[7] = 32'h0000001C;
//             opcode   source1   source2   dest      shift     Function...
instrbusin[7]={Rformat, 5'b11000, 5'b11001, 5'b11010, 5'b00000, ADD};

daddrbusout[7] = dontcare;
databusin[7] = 32'bzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz;
databusout[7] = dontcare;

//* SUBI  R17, R24, 6420
iaddrbusout[8] = 32'h00000020;
//            opcode source1   dest      Immediate...
instrbusin[8]={SUBI, 5'b11000, 5'b10001, 16'h6420};

daddrbusout[8] = dontcare;
databusin[8] = 32'bzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz;
databusout[8] = dontcare;

//* SUB   R27, R24, R25
iaddrbusout[9] = 32'h00000024;
//             opcode   source1   source2   dest      shift     Function...
instrbusin[9]={Rformat, 5'b11000, 5'b11001, 5'b11011, 5'b00000, SUB};

daddrbusout[9] = dontcare;
databusin[9] = 32'bzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz;
databusout[9] = dontcare;

//* ANDI   R18, R24, #0             
iaddrbusout[10] = 32'h00000028;
//            opcode source1   dest      Immediate...
instrbusin[10]={ANDI, 5'b11000, 5'b10010, 16'h0000};

daddrbusout[10] = dontcare;
databusin[10] = 32'bzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz;
databusout[10] = dontcare;

//* AND    R28, R24, R0           
iaddrbusout[11] = 32'h0000002C;
//             opcode   source1   source2   dest      shift     Function...
instrbusin[11]={Rformat, 5'b11000, 5'b00000, 5'b11100, 5'b00000, AND};

daddrbusout[11] = dontcare;
databusin[11] = 32'bzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz;
databusout[11] = dontcare;

//* XORI   R19, R24, 6420
iaddrbusout[12] = 32'h00000030;
//            opcode source1   dest      Immediate...
instrbusin[12]={XORI, 5'b11000, 5'b10011, 16'h6420};

daddrbusout[12] = dontcare;
databusin[12] = 32'bzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz;
databusout[12] = dontcare;

//* XOR    R29, R24, R25
iaddrbusout[13] = 32'h00000034;
//             opcode   source1   source2   dest      shift     Function...
instrbusin[13]={Rformat, 5'b11000, 5'b11001, 5'b11101, 5'b00000, XOR};

daddrbusout[13] = dontcare;
databusin[13] = 32'bzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz;
databusout[13] = dontcare;

//* ORI    R20, R24, 6420
iaddrbusout[14] = 32'h00000038;
//            opcode source1   dest      Immediate...
instrbusin[14]={ORI, 5'b11000, 5'b10100, 16'h6420};

daddrbusout[14] = dontcare;
databusin[14] = 32'bzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz;
databusout[14] = dontcare;

//* OR     R30, R24, R25
iaddrbusout[15] = 32'h0000003C;
//             opcode   source1   source2   dest      shift     Function...
instrbusin[15]={Rformat, 5'b11000, 5'b11001, 5'b11110, 5'b00000, OR};

daddrbusout[15] = dontcare;
databusin[15] = 32'bzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz;
databusout[15] =  dontcare;

//* SW     0(R26),  R26
iaddrbusout[16] = 32'h00000040;
//            opcode source1   dest      Immediate...
instrbusin[16]={SW, 5'b11010, 5'b11010, 16'h0000};

daddrbusout[16] = 32'h77777776;
databusin[16] = 32'bzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz;
databusout[16] = 32'h77777776;

//18* SW     0(R17),  R27
iaddrbusout[17] = 32'h00000044;
//            opcode source1   dest      Immediate...
instrbusin[17]={SW, 5'b10001, 5'b11011, 16'h0000};

daddrbusout[17] = 32'hCCCC68AC;
databusin[17] = 32'bzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz;
databusout[17] = 32'h22222222;

//19* SW     1000(R18),  R28           
iaddrbusout[18] = 32'h00000048;
//            opcode source1   dest      Immediate...
instrbusin[18]={SW, 5'b10010, 5'b11100, 16'h1000};

daddrbusout[18] = 32'h00001000;
databusin[18] = 32'bzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz;
databusout[18] = 32'h00000000;

//20* SW     0(R19),  R29
iaddrbusout[19] = 32'h0000004C;
//            opcode source1   dest      Immediate...
instrbusin[19]={SW, 5'b10011, 5'b11101, 16'h0000};

daddrbusout[19] = 32'hCCCCA8EC;
databusin[19] = 32'bzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz;
databusout[19] = 32'h66666666;

//21* SW     0(R20),  R30
iaddrbusout[20] = 32'h00000050;
//            opcode source1   dest      Immediate...
instrbusin[20]={SW, 5'b10100, 5'b11110, 16'h0000};

daddrbusout[20] = 32'hCCCCECEC;
databusin[20] = 32'bzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz;
databusout[20] = 32'hEEEEEEEE;


//22* SLT  R1,  R0,  R21
iaddrbusout[21] = 32'h00000054;
//             opcode   source1   source2   dest      shift     Function...
instrbusin[21]={Rformat, 5'b00000, 5'b10101, 5'b00001, 5'b00000, SLT};
daddrbusout[21] = dontcare;
databusin[21]   = 32'bzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz;
databusout[21]  = dontcare;

//* ADDI R5,  R0, #1
iaddrbusout[22] = 32'h00000058;
//            opcode source1   dest      Immediate...
instrbusin[22]={ADDI, 5'b00000, 5'b00101, 16'h0001};
daddrbusout[22] = dontcare;
databusin[22] = 32'bzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz;
databusout[22] = dontcare;

//* ADDI R6,  R0, #1
iaddrbusout[23] = 32'h0000005C;
//            opcode source1   dest      Immediate...
instrbusin[23]={ADDI, 5'b00000, 5'b00110, 16'h0001};
daddrbusout[23] = dontcare;
databusin[23] =   32'bzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz;
databusout[23] =  dontcare;

//* BNE  R0,  R1, #10
iaddrbusout[24] = 32'h00000060;
//            opcode source1   dest      Immediate...
instrbusin[24]={BNE, 5'b00001, 5'b00000, 16'h000A};
daddrbusout[24] = dontcare;
databusin[24] = 32'bzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz;
databusout[24] = dontcare;

//* ADDI R8,  R0, #1
iaddrbusout[25] = 32'h00000064;
//            opcode source1   dest      Immediate...
instrbusin[25]={ADDI, 5'b00000, 5'b01000, 16'h0001};
daddrbusout[25] = dontcare;
databusin[25] = 32'bzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz;
databusout[25] = dontcare;

//* SLE  R2,  R0, R0
iaddrbusout[26] = 32'h0000008C;
//             opcode   source1   source2   dest      shift     Function...
instrbusin[26]={Rformat, 5'b00000, 5'b00000, 5'b00010, 5'b00000, SLE};
daddrbusout[26] = dontcare;
databusin[26] = 32'bzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz;
databusout[26] = dontcare;

//* NOP
iaddrbusout[27] = 32'h00000090;
//                   oooooosssssdddddiiiiiiiiiiiiiiii
instrbusin[27] = 32'b00000000000000000000000000000000;
daddrbusout[27] = dontcare;
databusin[27] = 32'bzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz;
databusout[27] = dontcare;

//* NOP
iaddrbusout[28] = 32'h00000094;
//                   oooooosssssdddddiiiiiiiiiiiiiiii
instrbusin[28] = 32'b00000000000000000000000000000000;
daddrbusout[28] = dontcare;
databusin[28]  = 32'bzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz;
databusout[28] = dontcare;

//* BEQ  R0,  R2, #25
iaddrbusout[29] = 32'h00000098;
//            opcode source1   dest      Immediate...
instrbusin[29]={BEQ, 5'b00010, 5'b00000, 16'h0019};
daddrbusout[29] = dontcare;
databusin[29] = 32'bzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz;
databusout[29] = dontcare;

//* NOP
iaddrbusout[30] = 32'h0000009C;
//                   oooooosssssdddddiiiiiiiiiiiiiiii
instrbusin[30] = 32'b00000000000000000000000000000000;
daddrbusout[30] = dontcare;
databusin[30] = 32'bzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz;
databusout[30] = dontcare;

//* BEQ  R2,  R2, #10
iaddrbusout[31] = 32'h000000A0;
//            opcode source1   dest      Immediate...
instrbusin[31]={BEQ, 5'b00010, 5'b00010, 16'h000A};
daddrbusout[31] = dontcare;
databusin[31] = 32'bzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz;
databusout[31] = dontcare;

//* ADDI R20, R0, #1
iaddrbusout[32] = 32'h000000A4;
//            opcode source1   dest      Immediate...
instrbusin[32]={ADDI, 5'b00000, 5'b10100, 16'h0001};
daddrbusout[32] = dontcare;
databusin[32] = 32'bzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz;
databusout[32] = dontcare;

//* NOP
iaddrbusout[33] = 32'h000000CC;
//                   oooooosssssdddddiiiiiiiiiiiiiiii
instrbusin[33] = 32'b00000000000000000000000000000000;
daddrbusout[33] = dontcare;
databusin[33] = 32'bzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz;
databusout[33] = dontcare;

//* NOP
iaddrbusout[34] = 32'h000000D0;
//                   oooooosssssdddddiiiiiiiiiiiiiiii
instrbusin[34] = 32'b00000000000000000000000000000000;
daddrbusout[34] = dontcare;
databusin[34] = 32'bzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz;
databusout[34] = dontcare;

//* NOP
iaddrbusout[35] = 32'h000000D4;
//                   oooooosssssdddddiiiiiiiiiiiiiiii
instrbusin[35] = 32'b00000000000000000000000000000000;
daddrbusout[35] = dontcare;
databusin[35] = 32'bzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz;
databusout[35] = dontcare;


// (no. instructions) + (no. loads) + 2*(no. stores) = 35 + 2 + 2*7 = 51
ntests = 51;

$timeformat(-9,1,"ns",12);

end


//assumes positive edge FF.
//testbench reads databus when clk high, writes databus when clk low.
assign databus = clkd ? 32'bz : databusk;

//Change inputs in middle of period (falling edge).
initial begin
  error = 0;
  clkd =1;
  clk=1;
  $display ("Time=%t\n  clk=%b", $realtime, clk);
  databusk = 32'bz;

  //extended reset to set up PC MUX
  reset = 1;
  $display ("reset=%b", reset);
  #5
  clk=0;
  clkd=0;
  $display ("Time=%t\n  clk=%b", $realtime, clk);
  #5

  clk=1;
  clkd=1;
  $display ("Time=%t\n  clk=%b", $realtime, clk);
  #5
  clk=0;
  clkd=0;
  $display ("Time=%t\n  clk=%b", $realtime, clk);
  #5
  $display ("Time=%t\n  clk=%b", $realtime, clk);

for (k=0; k<= 35; k=k+1) begin
    clk=1;
    $display ("Time=%t\n  clk=%b", $realtime, clk);
    #2
    clkd=1;
    #3
    $display ("Time=%t\n  clk=%b", $realtime, clk);
    reset = 0;
    $display ("reset=%b", reset);


    //set load data for 3rd previous instruction
    if (k >=3)
      databusk = databusin[k-3];

    //check PC for this instruction
    if (k >= 0) begin
      $display ("  Testing PC for instruction %d", k);
      $display ("    Your iaddrbus =    %b", iaddrbus);
      $display ("    Correct iaddrbus = %b", iaddrbusout[k]);
      if (iaddrbusout[k] !== iaddrbus) begin
        $display ("    -------------ERROR. A Mismatch Has Occured-----------");
        error = error + 1;
      end
    end

    //put next instruction on ibus
    instrbus=instrbusin[k];
    $display ("  instrbus=%b %b %b %b %b for instruction %d: %s", instrbus[31:26], instrbus[25:21], instrbus[20:16], instrbus[15:11], instrbus[10:0], k, iname[k]);

    //check data address from 3rd previous instruction
    if ( (k >= 3) && (daddrbusout[k-3] !== dontcare) ) begin
      $display ("  Testing data address for instruction %d:", k-3);
      $display ("  %s", iname[k-3]);
      $display ("    Your daddrbus =    %b", daddrbus);
      $display ("    Correct daddrbus = %b", daddrbusout[k-3]);
      if (daddrbusout[k-3] !== daddrbus) begin
        $display ("    -------------ERROR. A Mismatch Has Occured-----------");
        error = error + 1;
      end
    end

    //check store data from 3rd previous instruction
    if ( (k >= 3) && (databusout[k-3] !== dontcare) ) begin
      $display ("  Testing store data for instruction %d:", k-3);
      $display ("  %s", iname[k-3]);
      $display ("    Your databus =    %b", databus);
      $display ("    Correct databus = %b", databusout[k-3]);
      if (databusout[k-3] !== databus) begin
        $display ("    -------------ERROR. A Mismatch Has Occured-----------");
        error = error + 1;
      end
    end

    clk = 0;
    $display ("Time=%t\n  clk=%b", $realtime, clk);
    #2
    clkd = 0;
    #3
    $display ("Time=%t\n  clk=%b", $realtime, clk);
  end

  if ( error !== 0) begin
    $display("--------- SIMULATION UNSUCCESFUL - MISMATCHES HAVE OCCURED ----------");
    $display(" No. Of Errors = %d", error);
  end
  if ( error == 0)
    $display("---------YOU DID IT!! SIMULATION SUCCESFULLY FINISHED----------");
end

endmodule

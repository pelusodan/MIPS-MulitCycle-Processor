`timescale 1ns / 1ps


//cpu5 dut(.reset(reset),.clk(clk),.iaddrbus(iaddrbus),.ibus(instrbus),.daddrbus(daddrbus),.databus(databus));
module cpu5(reset, clk, iaddrbus, ibus, daddrbus, databus);
    input reset, clk;
    input [31:0] ibus;
    output [31:0] iaddrbus, daddrbus;
    inout [31:0] databus;
    
    wire [31:0] ibusToDecoder, iaddrIfidToAdder, logicalAdderToMux, pcAdderToMux, sllToAdder, shiftLeftAdderToMux ;
    wire [31:0] muxToPc,signExtDecoderToIdex, rdWire, rsWire, rtWire, loadWireDecoderToIdex ;
    wire [31:0] DselectMuxToEqualizer, abusRegToIdex, bbusRegToIdex, DSelectEqualizerToIdex, DselectIdexToExmem, signExtIdexToMux;
    wire equalizerToMux, immDecoderToIdex, cinDecoderToIdex;
    wire [1:0] branchDecoderToEqualizer, slDecoderToIdex, loadWireIdexToExMem , slIdexToAlu, loadWireExmemToMemwb, loadWireMemwbToMux ;
    wire [31:0] abusToAlu, bbusIdexToMux, bbusToAlu , aluToExmem;
    wire immIdexToMux, cinIdexToAlu; 
    wire [2:0] sDecoderToIdex, sIdexToAlu;
    wire [31:0] DselectExmemToMemwb, databusExmemToBuffer, daddrbusMemwbToMux, databusMemwbToMux, muxOutToReg, DselectMemwbToReg;
    wire [31:0] iaddrbusWire;
    pcReg pc(
        .clk(clk),
        .iaddrbusIn(muxToPc),
        .iaddrbusOut(iaddrbus),
        .reset(reset)   
    );
    adder pcAdder(
        .a(iaddrbus),
        .b(32'h00000004),
        .d(pcAdderToMux)
    );
    ifidReg ifid(
        .ibusIn(ibus),
        .clk(clk),
        .ibusOut(ibusToDecoder),
        .iaddrbusIn(pcAdderToMux),
        .iaddrbusOut(iaddrIfidToAdder)
    );
    adder shiftIaddrAdder(
        .a(iaddrIfidToAdder),
        .b(sllToAdder),
        .d(shiftLeftAdderToMux)
    );
    // if equal, then the next instruction will breanch to the shift
    mux2to1 mux4(
        .abus(pcAdderToMux),
        .bbus(shiftLeftAdderToMux),
        .S(equalizerToMux),
        .out(muxToPc)
     );
    decoderTop top(
        .ibus(ibusToDecoder),
        .Imm(immDecoderToIdex),
        .Cin(cinDecoderToIdex),
        .S(sDecoderToIdex),
        .signExt(signExtDecoderToIdex),
        .rd(rdWire),
        .rt(rtWire),
        .rs(rsWire),
        .loadSelector(loadWireDecoderToIdex),
        .branchIns(branchDecoderToEqualizer),
        .slInst(slDecoderToIdex)
    );
     assign sllToAdder = signExtDecoderToIdex << 2;// logical shift left for signExtImm
     mux2to1 mux1(
         .abus(rdWire),
         .bbus(rtWire),
         .S(immDecoderToIdex),
         .out(DselectMuxToEqualizer)
     );
     regfile register(
         .Aselect(rsWire),
         .Bselect(rtWire),
         .Dselect(DselectMemwbToReg),
         .dbus(muxOutToReg),
         .clk(clk),
         .abus(abusRegToIdex),
         .bbus(bbusRegToIdex)
     );
     equalizer eq(
        .a(abusRegToIdex),
        .b(bbusRegToIdex),
        .out(equalizerToMux),
        .DselectIn(DselectMuxToEqualizer),
        .DselectOut(DSelectEqualizerToIdex),
        .en(branchDecoderToEqualizer[0]),
        .fxn(branchDecoderToEqualizer[1])
     );
     idexReg idex(
         .muxOut(DSelectEqualizerToIdex),
         .Imm(immDecoderToIdex),
         .S(sDecoderToIdex),
         .Cin(cinDecoderToIdex),
         .clk(clk),
         .out(DselectIdexToExmem),
         .Immout(immIdexToMux),
         .Sout(sIdexToAlu),
         .Cinout(cinIdexToAlu),
         .signExtIn(signExtDecoderToIdex),
         .signExtOut(signExtIdexToMux),
         .abusIn(abusRegToIdex),
         .abusOut(abusToAlu),
         .bbusIn(bbusRegToIdex),
         .bbusOut(bbusIdexToMux),
         .loadSelectIn(loadWireDecoderToIdex),
         .loadSelectOut(loadWireIdexToExMem),
         .slInsIn(slDecoderToIdex),
         .slInsOut(slIdexToAlu)
     );
     mux2to1 mux2(
         .abus(bbusIdexToMux),
         .bbus(signExtIdexToMux),
         .S(immIdexToMux),
         .out(bbusToAlu)
     );
     alu32 alu(
         .d(aluToExmem),
         .a(abusToAlu),
         .b(bbusToAlu),
         .Cin(cinIdexToAlu),
         .S(sIdexToAlu),
         .slIns(slIdexToAlu)
     );
     exmemReg exmem(
         .aluOut(aluToExmem),
         .dSelectIn(DselectIdexToExmem),
         .clk(clk),
         .dbus(daddrbus),
         .dSelectOut(DselectExmemToMemwb),
         .loadSelectIn(loadWireIdexToExMem),
         .loadSelectOut(loadWireExmemToMemwb),
         .databusIn(bbusIdexToMux),
         .databusOut(databusExmemToBuffer)
     );
         triStateBuffer buffer(
         .S(loadWireExmemToMemwb),
         .in(databusExmemToBuffer),
         .out(databus)
     );
     memwbReg memwb(
         .daddrbusIn(daddrbus),
         .daddrbusOut(daddrbusMemwbToMux),
         .databusIn(databus),
         .databusOut(databusMemwbToMux),
         .loadSelectIn(loadWireExmemToMemwb[0]),
         .loadSelectOut(loadWireMemwbToMux),
         .DselectIn(DselectExmemToMemwb),
         .DselectOut(DselectMemwbToReg),
         .clk(clk)
     );
     mux2to1 mux3(
         .abus(daddrbusMemwbToMux),
         .bbus(databusMemwbToMux),
         .S(loadWireMemwbToMux),
         .out(muxOutToReg)
     );
    
endmodule

// MEMWB REGISTER
module memwbReg(daddrbusIn, daddrbusOut, databusIn, databusOut, loadSelectIn, loadSelectOut, clk, DselectIn, DselectOut);
    input [31:0] daddrbusIn, databusIn, DselectIn;
    input loadSelectIn, clk;
    output reg [31:0] daddrbusOut, databusOut, DselectOut;
    output reg loadSelectOut;
    always @ (posedge clk) begin
        daddrbusOut = daddrbusIn;
        databusOut = databusIn;
        loadSelectOut = loadSelectIn;
        DselectOut = DselectIn;
    end
endmodule
//TRI STATE MODULE
module triStateBuffer(S,in,out);
    input [1:0] S;
    input [31:0] in;
    output [31:0] out;
    assign out = (S==2'b00 || S==2'b10) ? in : 32'bz;   
endmodule
// EXMEM
module exmemReg(aluOut,dSelectIn,clk,dbus,dSelectOut,loadSelectIn,loadSelectOut, databusIn, databusOut);
    input [31:0] aluOut, dSelectIn, databusIn; 
    input clk;
    input [1:0] loadSelectIn;
    output reg [31:0] dbus, dSelectOut, databusOut;
    output reg [1:0] loadSelectOut;
    always @ (posedge clk) begin
        dbus = aluOut;
        dSelectOut = (loadSelectIn!=2'b01) ? dSelectIn: 32'bz;
        loadSelectOut = loadSelectIn;
        databusOut = databusIn;
   end
endmodule
// ALU
module alu32 (d, Cout, V, a, b, Cin, S, slIns);
   output[31:0] d;
   output Cout, V;
   input [31:0] a, b;
   input Cin;
   input [2:0] S;
   input [1:0] slIns;
   
   wire [31:0] c, g, p, res;
   wire gout, pout;
   alu_cell alucell[31:0] (
      .d(res),
      .g(g),
      .p(p),
      .a(a),
      .b(b),
      .c(c),
      .S(S)
   );
   lac5 laclevel5(
      .c(c),
      .gout(gout),
      .pout(pout),
      .Cin(Cin),
      .g(g),
      .p(p)
   );
   overflow over(
      .gout(gout),
      .pout(pout),
      .Cin(Cin),
      .c31(c[31]),
      .Cout(Cout),
      .V(V)
   );
   // if SLT and SLE
    assign d = (slIns==2'b01) ? {31'b0, (!(res==0) && !Cout)} : // SLT
        (slIns==2'b10) ? {31'b0, (!Cout || (res==0))} : // SLE
        (slIns!=2'b00) ? 32'b0 : res; // 0 if a set, alu out if  otherwise
  endmodule
module alu_cell (d, g, p, a, b, c, S);
   output d, g, p;
   input a, b, c;
   input [2:0] S;      
   reg g,p,d,cint,bint;
     
   always @(a,b,c,S,p,g) begin 
     bint = S[0] ^ b;
     g = a & bint;
     p = a ^ bint;
     cint = S[1] & c;
     if(S[2] == 0)
        d = p ^ cint;
     if(S[2] == 1 && S[1] == 0 && S[0] == 0)
        d = a | b;
     if(S[2] == 1 && S[1] == 0 && S[0] == 1)
        d = !(a | b);
     if(S[2] == 1 && S[1] == 1 && S[0] == 0)
        d = a & b;
   end
endmodule
module overflow (gout,pout,Cin,c31,Cout,V);
    input gout, pout, Cin, c31;
    output Cout, V;
    assign Cout = gout | (pout & Cin);
    assign V = Cout ^ c31;
endmodule
module lac(c, gout, pout, Cin, g, p);
   output [1:0] c;
   output gout;
   output pout;
   input Cin;
   input [1:0] g;
   input [1:0] p;
   assign c[0] = Cin;
   assign c[1] = g[0] | ( p[0] & Cin );
   assign gout = g[1] | ( p[1] & g[0] );
   assign pout = p[1] & p[0];	
endmodule
module lac2 (c, gout, pout, Cin, g, p);
   output [3:0] c;
   output gout, pout;
   input Cin;
   input [3:0] g, p;
   
   wire [1:0] cint, gint, pint;
   
   lac leaf0(
      .c(c[1:0]),
      .gout(gint[0]),
      .pout(pint[0]),
      .Cin(cint[0]),
      .g(g[1:0]),
      .p(p[1:0])
   );
   
   lac leaf1(
      .c(c[3:2]),
      .gout(gint[1]),
      .pout(pint[1]),
      .Cin(cint[1]),
      .g(g[3:2]),
      .p(p[3:2])
   );
   
   lac root(
      .c(cint),
      .gout(gout),
      .pout(pout),
      .Cin(Cin),
      .g(gint),
      .p(pint)
   );
endmodule   
module lac3 (c, gout, pout, Cin, g, p);
   output [7:0] c;
   output gout, pout;
   input Cin;
   input [7:0] g, p;
   
   wire [1:0] cint, gint, pint;
   
   lac2 leaf0(
      .c(c[3:0]),
      .gout(gint[0]),
      .pout(pint[0]),
      .Cin(cint[0]),
      .g(g[3:0]),
      .p(p[3:0])
   );
   
   lac2 leaf1(
      .c(c[7:4]),
      .gout(gint[1]),
      .pout(pint[1]),
      .Cin(cint[1]),
      .g(g[7:4]),
      .p(p[7:4])
   );
   
   lac root(
      .c(cint),
      .gout(gout),
      .pout(pout),
      .Cin(Cin),
      .g(gint),
      .p(pint)
   );
endmodule
module lac4 (c, gout, pout, Cin, g, p);
 output [15:0] c;
 output gout, pout;
 input Cin;
 input [15:0] g, p;
 
 wire [1:0] cint, gint, pint;
 
 lac3 leaf0(
    .c(c[7:0]),
    .gout(gint[0]),
    .pout(pint[0]),
    .Cin(cint[0]),
    .g(g[7:0]),
    .p(p[7:0])
 );
 
 lac3 leaf1(
    .c(c[15:8]),
    .gout(gint[1]),
    .pout(pint[1]),
    .Cin(cint[1]),
    .g(g[15:8]),
    .p(p[15:8])
 );
 
 lac root(
    .c(cint),
    .gout(gout),
    .pout(pout),
    .Cin(Cin),
    .g(gint),
    .p(pint)
 );
endmodule
module lac5 (c, gout, pout, Cin, g, p);
output [31:0] c;
 output gout, pout;
 input Cin;
 input [31:0] g, p;
 
 wire [1:0] cint, gint, pint;
 
 lac4 leaf0(
    .c(c[15:0]),
    .gout(gint[0]),
    .pout(pint[0]),
    .Cin(cint[0]),
    .g(g[15:0]),
    .p(p[15:0])
 );
 
 lac4 leaf1(
    .c(c[31:16]),
    .gout(gint[1]),
    .pout(pint[1]),
    .Cin(cint[1]),
    .g(g[31:16]),
    .p(p[31:16])
 );
 
 lac root(
    .c(cint),
    .gout(gout),
    .pout(pout),
    .Cin(Cin),
    .g(gint),
    .p(pint)
 );
endmodule
// IDEX REGISTER
module idexReg(muxOut,Imm,S,Cin,clk,out,Immout,Sout,Cinout, signExtIn, signExtOut, abusIn, abusOut, bbusIn, bbusOut, loadSelectIn, loadSelectOut, slInsIn, slInsOut);
        input [31:0] muxOut, signExtIn, abusIn, bbusIn;
        input Imm, Cin, clk;
        input [2:0] S;
        input [1:0] loadSelectIn, slInsIn;
        output reg [31:0] out, signExtOut, abusOut, bbusOut;
        output reg Immout, Cinout;
        output reg [2:0] Sout;
        output reg [1:0] loadSelectOut, slInsOut;
        always  @ (posedge clk) begin
            out = muxOut;
            Immout = Imm;
            Cinout = Cin;
            Sout = S;
            signExtOut = signExtIn;
            abusOut = abusIn;
            bbusOut = bbusIn;
            loadSelectOut = loadSelectIn;
            slInsOut = slInsIn;
        end
endmodule
//EQUALIZER                                     enable function
module equalizer(a,b,out, DselectIn, DselectOut, en, fxn);
    input [31:0] a, b, DselectIn;
    // 0 is BEQ, 1 is  BNE
    input en, fxn;
    output out;
    output [31:0] DselectOut;
    assign out = !en ? 0 : // default - not branching
    (a == b && fxn ==0) ?  1: // beq function
    (a != b && fxn==1) ? 1: 0; // bne function and the final default, 0
    // for avoiding WB in BEQ and BNE
    assign DselectOut = !en ? DselectIn:
     (a == b && fxn == 0)||(a != b && fxn == 1) ? 32'b0: DselectIn;
endmodule
// REGISTER FILE
module regfile(Aselect,Bselect,Dselect,dbus,clk,abus,bbus);
    input [31:0] Aselect, Bselect, Dselect,dbus;
    input clk;
    output [31:0] abus, bbus;
    assign abus = Aselect[0] ? 0 : 32'bz;
    assign bbus = Bselect[0] ? 0 : 32'bz;        
    negdffcell register_cells [30:0](
        .Aselect(Aselect[31:1]),
        .Bselect(Bselect[31:1]),
        .clk(clk),
        .Dselect(Dselect[31:1]),
        .dbus(dbus),
        .abus(abus),
        .bbus(bbus)
        );
endmodule
module negdffcell(Aselect,Bselect,clk,Dselect,dbus,abus,bbus);
    input Aselect, Bselect, clk, Dselect;
    input [31:0] dbus;
    output [31:0] abus, bbus;
    wire [31:0] Q;
    negdff flip(
        .D(dbus),
        .clk(clk),
        .Dselect(Dselect),
        .Q(Q)
        );
    buffer cellbuffer(
        .Q(Q),
        .Aselect(Aselect),
        .Bselect(Bselect),
        .abus(abus),
        .bbus(bbus)
    );
endmodule
module negdff(D,clk,Dselect,Q);
    input [31:0] D;
    input clk, Dselect;
    output reg [31:0] Q;
    wire newclk;
    assign newclk = clk & Dselect;
    always @(negedge newclk) begin
        if (Dselect==1'b1) Q = D;
    end
endmodule
module buffer(Q,Aselect,Bselect,abus,bbus);
    input [31:0] Q;
    input Aselect, Bselect;
    output [31:0] abus,bbus;
    assign abus = Aselect ? Q : 32'bz;
    assign bbus = Bselect ? Q : 32'bz;
endmodule
// FIRST MULTIPLEXER
module mux2to1(abus,bbus,S,out);
    input [31:0] abus, bbus;
    input S;
    output [31:0] out;
    wire sWire;
    assign sWire = (S===1'bx) ? 0 : S;
    assign out = sWire ? bbus: abus;
endmodule
// DECODER
module decoderTop(ibus,Imm,Cin,S,signExt,rd,rt,rs,loadSelector, branchIns, slInst);
    input [31:0] ibus;
    output Imm, Cin;
    output [1:0] loadSelector, branchIns, slInst;
    output [2:0] S;
    output [31:0] signExt, rd, rt, rs;
    dec5to32 rsDec(
        .in(ibus[25:21]),
        .out(rs));
    dec5to32 rtDec(
        .in(ibus[20:16]),
        .out(rt));
    dec5to32 rdDec(
        .in(ibus[15:11]),
        .out(rd));
    opcodeDec opcodedec(
        .ibus(ibus),
        .Imm(Imm),
        .S(S),
        .Cin(Cin),
        .loadSelector(loadSelector),
        .branchIns(branchIns),
        .slInst(slInst));
    assign signExt = {{16{ibus[15]}}, ibus[15:0]};
endmodule
module dec5to32(in,out);
    input [4:0] in;
    output reg [31:0] out;
    always @(in, out) begin
      // out = 32'h00000001 << in;
       case(in)
       5'b00000: out = 32'b00000000000000000000000000000001; 
       5'b00001: out = 32'b00000000000000000000000000000010; 
       5'b00010: out = 32'b00000000000000000000000000000100;
       5'b00011: out = 32'b00000000000000000000000000001000;
       5'b00100: out = 32'b00000000000000000000000000010000;
       5'b00101: out = 32'b00000000000000000000000000100000;
       5'b00110: out = 32'b00000000000000000000000001000000;
       5'b00111: out = 32'b00000000000000000000000010000000;
       5'b01000: out = 32'b00000000000000000000000100000000;
       5'b01001: out = 32'b00000000000000000000001000000000;
       5'b01010: out = 32'b00000000000000000000010000000000;
       5'b01011: out = 32'b00000000000000000000100000000000;
       5'b01100: out = 32'b00000000000000000001000000000000;
       5'b01101: out = 32'b00000000000000000010000000000000;
       5'b01110: out = 32'b00000000000000000100000000000000;
       5'b01111: out = 32'b00000000000000001000000000000000;
       5'b10000: out = 32'b00000000000000010000000000000000;
       5'b10001: out = 32'b00000000000000100000000000000000;
       5'b10010: out = 32'b00000000000001000000000000000000;
       5'b10011: out = 32'b00000000000010000000000000000000;
       5'b10100: out = 32'b00000000000100000000000000000000;
       5'b10101: out = 32'b00000000001000000000000000000000;
       5'b10110: out = 32'b00000000010000000000000000000000;
       5'b10111: out = 32'b00000000100000000000000000000000;
       5'b11000: out = 32'b00000001000000000000000000000000;
       5'b11001: out = 32'b00000010000000000000000000000000;
       5'b11010: out = 32'b00000100000000000000000000000000;
       5'b11011: out = 32'b00001000000000000000000000000000;
       5'b11100: out = 32'b00010000000000000000000000000000;
       5'b11101: out = 32'b00100000000000000000000000000000;
       5'b11110: out = 32'b01000000000000000000000000000000;
       5'b11111: out = 32'b10000000000000000000000000000000;
       endcase
    end
endmodule 
module opcodeDec(ibus,Imm,S,Cin,loadSelector, branchIns, slInst);
    input [31:0] ibus;
    output reg Imm, Cin;
    output reg [1:0] loadSelector, branchIns, slInst;
    output reg [2:0] S;
    always @ (ibus, Imm, S, Cin, loadSelector, branchIns, slInst) begin
                case(ibus[31:26])
                    // ADDI
                    6'b000011:  begin Imm = 1; S = 3'b010; Cin = 0; loadSelector = 2'b00; branchIns = 2'b00; slInst = 2'b00;  end
                    // SUBI
                    6'b000010: begin Imm = 1; S = 3'b011; Cin = 1; loadSelector = 2'b00; branchIns = 2'b00; slInst = 2'b00;  end
                    // XORI
                    6'b000001:  begin Imm = 1; S = 3'b000; Cin = 0; loadSelector = 2'b00; branchIns = 2'b00; slInst = 2'b00;  end
                    // ANDI
                    6'b001111:  begin Imm = 1; S = 3'b110; Cin = 0; loadSelector = 2'b00; branchIns = 2'b00; slInst = 2'b00;  end
                    // ORI
                    6'b001100:  begin Imm = 1; S = 3'b100; Cin = 0; loadSelector = 2'b00; branchIns = 2'b00; slInst = 2'b00;end
                    // STORE WORD
                    6'b011111:  begin Imm = 1; S = 3'b010; Cin = 0; loadSelector = 2'b10; branchIns = 2'b00; slInst = 2'b00;end
                    // LOAD WORD
                    6'b011110:  begin Imm = 1; S = 3'b010; Cin = 0; loadSelector = 2'b11; branchIns = 2'b00; slInst = 2'b00;end 
                    // BEQ
                    6'b110000:  begin Imm = 1; S = 3'b010; Cin = 0; loadSelector = 2'b00; branchIns = 2'b01; slInst = 2'b00;end 
                    // BNE
                    6'b110001:  begin Imm = 1; S = 3'b010; Cin = 0; loadSelector = 2'b00; branchIns = 2'b11; slInst = 2'b00;end 

                    // R format
                    6'b000000: 
                        case(ibus[5:0]) 
                            // add
                            6'b000011: begin Imm = 0; S = 3'b010; Cin = 0; loadSelector = 2'b00; branchIns = 2'b00; slInst = 2'b00;end
                            // sub
                            6'b000010: begin Imm = 0; S = 3'b011; Cin = 1; loadSelector = 2'b00; branchIns = 2'b00; slInst = 2'b00;end
                            // xor
                            6'b000001: begin Imm = 0; S = 3'b000; Cin = 0; loadSelector = 2'b00; branchIns = 2'b00; slInst = 2'b00;end
                            // and
                            6'b000111: begin Imm = 0; S = 3'b110; Cin = 0; loadSelector = 2'b00; branchIns = 2'b00; slInst = 2'b00;end
                            // or
                            6'b000100: begin Imm = 0; S = 3'b100; Cin = 0; loadSelector = 2'b00; branchIns = 2'b00; slInst = 2'b00;end
                            // SLT                                                               
                            6'b110110: begin Imm = 0; S = 3'b011; Cin = 1; loadSelector = 2'b00; branchIns = 2'b00; slInst = 2'b01;end
                            // SLE
                            6'b110111: begin Imm = 0; S = 3'b011; Cin = 1; loadSelector = 2'b00; branchIns = 2'b00; slInst = 2'b10;end
                            // NOP
                            default: begin Imm = 0; S = 3'b010; Cin = 0; loadSelector = 2'b00; branchIns = 2'b00; slInst = 2'b00;end
                        endcase      
                endcase
            end
endmodule
// IFID REGISTER
module ifidReg(ibusIn, clk, ibusOut, iaddrbusIn, iaddrbusOut);
    input [31:0] ibusIn, iaddrbusIn;
    input clk;
    output reg [31:0] ibusOut, iaddrbusOut;
    always @(posedge clk)
    begin
        ibusOut = ibusIn;
        iaddrbusOut = iaddrbusIn;
    end
endmodule
//ADDER
module adder(a, b, d);
    input [31:0] a, b;
    output [31:0] d;
    assign d = a + b;
endmodule
//PC REGISTER
module pcReg(clk, iaddrbusIn, iaddrbusOut, reset);
    input clk, reset;
    input [31:0] iaddrbusIn;
    output reg [31:0] iaddrbusOut;
    initial begin
        iaddrbusOut = 32'b0;
    end
    always @ (posedge clk)  begin
        if(reset) iaddrbusOut = 32'b0;
        else iaddrbusOut = iaddrbusIn;
    end
endmodule

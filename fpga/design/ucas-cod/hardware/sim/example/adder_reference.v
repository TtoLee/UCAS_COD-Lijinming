/* Generated by Yosys 0.12+36 (git sha1 60c3ea367, clang 10.0.0-4ubuntu1 -fPIC -Os) */

(* src = "adder.v:3.1-11.10" *)
module adder_reference(operand0, operand1, result);
  wire _00_;
  wire _01_;
  wire _02_;
  wire _03_;
  wire _04_;
  wire _05_;
  wire _06_;
  wire _07_;
  wire _08_;
  wire _09_;
  wire _10_;
  wire _11_;
  wire _12_;
  wire _13_;
  wire _14_;
  wire _15_;
  wire _16_;
  wire _17_;
  wire _18_;
  wire _19_;
  wire _20_;
  wire _21_;
  wire _22_;
  wire _23_;
  wire _24_;
  wire _25_;
  wire _26_;
  wire _27_;
  wire _28_;
  wire _29_;
  wire _30_;
  wire _31_;
  (* src = "adder.v:4.17-4.25" *)
  input [7:0] operand0;
  (* src = "adder.v:5.17-5.25" *)
  input [7:0] operand1;
  (* src = "adder.v:6.18-6.24" *)
  output [7:0] result;
  assign _00_ = operand1[1] ^ operand0[1];
  assign _01_ = ~(operand1[0] & operand0[0]);
  assign result[1] = ~(_01_ ^ _00_);
  assign _02_ = ~(operand1[2] ^ operand0[2]);
  assign _03_ = operand1[1] & operand0[1];
  assign _04_ = _00_ & ~(_01_);
  assign _05_ = _04_ | _03_;
  assign result[2] = ~(_05_ ^ _02_);
  assign _06_ = ~(operand1[3] ^ operand0[3]);
  assign _07_ = ~(operand1[2] & operand0[2]);
  assign _08_ = _05_ & ~(_02_);
  assign _09_ = _07_ & ~(_08_);
  assign result[3] = _09_ ^ _06_;
  assign _10_ = ~(operand1[4] ^ operand0[4]);
  assign _11_ = ~(operand1[3] & operand0[3]);
  assign _12_ = ~(_07_ | _06_);
  assign _13_ = _11_ & ~(_12_);
  assign _14_ = _06_ | _02_;
  assign _15_ = _05_ & ~(_14_);
  assign _16_ = _13_ & ~(_15_);
  assign result[4] = _16_ ^ _10_;
  assign _17_ = ~(operand1[5] ^ operand0[5]);
  assign _18_ = ~(operand1[4] & operand0[4]);
  assign _19_ = ~(_16_ | _10_);
  assign _20_ = _18_ & ~(_19_);
  assign result[5] = _20_ ^ _17_;
  assign _21_ = ~(operand1[6] ^ operand0[6]);
  assign _22_ = ~(operand1[5] & operand0[5]);
  assign _23_ = ~(_18_ | _17_);
  assign _24_ = _22_ & ~(_23_);
  assign _25_ = _17_ | _10_;
  assign _26_ = ~(_25_ | _16_);
  assign _27_ = _24_ & ~(_26_);
  assign result[6] = _27_ ^ _21_;
  assign _28_ = ~(operand1[7] ^ operand0[7]);
  assign _29_ = ~(operand1[6] & operand0[6]);
  assign _30_ = ~(_27_ | _21_);
  assign _31_ = _29_ & ~(_30_);
  assign result[7] = _31_ ^ _28_;
  assign result[0] = operand1[0] ^ operand0[0];
endmodule

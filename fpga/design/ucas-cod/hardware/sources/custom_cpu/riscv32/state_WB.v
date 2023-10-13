`timescale 10ns / 1ns

module state_WB(
    input               clk,
    input               rst,

    input       [31:0]  PC_input,       //输入PC值(待退役)
    input               complete_pre,   //前序阶段完成信号
    input       [ 4:0]  RF_waddr,       //寄存器写地址
    input       [31:0]  RF_wdata,       //寄存器写数据

    output              RF_wen,         //寄存器写使能
    output  [69:0]  inst_retire     //退役相关信号
);
    reg tag;

    assign RF_wen = (complete_pre & (|RF_waddr));
    assign inst_retire = (complete_pre)? {RF_wen, RF_waddr, RF_wdata, PC_input} : 70'b0;

    always @ (posedge clk) begin
        if (rst) tag <= 1'b0;
        else if (complete_pre)
            tag <= 1'b1;
        else tag <= 1'b0;
    end
                                    
endmodule
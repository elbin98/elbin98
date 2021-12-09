/*
* conv_module.v
*/

module conv_module 
  #(
    parameter integer C_S00_AXIS_TDATA_WIDTH = 32
  )
  (
    input wire clk,
    input wire rstn,

    output wire S_AXIS_TREADY,
    input wire [C_S00_AXIS_TDATA_WIDTH-1 : 0] S_AXIS_TDATA,
    input wire [(C_S00_AXIS_TDATA_WIDTH/8)-1 : 0] S_AXIS_TKEEP, 
    input wire S_AXIS_TUSER, 
    input wire S_AXIS_TLAST, 
    input wire S_AXIS_TVALID, 

    input wire M_AXIS_TREADY, 
    output wire M_AXIS_TUSER, 
    output wire [C_S00_AXIS_TDATA_WIDTH-1 : 0] M_AXIS_TDATA, 
    output wire [(C_S00_AXIS_TDATA_WIDTH/8)-1 : 0] M_AXIS_TKEEP, 
    output wire M_AXIS_TLAST, 
    output wire M_AXIS_TVALID, 

    input conv_start, 
    output reg conv_done,
    
    //////////////////////////////////////////////////////////////////////////
    // TODO : Add ports if you need them
    //////////////////////////////////////////////////////////////////////////
    input [2:0] COMMAND,    // IDLE : 3'b000  //    Feature receive, 3'b001  // Bias receive :   3'b010 // Calculation start  3'b011 // Data Transmit 3'b100 //
    input [8:0] InCh,        // Number of Input Channel -> Maximum : 256
    input [8:0] OutCh,       // Number of Output Channel -> Maximum : 256
    input [5:0] FLengthm,
    output reg F_writedone,
    output reg B_writedone,
    output reg rdy_to_transmit,
    output reg transmit_done,
    input  F_writedone_respond,
    input  B_writedone_respond,
    input  rdy_to_transmit_respond,
    input  transmit_done_respond
  );
  
  localparam STATE_IDLE = 4'd0,
  STATE_RECEIVE_FEATURE = 4'd1,
  STATE_RECEIVE_BIAS = 4'd2,
  STATE_READ_BIAS = 4'd3,
  STATE_RECEIVE_WEIGHT = 4'd4,
  STATE_READ_FEATURE = 4'd5,
  STATE_COMPUTE = 4'd6,
  STATE_ADD_BIAS = 4'd7,
  STATE_WRITE_RESULT = 4'd8,
  STATE_SEND_RESULT = 4'd9;
  
  reg [3:0] state;
  
  reg                                           m_axis_tuser;
  reg [C_S00_AXIS_TDATA_WIDTH-1 : 0]            m_axis_tdata;
  reg [(C_S00_AXIS_TDATA_WIDTH/8)-1 : 0]        m_axis_tkeep;
  reg                                           m_axis_tlast;
  reg                                           m_axis_tvalid;
  reg                                          s_axis_tready;
  
  assign S_AXIS_TREADY = s_axis_tready;
  assign M_AXIS_TDATA = m_axis_tdata;
  assign M_AXIS_TLAST = m_axis_tlast;
  assign M_AXIS_TVALID = m_axis_tvalid;
  assign M_AXIS_TUSER = 1'b0;
  assign M_AXIS_TKEEP = {(C_S00_AXIS_TDATA_WIDTH/8) {1'b1}};
  
  reg f_bram_en;
  reg [10:0] f_addr;
  reg f_we;  
  wire [31:0] f_din;
  wire [31:0] f_dout;
  assign f_din = S_AXIS_TDATA;
  sram_32x2048 feature_sram_32x2048(
    .addra(f_addr),
    .clka(clk),
    .dina(f_din),
    .douta(f_dout),
    .ena(f_bram_en),
    .wea(f_we)
  );
  
  reg b_bram_en;
  reg [5:0] b_addr;
  reg b_we;  
  wire [31:0] b_din;
  wire [31:0] b_dout;  
  assign b_din = S_AXIS_TDATA;
  sram_32x64 bias_sram_32x64(
    .addra(b_addr),
    .clka(clk),
    .dina(b_din),
    .douta(b_dout),
    .ena(b_bram_en),
    .wea(b_we)
  ); 
  
  reg o_bram_en;
  wire [7:0] o_addr;
  reg o_we;  
  wire [31:0] o_din;
  wire [31:0] o_dout;
  
  sram_32x256 output_sram_32x256(
    .addra(o_addr),
    .clka(clk),
    .dina(o_din),
    .douta(o_dout),
    .ena(o_bram_en),
    .wea(o_we)
  );  
  
  reg  pe_en[15:0];
  reg [7:0] data_a[15:0];
  reg [7:0] data_b[15:0];
  reg [26:0] data_c[15:0];
  reg [7:0] data_in;
  wire [26:0] partial_sum[15:0];
  wire pe_done[15:0];
  wire [26:0] result[15:0];
  reg [5:0] i1, i2, i3, i4, i5, i6, i7;
  
  genvar i;
  generate
    for(i=0;i<16;i=i+1) begin: u_pe_module
        if (i==0) begin
            PE u_pe(
              .CLK(clk),
              .EN(pe_en[i]),
              .RSTn(rstn),
              .DATA_A(data_a[i]), 
              .DATA_B(data_b[i]),
              .DATA_C(data_c[i]),
              .DATA_OUT_A(),
              .MOUT(result[i]),
              .DONE(pe_done[i])
            );
        end
        else if (i==15) begin
            PE u_pe(
              .CLK(clk),
              .EN(pe_en[i]),
              .RSTn(rstn),
              .DATA_A(data_a[i]), 
              .DATA_B(data_b[i]),
              .DATA_C(data_c[i]),
              .DATA_OUT_A(),
              .MOUT(result[i]),
              .DONE(pe_done[i])
            );
        end
        else begin
            PE u_pe(
              .CLK(clk),
              .EN(pe_en[i]),
              .RSTn(rstn),
              .DATA_A(data_a[i]), 
              .DATA_B(data_b[i]),
              .DATA_C(data_c[i]),
              .DATA_OUT_A(),
              .MOUT(result[i]),
              .DONE(pe_done[i])
            );
        end
    end
  endgenerate
  
  reg [7:0] weight_num; // 몇번째 weigth에서 연산중인지
  wire [8:0] kernel_num;  //몇번째 kernel에서 연산중인지
  reg [1:0] bias_num1; //remainder of kernel_num
  reg [6:0] bias_num2; //quotient of kernel_num
  reg [8:0] InCh_num; //몇번째 깊이(input channel)에서 연산중인지
  reg [7:0] bias;   //bias 값 1개 저장
  reg [7:0] weight_nine [8:0];
  reg [7:0] weight_remain [2:0];
  reg [7:0] zero_padding [33:0] [33:0];
  reg [26:0] temp_sum [1023:0];
  reg compute_done;
  reg [10:0]read_feature_cnt;    //feature의 몇번째 원소를 zero padding 하고 있는지
  reg [3:0] nine_cnt;     //pe에 몇번째 원소가 들어가고 있는지

  reg [10:0]write_cnt;
  wire[7:0] q_result1, q_result2, q_result3, q_result4;
  reg [7:0] send_cnt;
  assign partial_sum[0] = result[0];
  assign partial_sum[1] = result[1];
  assign partial_sum[2] = result[2];
  assign partial_sum[3] = result[3];
  assign partial_sum[4] = result[4];
  assign partial_sum[5] = result[5];
  assign partial_sum[6] = result[6];
  assign partial_sum[7] = result[7];
  assign partial_sum[8] = result[8];
  assign partial_sum[9] = result[9];
  assign partial_sum[10] = result[10];
  assign partial_sum[11] = result[11];
  assign partial_sum[12] = result[12];
  assign partial_sum[13] = result[13];
  assign partial_sum[14] = result[14];
  assign partial_sum[15] = result[15];
  
  assign kernel_num = 4 * bias_num2 + bias_num1;
  assign o_din = {q_result1, q_result2, q_result3, q_result4};
  assign o_addr = (state==STATE_WRITE_RESULT) ? (kernel_num*FLengthm*FLengthm/4 + write_cnt) : (send_cnt);
  assign q_result1 = (temp_sum[4*write_cnt][26]) ? 0 : ((temp_sum[4*write_cnt][25:13] == 13'b0000000000000) ? {1'b0, temp_sum[4*write_cnt][12:6]} : 8'b01111111);
  assign q_result2 = (temp_sum[4*write_cnt+1][26]) ? 0 : ((temp_sum[4*write_cnt+1][25:13] == 13'b0000000000000) ? {1'b0, temp_sum[4*write_cnt+1][12:6]} : 8'b01111111);
  assign q_result3 = (temp_sum[4*write_cnt+2][26]) ? 0 : ((temp_sum[4*write_cnt+2][25:13] == 13'b0000000000000) ? {1'b0, temp_sum[4*write_cnt+2][12:6]} : 8'b01111111);
  assign q_result4 = (temp_sum[4*write_cnt+3][26]) ? 0 : ((temp_sum[4*write_cnt+3][25:13] == 13'b0000000000000) ? {1'b0, temp_sum[4*write_cnt+3][12:6]} : 8'b01111111);
  
  ////////////////////////////////////////////////////////////////////////////
  // TODO : Write your code here
  ////////////////////////////////////////////////////////////////////////////
  //state control
  always @(posedge clk or negedge rstn) begin
    if(!rstn) begin
        state <= STATE_IDLE;
    end
    
    else begin
        case(state)
            STATE_IDLE:begin
                //if(conv_start)begin
                    
                    if(COMMAND == 1) begin
                        state <= STATE_RECEIVE_FEATURE;
                        pe_en[0] <= 1;
                        weight_num <= 0;
                        bias_num1 <= 0;
                        bias_num2 <= 0;
                        InCh_num <= 0;
                        bias <= 0;
                        compute_done <= 0;
                        read_feature_cnt <= 0;
                        nine_cnt <= 0;
                        o_we <= 0;
                        o_bram_en <= 0;
                        write_cnt <= 0;
                        send_cnt <= 0;
                        for(i6=0;i6<34;i6=i6+1)begin
                            zero_padding[0][i6] <= 8'b0;
                            zero_padding[i6][0] <= 8'b0;
                        end
                        zero_padding[FLengthm+1][FLengthm+1] <= 0;
                        //zero_padding, weight_remain 초기화
                    end
                    
                //end
            end
            STATE_RECEIVE_FEATURE:begin
                if(COMMAND==2) begin
                    state <= STATE_RECEIVE_BIAS;
                end
                if(S_AXIS_TLAST && S_AXIS_TVALID) begin
                    //state <= STATE_RECEIVE_BIAS;
                    F_writedone <= 1;
                end
            end
            STATE_RECEIVE_BIAS:begin
                if(COMMAND==3) begin
                    state <= STATE_READ_BIAS;
                end
                if(S_AXIS_TLAST && S_AXIS_TVALID) begin
                    //state <= STATE_READ_BIAS;
                    B_writedone <= 1;
                end
            end
            STATE_READ_BIAS:begin
                state <= STATE_RECEIVE_WEIGHT;
                s_axis_tready <= 1;
            end
            
            STATE_RECEIVE_WEIGHT:begin
                if(weight_num >= 5) begin
                    state <= STATE_READ_FEATURE;
                    weight_num <= weight_num - 5;
                    //InCh_num <= InCh_num+1;
                    s_axis_tready <= 0;
                end
                else begin
                    weight_num <= weight_num + 4;
                end
            end
            
            STATE_READ_FEATURE:begin
                if(read_feature_cnt == FLengthm * FLengthm - 4) begin
                    f_addr <= f_addr + 1;
                    state <= STATE_COMPUTE;
                    read_feature_cnt <= 0;
                end
                else begin
                    read_feature_cnt <= read_feature_cnt + 4;
                    f_addr <= f_addr + 1;
                end
            end
            
            STATE_COMPUTE:begin
                if(read_feature_cnt > FLengthm * FLengthm - 16) begin
                    if(InCh_num == InCh - 1) begin
                        //data_in <= 0;
                        for(i1 = 0; i1 < 16; i1 = i1 + 1) begin
                            data_a[i1] <= 0;
                            data_b[i1] <= 0;
                        end
                        InCh_num <= 0;
                        f_addr <= 0;
                        state <= STATE_ADD_BIAS;
                    end
                    else begin
                        InCh_num <= InCh_num + 1;
                        state <= STATE_RECEIVE_WEIGHT;
                        s_axis_tready <= 1; //추가
                    end
                    read_feature_cnt <= 0;
                end
                else begin
                    if(nine_cnt == 8) begin
                        nine_cnt <= 0;
                        read_feature_cnt <= read_feature_cnt + 16;
                    end
                    else begin
                        nine_cnt <= nine_cnt + 1;
                    end
                end
            end
            
            STATE_ADD_BIAS:begin
                if(read_feature_cnt == FLengthm*FLengthm) begin
                    state <= STATE_WRITE_RESULT;
                    o_bram_en <= 1;
                    o_we <= 1;
                    read_feature_cnt <= 0;
                end
                else begin
                    //data_in <= bias;
                    for(i2 = 0; i2 < 16; i2 = i2 + 1) begin
                        data_a[i2] <= bias;
                        data_b[i2] <= 8'b01000000;
                        temp_sum[read_feature_cnt+i2] <= result[i2];
                    end
                    
                    data_c[0] <= temp_sum[read_feature_cnt];
                    data_c[1] <= temp_sum[read_feature_cnt+1];
                    data_c[2] <= temp_sum[read_feature_cnt+2];
                    data_c[3] <= temp_sum[read_feature_cnt+3];
                    data_c[4] <= temp_sum[read_feature_cnt+4];
                    data_c[5] <= temp_sum[read_feature_cnt+5];
                    data_c[6] <= temp_sum[read_feature_cnt+6];
                    data_c[7] <= temp_sum[read_feature_cnt+7];
                    data_c[8] <= temp_sum[read_feature_cnt+8];
                    data_c[9] <= temp_sum[read_feature_cnt+9];
                    data_c[10] <= temp_sum[read_feature_cnt+10];
                    data_c[11] <= temp_sum[read_feature_cnt+11];
                    data_c[12] <= temp_sum[read_feature_cnt+12];
                    data_c[13] <= temp_sum[read_feature_cnt+13];
                    data_c[14] <= temp_sum[read_feature_cnt+14];
                    data_c[15] <= temp_sum[read_feature_cnt+15];
                    read_feature_cnt <= read_feature_cnt + 16;
                    
                end
            end
            
            STATE_WRITE_RESULT:begin
                
              
            end
            
            STATE_SEND_RESULT:begin
                if(conv_done==1 && COMMAND==0) state <= STATE_IDLE;
            end
            
        endcase
    end
  end
  
  //datapath
  always @(posedge clk or negedge rstn) begin
    if(!rstn) begin
        s_axis_tready <= 0;
    end
    
    else begin
        case(state)
            STATE_IDLE:begin
                if(COMMAND==1) begin
                    s_axis_tready <= 1;
                    f_addr <= 0;
                    f_we <= 1;
                    f_bram_en <= 1; 
                end
            end
            STATE_RECEIVE_FEATURE:begin
                if(COMMAND ==2) begin
                    s_axis_tready <= 1;
                    b_addr <= 0;
                    b_bram_en <= 1;
                    b_we <= 1;
                end
                else if(S_AXIS_TVALID == 0) begin
                    f_we <= 0;
                    f_addr <= 0;
                end
                else if(S_AXIS_TLAST && S_AXIS_TVALID) begin
                    f_addr <= f_addr + 1;
                    s_axis_tready <= 0;
                end
                else if(S_AXIS_TVALID&&s_axis_tready) begin
                        f_addr <= f_addr + 1;
                end
            end
            STATE_RECEIVE_BIAS:begin
                if(S_AXIS_TLAST && S_AXIS_TVALID) begin
                    b_addr <= 0;
                    b_we <= 0;
                    s_axis_tready <= 0;
                    
                end
                else if(S_AXIS_TVALID&&s_axis_tready) begin
                    b_addr <= b_addr + 1;
                end
            end
            STATE_READ_BIAS:begin
               s_axis_tready <= 1;
               case(bias_num1)
                  0:begin
                    bias <= b_dout[31:24];
                  end
                  1:begin
                    bias <= b_dout[23:16];
                  end
                  2:begin
                    bias <= b_dout[15:8];
                  end
                  3:begin
                    bias <= b_dout[7:0];
                  end
               endcase
               
            end
            
            STATE_RECEIVE_WEIGHT:begin
                case(weight_num)
                    0:begin
                        weight_nine[0] <= S_AXIS_TDATA[31:24];
                        weight_nine[1] <= S_AXIS_TDATA[23:16];
                        weight_nine[2] <= S_AXIS_TDATA[15:8];
                        weight_nine[3] <= S_AXIS_TDATA[7:0];
                    end
                    1:begin
                        weight_nine[0] <= weight_remain[0];
                        weight_nine[1] <= S_AXIS_TDATA[31:24];
                        weight_nine[2] <= S_AXIS_TDATA[23:16];
                        weight_nine[3] <= S_AXIS_TDATA[15:8];
                        weight_nine[4] <= S_AXIS_TDATA[7:0];
                    end
                    2:begin
                        weight_nine[0] <= weight_remain[0];
                        weight_nine[1] <= weight_remain[1];
                        weight_nine[2] <= S_AXIS_TDATA[31:24];
                        weight_nine[3] <= S_AXIS_TDATA[23:16];
                        weight_nine[4] <= S_AXIS_TDATA[15:8];
                        weight_nine[5] <= S_AXIS_TDATA[7:0];
                    end
                    3:begin
                        weight_nine[0] <= weight_remain[0];
                        weight_nine[1] <= weight_remain[1];
                        weight_nine[2] <= weight_remain[2];
                        weight_nine[3] <= S_AXIS_TDATA[31:24];
                        weight_nine[4] <= S_AXIS_TDATA[23:16];
                        weight_nine[5] <= S_AXIS_TDATA[15:8];
                        weight_nine[6] <= S_AXIS_TDATA[7:0];
                    end
                    4:begin
                        weight_nine[4] <= S_AXIS_TDATA[31:24];
                        weight_nine[5] <= S_AXIS_TDATA[23:16];
                        weight_nine[6] <= S_AXIS_TDATA[15:8];
                        weight_nine[7] <= S_AXIS_TDATA[7:0];
                    end
                    5:begin
                        weight_nine[5] <= S_AXIS_TDATA[31:24];
                        weight_nine[6] <= S_AXIS_TDATA[23:16];
                        weight_nine[7] <= S_AXIS_TDATA[15:8];
                        weight_nine[8] <= S_AXIS_TDATA[7:0];
                    end
                    6:begin
                        weight_nine[6] <= S_AXIS_TDATA[31:24];
                        weight_nine[7] <= S_AXIS_TDATA[23:16];
                        weight_nine[8] <= S_AXIS_TDATA[15:8];
                        weight_remain[0] <= S_AXIS_TDATA[7:0];
                    end
                    7:begin
                        weight_nine[7] <= S_AXIS_TDATA[31:24];
                        weight_nine[8] <= S_AXIS_TDATA[23:16];
                        weight_remain[0] <= S_AXIS_TDATA[15:8];
                        weight_remain[1] <= S_AXIS_TDATA[7:0];
                    end
                    8:begin
                        weight_nine[8] <= S_AXIS_TDATA[31:24];
                        weight_remain[0] <= S_AXIS_TDATA[23:16];
                        weight_remain[1] <= S_AXIS_TDATA[15:8];
                        weight_remain[2] <= S_AXIS_TDATA[7:0];
                    end
                endcase
            end
            
            STATE_READ_FEATURE:begin
                zero_padding[(read_feature_cnt)/FLengthm+1][(read_feature_cnt)%FLengthm+1] <= f_dout[31:24];
                zero_padding[(read_feature_cnt)/FLengthm+1][(read_feature_cnt)%FLengthm+2] <= f_dout[23:16];
                zero_padding[(read_feature_cnt)/FLengthm+1][(read_feature_cnt)%FLengthm+3] <= f_dout[15:8];
                zero_padding[(read_feature_cnt)/FLengthm+1][(read_feature_cnt)%FLengthm+4] <= f_dout[7:0];
                
                if(read_feature_cnt>=FLengthm*FLengthm-FLengthm) begin
                    zero_padding[FLengthm+1][(read_feature_cnt)%FLengthm+1] <=0;
                    zero_padding[FLengthm+1][(read_feature_cnt)%FLengthm+2] <=0;
                    zero_padding[FLengthm+1][(read_feature_cnt)%FLengthm+3] <=0;
                    zero_padding[FLengthm+1][(read_feature_cnt)%FLengthm+4] <=0;
                end
                
                if((read_feature_cnt%FLengthm)==FLengthm-4) begin
                    zero_padding[(read_feature_cnt)/FLengthm+1][FLengthm+1] <= 0;
                end
            end
            
            STATE_COMPUTE:begin
                //for(i3 = 0; i3 < 16; i3 = i3 + 1) begin
                //    partial_sum[i3] <= result[i3];
                //end
                if(InCh_num == InCh - 1) begin
                
                end
                
                else begin
            
                    if(nine_cnt == 8) begin
                        for(i4 = 0; i1 < 16; i4 = i4 + 1) begin
                            temp_sum[read_feature_cnt+i4] <= temp_sum[read_feature_cnt+i4] + result[i4];
                        end
                    end
                    //data_in <= weight_nine[nine_cnt];
                    for(i5 = 0; i5 < 16; i5 = i5 + 1) begin
                        case(nine_cnt)
                            0:begin
                                data_a[i5] <= weight_nine[nine_cnt];
                                data_b[i5] <= zero_padding[(read_feature_cnt+i5)/FLengthm][(read_feature_cnt+i5)%FLengthm];
                                data_c[i5] <= 0;
                            end
                            1:begin
                                data_a[i5] <= weight_nine[nine_cnt];
                                data_b[i5] <= zero_padding[(read_feature_cnt+i5)/FLengthm][(read_feature_cnt+i5)%FLengthm+1];
                                data_c[i5] <= partial_sum[i5];
                            end
                            2:begin
                                data_a[i5] <= weight_nine[nine_cnt];
                                data_b[i5] <= zero_padding[(read_feature_cnt+i5)/FLengthm][(read_feature_cnt+i5)%FLengthm+2];
                                data_c[i5] <= partial_sum[i5];
                            end
                            3:begin
                                data_a[i5] <= weight_nine[nine_cnt];
                                data_b[i5] <= zero_padding[(read_feature_cnt+i5)/FLengthm+1][(read_feature_cnt+i5)%FLengthm];
                                data_c[i5] <= partial_sum[i5];
                            end
                            4:begin
                                data_a[i5] <= weight_nine[nine_cnt];
                                data_b[i5] <= zero_padding[(read_feature_cnt+i5)/FLengthm+1][(read_feature_cnt+i5)%FLengthm+1];
                                data_c[i5] <= partial_sum[i5];
                            end
                            5:begin
                                data_a[i5] <= weight_nine[nine_cnt];
                                data_b[i5] <= zero_padding[(read_feature_cnt+i5)/FLengthm+1][(read_feature_cnt+i5)%FLengthm+2];
                                data_c[i5] <= partial_sum[i5];
                            end
                            6:begin
                                data_a[i5] <= weight_nine[nine_cnt];
                                data_b[i5] <= zero_padding[(read_feature_cnt+i5)/FLengthm+2][(read_feature_cnt+i5)%FLengthm];
                                data_c[i5] <= partial_sum[i5];
                            end
                            7:begin
                                data_a[i5] <= weight_nine[nine_cnt];
                                data_b[i5] <= zero_padding[(read_feature_cnt+i5)/FLengthm+2][(read_feature_cnt+i5)%FLengthm+1];
                                data_c[i5] <= partial_sum[i5];
                            end
                            8:begin
                                data_a[i5] <= weight_nine[nine_cnt];
                                data_b[i5] <= zero_padding[(read_feature_cnt+i5)/FLengthm+2][(read_feature_cnt+i5)%FLengthm+2];
                                data_c[i5] <= partial_sum[i5];
                            end
                        endcase
                        //data_c[i5] <= partial_sum[i5];
                    end
                
                end
            end
            
            STATE_WRITE_RESULT:begin
                if((write_cnt + 1) * 4 == FLengthm * FLengthm) begin
                    if(kernel_num == OutCh - 1) begin
                        rdy_to_transmit <= 1;
                        if(COMMAND == 4) begin
                            state <= STATE_SEND_RESULT;
                            o_bram_en <= 1;
                            m_axis_tvalid <= 1;
                            send_cnt <= 0;
                            write_cnt <= 0;
                            bias_num1 <= 0;
                            bias_num2 <= 0;
                        end
                    end
                    else begin
                        state <= STATE_READ_BIAS;
                        if(bias_num1 == 3) begin
                            bias_num1 <= 0;
                            bias_num2 <= bias_num2 + 1;
                            b_addr <= b_addr + 1;
                        end
                        else begin
                            bias_num1 <= bias_num1 + 1;
                        end
                        write_cnt <= 0;
                    end
                    o_we <= 0;
                end
                else begin
                    write_cnt <= write_cnt + 1;
                end
            end
            
            STATE_SEND_RESULT:begin
               if(send_cnt == FLengthm*FLengthm*OutCh/4-1) begin
                   m_axis_tlast <= 1;
                   transmit_done <= 1;
                   state <= STATE_IDLE;
               end
               else begin
                   send_cnt <= send_cnt + 1;
                   
               end
               m_axis_tdata <= o_dout;
            end
            
        endcase
    end
  end
endmodule

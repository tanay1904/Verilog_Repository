`timescale 1ns / 1ps

module double_Integrator #(
    parameter integer WIDTH    = 16,  // sfix16_En10
    parameter integer DT_WIDTH = 12,  // ufix12_En12
    parameter integer FRAC_DT  = 12   // dt fractional bits
)(
    input                       clk,
    input                       reset,
    input                       clk_enable,

    input      [DT_WIDTH-1:0]   dt,   // unsigned fixed-point, ufix12_En12
    input signed [WIDTH-1:0]    u,    // acceleration, sfix16_En10
    output reg signed [WIDTH-1:0] y   // position, sfix16_En10
);

    // =========================================================
    // Widths
    // =========================================================

    localparam integer VEL_PROD_WIDTH = WIDTH + DT_WIDTH;
    localparam integer DT2_WIDTH      = 2 * DT_WIDTH;
    localparam integer ACC_PROD_WIDTH = WIDTH + DT2_WIDTH;
    localparam integer POS_SUM_WIDTH  = WIDTH + 2;

    // =========================================================
    // Internal states
    // =========================================================

    reg signed [WIDTH-1:0] v_reg;
    reg signed [WIDTH-1:0] x_reg;

    // =========================================================
    // 1) Velocity integration
    //
    // v(k+1) = v(k) + u(k)*dt
    //
    // u      : sfix16_En10
    // dt     : ufix12_En12
    // u*dt   : sfix28_En22
    // >>> 12 : sfix28_En10
    // =========================================================

    wire signed [VEL_PROD_WIDTH-1:0] u_ext_v;
    wire signed [VEL_PROD_WIDTH-1:0] dt_ext_v;

    assign u_ext_v  = {{(VEL_PROD_WIDTH-WIDTH){u[WIDTH-1]}}, u};
    assign dt_ext_v = {{(VEL_PROD_WIDTH-DT_WIDTH-1){1'b0}}, 1'b0, dt};

    wire signed [VEL_PROD_WIDTH-1:0] prod_v;

    assign prod_v = u_ext_v * dt_ext_v;

    // Round before converting En22 -> En10 (add half-LSB bias: 2^(FRAC_DT-1) = 2^11 = 2048)
    wire signed [VEL_PROD_WIDTH-1:0] round_bias_v;
    wire signed [VEL_PROD_WIDTH-1:0] prod_v_round;
    wire signed [VEL_PROD_WIDTH-1:0] dv_full;

    assign round_bias_v =
        $signed({{(VEL_PROD_WIDTH-1){1'b0}}, 1'b1})
        <<< (FRAC_DT - 1);

    assign prod_v_round = prod_v + round_bias_v;

    assign dv_full = prod_v_round >>> FRAC_DT;

    // Keep the original WIDTH-bit state behavior
    wire signed [WIDTH:0] v_sum_full;

    assign v_sum_full =
        {v_reg[WIDTH-1], v_reg} +
        {{(WIDTH+1-WIDTH){dv_full[WIDTH-1]}},
         dv_full[WIDTH-1:0]};

    wire signed [WIDTH-1:0] v_next;

    assign v_next = v_sum_full[WIDTH-1:0];

    // =========================================================
    // 2) Position integration
    //
    // Correct constant-acceleration equation:
    //
    // x(k+1) = x(k)
    //        + v(k)*dt
    //        + 0.5*u(k)*dt^2
    //
    // IMPORTANT:
    //      uses v_reg (old velocity), NOT v_next.
    // =========================================================

    // ---------------------------------------------------------
    // v(k) * dt
    //
    // v       : sfix16_En10
    // dt      : ufix12_En12
    // product : sfix28_En22
    // >>> 12  : sfix28_En10
    // ---------------------------------------------------------

    wire signed [VEL_PROD_WIDTH-1:0] v_ext_x;
    wire signed [VEL_PROD_WIDTH-1:0] dt_ext_x;
    wire signed [VEL_PROD_WIDTH-1:0] prod_x_vel;

    assign v_ext_x  = {{(VEL_PROD_WIDTH-WIDTH){v_reg[WIDTH-1]}}, v_reg};
    assign dt_ext_x = {{(VEL_PROD_WIDTH-DT_WIDTH-1){1'b0}}, 1'b0, dt};

    assign prod_x_vel = v_ext_x * dt_ext_x;

    wire signed [VEL_PROD_WIDTH-1:0] round_bias_x_vel;
    wire signed [VEL_PROD_WIDTH-1:0] prod_x_vel_round;
    wire signed [VEL_PROD_WIDTH-1:0] dx_vel_full;

    assign round_bias_x_vel =
        $signed({{(VEL_PROD_WIDTH-1){1'b0}}, 1'b1})
        <<< (FRAC_DT - 1);

    assign prod_x_vel_round = prod_x_vel + round_bias_x_vel;

    assign dx_vel_full = prod_x_vel_round >>> FRAC_DT;

    // ---------------------------------------------------------
    // u * dt^2
    //
    // dt      : ufix12_En12
    // dt^2    : ufix24_En24
    //
    // u       : sfix16_En10
    // u*dt^2  : sfix40_En34
    //
    // We need:
    //
    //     0.5*u*dt^2
    //
    // Result required: En10
    //
    // En34 -> En10 requires 24 shifts (2 * FRAC_DT).
    // Additional /2 factor requires one more shift (+ 1).
    //
    // Total shift = 2 * FRAC_DT + 1 = 25.
    // ---------------------------------------------------------

    wire [DT2_WIDTH-1:0] dt_squared;

    assign dt_squared = dt * dt;

    wire signed [ACC_PROD_WIDTH-1:0] u_ext_acc;
    wire signed [ACC_PROD_WIDTH-1:0] dt2_ext_acc;
    wire signed [ACC_PROD_WIDTH-1:0] prod_x_acc;

    assign u_ext_acc =
        {{(ACC_PROD_WIDTH-WIDTH){u[WIDTH-1]}}, u};

    assign dt2_ext_acc =
        {{(ACC_PROD_WIDTH-DT2_WIDTH){1'b0}}, dt_squared};

    assign prod_x_acc = u_ext_acc * dt2_ext_acc;

    // Total fixed-point shift:
    //   34 fractional bits -> 10 fractional bits = 24
    //   factor 0.5 = one additional bit (+1)
    //   total = 25
    localparam integer ACC_SHIFT = (2 * FRAC_DT) + 1;

    wire signed [ACC_PROD_WIDTH-1:0] round_bias_x_acc;
    wire signed [ACC_PROD_WIDTH-1:0] prod_x_acc_round;
    wire signed [ACC_PROD_WIDTH-1:0] dx_acc_full;

    assign round_bias_x_acc =
        $signed({{(ACC_PROD_WIDTH-1){1'b0}}, 1'b1})
        <<< (ACC_SHIFT - 1);

    assign prod_x_acc_round = prod_x_acc + round_bias_x_acc;

    assign dx_acc_full = prod_x_acc_round >>> ACC_SHIFT;

    // =========================================================
    // Convert both position increments to WIDTH-bit En10
    // =========================================================

    wire signed [WIDTH-1:0] dx_vel;
    wire signed [WIDTH-1:0] dx_acc;

    assign dx_vel = dx_vel_full[WIDTH-1:0];
    assign dx_acc = dx_acc_full[WIDTH-1:0];

    // =========================================================
    // Position update
    //
    // x(k+1) = x(k) + dx_vel + dx_acc
    //
    // Use extra bits for the addition, then retain WIDTH bits.
    // =========================================================

    wire signed [POS_SUM_WIDTH-1:0] x_sum_full;

    assign x_sum_full =
        {{(POS_SUM_WIDTH-WIDTH){x_reg[WIDTH-1]}}, x_reg} +
        {{(POS_SUM_WIDTH-WIDTH){dx_vel[WIDTH-1]}}, dx_vel} +
        {{(POS_SUM_WIDTH-WIDTH){dx_acc[WIDTH-1]}}, dx_acc};

    wire signed [WIDTH-1:0] x_next;

    assign x_next = x_sum_full[WIDTH-1:0];

    // =========================================================
    // State update
    //
    // Both states update on the same enabled clock edge.
    // Because these are nonblocking assignments, RHS values
    // are evaluated from the OLD states:
    //
    //   v_next uses v_reg
    //   x_next uses v_reg and x_reg
    //
    // This is exactly what we want.
    // =========================================================

    always @(posedge clk) begin
        if (reset) begin
            v_reg <= 'b0;
            x_reg <= 'b0;
            y     <= 'b0;
        end
        else if (clk_enable) begin
            v_reg <= v_next;
            x_reg <= x_next;
            y     <= x_next;
        end
    end

endmodule
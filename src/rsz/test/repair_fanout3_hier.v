/*
 
 Hierarchical repeater case

 Exercise the case when we want to expose the primary output
 net.
 
 */


module hi_fanout (clk1, net0);
   input clk1;
   output net0;

   DFF_X1 drvr (.CK(clk1),
		.Q(net0));
   
   hi_fanout_child hi_fanout_inst1(net0,clk1);
   hi_fanout_child hi_fanout_inst2(net0,clk1);   
endmodule


module hi_fanout_child (net0,clk1);
   input net0; //34 loads
   input clk1; //34 loads

 DFF_X1 load0 (.D(net0),
    .CK(clk1));
 DFF_X1 load1 (.D(net0),
    .CK(clk1));
 DFF_X1 load2 (.D(net0),
    .CK(clk1));
 DFF_X1 load3 (.D(net0),
    .CK(clk1));
 DFF_X1 load4 (.D(net0),
    .CK(clk1));
 DFF_X1 load5 (.D(net0),
    .CK(clk1));
 DFF_X1 load6 (.D(net0),
    .CK(clk1));
 DFF_X1 load7 (.D(net0),
    .CK(clk1));
 DFF_X1 load8 (.D(net0),
    .CK(clk1));
 DFF_X1 load9 (.D(net0),
    .CK(clk1));
 DFF_X1 load10 (.D(net0),
    .CK(clk1));
 DFF_X1 load11 (.D(net0),
    .CK(clk1));
 DFF_X1 load12 (.D(net0),
    .CK(clk1));
 DFF_X1 load13 (.D(net0),
    .CK(clk1));
 DFF_X1 load14 (.D(net0),
    .CK(clk1));
 DFF_X1 load15 (.D(net0),
    .CK(clk1));
 DFF_X1 load16 (.D(net0),
    .CK(clk1));
 DFF_X1 load17 (.D(net0),
    .CK(clk1));
 DFF_X1 load18 (.D(net0),
    .CK(clk1));
 DFF_X1 load19 (.D(net0),
    .CK(clk1));
 DFF_X1 load20 (.D(net0),
    .CK(clk1));
 DFF_X1 load21 (.D(net0),
    .CK(clk1));
 DFF_X1 load22 (.D(net0),
    .CK(clk1));
 DFF_X1 load23 (.D(net0),
    .CK(clk1));
 DFF_X1 load24 (.D(net0),
    .CK(clk1));
 DFF_X1 load25 (.D(net0),
    .CK(clk1));
 DFF_X1 load26 (.D(net0),
    .CK(clk1));
 DFF_X1 load27 (.D(net0),
    .CK(clk1));
 DFF_X1 load28 (.D(net0),
    .CK(clk1));
 DFF_X1 load29 (.D(net0),
    .CK(clk1));
 DFF_X1 load30 (.D(net0),
    .CK(clk1));
 DFF_X1 load31 (.D(net0),
    .CK(clk1));
 DFF_X1 load32 (.D(net0),
    .CK(clk1));
 DFF_X1 load33 (.D(net0),
    .CK(clk1));
 DFF_X1 load34 (.D(net0),
    .CK(clk1));
   
endmodule

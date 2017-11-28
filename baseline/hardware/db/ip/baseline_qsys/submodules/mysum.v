module mysum ( 
	       input wire 	  clk,
	       input wire 	  reset,
	       
	       input wire 	  address,
	       input wire 	  read,
	       output wire [31:0] readdata,
	       input wire 	  write, 
	       input wire [31:0]  writedata,
               input wire [3:0]   byteenable);
   
   reg [31:0] 			  hw_acc;
   
   wire 			  write_data;
   wire 			  clear_acc;
   wire 			  read_acc;
   
   wire [31:0] 			  sum;
   
   always @(posedge clk or posedge reset)
     if (reset)
       hw_acc      <= 32'h0;
     else
       hw_acc      <= clear_acc    ? 32'h0 :
                      write_data   ? sum : hw_acc;
   
   assign sum = hw_acc + 
                (byteenable[0] ? writedata[ 7: 0] : 8'h0) +
                (byteenable[1] ? writedata[15: 8] : 8'h0) +
                (byteenable[2] ? writedata[23:16] : 8'h0) +
                (byteenable[2] ? writedata[31:24] : 8'h0);
   
   assign write_data    = (write & (address == 1'h0));
   assign clear_acc     = (write & (address == 1'h1));
   assign read_acc      = (read  & (address == 1'h0));
   
   assign readdata = read_acc ? hw_acc : 32'h0;
   
endmodule					   

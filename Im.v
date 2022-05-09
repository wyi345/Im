module Intersection_Manager(clk,require_signal,
                            clock_synchronization_request,
                            car_Idnetity,xc,global_time,
                            Actuation_time,Target_Position,Target_time,
                            Target_velocity,finish,x_0,tc,l_max,l_b,v_min);
    
    input clk;//global system clock
    input clock_synchronization_request;//sent by vehicle
    input require_signal;//sent by vehicle
    input [10:0] car_Idnetity;
    input rst_n;//System reset signal
    input [10:0]v_0;//The speed at which the car sends a request to enter the intersection.
    input [10:0]x_0;//The position of the vehicle at the time of the request signal.
    input [10:0]d_max;//Maximum vehicle deceleration
    input [10:0]a_max;//maximum acceleration
    
    input [10:0]xc;//The outer query gets the collision location.
    input [10:0]tc;//
    input [10:0]global_time;//external time
    input [10:0]t_wcrtd;//worst case delay
    input finish;//The vehicle successfully passed the intersection signal.
    
    input[5:0] l_max;
    input [5:0]l_b;
    input [10:0] v_min;
    
    output  reg[10:0]clk_out;//The output clock is used for synchronization signals.
    output reg[10:0] Target_velocity;
    output reg[10:0] Target_time;
    output reg[10:0] Target_Position;
    output reg[10:0] Actuation_time;
    output reg [10:0] x_act;
    
    
    
    reg[3:0] cars;//number of cars
    reg[10:0]tacc;//Time of acceleration
     
    
    reg require_signalr;
    reg require_signalr1;//Sync Request signal rising edge detection.
    wire require;
    always@(posedge clk or negedge rst_n)
    begin
        if(!rst_n)
            begin
                require_signalr <= 1'b0;
                require_signalr1 <= 1'b0;
            end 
        else
            begin
                require_signalr <= clock_synchronization_request;
                require_signalr1 <= require_signalr;
            end 
        
    end 
    
    assign sync_require = require_signalr && (!require_signalr1);
    
    reg en_sync;
    always@(posedge clk or negedge rst_n)//After the detection of the rising edge of the signal is completed, the number of vehicles in the intersection is increased by one and the clock synchronization signal is transmitted at the same time.
    begin
        if(!rst_n)
         begin
            cars <= 4'd0;
            en_sync <= 1'b0;
         end 
        else if(sync_require == 1'b1)
            begin
                en_sync <= 1'b1;
                cars <= cars + 1'b1;
            end     
        else if(finish)
            begin
                cars <= cars - 1'b1;
                en_sync <= 1'b0;
            end 
    end 
    
    
    reg require_signalr_sync;
    reg require_signalr1_sync;//Request signal rising edge detection.
    wire require;
    always@(posedge clk or negedge rst_n)
    begin
        if(!rst_n)
            begin
                require_signalr_sync <= 1'b0;
                require_signalr1_sync <= 1'b0;
            end 
        else
            begin
                require_signalr_sync <= require_signal;
                require_signalr1_sync <= require_signalr_sync;
            end 
        
    end 
    
    assign request = require_signalr_sync && (!require_signalr1_sync);
    
    assign clk_out = (?en_sync) clk : 1'b0;
    //Record car request time
    reg [10:0]t_0;
    always@(posedge clk ot negedge rst_n)
        begin
            if(!rst_n)
                t_0 <= 'd0;
            else if(request == 1'b1)
                t_0 <= global_time;
            else if(finish)
                t_0 <= 'd0
            
        end 
                
    always@(posedge clk or negedge rst_n)
        begin
            if(!rst_n)
                Actuation_time <= 'd0;
            else if(en_sync)
                Actuation_time <= t_0 + t_wcrtd;
            else if(finish)
                Actuation_time <= 'd0;
        end 

    always@(*)//Calculate vehicle action time
     begin
        if(!rst_n)
            x_act <= 'd0;
         else if(en_sync)
            x_act <= x_0 + v_0 * t_wcrtd;
         else if(finish)
             x_act <= 'd0;
     end 
    
    wire [10:0] t_safety;
    assign t_safety = (l_max + l_b)/v_min;           
            
    reg [10:0] t_s;      
    always@(*)//Calculate vehicle arriving at the collision time 
     begin
        if(!rst_n)
           t_s <= 'd0;
         else if(en_sync)
             t_s <= tc + t_safety;
         else if(finish)
             t_s <= 'd0;
     end            

    
      //Calculate vehicle target speed      
    always@(*)
        begin
            if(!rst_n)
                Target_velocity <= 'd0;
            else if(en_sync)
                Target_velocity <= (xc-x_act)/t_s-Actuation_time;
            else if(finish)
                Target_velocity <= 'd0;
        end 
    
            always@(posedge clk or negedge rst_n)//Since there are countless solutions to this problem, in this project we can only take a solution within one interval, and in future more complex projects, we can decide according to the situation.
                if(!rst_n)
                    Target_time <= 'd0;
            else if(en_sync)
                Target_time <= (t_s + Actuation_time)/2;
            else if(finish)
                Target_time <= 'd0;
                
    reg [10:0] t_acc;
   assign t_acc = Target_time - Actuation_time；



endmodule
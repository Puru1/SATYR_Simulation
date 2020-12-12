classdef RB
   properties
      mass; % scalr
      idx_pos; % int
      pos;     % double array 3x1
      idx_link; % int
      iner;     % scalar
      ori;      % double array 3x1
   end

   methods
         
       function obj = RB(m, idx_pos, p, l, i, o)
           if nargin == 0
               obj.mass = 0;
               obj.idx_pos = 0;
               obj.pos = [0;0;0];
               obj.idx_link = -1;
               obj.iner = 0;
               obj.ori = [0;0;0];
           else
               obj.mass= m;
               if isrow(p)
                   p = transpose(p);
               end
               obj.pos = p;
               obj.idx_link = l;
               obj.iner = i;
           end
       end
   
              
       function obj = set.mass(obj,min)
           obj.mass= min;
       end
        
       function out= get.mass(obj)
           out = obj.mass;
       end
       
       
       function obj = set.idx_pos(obj,ipos)
           obj.idx_pos= ipos;
       end
        
       function out= get.idx_pos(obj)
           out = obj.idx_pos;
       end
       
       
       function obj = set.pos(obj,vin)
           if isrow(vin)
               vin = transpose(vin);
           end
           obj.pos= vin;    
       end
        
       function [out]= get.pos(obj)
           out = obj.pos;
       end
       

       function obj = set.idx_link(obj,lin)
           obj.idx_link= lin;
       end
        
       function out= get.idx_link(obj)
           out = obj.idx_link;
       end

       function obj = set.iner(obj,iin)
           obj.iner= iin;
       end
        
       function out= get.iner(obj)
           out = obj.iner;
       end

   
       function obj = set.ori(obj,oin)
           if isrow(oin)
               oin = transpose(oin);
           end
           obj.ori= oin;    
       end
        
       function [out]= get.ori(obj)
           out = obj.ori;
       end
   
   end
end
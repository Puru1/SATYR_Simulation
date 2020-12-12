function out = ht(R, v)
if isrow(v)
    v = transpose(v);
end    
out = [R, v; 0 0 0 1];
function out = getDisp(T)
if isequal(size(T), [4,4])
    out = T(1:3, 4);
else
    disp("dimension of argument must be 4x4")
end

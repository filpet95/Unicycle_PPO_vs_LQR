function sat_value = saturate(inp,sat_val)
    sat_value = zeros(size(inp));
    for i = 1:length(inp)
        sat_value(i) = sign(inp(i))*min(abs(inp(i)),sat_val);
    end
end
function mat2latex(A)

sz = size(A);
fprintf("\\begin{bmatrix}\n")
for i = 1:sz(1)
    for j = 1:sz(2)
        fprintf("%.4f\t",A(i,j))
        if j ~= sz(2)
            fprintf("&")
        end
    end
    if i ~= sz(1)
        fprintf("\\\\\n")
    end
end
fprintf("\n\\end{bmatrix}\n")

end
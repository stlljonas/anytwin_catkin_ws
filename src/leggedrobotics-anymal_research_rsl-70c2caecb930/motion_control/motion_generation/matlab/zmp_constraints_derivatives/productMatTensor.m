function mat_out = productMatTensor(mat_in,T,dim)
[a,b,c] = size(T);
[n,m] = size(mat_in);

if dim == 2
    mat_out = sym(zeros(n,a,c));
elseif dim == 3
    % todo
end

for k=1:n
    if dim == 2
        mat_out(:,:,k) = mat_in*T(:,:,k);
    elseif dim == 3
        % todo
    end
end

end
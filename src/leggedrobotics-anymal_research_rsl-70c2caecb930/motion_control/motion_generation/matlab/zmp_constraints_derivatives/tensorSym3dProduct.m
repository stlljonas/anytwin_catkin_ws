function mat = tensorSym3dProduct(T,vec,dim)

[a,b,c] = size(T);
n = length(vec);

if dim == 2
    mat = sym(zeros(a,c));
elseif dim == 3
    mat = sym(zeros(a,b));
end

for k=1:n
    if dim == 2
        mat = mat + reshape(T(:,k,:),a,c)*vec(k);
    elseif dim == 3
        mat = mat + reshape(T(:,:,k),a,b)*vec(k);
    end
end

end
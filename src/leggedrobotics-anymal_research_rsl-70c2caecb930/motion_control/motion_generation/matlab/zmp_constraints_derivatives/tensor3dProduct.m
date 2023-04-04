function mat = tensor3dProduct(T,vec,dim)

[a,b,c] = size(T);
n = length(vec);

if dim == 2
    mat = zeros(a,c);
elseif dim == 3
    mat = zeros(a,b);
end

for k=1:n
    if dim == 2
        mat = mat + reshape(T(:,k,:),a,c)*vec(k);
    elseif dim == 3
        mat = mat + reshape(T(:,:,k),a,b)*vec(k);
    end
end

end
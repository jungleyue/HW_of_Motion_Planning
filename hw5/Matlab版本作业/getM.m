function M = getM(n_seg, n_order, ts)
    M = [];
    for k = 1:n_seg
        %#####################################################
        % STEP 1.1: calculate M_k of the k-th segment 
        M_k=zeros(n_order+1,n_order+1);
        M_k(1,1)=1;
        M_k(2,2)=1;
        M_k(3,3)=2;
        M_k(4,4)=6;
        for i=5:n_order+1
            for j=i-4:n_order+1
                M_k(i,j)=ts(k)^(j-(i-4))*factorial(j-1)/factorial(j-i+4);
            end
        end
        M = blkdiag(M, M_k);
    end
end
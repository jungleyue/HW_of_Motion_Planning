function [Q, M] = getQM(n_seg, n_order, ts)
    Q = [];
    M = [];
    M_k = getM(n_order);
    for k = 1:n_seg
        %#####################################################
        % STEP 2.1 calculate Q_k of the k-th segment 
        Q_k = zeros(n_order+1);
        for i=5:n_order+1
            for j=5:n_order+1
                Q_k(i,j)=factorial(i-1)/factorial(i-5)*factorial(j-1)/factorial(j-5)/((i+j)-9)*ts(k)^(i+j-9);
            end
        end
        Q = blkdiag(Q, Q_k);
        M = blkdiag(M, M_k);
    end
end
function Ct = getCt(n_seg, n_order)
    %#####################################################
    % STEP 2.1: finish the expression of Ct
    Ct=zeros(8*n_seg,4*n_seg+4);
    for i=1:4
        Ct(i,i)=1;
    end
    
    for k=1:n_seg-1
        Ct(8*k+1-4,4+k)=1;
        Ct(8*k+2-4,n_seg+7+3*(k-1)+1)=1;
        Ct(8*k+3-4,n_seg+7+3*(k-1)+2)=1;
        Ct(8*k+4-4,n_seg+7+3*(k-1)+3)=1;
   
        Ct(8*k+5-4,4+k)=1;
        Ct(8*k+6-4,n_seg+7+3*(k-1)+1)=1;
        Ct(8*k+7-4,n_seg+7+3*(k-1)+2)=1;
        Ct(8*k+8-4,n_seg+7+3*(k-1)+3)=1;
    end
    
    for i=1:4
        Ct(8*n_seg-4+i,n_seg+3+i)=1;
    end
    
end
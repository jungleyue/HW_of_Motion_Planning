function [Aeq, beq] = getAbeq(n_seg, n_order, ts, start_cond, end_cond)
    n_all_poly = n_seg*(n_order+1);
    M=[];
    M_k = getM(n_order);
    for k = 1:n_seg
        M = blkdiag(M, M_k);
    end
    %#####################################################
    % p,v,a,j constraint in start, 
    Aeq_start = zeros(4, n_all_poly);
    % STEP 2.1: write expression of Aeq_start and beq_start
    Aeq_start(1,1)=1;
    Aeq_start(2,2)=1;
    Aeq_start(3,3)=2;
    Aeq_start(4,4)=6;
    beq_start=[start_cond';0];
    
    %#####################################################
    % p,v,a constraint in end
    Aeq_end = zeros(4, n_all_poly);
    % STEP 2.2: write expression of Aeq_end and beq_end
    tt=ts(n_seg);
    for i=1:4
        for j=i:(n_order+1)
            Aeq_end(i,n_all_poly-n_order-1+j)=tt^(j-i)*factorial(j-1)/factorial(j-i);
        end
    end
    beq_end =[end_cond';0];

    %#####################################################
    % position continuity constrain between each 2 segments
    Aeq_con_p = zeros(n_seg-1, n_all_poly);
    beq_con_p = zeros(n_seg-1, 1);
    % STEP 2.4: write expression of Aeq_con_p and beq_con_p
    for i=1:n_seg-1
        for j=1:n_order+1
            Aeq_con_p(i,(i-1)*(n_order+1)+j)=ts(i)^(j-1);
        end
        Aeq_con_p(i,i*(n_order+1)+1)=-1;
    end
 
    %#####################################################
    % velocity continuity constrain between each 2 segments
    Aeq_con_v = zeros(n_seg-1, n_all_poly);
    beq_con_v = zeros(n_seg-1, 1);
    % STEP 2.5: write expression of Aeq_con_v and beq_con_v
    for i=1:n_seg-1
        for j=2:n_order+1
            Aeq_con_v(i,(i-1)*(n_order+1)+j)=ts(i)^(j-2)*(j-1);
        end
        Aeq_con_v(i,i*(n_order+1)+2)=-1;
    end

    %#####################################################
    % acceleration continuity constrain between each 2 segments
    Aeq_con_a = zeros(n_seg-1, n_all_poly);
    beq_con_a = zeros(n_seg-1, 1);
    % STEP 2.6: write expression of Aeq_con_a and beq_con_a
    
    for i=1:n_seg-1
        for j=3:n_order+1
            Aeq_con_a(i,(i-1)*(n_order+1)+j)=ts(i)^(j-3)*(j-1)*(j-2);
        end
        Aeq_con_a(i,i*(n_order+1)+3)=-1*2;
    end
  
    %#####################################################
    % combine all components to form Aeq and beq   
    Aeq_con = [Aeq_con_p; Aeq_con_v; Aeq_con_a];
    beq_con = [beq_con_p; beq_con_v; beq_con_a];
    Aeq = [Aeq_start; Aeq_end; Aeq_con];
    Aeq=Aeq*M;
    beq = [beq_start; beq_end; beq_con];
end
function [Aieq, bieq] = getAbieq(n_seg, n_order, corridor_range, ts, v_max, a_max)
    n_all_poly = n_seg*(n_order+1);
    %#####################################################
    % STEP 3.2.1 p constraint
    Aieq_p = [-eye(n_all_poly);eye(n_all_poly)];
    bieq_p = zeros(n_all_poly*2, 1);
    for i=1:n_seg
        bieq_p((i-1)*(n_order+1)+1:i*(n_order+1)) = -corridor_range(i,1);
        bieq_p(n_all_poly+(i-1)*(n_order+1)+1:n_all_poly+i*(n_order+1)) = corridor_range(i,2);
    end
 
    %#####################################################
    % STEP 3.2.2 v constraint   
    Aieq_v = zeros(2*(n_all_poly-1),n_all_poly);
    bieq_v = zeros(2*(n_all_poly-1),1);
    for i = 1:n_all_poly-1
        Aieq_v(i, i+1) = -n_order;
        Aieq_v(i, i) = n_order;
        bieq_v(i) = v_max;
        
        Aieq_v(i+n_all_poly-1, i+1) = n_order;
        Aieq_v(i+n_all_poly-1, i) = -n_order;
        bieq_v(i+n_all_poly-1) = v_max;
    end
   
    %#####################################################
    % STEP 3.2.3 a constraint   
    Aieq_a = zeros(2*(n_all_poly-2),n_all_poly);
    bieq_a = zeros(2*(n_all_poly-2),1);
    for i = 1:n_all_poly-2
        Aieq_a(i, i+2) = -n_order*(n_order-1);
        Aieq_a(i, i+1) = 2*n_order*(n_order-1);
        Aieq_a(i, i) = -n_order*(n_order-1);
        bieq_a(i) = a_max;
        
        Aieq_a(i+n_all_poly-2, i+2) = n_order*(n_order-1);
        Aieq_a(i+n_all_poly-2, i+1) = -2*n_order*(n_order-1);
        Aieq_a(i+n_all_poly-2, i) = n_order*(n_order-1);
        bieq_a(i+n_all_poly-2) = a_max;   
    end
    

    %#####################################################
    % combine all components to form Aieq and bieq   
    Aieq = [Aieq_p; Aieq_v; Aieq_a];
    bieq = [bieq_p; bieq_v; bieq_a];
    
end
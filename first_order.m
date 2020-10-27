function dydt = first_order(x)
    
    y = x(3);
    w = x(2);
    mu = x(1);
    
    dydt = w-mu*y;
    
end


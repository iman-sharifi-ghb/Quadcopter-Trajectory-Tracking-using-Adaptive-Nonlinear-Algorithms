function x = RK4(func, x, u)
    
    global dt
    k1 = func(x,         u);
    k2 = func(x+dt/2*k1, u);
    k3 = func(x+dt/2*k2, u);
    k4 = func(x+dt*k3,   u);    
    x  = x + dt/6*(k1+2*k2+2*k3+k4);  
    
end


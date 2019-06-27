function fun = optE(x)
    global Sc;
    %Parameter
    rho = 800; 
    mu = 0.9;
    R = 1.53; 
    m = 2.5; 
    g = 9.81;
    k = 29.4;
    d = 0.2;
    h_c = 0.3;
    
    %Input V, l, w, h, r
    xs = x./Sc;
    V = xs(1);
    l = xs(2);
    w = xs(3);
    h = xs(4);
    r = xs(5);
    
    %Equation
    V_c = V;
    M = rho*w*l*h;
    f = mu*(M+m)*g; 
    T_max = V*k/R;
    a_max = (4*T_max*d/2-f)/(M+m);
    v = d*(V*k-f*R/(2*d))/(2*k^2);
    M_x = (M+m)*g*l/2-m*a_max*h_c/2-M*a_max*(h/2+h_c);
    M_y = (M+m)*g*l/2-m*v^2*h_c/(2*r)-M*v^2*(h/2+h_c)/r;
    s = dubins(0,0,0,17,23,pi,r,0);
    t = s/v; 
    l_c = l;
    w_c = w;
    v_c = v;
    
    %Objective function
    fun = t/(M*60);
end
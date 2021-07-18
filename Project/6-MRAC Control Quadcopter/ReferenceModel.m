function x_dot = ReferenceModel(x, u)

global Am Bm
x_dot = Am*x + Bm*u;

end


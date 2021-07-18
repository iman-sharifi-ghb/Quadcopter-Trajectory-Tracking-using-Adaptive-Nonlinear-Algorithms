function K = SF_Controller(Type)

global A B desiredPoles 
global Q R N

Type = lower(Type);

if isempty(Type)
    Type = 'place';
end

switch Type
    case 'place'
        K = place(A, B, desiredPoles);
    case 'lqr'
        [K, P, e] = lqr(A, B, Q, R, N);
end

end


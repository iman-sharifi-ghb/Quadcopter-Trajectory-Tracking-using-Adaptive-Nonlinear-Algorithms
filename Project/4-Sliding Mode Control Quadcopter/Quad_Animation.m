function Quad_Animation(t,q1,q2,q3,q4,q5,q6,q1d,q2d,q3d)
%ANIMATION PART
global l
r = l/3;
a = 1.1*l+r;
t1 = 0:0.01:2*pi;
hold on
Len=length(t);
for i=1:Len
    
    R=[                          cos(q4(i))*cos(q5(i)),                           cos(q5(i))*sin(q4(i)),        -sin(q5(i))
        cos(q4(i))*sin(q5(i))*sin(q6(i)) - cos(q6(i))*sin(q4(i)), cos(q4(i))*cos(q6(i)) + sin(q4(i))*sin(q5(i))*sin(q6(i)), cos(q5(i))*sin(q6(i))
        sin(q4(i))*sin(q6(i)) + cos(q4(i))*cos(q6(i))*sin(q5(i)), cos(q6(i))*sin(q4(i))*sin(q5(i)) - cos(q4(i))*sin(q6(i)), cos(q5(i))*cos(q6(i))];

    %Quad Motors Location
    P1 = [q1(i);q2(i);q3(i)]+R*[l;0;0];
    P2 = [q1(i);q2(i);q3(i)]+R*[-l;0;0];
    P3 = [q1(i);q2(i);q3(i)]+R*[0;l;0];
    P4 = [q1(i);q2(i);q3(i)]+R*[0;-l;0];   

    P11 = [q1(i);q2(i);q3(i)]+R*[l-r;0;0];
    P22 = [q1(i);q2(i);q3(i)]+R*[-(l-r);0;0];
    P33 = [q1(i);q2(i);q3(i)]+R*[0;l-r;0];
    P44 = [q1(i);q2(i);q3(i)]+R*[0;-(l-r);0];     
    
    V12 = [P11,P22]; V34 = [P33,P44]; 

    plot3(q1d,q2d,q3d,'-g','linewidth',2);hold on
    
    C1 = P1 + r*[cos(t1);sin(t1);zeros(size(t1))];
    plot3(C1(1,:),C1(2,:),P1(3)*ones(1,length(C1)),'-k','linewidth',2);hold on
    
    C2 = P2 + r*[cos(t1);sin(t1);zeros(size(t1))];
    plot3(C2(1,:),C2(2,:),P2(3)*ones(1,length(C2)),'-k','linewidth',2);hold on
    
    C3 = P3 + r*[cos(t1);sin(t1);zeros(size(t1))];
    plot3(C3(1,:),C3(2,:),P3(3)*ones(1,length(C3)),'-k','linewidth',2);hold on
    
    C4 = P4 + r*[cos(t1);sin(t1);zeros(size(t1))];
    plot3(C4(1,:),C4(2,:),P4(3)*ones(1,length(C4)),'-k','linewidth',2);hold on
    
    plot3(V12(1,:),V12(2,:),V12(3,:),'k','linewidth',3);title('Quadcopter')
    xlabel('X');ylabel('Y');zlabel('Z');hold on
    
    plot3(V34(1,:),V34(2,:),V34(3,:),'k','linewidth',3);hold on
    plot3(q1(1:i),q2(1:i),q3(1:i),'b','linewidth',1)

    axis equal
    grid on
    axis([min(q1)-a  max(q1)+a min(q2)-a  max(q2)+a  min(q3)-a  max(q3)+a])
    str=['Time = ',num2str(t(i))];
    text(min(q1),min(q1),str)
    
    hold off
    pause(0.0001)
    
end
end


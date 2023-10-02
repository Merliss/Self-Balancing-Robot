 A=[0 1 0 0;
    0 0 -0.717 0;
    0 0 0 1;
    0 0 15.79 0];
 B=[0; 0.976; 0; -1.463;];
 C=eye(4,4);
 D=zeros(4,1);
 [l,m]=ss2tf(A,B,C,D);

% regulator stanu
rankS=rank(ctrb(A,B));
eig(A);

alfa=6;
pi=[-0.4240+1.2630*j;-0.4240-1.2630*j;-0.6260+0.4141*j;-0.6260-0.4141*j]; %ITAE
pb=[-0.6573+0.3302*j;-0.6573-0.3302*j;-0.9047+0.2711*j;-0.9047-0.2711*j]; %Bessel

%for alfa=1:1:4;
p=alfa*pb;
K=place(A,B,p);
Ac=A-B*K;
C1=[1 0 0 0];
D1=[0];
P=[0;0;0;0;1];
L=[A B;C1 D1];
Nn=L\P;
K=place(A,B,p);
Nx=Nn(1:4,:);
Nu=Nn(5:5,:);
N=K*Nx+Nu;

Ac=A-B*K;
Bc=B*N;
Cc=C-D*K;
Dc=D*N;

t=0:0.001:4;
[lz,mz]=ss2tf(Ac,Bc,Cc,Dc);
y=step(lz,mz,t);
plot(t,y);
grid on;
%end
lgd=legend({'położenie ','prędkość','kąt','prędkość kątowa'},'Location','northwest','Orientation','horizontal');
%lgd.FontSize = 14;
%hold on







clc
clear
b=0.99;
w11=0.1*[1,-1;1,-1;1,-1];
w21=[1.9,0.3,3]; 
w11_1=w11;w11_2=w11_1;
w21_1=w21;w21_2=w21_1;

%% Initialization
rate2=0.000000000000001;rate3=0.1; 

y_1=0;y_2=y_1;y_3=y_2;  
ym_1=0;ym_2=ym_1;ym_3=ym_2;   
u_1=0;u_2=u_1;u_3=u_2;   
v_1=0;v_2=v_1;v_3=v_2;   
uh_1=0;uh_2=uh_1;uh_3=uh_2;   
h1i=zeros(3,1);h1i_1=h1i;  
x1i=zeros(3,1);x2i=x1i;x3i=x2i;x1i_1=x1i;x2i_1=x2i;x3i_1=x3i;  

for k=1:1:2000
  % system model
  y(k)=0.6*y_1-0.2*y_2+1*uh_1+1.2*uh_2+0.5*(sin(y_1+y_2+uh_1+uh_2)-(y_1+y_2+uh_1+uh_2)/(1+y_1^2+y_2^2+uh_1^2+uh_2^2));+0.1*y_1*sin(y_1+y_2+uh_1+uh_2); +0.001*sin(y_1^2+y_2^2); %第2个被控对象 VSC-HVDC
  ym(k)=0.6*ym_1-0.2*ym_2+1*u_1+1.2*u_2; 
  f(k)=0.5*(sin(y_1+y_2+uh_1+uh_2)-(y_1+y_2+uh_1+uh_2)/(1+y_1^2+y_2^2+uh_1^2+uh_2^2));
 
  %Desired input signal         
   w(k)=0.5*sin(0.001*pi*k);
 % w(k)=0.4*(1-exp(-0.1*k));
 
    w1(k)=0.5*sin(0.001*pi*k);
 %  w1(k)=0.4;
    u(k)=(-1*u_1+w(k)-0.6*y(k)+0.2*y_1)/1.2;   % u_1
    %% PIDNN
    x1o=[y(k);ym(k)]; 
  
    x1i=w11*x1o; 
  
    xp=[x1i(1)];
    qp=xp;
    h1i(1)=qp;
    xi=[x1i(2)];
    qi=[0];qi_1=[h1i(2)];
    qi=qi_1+xi; 
    h1i(2)=qi;

    xd=[x1i(3)];
    qd=[0];
    xd_1=[x1i_1(3)];
    qd=xd-xd_1;
    h1i(3)=qd;

    wo=[w21];
    qo=[h1i'];qo=qo';
    v(k)=wo*qo; 
    uh(k)=-v(k)+u(k); %nonlinear control law
    fm(k)=v(k)+1.2*v_1;
    em(k)=f(k)-fm(k);
    %uh(k)=u(k);  %linear control law 
    error=[y(k)-ym(k)];  
    error1(k)=error(1);
    J(k)=0.5*(error(1)^2);
    ypc=[y(k)-y_1(1)];
    uhc=[v_1(1)-v_2(1)];
    if uhc(1)==0
        uhc(1)=uhc(1)+0.00001;
    end
    z21=ypc./(uhc(1));
    Sig1=sign(z21);
    Sig1=z21;
    dw21=sum(error.*Sig1)*qo';
    w21=w21+rate2*dw21+rate3*(w21_1-w21_2); 

 %% Update parameters
    u_3=u_2;u_2=u_1;u_1=u(k);
    v_3=v_2;v_2=v_1;v_1=v(k);
    uh_3=uh_2;uh_2=uh_1;uh_1=uh(k);
    y_2=y_1;y_1=y(k);
    ym_2=ym_1;ym_1=ym(k);
    h1i_1=h1i;
    x1i_1=x1i;x2i_1=x2i;x3i_1=x3i;
    w11_2=w11_1;w11_1=w11;
    w21_2=w21_1;w21_1=w21;
   
end
time=0.001*(1:k);
Time=0.001*(1:1);
m=1;

figure(1)
plot(time,w1,'r-',time,y,'b--',time,w1-y,'g--','LineWidth',2);
xlabel('Time(s)','fontsize',20,'FontName','Times New Roman');
ylabel('System output','fontsize',20,'FontName','Times New Roman');
legend('desired signal','actual system output signal','tracking error',12);
xlim([0, 2]);
ylim([-0.8, 0.8]);
set(gca,'XTick',[0:0.4:2]) 
set(gca,'YTick',[-0.8:0.4:0.8]) 
set(gca,'FontSize',12);
set(0,'defaultfigurecolor','w');

figure(2)
plot(time,f,'r-',time,fm,'b--',time,f-fm,'g--','LineWidth',2);
xlabel('Time(s)','fontsize',20,'FontName','Times New Roman');
ylabel('Unmodeled dynamic','fontsize',20,'FontName','Times New Roman');
legend('UD','estimated values of UD','estimation error',12);
xlim([0, 2]);
ylim([-0.08, 0.08]);
set(gca,'XTick',[0:0.4:2]) 
set(gca,'YTick',[-0.08:0.04:0.08]) 
set(gca,'FontSize',12);
set(0,'defaultfigurecolor','w');

figure(3)
plot(time,y,'r--',time,ym,'b--',time,y-ym,'g--','LineWidth',2);
xlabel('Time(s)','fontsize',20,'FontName','Times New Roman');
%ylabel('w and y','fontsize',20,'FontName','Times New Roman');
legend('actual system output signal ','virtual system output signal ','error caused by UD',12);
%title('virtual system output signal $$\hat{o}$$','interpreter','latex')
xlim([0, 2]);
ylim([-0.5, 1]);
set(gca,'XTick',[0:0.4:2]) 
set(gca,'FontSize',12);
set(0,'defaultfigurecolor','w');



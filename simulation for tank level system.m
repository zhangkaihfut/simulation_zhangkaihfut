
clc
clear
b=0.99;
w11=0.5*[1,-1;1,-1;1,-1];
w21=0.3*[1.9,0.3,3]; 
w11_1=w11;w11_2=w11_1;
w21_1=w21;w21_2=w21_1;

%% 初始化数据
rate2=0.000001;rate3=0.1; 

y_1=0;y_2=y_1;y_3=y_2;   
ym_1=0;ym_2=ym_1;ym_3=ym_2;   
u_1=0;u_2=u_1;u_3=u_2;   
v_1=0;v_2=v_1;v_3=v_2;   
uh_1=0;uh_2=u_1;uh_3=u_2;   
h1i=zeros(3,1);h1i_1=h1i;  
x1i=zeros(3,1);x2i=x1i;x3i=x2i;x1i_1=x1i;x2i_1=x2i;x3i_1=x3i;   

for k=1:1:5000
  y(k)=0.819*y_1-0.8186*y_2+0.4629*uh_1+0.2*sin(0.1*y_1^2+0.2*y_2^2+0.001*uh_1^2)+0.1*sin(0.001*k*pi)+0.01*randn(1);  % actual system
  ym(k)=0.819*ym_1(1)-0.8186*ym_2(1)+0.4629*u_1;                                                                      % virtual system
  f(k)=0.2*sin(0.1*y_1^2+0.2*y_2^2+0.001*uh_1^2)+0.1*sin(0.001*k*pi)+0.01*randn(1);                                   % unmodeled dynamic                                
  w(k)=5*(1-exp(-0.01*k));
  w1(k)=5; 
  h1=0.03;
  u(k)=(w(k)-0.819*y(k)+0.8186*y_1+h1*u_1)/(0.4629+h1);                                                              %linear control law
    %% PIDNN
    x1o=[ym(k);y(k)]; 
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
   uh(k)=v(k)+u(k); %nonlinear control law
   % uh(k)=u(k);  %linear control law
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

 %% undate paremeters
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
set(0,'defaultfigurecolor','w');
figure(1)
plot(time,w1,'r-',time,y,'g--',time,w1-y,'b--','LineWidth',2.5);
%plot(time,R11,'k:',time,y,'r-',Time,m,'g--',Time,m,'b--','LineWidth',2);
xlabel('Time(s)','fontsize',20,'FontName','Times New Roman');
ylabel('Liquid level height(cm)','fontsize',20,'FontName','Times New Roman');
legend('desired signal','actual system output signal','tracking error',12);
xlim([0, 5]);
ylim([-4, 8]);
set(gca,'XTick',[0:1:5]) 
set(gca,'YTick',[-4:4:8])
set(gca,'FontSize',12);
%set(0,'defaultfigurecolor','w');

figure(2)
plot(time,f,'r-',time,-0.4629*v,'g--',time,f+0.4629*v,'b--','LineWidth',2);
%plot(time,R11,'k:',time,y,'r-',Time,m,'g--',Time,m,'b--','LineWidth',2);
xlabel('Time(s)','fontsize',20,'FontName','Times New Roman');
ylabel('Unmodeled dynamic','fontsize',20,'FontName','Times New Roman');
legend('UD','estimated values of UD','estimation error',12);
xlim([0, 5]);
ylim([-0.4, 0.5]);
set(gca,'XTick',[0:1:5]) 
set(gca,'YTick',[-0.4:0.3:0.5]) 
set(gca,'FontSize',12);

figure(3)
plot(time,ym,'r-',time,y,'g--',time,ym-y,'b--','LineWidth',2.5);
%plot(time,R11,'k:',time,y,'r-',Time,m,'g--',Time,m,'b--','LineWidth',2);
xlabel('Time(s)','fontsize',20,'FontName','Times New Roman');
ylabel('System output','fontsize',20,'FontName','Times New Roman');
legend('virtual system output signal','actual system output signal','error caused by UD',12);
xlim([0, 5]);
ylim([-3, 7]);
set(gca,'XTick',[0:1:5]) 
set(gca,'YTick',[-3:2:7]) 
set(gca,'FontSize',12);


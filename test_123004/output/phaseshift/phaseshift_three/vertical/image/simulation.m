noise1=5*unifrnd(-0.5,0.5,1,401);
noise2=5*unifrnd(-0.5,0.5,1,401);
noise3=5*unifrnd(-0.5,0.5,1,401);
I1=10*rand(1,401);
I2=100+20*rand(1,401);

x=0:pi/100:4*pi;

y1=I1+I2.*(cos(x-2*pi/3)+1)+noise1;
y2=I1+I2.*(cos(x)+1)+noise2;
y3=I1+I2.*(cos(x+2*pi/3)+1)+noise3;
phase=atan2(sqrt(3)*(y1-y3),(2*y2-y1-y3))/(2*pi);

figure('NumberTitle','off','Name','ideal three phaseshift');
plot(x,y1,'r',x,y2,'g',x,y3,'b',x,I1,'c',x,I2,'m',x,noise1,'r',x,noise2,'g',x,noise3,'b');
axis([0,4*pi,0,300]);       %set axis range
set(gca,'XTick',0:pi/3:4*pi);    %set x-axis's  tick and label
set(gca,'XTickLabel',{'0' 'pi/3' 'pi*2/3' 'pi' 'pi*4/3' 'pi*5/3' '2*pi' 'pi*7/3' 'pi*8/3' '3*pi' 'pi*10/3' 'pi*11/3' '4*pi'});
xlabel('phase');    %set axis'slabel 
ylabel('value');
grid;

hleg1=legend('phase-1','phase-2','phase-3','background','albedo','noise1','noise2','noise3');

figure('NumberTitle','off','Name','ideal wrapped phase');
plot(x,phase);

%assum camera response curve
figure('NumberTitle','off','Name','camera response curve');
z=0:1:255;
f=127.5*(sin(z*pi/255-pi/2)+1);
plot(f);

%input value multiply camera response curve
r1=127.5*(sin(y1*pi/255-pi/2)+1);
r2=127.5*(sin(y2*pi/255-pi/2)+1);
r3=127.5*(sin(y3*pi/255-pi/2)+1);
real_phase=atan2(sqrt(3)*(r1-r3),(2*r2-r1-r3))/(2*pi);

figure('NumberTitle','off','Name','real three phaseshift');
plot(x,r1,'r',x,r2,'g',x,r3,'b',x,I1,'c',x,I2,'m',x,noise1,'r',x,noise2,'g',x,noise3,'b');
axis([0,4*pi,0,300]);       %set axis range
set(gca,'XTick',0:pi/3:4*pi);    %set x-axis's  tick and label
set(gca,'XTickLabel',{'0' 'pi/3' 'pi*2/3' 'pi' 'pi*4/3' 'pi*5/3' '2*pi' 'pi*7/3' 'pi*8/3' '3*pi' 'pi*10/3' 'pi*11/3' '4*pi'});
xlabel('phase');    %set axis'slabel 
ylabel('value');
grid;
legend('phase-1','phase-2','phase-3','background','albedo','noise1','noise2','noise3');

figure('NumberTitle','off','Name','real wrapped phase');
plot(x,real_phase);

%start from (580,300)
sample_point=[19,19,19,19,19,21,22,22,22,22,...
                       25,28,29,32,33,38,40,45,49,57,...
                       62,66,75,81,90,102,108,116,125,138,...
                       154,167,166,171,171,177,193,202,205,218,...
                       226,227,233,220,225,220,227,212,202,202,...
                       193,187,187,176,153,152,141,139,125,119,...
                       113,101,92,82,73,64,59,51,45,42,...
                       37,30,28,29,26,24,21,21,19,21,...
                       18,19,18,20,18,18,18,18,18,18,...
                       19,19,20,21,22,22,23,28,31,30]

figure('NumberTitle','off','Name','real camera response');
x=1:1:100;
plot(x,sample_point,'r',x,105*(cos((x-2)*pi/42+pi)+1)+20,'g');





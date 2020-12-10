%definir las variables a utilizar en el sistema
m_disco=0.045; %masa del disco de inercia kilogramos
m_estructura=0.316; %masa de la estructura de la motocicleta en kilogramos
m=m_disco+m_estructura+2*0.06158; %ruedas, disco, estructura kilogramos 
r_disco=0.04; %radio del disco en metros
l=0.08; %longitud al centro del disco en metros
I=1/12*m_estructura*0.04; %inecia del brazo 1/12*M*e
Ir=0.5*m_disco*r_disco; %volante de inercia 1/2 M*r
g=9.775; %gravedad en La Paz
k=0.009; % constante entre el torque y señales de voltaje

a=m*g*l/I;
b=k/I;
c=k/Ir;
x=pi/2;

%comenzamos definiendo el espacio de estados
A=[0,1,0; a*cos(x),0,0;0,0,0];
B=[0;-b;c];
C=[1,0,0];
D=[0];

%estabilidad del sistema
polos = eig(A); %polos 

sys=ss(A,B,C,D); %determinando el las matrices como parte del espacio de estados
FuncionTransferencia = tf(sys) %obtenemos la funcion de transferencia

%Parametros para el control LQR
Q = [ 0.47 0 0; 0 0.09 0; 0 0 0.01];
R = [0.47];
[K,S,e] = lqr(A,B,Q,R);

%actualizamos el espacio de estados en base a la matriz k obtenida
n = length(K);
AA = A - B * K;
for i=1:n
    BB(:,i)=B * K(i);
end
CC=C;
DD=D;
for i=1:n
     sys(:,i)=ss(AA,BB(:,i),CC,DD);
end

%Obtenemos la respuesta del sistema LQR
[num,den]=ss2tf(AA,BB(:,1),CC,DD);
sys_tf=tf(num,den);
step(sys_tf)
Datos = stepinfo(sys_tf)
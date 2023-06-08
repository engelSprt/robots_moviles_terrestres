%Limpieza de pantalla
clear all
close all
clc

%Declaración de variables simbólicas
syms th1(t) th2(t) th3(t) th4(t) th5(t) th6(t) t a0 a1 a2 a3

n = 3; % Tamaño de la matriz identidad
matriz_identidad = sym(eye(n));


%Configuración del robot, 0 para junta rotacional, 1 para junta prismática
RP=[0 0 0 0 0 0];

%Creamos el vector de coordenadas articularesS
Q= [th1, th2, th3, th4, th5, th6];
%disp('Coordenadas generalizadas');
%pretty (Q);

%Creamos el vector de velocidades generalizadas
Qp= diff(Q, t);
%disp('Velocidades generalizadas');
%pretty (Qp);
%Número de grado de libertad del robot
GDL= size(RP,2);
GDL_str= num2str(GDL);


%Trama 2 
%Posición de la articulación 2 respecto a 1
P(:,:,1)= [a0;a0;a0];
%Matriz de rotación de la junta 2 respecto a 1.... 
R(:,:,1)= rotacion('x',-90+th2);
R(:,:,1)= R(:,:,1)*rotacion('y',90+th1);

%Trama 3
%Posición de la articulación 3 respecto a 2
P(:,:,2)= [0;0;0];
%Matriz de rotación de la junta 3 respecto a 2
R(:,:,2)= rotacion('y',-90+th1);
R(:,:,2)= R(:,:,2)*rotacion('z', -90+th3);
 
%Trama 4 
%Posición de la articulación 4 respecto a 3
P(:,:,3)= [a1;0;0];
%Matriz de rotación de la junta 4 respecto a 3
R(:,:,3)= R(:,:,2);

%Trama 5 
%Posición de la articulación 5 respecto a 4
P(:,:,4)= [a2;0;0];
%Matriz de rotación de la junta 5 respecto a 4
R(:,:,4)= rotacion('z', 90+th5);

%Trama 6 
%Posición de la articulación 6 respecto a 5
P(:,:,5)= [0;0;0];
%Matriz de rotación de la junta 6 respecto a 5
R(:,:,5)= rotacion('y', 90+th1);
R(:,:,5)= R(:,:,5) * rotacion('z', 90+th6);

%Trama 7 
%Posición de la articulación 7 respecto a 6
P(:,:,6)= [0;0;a3];
%Matriz de rotación de la junta 7 respecto a 6
R(:,:,6)= R(:,:,5);


%Creamos un vector de ceros
Vector_Zeros= zeros(1, 3);

%Inicializamos las matrices de transformación Homogénea locales
A(:,:,GDL)=simplify([R(:,:,GDL) P(:,:,GDL); Vector_Zeros 1]);
%Inicializamos las matrices de transformación Homogénea globales
T(:,:,GDL)=simplify([R(:,:,GDL) P(:,:,GDL); Vector_Zeros 1]);
%Inicializamos las posiciones vistas desde el marco de referencia inercial
PO(:,:,GDL)= P(:,:,GDL); 
%Inicializamos las matrices de rotación vistas desde el marco de referencia inercial
RO(:,:,GDL)= R(:,:,GDL); 


for i = 1:GDL
    i_str= num2str(i);
   %disp(strcat('Matriz de Transformación local A', i_str));
    A(:,:,i)=simplify([R(:,:,i) P(:,:,i); Vector_Zeros 1]);
   %pretty (A(:,:,i));

   %Globales
    try
       T(:,:,i)= T(:,:,i-1)*A(:,:,i);
    catch
       T(:,:,i)= A(:,:,i);
    end
    disp(strcat('Matriz de Transformación global T', i_str));
    T(:,:,i)= simplify(T(:,:,i));
    pretty(T(:,:,i))

    RO(:,:,i)= T(1:3,1:3,i);
    PO(:,:,i)= T(1:3,4,i);
    %pretty(RO(:,:,i));
    %pretty(PO(:,:,i));
end



%Calculamos el jacobiano lineal de forma analítica
Jv_a(:,GDL)=PO(:,:,GDL);
Jw_a(:,GDL)=PO(:,:,GDL);

for k= 1:GDL
    if RP(k)==0 
       %Para las juntas de revolución
        try
            Jv_a(:,k)= cross(RO(:,3,k-1), PO(:,:,GDL)-PO(:,:,k-1));
            Jw_a(:,k)= RO(:,3,k-1);
        catch
            Jv_a(:,k)= cross([0,0,1], PO(:,:,GDL));%Matriz de rotación de 0 con respecto a 0 es la Matriz Identidad, la posición previa tambien será 0
            Jw_a(:,k)=[0,0,1];%Si no hay matriz de rotación previa se obtiene la Matriz identidad
         end
     else
%         %Para las juntas prismáticas
        try
            Jv_a(:,k)= RO(:,3,k-1);
        catch
            Jv_a(:,k)=[0,0,1];
        end
            Jw_a(:,k)=[0,0,0];
     end
 end    

Jv_a= simplify (Jv_a);
Jw_a= simplify (Jw_a);
%disp('Jacobiano lineal obtenido de forma analítica');
%pretty (Jv_a);
%disp('Jacobiano ángular obtenido de forma analítica');
%pretty (Jw_a);


disp('Velocidad lineal obtenida mediante el Jacobiano lineal');
V=simplify (Jv_a*Qp');
pretty(V);
disp('Velocidad angular obtenida mediante el Jacobiano angular');
W=simplify (Jw_a*Qp');
pretty(W);
    
%% Funcion para hacer rotacion en cualquiera de los 3 ejes

function matriz_rotacion = rotacion(eje, theta)
    
    % Validar el eje de rotación y construir la matriz de rotación correspondiente
    switch eje
        case 'x'
            matriz_rotacion = [1       0              0;
                               0  cos(theta) -sin(theta);
                               0  sin(theta)  cos(theta)];
        case 'y'
            matriz_rotacion = [ cos(theta)  0  sin(theta);
                                      0       1       0    ;
                               -sin(theta)  0  cos(theta)];
        case 'z'
            matriz_rotacion = [cos(theta)  -sin(theta)  0;
                               sin(theta)   cos(theta)  0;
                                   0              0       1];
        otherwise
            error('Eje de rotación no válido. Los valores válidos son "x", "y" o "z"');
    end
end

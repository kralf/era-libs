


a_3=23.05 ;
a_4=22.4  ;
a_5=18.8  ;

%color = zeros(140,100,70);
for i=1:10
zmin = (i-1)*5;
zmax = (i)*5;

resolution = 10; %degree
Workspace = [];
for t1= 0 : 120/resolution
    for t2 = 0: 90/resolution
        for t3 = 0: 110/resolution
            for t4 = 0: 120/resolution
                theta_1 = (-90 + t1 *resolution) *pi/180 ;
                theta_2 = ( t2 *resolution )*pi/180;
                theta_3 = (-20 + t3 *resolution) *pi/180;
                theta_4 = ( t4 *resolution) *pi/180;
                theta_5 =  pi/2- theta_3 - theta_4;
                
                if(theta_5>0)
                pos6 = [
                     (-((-cos(theta_1)*sin(theta_2)*cos(theta_3)+sin(theta_1)*sin(theta_3))*cos(theta_4)+(cos(theta_1)*sin(theta_2)*sin(theta_3)+sin(theta_1)*cos(theta_3))*sin(theta_4))*sin(theta_4+theta_3)-(-(-cos(theta_1)*sin(theta_2)*cos(theta_3)+sin(theta_1)*sin(theta_3))*sin(theta_4)+(cos(theta_1)*sin(theta_2)*sin(theta_3)+sin(theta_1)*cos(theta_3))*cos(theta_4))*cos(theta_4+theta_3))*a_5-(-cos(theta_1)*sin(theta_2)*cos(theta_3)+sin(theta_1)*sin(theta_3))*cos(theta_4)*a_4-(cos(theta_1)*sin(theta_2)*sin(theta_3)+sin(theta_1)*cos(theta_3))*sin(theta_4)*a_4+cos(theta_1)*sin(theta_2)*cos(theta_3)*a_3-sin(theta_1)*sin(theta_3)*a_3;
                     (-((-sin(theta_1)*sin(theta_2)*cos(theta_3)-cos(theta_1)*sin(theta_3))*cos(theta_4)+(sin(theta_1)*sin(theta_2)*sin(theta_3)-cos(theta_1)*cos(theta_3))*sin(theta_4))*sin(theta_4+theta_3)-(-(-sin(theta_1)*sin(theta_2)*cos(theta_3)-cos(theta_1)*sin(theta_3))*sin(theta_4)+(sin(theta_1)*sin(theta_2)*sin(theta_3)-cos(theta_1)*cos(theta_3))*cos(theta_4))*cos(theta_4+theta_3))*a_5-(-sin(theta_1)*sin(theta_2)*cos(theta_3)-cos(theta_1)*sin(theta_3))*cos(theta_4)*a_4-(sin(theta_1)*sin(theta_2)*sin(theta_3)-cos(theta_1)*cos(theta_3))*sin(theta_4)*a_4+sin(theta_1)*sin(theta_2)*cos(theta_3)*a_3+cos(theta_1)*sin(theta_3)*a_3;                                                                                                                                                                                                                                                                     
                     (-(cos(theta_2)*cos(theta_3)*cos(theta_4)-cos(theta_2)*sin(theta_3)*sin(theta_4))*sin(theta_4+theta_3)-(-cos(theta_2)*cos(theta_3)*sin(theta_4)-cos(theta_2)*sin(theta_3)*cos(theta_4))*cos(theta_4+theta_3))*a_5-cos(theta_2)*cos(theta_3)*cos(theta_4)*a_4+cos(theta_2)*sin(theta_3)*sin(theta_4)*a_4-cos(theta_2)*cos(theta_3)*a_3+a_3+a_4+a_5;
                     ];
                 
                 
               %  color( round(pos6(1))+80, round(pos6(2))+1, round(pos6(3))+1 ) = color( round(pos6(1))+80, round(pos6(2))+1, round(pos6(3))+1 ) +1;
           
                if( zmin+ a_5 < (pos6(3)) && (pos6(3))< zmax + a_5) 
                    Workspace = [Workspace pos6];
                end
                %if( zmin+ a_5 < round(pos6(3)) && round(pos6(3))< zmax + a_5) 
                %    Workspace = [Workspace round(pos6)];
                %end
                end
            end
        end
    end
end

Workspace(3,:) = Workspace(3,:) - a_5;

%figure(5)
%X = Workspace';
%K = convhulln(X);
%trisurf(K,X(:,1),X(:,2),X(:,3))



if(1)
figure(i)
plot(Workspace(1,:),Workspace(2,:),'x',0,0,'ob')
xlabel( 'x [cm]',  'fontsize', 30);
ylabel( 'y [cm]',  'fontsize', 30);
%axes( 'fontsize', 18);
%axis([-40,80,-60,80], 'equal');
axis([-80,117,-60,80]);
text(-70,-45,[num2str(zmin) ' < z [cm] < ' num2str(zmax) ], 'fontsize', 30, 'EdgeColor','black')
end

end

if(0)
figure(1)
plot3(Workspace(1,:),Workspace(2,:),Workspace(3,:),'x')
xlabel( 'x' )
ylabel( 'y' )
zlabel( 'z' )
end

if(0)
figure(3)

%axes( 'fontsize', 18);

scatter(Workspace(1,:),Workspace(2,:),4,Workspace(3,:))
xlabel( 'x', 'fontsize', 30 )
ylabel( 'y', 'fontsize', 30 )
zlabel( 'y', 'fontsize', 30 )
axis([-40,80,-60,80], 'equal');
end

if(0)
figure(1)
subplot(2,2,1)
plot(Workspace(1,:),Workspace(2,:),'x')
xlabel( 'x',  'fontsize', 30);
ylabel( 'y',  'fontsize', 30);
%axes( 'fontsize', 18);
axis([-40,80,-60,80], 'equal');


subplot(2,2,2)
plot(Workspace(3,:),Workspace(2,:),'x')
xlabel( 'z',  'fontsize', 30);
ylabel( 'y',  'fontsize', 30);
%axes( 'fontsize', 18);
axis([-40,80,-60,80], 'equal');


subplot(2,2,3)
plot(Workspace(1,:),Workspace(3,:),'x')
xlabel( 'x',  'fontsize', 30);
ylabel( 'z',  'fontsize', 30);
%axes( 'fontsize', 18);
axis([-40,80,-60,80], 'equal');


end


if(0)
figure(1)
plot3(color(1,:),color(2,:),color(3,:),'x')
xlabel( 'x' )
ylabel( 'y' )
zlabel( 'z' )
end
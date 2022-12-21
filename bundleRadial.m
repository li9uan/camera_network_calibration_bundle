% bundle(Kc,Rc,tc,Rp,tp,x,y)

function [Kc,Rc,tc,Rp,tp]=bundleRadial(Kc0,Rc0,tc0,Rp0,tp0)

global nc np xgrid ygrid kkRadial

% nx=5; ny=8;
% sideLengthX = 120;
% sideLengthY = 120;
% xgrid=reshape((ones(ny,1)*[0:nx-1])'*sideLengthX, ny*nx, 1)';
% ygrid=reshape(ones(nx,1)*[ny-1:-1:0]*sideLengthY, ny*nx, 1)';

% if (nargin==4) % testmode
%     nc=Kc0; np=Rc0; noise=tc0; noise2=Rp0;
%     [Kc0,Rc0,tc0,Rp0,tp0,x,y]=generate_test(nc,np,noise,noise2);
% end

param0=param(Kc0,Rc0,tc0,Rp0,tp0);
opt=optimset('Display','iter');
paramopt=lsqnonlin(@residuals,param0,[],[],opt);
[Kc,Rc,tc,Rp,tp]=deparam(paramopt);

if (nargin==4) % testmode
    errKc=norm(Kc0-Kc,'fro')/norm(Kc0)
    errRc=norm(Rc0-Rc,'fro')/norm(Rc0)
    errtc=norm(tc0-tc)/norm(tc0)
    errRp=norm(Rp0-Rp,'fro')/norm(Rp0)
    errtp=norm(tp0-tp)/norm(tp0)
end

%-------------------------------------------
function [projx, projy]=projection(params)

global nc np xgrid ygrid kkRadial

[Kc,Rc,tc,Rp,tp]=deparam(params);

projx=[]; projy=[];
for i=1:nc,
    PciE=[Rc(i*3+(-2:0),:) tc(i*3+(-2:0))];
    PciI=Kc(i*3+(-2:0),:);
    projxi=[];projyi=[];
    for j=1:np
        if(i==2&&j==8)
            test=1;
        end;
        tmp=PciE*[Rp(j*3+(-2:0),:) tp(j*3+(-2:0)); 0 0 0 1]*[xgrid; ygrid; zeros(size(xgrid)); ones(size(xgrid))];
        tmpR = [tmp(1,:)./tmp(3,:);tmp(2,:)./tmp(3,:)];
        Rsquare = tmpR(1,:).*tmpR(1,:)+tmpR(2,:).*tmpR(2,:);
        tmpUD(1,:) = (1+kkRadial(i,1)*Rsquare+kkRadial(i,2)*Rsquare.*Rsquare).*tmpR(1,:);
        tmpUD(2,:) = (1+kkRadial(i,1)*Rsquare+kkRadial(i,2)*Rsquare.*Rsquare).*tmpR(2,:);
        tmpUD(3,:) = ones(1, size(tmpR,2));         
        tmp = PciI*tmpUD;
        
        projxi=[projxi tmp(1,:)./tmp(3,:)];
        projyi=[projyi tmp(2,:)./tmp(3,:)];
    end
    projx=[projx; projxi];
    projy=[projy; projyi];
end

%-------------------------------------------
function res=residuals(params)
global x y xgrid ygrid nc np kkRadial

[projx, projy]=projection(params);
use=(x>0).*(y>0); % (0,0) coordinates would indicate something was not observed in the corresponding view
res=[use.*abs((projx-x)); use.*abs((projy-y))];

%-------------------------------------------
function params=param(Kc,Rc,tc,Rp,tp)

nc=size(tc,1)/3; 
np=size(tp,1)/3;
params=zeros(nc*(4+4+3)+np*(4+3),1);
for i=1:nc,
    Kci=Kc(i*3+(-2:0),:);
    Rci=Rc(i*3+(-2:0),:);
    tci=tc(i*3+(-2:0));
    params(i*11+(-10:0))=[Kci(1,1) Kci(2,2) Kci(1,3) Kci(2,3) R2q(Rci) tci']';
end
for i=1:np,
    Rpi=Rp(i*3+(-2:0),:);
    tpi=tp(i*3+(-2:0));
    params(nc*11+i*7+(-6:0))=[R2q(Rpi) tpi']';
end
params;

%-------------------------------------------
function [Kc,Rc,tc,Rp,tp]=deparam(params,nc,np)
global nc np 

Kc=zeros(3*nc,3); Rc=Kc; tc=zeros(3*nc,1);
Rp=zeros(3*np,3); tp=zeros(3*np,1);
for i=1:nc,
    Kc(i*3+(-2:0),:)=[params(i*11-10) 0 params(i*11-8);
                       0 params(i*11-9) params(i*11-7);
                       0 0 1];
    Rc(i*3+(-2:0),:)=q2R(params(i*11+(-6:-3)));
    tc(i*3+(-2:0))=params(i*11+(-2:0));
end
for i=1:np,
    Rp(i*3+(-2:0),:)=q2R(params(nc*11+i*7+(-6:-3)));
    tp(i*3+(-2:0))=params(nc*11+i*7+(-2:0));
end

%-------------------------------------------
function R=q2R(q)
q=q/norm(q); %normalize, just to make sure
a=q(1); b=q(2); c=q(3); d=q(4);
R=[a*a+b*b-c*c-d*d 2*b*c-2*a*d 2*a*c+2*b*d;
   2*a*d+2*b*c a*a-b*b+c*c-d*d 2*c*d-2*a*b;
   2*b*d-2*a*c 2*a*b+2*c*d a*a-b*b-c*c+d*d];

%-------------------------------------------
function q=R2q(R)
[V,D]=eig(R);
d=diag(D);
[tmp,i]=max(real(d)); % find index of eigenvector corresponding to eigenvalue 1, i.e. rotation axis
[tmp,i2]=min(real(d));
dmin=real(d(i2));
cosa=sqrt((1+dmin)/2);
sina=sqrt(1-cosa^2);
q1(1,1:4)=[cosa sina*V(:,i)'];
q1(2,1:4)=[cosa -sina*V(:,i)'];
%q1(3,1:4)=[-cosa sina*V(:,i)']
%q1(4,1:4)=[-cosa -sina*V(:,i)']

mindif=100;
for i=1:2,
    dif=norm(q2R(q1(i,:))-R,'fro');
    if (dif<mindif),
        mindif=dif;
        q=q1(i,:);
        i;
    end
end
norm(q2R(q)-R,'fro');
return

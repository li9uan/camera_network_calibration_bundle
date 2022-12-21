function [Kc,Rc,tc,Rp,tp]=bundleRadialFixK(Rc0,tc0,Rp0,tp0)

global fixK nc np xgrid ygrid kkRadial

param0=paramFixK(Rc0,tc0,Rp0,tp0);

opt=optimset('Display','iter'); %,'TolFun',1e-10,'TolX',1e-10);
    
paramopt=lsqnonlin(@residualsFixK,param0,[],[],opt);
[Rc,tc,Rp,tp]=deparamFixK(paramopt);
Kc = fixK;

%-------------------------------------------
function [projx, projy]=projectionFixK(params)
global fixK nc np xgrid ygrid kkRadial x y

% decompose optimization parameters into camera rotations & translations
% and board rotations & translations
[Rc,tc,Rp,tp]=deparamFixK(params);
projx=[]; projy=[];

% loop through cameras
for i=1:nc
    % construct extrinsics
    PciE=[Rc(i*3+(-2:0),:) tc(i*3+(-2:0))];
    % construct intrinsics
    PciI=fixK(i*3+(-2:0),:);
    projxi=[];projyi=[];
    % loop through checkerboards
    for j=1:np
        tmp = PciE*[Rp(j*3+(-2:0),:) tp(j*3+(-2:0)); 0 0 0 1]*[xgrid; ygrid; zeros(size(xgrid)); ones(size(xgrid))];
        tmpR = [tmp(1,:)./tmp(3,:);tmp(2,:)./tmp(3,:)];
        Rsquare = tmpR(1,:).*tmpR(1,:)+tmpR(2,:).*tmpR(2,:);
        
        % here radial distortion is fixed to be zero
        tmpUD(1,:) = (1+kkRadial(i,1)*Rsquare+kkRadial(i,2)*Rsquare.*Rsquare).*tmpR(1,:);
        tmpUD(2,:) = (1+kkRadial(i,1)*Rsquare+kkRadial(i,2)*Rsquare.*Rsquare).*tmpR(2,:);
        tmpUD(3,:) = ones(1, size(tmpR,2));         
        tmp = PciI*tmpUD;
        
        % normalize image coordinates
        projxi = [projxi tmp(1,:)./tmp(3,:)];
        projyi = [projyi tmp(2,:)./tmp(3,:)];
    end
    
    % stack reprojection to projx & projy
    projx = [projx; projxi];
    projy = [projy; projyi];
end

%-------------------------------------------
function res=residualsFixK(params)
global x y xgrid ygrid nc np kkRadial

[projx, projy]=projectionFixK(params);

% use is a flag to indicate whether the board pose is observed from the
% camera
use=(x>0).*(y>0); 
projx = projx.*use;
projy = projy.*use;

% residual is diff between the reprojection and the image corner detection
% when performing evaluation in lsqnonlin, square of each element in res
% will be computed, so it is computing the square root as cost value
res=[(projx-x); (projy-y)];

%-------------------------------------------
function params=paramFixK(Rc,tc,Rp,tp)

nc=size(tc,1)/3; 
np=size(tp,1)/3;
params=zeros(nc*(4+4+3)+np*(4+3),1);
for i=1:nc
    Rci=Rc(i*3+(-2:0),:);
    tci=tc(i*3+(-2:0));
    params(i*7+(-6:0))=[R2q(Rci) tci']';
end
for i=1:np
    Rpi=Rp(i*3+(-2:0),:);
    tpi=tp(i*3+(-2:0));
    params(nc*7+i*7+(-6:0))=[R2q(Rpi) tpi']';
end
params;

%-------------------------------------------
function [Rc,tc,Rp,tp]=deparamFixK(params,nc,np)
global nc np 

Rc=zeros(3*nc,3); tc=zeros(3*nc,1);
Rp=zeros(3*np,3); tp=zeros(3*np,1);
for i=1:nc
    Rc(i*3+(-2:0),:)=q2R(params(i*7+(-6:-3)));
    tc(i*3+(-2:0))=params(i*7+(-2:0));
end
for i=1:np
    Rp(i*3+(-2:0),:)=q2R(params(nc*7+i*7+(-6:-3)));
    tp(i*3+(-2:0))=params(nc*7+i*7+(-2:0));
end
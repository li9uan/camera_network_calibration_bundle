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

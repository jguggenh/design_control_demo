
clear all

% liftbox_Jacob1=rosbag('liftbox1_jacob.bag');
% liftbox_Jacob2=rosbag('liftbox1_jacob.bag');
% liftbox_Jacob3=rosbag('liftbox3_jacob.bag');
% mess1=rosbag('MessAround_jacob1.bag');
% mess2=rosbag('MessAround_jacob2.bag');
% mess3=rosbag('MessAround_jacob3.bag');
% noise=rosbag('Zero_Jacob.bag');
% lift_lighterbox_Jacob1=rosbag('lift_lighterbox_Jacob1.bag');
% lift_lighterbox_Jacob2=rosbag('lift_lighterbox_Jacob2.bag');
% lift_lighterbox_Jacob3=rosbag('lift_lighterbox_Jacob3.bag');
% 

liftbox_Jacob1_left=readMessages(select(rosbag('liftbox1_jacob.bag'),'Topic','/forceGlovePubLeft'));
liftbox_Jacob1_right=readMessages(select(rosbag('liftbox1_jacob.bag'),'Topic','/forceGlovePubRight'));
liftbox_Jacob2_left=readMessages(select(rosbag('liftbox2_jacob.bag'),'Topic','/forceGlovePubLeft'));
liftbox_Jacob2_right=readMessages(select(rosbag('liftbox2_jacob.bag'),'Topic','/forceGlovePubRight'));
liftbox_Jacob3_left=readMessages(select(rosbag('liftbox3_jacob.bag'),'Topic','/forceGlovePubLeft'));
liftbox_Jacob3_right=readMessages(select(rosbag('liftbox3_jacob.bag'),'Topic','/forceGlovePubRight'));

lift_lighterbox_Jacob1_left=readMessages(select(rosbag('lift_lighterbox_Jacob1.bag'),'Topic','/forceGlovePubLeft'));
lift_lighterbox_Jacob1_right=readMessages(select(rosbag('lift_lighterbox_Jacob1.bag'),'Topic','/forceGlovePubRight'));
lift_lighterbox_Jacob2_left=readMessages(select(rosbag('lift_lighterbox_Jacob2.bag'),'Topic','/forceGlovePubLeft'));
lift_lighterbox_Jacob2_right=readMessages(select(rosbag('lift_lighterbox_Jacob2.bag'),'Topic','/forceGlovePubRight'));
lift_lighterbox_Jacob3_left=readMessages(select(rosbag('lift_lighterbox_Jacob3.bag'),'Topic','/forceGlovePubLeft'));
lift_lighterbox_Jacob3_right=readMessages(select(rosbag('lift_lighterbox_Jacob3.bag'),'Topic','/forceGlovePubRight'));

MessAround_Jacob1_left=readMessages(select(rosbag('MessAround_jacob1.bag'),'Topic','/forceGlovePubLeft'));
MessAround_Jacob1_right=readMessages(select(rosbag('MessAround_jacob1.bag'),'Topic','/forceGlovePubRight'));
MessAround_Jacob2_left=readMessages(select(rosbag('MessAround_jacob2.bag'),'Topic','/forceGlovePubLeft'));
MessAround_Jacob2_right=readMessages(select(rosbag('MessAround_jacob2.bag'),'Topic','/forceGlovePubRight'));
MessAround_Jacob3_left=readMessages(select(rosbag('MessAround_jacob3.bag'),'Topic','/forceGlovePubLeft'));
MessAround_Jacob3_right=readMessages(select(rosbag('MessAround_jacob3.bag'),'Topic','/forceGlovePubRight'));

Noise_Jacob_left=readMessages(select(rosbag('Zero_Jacob.bag'),'Topic','/forceGlovePubLeft'));
Noise_Jacob_right=readMessages(select(rosbag('Zero_Jacob.bag'),'Topic','/forceGlovePubRight'));


for i=1:min(length(liftbox_Jacob1_left),length(liftbox_Jacob1_right))    
liftbox_Jacob1_data(i,:)=[liftbox_Jacob1_left{i}.Data',liftbox_Jacob1_right{i}.Data'];
end

for i=1:min(length(liftbox_Jacob2_left),length(liftbox_Jacob2_right))
liftbox_Jacob2_data(i,:)=[liftbox_Jacob2_left{i}.Data',liftbox_Jacob2_right{i}.Data'];
end

for i=1:min(length(liftbox_Jacob3_left),length(liftbox_Jacob3_right))
liftbox_Jacob3_data(i,:)=[liftbox_Jacob3_left{i}.Data',liftbox_Jacob3_right{i}.Data'];
end

for i=1:min(length(lift_lighterbox_Jacob1_left),length(lift_lighterbox_Jacob1_right))
lift_lighterbox_Jacob1_data(i,:)=[lift_lighterbox_Jacob1_left{i}.Data', lift_lighterbox_Jacob1_right{i}.Data'];
end

for i=1:min(length(lift_lighterbox_Jacob2_left),length(lift_lighterbox_Jacob2_right))
lift_lighterbox_Jacob2_data(i,:)=[lift_lighterbox_Jacob2_left{i}.Data', lift_lighterbox_Jacob2_right{i}.Data'];
end

for i=1:min(length(lift_lighterbox_Jacob3_left),length(lift_lighterbox_Jacob3_right))
lift_lighterbox_Jacob3_data(i,:)=[lift_lighterbox_Jacob3_left{i}.Data', lift_lighterbox_Jacob3_right{i}.Data'];
end

for i=1:min(length(MessAround_Jacob1_left),length(MessAround_Jacob1_right))
MessAround_Jacob1_data(i,:)=[MessAround_Jacob1_left{i}.Data',MessAround_Jacob1_right{i}.Data'];
end

for i=1:min(length(MessAround_Jacob2_left),length(MessAround_Jacob2_right))
MessAround_Jacob2_data(i,:)=[MessAround_Jacob2_left{i}.Data',MessAround_Jacob2_right{i}.Data'];
end

for i=1:min(length(MessAround_Jacob3_left),length(MessAround_Jacob3_right))
MessAround_Jacob3_data(i,:)=[MessAround_Jacob3_left{i}.Data',MessAround_Jacob3_right{i}.Data'];
end

for i=1:min(length(Noise_Jacob_left),length(Noise_Jacob_right))
Noise_Jacob_Data(i,:)=[Noise_Jacob_left{i}.Data',Noise_Jacob_right{i}.Data'];
end

[lb1coeff,lb1score,lb1latent,lb1tsquared,lb1explained,lb1mu]=pca(double(lift_lighterbox_Jacob1_data));
%[lb2coeff,lb2score,lb2latent,lb2tsquared,lb2explained,lb2mu]=pca(double(lift_lighterbox_Jacob2_data));
%[lb3coeff,lb3score,lb3latent,lb3tsquared,lb3explained,lb3mu]=pca(double(lift_lighterbox_Jacob3_data));
%[lbcoeff,lbscore,lblatent,lbtsquared,lbexplained,lbmu]=pca([double(lift_lighterbox_Jacob1_data);double(lift_lighterbox_Jacob2_data);double(lift_lighterbox_Jacob3_data)]);

%reconstruct which sensors contribute to  blatent 6,7,8,9,10
%pick some that seen doable.
%check to see if exists in nullspace
%pick three fingers on left or right hand, plot those, then draw on three axes from pca from those fingers.
%fancy shading with oval
%LOOK FOR GRASPING PAPER NULLSPACE CONVENTION 
[b1coeff,b1score,b1latent,b1tsquared,b1explained,b1mu]=pca(double(liftbox_Jacob1_data));
[b2coeff,b2score,b2latent,b2tsquared,b2explained,b2mu]=pca(double(liftbox_Jacob2_data));
[b3coeff,b3score,b3latent,b3tsquared,b3explained,b3mu]=pca(double(liftbox_Jacob3_data));
[bcoeff,bscore,blatent,btsquared,bexplained,bmu]=pca([double(liftbox_Jacob1_data);double(liftbox_Jacob2_data);double(liftbox_Jacob3_data)]);


%%
close all
figure
hold on
l=[1,3,2];
plot3(double(MessAround_Jacob1_data(:,l(1))),double(MessAround_Jacob1_data(:,l(2))),double(MessAround_Jacob1_data(:,l(3))),'ro')
plot3(double(lift_lighterbox_Jacob1_data(:,l(1))),double(lift_lighterbox_Jacob1_data(:,l(2))),double(lift_lighterbox_Jacob1_data(:,l(3))),'b.')
xlabel(l(1))
ylabel(l(2))
zlabel(l(3))
view(45,15)

figure
hold on
l=[6,8,7];
plot3(double(MessAround_Jacob1_data(:,l(1))),double(MessAround_Jacob1_data(:,l(2))),double(MessAround_Jacob1_data(:,l(3))),'ro')
plot3(double(lift_lighterbox_Jacob1_data(:,l(1))),double(lift_lighterbox_Jacob1_data(:,l(2))),double(lift_lighterbox_Jacob1_data(:,l(3))),'b.')
xlabel(l(1))
ylabel(l(2))
zlabel(l(3))
view(45,15)


figure
plot3(bscore(:,1),bscore(:,2),bscore(:,3),'g.')
figure
plot(bscore(:,1),bscore(:,2),'k.')
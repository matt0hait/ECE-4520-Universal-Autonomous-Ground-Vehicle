load('Motor_calc');
tDesc = 0.002;

%Uncoment below to grab data from Simulink
    %DesiredvsOutput(:,1) = Desired.time
    %DesiredvsOutput(:,2) = Desired.signals.values
    %DesiredvsOutput(:,3) = WheelSpeed.signals.values(1,1,1:end)

% Create Discrete-Time TF Dynamic Model
data = iddata(DesiredvsOutput(:,3),DesiredvsOutput(:,2),tDesc);
DTsys = tfest(data,1);

% Graph DTF Sym vs Input
f1 = figure;
set(f1,'name','Wheel Speed - Desired vs Measured [m/s]','numbertitle','off');
subplot(2,1,1);
plot(data);
subplot(2,1,2);
bode(DTsys);

% Show Root Locus of DTF
f2 = figure;
set(f2,'name','Wheel Speed Root Locus','numbertitle','off');
rlocus(DTsys);

% Create State-Space Dynamic Model
SSorder = 10;
SSsys = n4sid(data,SSorder);

% Compare State-Space Model Orders 1-10
    %{
        cost_func = 'NRMSE';
        for c = 1:10
            SSorder = c;
            eval(['SSsys' num2str(c) '= n4sid(data,SSorder)']);
            eval(['y_est = sim(SSsys' num2str(c) ',data)']);
            fit(c) = goodnessOfFit(y_est.y,DesiredvsOutput(:,3),cost_func);
        end
        SSFIT = figure;
        set(SSFIT,'name','State-Space Model Fit Comparison','numbertitle','off');
        compare(data,SSsys1,SSsys3,SSsys5,SSsys7,SSsys9,SSsys10);
        xlabel('Seconds')
        ylabel('Speed [m/s]')
    %}

% Compare SS vs DTF Models
ffite = figure;
compare(data,DTsys,SSsys);
legend('Measured','DTF Sim','SS Sim')
xlabel('Seconds')
ylabel('Speed [m/s]')
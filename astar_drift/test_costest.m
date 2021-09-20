%Finding minimum cost balanced between Air resistance and boardnett power
v=0:0.1:36.2;
d=1;
Ebnair=Pbn.*d./v+fair.*d*v.^2;
figure
plot(v,Ebnair/1000);
xlabel('velocity (m/s)');
ylabel('Energy (kJ)');
min(Ebnair)
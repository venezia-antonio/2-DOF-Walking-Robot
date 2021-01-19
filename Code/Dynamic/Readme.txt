Lo script di Matlab presente in questo folder permette di determinare le coppie elastiche e attuate che agiscono sul walking robot quando esso compie un passo di camminata. Nel modello dinamico vengono considerate in questo caso momenti e forze di inerzia.
Varie simulazioni sono state effettuate e salvate in opportuni file .mat, come di seguito:
FirstSimulation.mat -> w = 0.1rad/s, bodyweight = 16kg, comBody = [P8W(1,end);P8W(2,end);P8W(3,end);1]
SecondSimulation.mat -> w = 0.1rad/s, bodyweight = 16kg, comBody = [P8W(1,end)+0.3;P8W(2,end);P8W(3,end);1]
ThirdSimulation.mat -> w = 0.1rad/s, bodyweight = 30kg, comBody = [P8W(1,end)+0.3;P8W(2,end);P8W(3,end);1]
Fourthimulation.mat -> w = 5rad/s, bodyweight = 20kg, comBody = [P8W(1,end)+0.3;P8W(2,end);P8W(3,end);1]

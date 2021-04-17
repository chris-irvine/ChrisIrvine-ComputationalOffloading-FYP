function initialiseText(offload, tRRT, tAStar, a, time2goalRRT, time2goalAStar, c)

% Gnerate Additional parameters
tNet = 0.01;
b = time2goalAStar/(time2goalRRT*2500);
RHS = tNet + tRRT;
LHS = tAStar + b + c;

% Convert numbers to stings
oflS = num2str(offload);
rrtS = num2str(tRRT,3);
astarS = num2str(tAStar,3);
aS = num2str(a);
time2goalRRTS = num2str(time2goalRRT,5);
time2goalAStarS = num2str(time2goalAStar,5); 
cS = num2str(c);
RHSS = num2str(RHS);
LHSS = num2str(LHS);

if offload
    signage = {'  <  '};
else
    signage = {'  >  '};
end

% Draw sting on plot
rrtTxt = text(14.0804+7.5, 16.2976-2.5, strcat({'RRT Path Plan:  '},rrtS));
astarTxt = text(14.0804+7.5, 16.2976-3.5, strcat({'A* Path Plan:  '},astarS));
aTxt = text(14.0804+7.5, 16.2976-4.5, strcat({'a:  '},aS));
time2goalRRTTxt = text(14.0804+7.5, 16.2976-5.5, strcat({'T2G RRT:  '},time2goalRRTS));
time2goalAStarTxt = text(14.0804+7.5, 16.2976-6.5, strcat({'T2G A*:  '},time2goalAStarS));
cTxt = text(14.0804+7.5, 16.2976-7.5, strcat({'c:  '},cS));
lblTxt = text(14.0804+7.5, 16.2976-9.5, ' Remote        Local', 'FontSize', 14, 'FontWeight','Bold');
equTxt = text(14.0804+7.5, 16.2976-11, strcat(RHSS, signage, LHSS), 'FontSize', 14, 'FontWeight','Bold');
offloadTxt = text(14.0804+7.5, 16.2976-12.5, strcat({'Offload:  '},oflS),'FontSize', 14, 'FontWeight','Bold');

end


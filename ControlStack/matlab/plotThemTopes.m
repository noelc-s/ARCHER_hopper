system('sshpass -p "amberlab" scp noelcs@10.0.0.6:~/ARCHER_hopper/ControlStack/output.m ~/repos/ARCHER_hopper/ControlStack/')
system('sshpass -p "amberlab" scp noelcs@10.0.0.6:~/ARCHER_hopper/ControlStack/stored_graph.m ~/repos/ARCHER_hopper/ControlStack/')

run('../output.m')
run('../stored_graph.m')


clf

hold on;
for i = 1:10:size(NominalEdges,1)
plot([Points(NominalEdges(i,1)+1,1) Points(NominalEdges(i,2)+1,1)],[Points(NominalEdges(i,1)+1,2) Points(NominalEdges(i,2)+1,2)],'k')
i
end
axis equal

hold on;
for i = 1:size(Edges,1)
plot([Points(Edges(i,1)+1,1) Points(Edges(i,2)+1,1)],[Points(Edges(i,1)+1,2) Points(Edges(i,2)+1,2)],'r')
i
end
axis equal

s = load('s.txt');
s1 = s(:,2:4); s2 = s(:,5:7);
s1 = sortrows(s1, 1);
s2 = sortrows(s2, 1);

x = s1(:,1);
y = s1(:,2);

x2 = s2(:,1);
y2 = s2(:,2);

plot(x,y);
hold all
plot(x2,y2);
hold off
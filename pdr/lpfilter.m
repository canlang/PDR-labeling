function y = lpfilter(X,order,freqHZ)
Fs=3600;
n=order;
Wn=freqHZ;
Fn=Fs/2;
ftype = 'low';
[b,a] = butter(n,Wn/Fn,ftype);
y=filter(b,a,X);


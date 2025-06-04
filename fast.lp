#const h=40.                 % horizon (override with -c h=NN)
t(0..h). step(1..h). dir(1,0;-1,0;0,1;0,-1).

node(N,X,Y):-init(object(node,N),value(at,pair(X,Y))).
hw(X,Y)    :-init(object(highway,_),value(at,pair(X,Y))).
st(P,X,Y)  :-init(object(pickingStation,P),value(at,pair(X,Y))).
bot(R)     :-init(object(robot,R),_).
sh(S)      :-init(object(shelf,S),_).
pr(I)      :-init(object(product,I),_).
ord(O)     :-init(object(order,O),value(pickingStation,_)).

at(R,X,Y,0):-init(object(robot,R),value(at,pair(X,Y))).
at(S,X,Y,0):-init(object(shelf,S),value(at,pair(X,Y))).
stock(S,I,Q,0):-init(object(product,I),value(on,pair(S,Q))).
need(O,I,Q,0):-init(object(order,O),value(line,pair(I,Q))).
os(O,P)       :-init(object(order,O),value(pickingStation,P)).
stxy(O,X,Y)   :-os(O,P),st(P,X,Y).

{ act(R,move(DX,DY),T):dir(DX,DY); act(R,pick,T); act(R,drop,T);
  act(R,del(O,I),T):ord(O),pr(I) }<=1 :-bot(R),step(T).

at(R,X2,Y2,T):-act(R,move(DX,DY),T),at(R,X1,Y1,T-1),
               X2=X1+DX,Y2=Y1+DY,node(_,X2,Y2).
mov(R,T):-act(R,move(_,_),T). at(R,X,Y,T):-bot(R),at(R,X,Y,T-1),not mov(R,T).

carry(R,S,T):-act(R,pick,T),at(R,X,Y,T-1),at(S,X,Y,T-1).
carry(R,S,T):-carry(R,S,T-1),not act(R,drop,T).

at(S,X,Y,T):-act(R,drop,T),carry(R,S,T-1),at(R,X,Y,T-1).
chg(S,T):-act(_,pick,T),carry(_,S,T);act(_,drop,T),carry(_,S,T-1).
at(S,X,Y,T):-sh(S),at(S,X,Y,T-1),not chg(S,T).
shelfAt(X,Y,T):-sh(S),at(S,X,Y,T).

:-act(_,del(O,I),T),need(O,I,0,T-1).
:-act(R,del(O,I),T),carry(R,S,T-1),stock(S,I,0,T-1).
:-act(R,del(O,_),T),at(R,X,Y,T-1),not stxy(O,X,Y).

stock(S,I,Q-1,T):-act(_,del(_,I),T),carry(_,S,T-1),stock(S,I,Q,T-1),Q>0.
stock(S,I,Q,T):-stock(S,I,Q,T-1),not stc(S,I,T).
stc(S,I,T):-act(_,del(_,I),T),carry(_,S,T-1).

need(O,I,Q-1,T):-act(_,del(O,I),T),need(O,I,Q,T-1),Q>0.
need(O,I,Q,T):-need(O,I,Q,T-1),not ndc(O,I,T).
ndc(O,I,T):-act(_,del(O,I),T).

:-at(R1,X,Y,T),at(R2,X,Y,T),bot(R1;R2),R1<R2.
:-at(S1,X,Y,T),at(S2,X,Y,T),sh(S1;S2),S1<S2.
:-at(R,X,Y,T),at(S,X,Y,T),bot(R),sh(S),not carry(R,S,T).
:-act(R1,move(DX,DY),T),act(R2,move(-DX,-DY),T),
  at(R1,X,Y,T-1),X2=X+DX,Y2=Y+DY,at(R2,X2,Y2,T-1).
:-act(R,pick,T),carry(R,_,T-1).
:-act(R,pick,T),at(R,X,Y,T-1),not shelfAt(X,Y,T-1).
:-act(R,drop,T),not carry(R,_,T-1).
:-act(R,drop,T),at(R,X,Y,T-1),hw(X,Y).

:-need(_,_,Q,h),Q>0. latest(T):-act(_,_,T).
#minimize{T@1:latest(T)}.  #show act/3.

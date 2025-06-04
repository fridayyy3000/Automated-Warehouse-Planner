
% ====================================================================
%  smart_store.lp  --  compact ASP model with fresh names
% ====================================================================

#const H = 24.           % planning horizon (override: -c H=40)
step(0..H).

% -------- static data from instance ---------------------------------
agent(R)                 :- init(object(robot,R),_).
box(S)                   :- init(object(shelf,S),_).

coord(agent(R),0,X,Y)    :- init(object(robot,R), value(at,pair(X,Y))).
coord(box(S),0,X,Y)      :- init(object(shelf,S), value(at,pair(X,Y))).

fastlane(X,Y)            :- init(object(highway,_), value(at,pair(X,Y))).

need(O,P,Q)              :- init(object(order,O), value(line,pair(P,Q))).
dest(O,Stn)              :- init(object(order,O), value(pickingStation,Stn)).
dpos(Stn,X,Y)            :- init(object(pickingStation,Stn), value(at,pair(X,Y))).

inv(P,S,Q)               :- init(object(product,P), value(on,pair(S,Q))).

cell(X,Y)                :- init(object(node,_), value(at,pair(X,Y))).

% initial quantities
leftInv(P,S,0,Q)         :- inv(P,S,Q).
leftNeed(O,P,0,Q)        :- need(O,P,Q).

% -------- directions -------------------------------------------------
dir(north; south; east; west).
vec(north,0,1). vec(south,0,-1). vec(east,1,0). vec(west,-1,0).

% ====================================================================
%  ACTIONS AND EFFECTS
% ====================================================================
{ move(R,D,T):dir(D); take(R,T); drop(R,T); serve(R,O,P,T):leftNeed(O,P,T-1,_) } 1
    :- agent(R), step(T;T>0).

% robot motion
coord(agent(R),T,X2,Y2) :-
    move(R,D,T), coord(agent(R),T-1,X1,Y1), vec(D,DX,DY),
    X2=X1+DX, Y2=Y1+DY, cell(X2,Y2).
coord(agent(R),T,X,Y) :- coord(agent(R),T-1,X,Y), not move(R,_,T).

% carrying relation
carry(R,S,T) :- take(R,T), coord(agent(R),T-1,X,Y), coord(box(S),T-1,X,Y).
carry(R,S,T) :- carry(R,S,T-1), not drop(R,T).

% box position
coord(box(S),T,X,Y) :- carry(_,S,T), coord(agent(_),T,X,Y).
coord(box(S),T,X,Y) :- coord(box(S),T-1,X,Y), not carry(_,S,T).

% amount shipped at step
ship(R,O,P,S,T,N) :-
    serve(R,O,P,T), carry(R,S,T-1),
    leftInv(P,S,T-1,I), leftNeed(O,P,T-1,Rq),
    N = #min { I ; Rq }.

sentBox(P,S,T,Sum) :- Sum = #sum { N : ship(_,_,P,S,T,N) }.
sentOrder(O,P,T,Sum) :- Sum = #sum { N : ship(_,O,P,_,T,N) }.

leftInv(P,S,T,I2) :-
    leftInv(P,S,T-1,I1), sentBox(P,S,T,Sent), I2 = I1 - Sent.
leftNeed(O,P,T,R2) :-
    leftNeed(O,P,T-1,R1), sentOrder(O,P,T,Sent), R2 = R1 - Sent.

% ====================================================================
%  CONSTRAINTS
% ====================================================================
:- serve(R,O,P,T), coord(agent(R),T-1,X,Y),
   dest(O,Stn), dpos(Stn,Xs,Ys), (X!=Xs;Y!=Ys).
:- serve(R,O,P,T), not carry(R,_,T-1).

:- coord(agent(R1),T,X,Y), coord(agent(R2),T,X,Y), R1<R2.
:- coord(box(S1),T,X,Y), coord(box(S2),T,X,Y), S1<S2,
   not carry(_,S1,T), not carry(_,S2,T).
:- coord(box(S),T,X,Y), fastlane(X,Y), not carry(_,S,T).

% ====================================================================
%  OPTIMISATION
% ====================================================================
done(T) :- step(T), not leftNeed(_,_,T,_).
#minimize { T@1 : done(T) }.

#show move/3.
#show take/2.
#show drop/2.
#show serve/4.

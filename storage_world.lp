
% ====================================================================
%  storage_world.lp  –  Incremental ASP encoding (renamed predicates)
% ====================================================================

#program base.

dir(up;down;left;right).
vec(up,0,1). vec(down,0,-1). vec(left,-1,0). vec(right,1,0).

bot(B)                 :- init(object(robot,B),_).
crate(C)               :- init(object(shelf,C),_).

pos(bot(B),0,X,Y)      :- init(object(robot,B), value(at,pair(X,Y))).
pos(crate(C),0,X,Y)    :- init(object(shelf,C), value(at,pair(X,Y))).

fastlane(X,Y)          :- init(object(highway,_), value(at,pair(X,Y))).

order(O,Pr,Qty)        :- init(object(order,O), value(line,pair(Pr,Qty))).
target(O,S)            :- init(object(order,O), value(pickingStation,S)).
tpos(S,X,Y)            :- init(object(pickingStation,S), value(at,pair(X,Y))).

stock(Pr,C,Qty)        :- init(object(product,Pr), value(on,pair(C,Qty))).

grid(X,Y)              :- init(object(node,_), value(at,pair(X,Y))).

remainStock(Pr,C,0,Q)  :- stock(Pr,C,Q).
remainNeed(O,Pr,0,Q)   :- order(O,Pr,Q).

carried(C,T) :- carry(_,C,T).

#program step(k).

time(k).

{ move(B,D,k):dir(D); pick(B,k); drop(B,k); sup(B,O,P,k):remainNeed(O,P,k-1,_) } 1 :- bot(B).

pos(bot(B),k,X2,Y2) :- move(B,D,k), pos(bot(B),k-1,X1,Y1), vec(D,DX,DY),
    X2=X1+DX, Y2=Y1+DY, grid(X2,Y2).
pos(bot(B),k,X,Y) :- pos(bot(B),k-1,X,Y), not move(B,_,k).

carry(B,C,k) :- pick(B,k), pos(bot(B),k-1,X,Y), pos(crate(C),k-1,X,Y).
carry(B,C,k) :- carry(B,C,k-1), not drop(B,k).

pos(crate(C),k,X,Y) :- carry(_,C,k), pos(bot(_),k,X,Y).
pos(crate(C),k,X,Y) :- pos(crate(C),k-1,X,Y), not carried(C,k).

shipAmt(B,O,P,C,k,A) :- sup(B,O,P,k), carry(B,C,k-1),
    remainStock(P,C,k-1,S), remainNeed(O,P,k-1,R), A = #min{ S ; R }.

shipped(P,C,k,Sum)   :- remainStock(P,C,k-1,_), Sum = #sum {A: shipAmt(_,_,P,C,k,A)}.
sent(O,P,k,Sum)      :- remainNeed(O,P,k-1,_),  Sum = #sum {A: shipAmt(_,O,P,_,k,A)}.

remainStock(P,C,k,Q2) :- remainStock(P,C,k-1,Q1), shipped(P,C,k,S), Q2 = Q1 - S.
remainNeed(O,P,k,R2)  :- remainNeed(O,P,k-1,R1), sent(O,P,k,S),   R2 = R1 - S.

:- sup(B,O,P,k), pos(bot(B),k-1,X,Y), target(O,S), tpos(S,Xs,Ys), (X!=Xs;Y!=Ys).
:- sup(B,O,P,k), not carry(B,_,k-1).

:- pos(bot(B1),k,X,Y), pos(bot(B2),k,X,Y), B1 < B2.
:- move(B1,D1,k), move(B2,D2,k), B1 < B2,
   pos(bot(B1),k-1,X1,Y1), pos(bot(B2),k-1,X2,Y2),
   vec(D1,DX1,DY1), vec(D2,DX2,DY2),
   X1+DX1=X2, Y1+DY1=Y2, X2+DX2=X1, Y2+DY2=Y1.

:- pos(crate(C1),k,X,Y), pos(crate(C2),k,X,Y), C1<C2,
   not carried(C1,k), not carried(C2,k).

:- pos(crate(C),k,X,Y), fastlane(X,Y), not carried(C,k).

goal(k) :- time(k), not remainNeed(_,_,k,_).
#minimize { k@1 : goal(k) }.

#show move/3.
#show pick/2.
#show drop/2.
#show sup/4.

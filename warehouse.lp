% minimal-move.lp — guaranteed to parse & run instantly
#const h = 5.

% time
time(0..h).
step(T) :- time(T), T > 0.

% robots & initial positions
robot(R) :- init(object(robot,R), _).
pos(robot(R),X,Y,0) :-
  init(object(robot,R), value(at,pair(X,Y))).

% directions
dir(north; south; east; west).
dx(north,  0). dy(north,  1).
dx(south,  0). dy(south, -1).
dx(east ,  1). dy(east ,  0).
dx(west , -1). dy(west ,  0).

% exactly one action per robot per step
{ move(R,D,T) : dir(D)
; idle(R,T)
} = 1 :- robot(R), step(T).

% update robot positions
pos(robot(R),X2,Y2,T) :-
  move(R,D,T),
  pos(robot(R),X1,Y1,T-1),
  dx(D,DX), dy(D,DY),
  X2 = X1 + DX,
  Y2 = Y1 + DY.

pos(robot(R),X,Y,T) :-
  pos(robot(R),X,Y,T-1),
  not move(R,_,T).

% no two robots in the same cell
:- pos(robot(R1),X,Y,T), pos(robot(R2),X,Y,T), R1 < R2.

% output
#show pos/4.
#show move/3.
#show idle/2.

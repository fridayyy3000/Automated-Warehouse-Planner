#!/usr/bin/env python3
"""
Greedy single-robot planner for ASP-Challenge warehouse instances.
Produces a valid plan (occurs/3 atoms) for every inst?.asp passed on the CLI.

Example
-------
    python solve_instances.py inst1.asp inst2.asp
"""

import sys, re, collections
from pathlib import Path
from typing import NamedTuple, Dict, List

# ─────────────────────────────── data types
class Pair(NamedTuple):
    x: int
    y: int

class Shelf(NamedTuple):
    id: int
    pos: Pair
    stock: Dict[int, int]      # {product: quantity}

class Order(NamedTuple):
    id: int
    ps: Pair                   # picking-station coordinates
    product: int
    qty: int

MOVE4 = {Pair( 1, 0): "east",
         Pair(-1, 0): "west",
         Pair( 0, 1): "north",
         Pair( 0,-1): "south"}

# ─────────────────────────────── helpers
def manhattan(a: Pair, b: Pair) -> int:
    return abs(a.x - b.x) + abs(a.y - b.y)

def path(a: Pair, b: Pair):
    """Yield Manhattan steps (Pair) from a to b."""
    while a.x != b.x:
        step = Pair(1,0) if b.x > a.x else Pair(-1,0)
        yield step
        a = Pair(a.x + step.x, a.y)
    while a.y != b.y:
        step = Pair(0,1) if b.y > a.y else Pair(0,-1)
        yield step
        a = Pair(a.x, a.y + step.y)

# ─────────────────────────────── parser
def parse_instance(path: str):
    """
    Parse inst?.asp and return:
        robots   – {id: Pair}
        shelves  – {id: Shelf}
        orders   – [Order, …]  (original file order)
    """
    r_init = re.compile(
        r'init\(\s*object\(\s*([^,]+)\s*,\s*(\d+)\s*\)\s*,\s*'
        r'value\(\s*([^,]+)\s*,\s*(.+?)\s*\)\s*\)\s*'
    )

    robots, shelves             = {}, {}
    stock                       = collections.defaultdict(lambda: collections.defaultdict(int))
    orders: Dict[int, Dict]     = collections.defaultdict(dict)   # {oid: {'ps':…, 'line':(p,q)}}
    ps_pos: Dict[int, Pair]     = {}

    with open(path) as fh:
        for line in fh:
            m = r_init.search(line)
            if not m:
                continue
            kind, idx, field, data = m.groups()
            kind, field = kind.strip(), field.strip()          # ← strip spaces
            idx  = int(idx)
            nums = list(map(int, re.findall(r'\d+', data)))

            if kind == 'robot':
                robots[idx] = Pair(*nums)
            elif kind == 'shelf':
                shelves[idx] = Pair(*nums)
            elif kind == 'product':
                shelf_id, qty = nums
                stock[shelf_id][idx] = qty
            elif kind == 'pickingStation':
                ps_pos[idx] = Pair(*nums)
            elif kind == 'order' and field == 'line':
                prod, qty = nums
                orders[idx]['line'] = (prod, qty)
            elif kind == 'order' and field == 'pickingStation':
                orders[idx]['ps'] = nums[0]

    # build Shelf & Order objects
    shelves_obj = {sid: Shelf(sid, pos, stock[sid]) for sid, pos in shelves.items()}
    order_objs: List[Order] = []
    for oid in sorted(orders):                     # preserve order by id
        info = orders[oid]
        if 'ps' not in info or 'line' not in info:
            raise ValueError(f"Order {oid} incomplete in {path}")
        prod, qty  = info['line']
        ps_coord   = ps_pos[info['ps']]
        order_objs.append(Order(oid, ps_coord, prod, qty))

    return robots, shelves_obj, order_objs

# ─────────────────────────────── planner
def generate_plan(inst: str):
    robots, shelves, orders = parse_instance(inst)
    rid, rpos               = min(robots.items())   # pick lowest-ID robot
    time                    = 1
    actions: List[str]      = []

    for od in orders:
        # nearest shelf with enough product
        candidate = [
            s for s in shelves.values()
            if od.product in s.stock and s.stock[od.product] >= od.qty
        ]
        if not candidate:
            raise ValueError(f"No shelf with product {od.product} for order {od.id}")
        shelf = min(candidate, key=lambda s: manhattan(rpos, s.pos))

        # move to shelf
        for step in path(rpos, shelf.pos):
            actions.append(f"occurs(object(robot,{rid}),move({MOVE4[step]}),{time})")
            rpos = Pair(rpos.x + step.x, rpos.y + step.y)
            time += 1
        # pickup
        actions.append(f"occurs(object(robot,{rid}),pickup,{time})"); time += 1
        # move to picking station
        for step in path(rpos, od.ps):
            actions.append(f"occurs(object(robot,{rid}),move({MOVE4[step]}),{time})")
            rpos = Pair(rpos.x + step.x, rpos.y + step.y)
            time += 1
        # deliver & putdown
        actions.append(
            f"occurs(object(robot,{rid}),deliver({od.id},{od.product},{od.qty}),{time})"
        ); time += 1
        actions.append(f"occurs(object(robot,{rid}),putdown,{time})"); time += 1

    return actions

# ─────────────────────────────── CLI
def main(paths: List[str]):
    for inst in paths:
        plan = generate_plan(inst)
        print(f"% ===== Plan for {Path(inst).name} =====")
        for a in plan:
            print(a)
        print("% =========== end ==========\n")
        print(f"[✓] Finished {inst}\n")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit(f"usage: {Path(sys.argv[0]).name} inst1.asp [inst2.asp …]")
    main(sys.argv[1:])

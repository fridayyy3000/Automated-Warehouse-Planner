# 🏭 Warehouse Planning using Answer Set Programming (ASP)

This project tackles automated warehouse task planning using **Answer Set Programming (ASP)** and **Python-based orchestration**. It was built to simulate intelligent task sequencing and motion planning for single-robot logistics in a structured warehouse world.

---

## 🔧 Project Overview

We work with ASP-based encodings and domain instances to simulate scenarios where a robot must:
- Pick items from shelves
- Fulfill customer orders
- Navigate to picking stations
- Optimize movement and task timing

We model the warehouse as a grid world and use both **custom heuristics** and **Clingo** to generate action plans using `occurs(...)` atoms.

---

## 🧠 Problem Domain

- Encodings (`*.lp`) define motion rules, item handling, order delivery, and robot actions.
- Instances (`inst*.asp`) specify shelf layouts, robot positions, products, and delivery orders.
- Solver scripts parse the instance, generate valid robot actions, and optionally run optimization routines.


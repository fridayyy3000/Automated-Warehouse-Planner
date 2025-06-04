# pip install clingo   # if you haven’t already
import clingo

ctl = clingo.Control(["-c", "h=40", "--opt-mode=opt"])
ctl.load("warehouse.lp")      # your encoding
ctl.load("inst1.asp")         # the instance file
ctl.ground([("base", [])])    # ground the combined program

# iterate over optimum models
def on_model(model):
    print("Model cost:", model.cost)
    for atom in model.symbols(atoms=True):
        if atom.name == "occurs":
            print(atom)
            
ctl.solve(on_model=on_model)

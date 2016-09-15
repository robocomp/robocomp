from libagm import *

acho = AGM("gualzru.pddl", "gualzru.agmbd")

model1  = AGMModel()
model1.resetLastId()
model1.insertSymbol(AGMModelSymbol(1, "a"))

model2  = AGMModel()
model2.resetLastId()
model2.insertSymbol(AGMModelSymbol(2, "b"))

d = model1.generatePDDLProblem(model2, 3, "domainName", "problemName")
print d






from z3 import *
p = Bool('p')
q = Bool('q')
r = Bool('r')
solve(Implies(p, q), r == Not(q), Or(Not(p), r))

p = Bool('p')
q = Bool('q')
x = Real('x')
print And(p, q, True)
print simplify(Or(x < 5, x > 10,x > 9,x==0,x==87))
print simplify(And(p, False))

p = Bool('p')
x = Real('x')
solve(Or(x < 5, x > 10,x > 9,x==0,x==87), Or(p, x**2 == 2), Not(p))
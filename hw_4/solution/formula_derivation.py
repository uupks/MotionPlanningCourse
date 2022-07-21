from sympy import *

# duration
T = symbols('T')
# initial state
px_s, py_s, pz_s, vx_s, vy_s, vz_s = symbols('px_s, py_s, pz_s, vx_s, vy_s, vz_s')
# final state
px_f, py_f, pz_f, vx_f, vy_f, vz_f = symbols('px_f, py_f, pz_f, vx_f, vy_f, vz_f')


A = Matrix([
    [-12/(T**3),          0,          0, 6/(T**2),        0,        0],
    [         0, -12/(T**3),          0,        0, 6/(T**2),        0],
    [         0,          0, -12/(T**3),        0,        0, 6/(T**2)],
    [  6/(T**2),          0,          0,   -2/(T),        0,        0],
    [         0,   6/(T**2),          0,        0,   -2/(T),        0],
    [         0,          0,   6/(T**2),        0,        0,   -2/(T)]
])

print(latex(A))

b = Matrix([
    [px_f - vx_s * T - px_s],
    [py_f - vy_s * T - py_s],
    [pz_f - vz_s * T - pz_s],
    [vx_f - vx_s],
    [vy_f - vy_s],
    [vz_f - vz_s]
]
)

print(latex(b))

coeffs = simplify(A * b)
print(latex(coeffs))

a1 = coeffs[0]
a2 = coeffs[1]
a3 = coeffs[2]
b1 = coeffs[3]
b2 = coeffs[4]
b3 = coeffs[5]

J = T + (a1**2)*(T**3)/3 + a1*b1*(T**2) + (b1**2)*T + (a2**2)*(T**3)/3 + a2*b2*(T**2) + (b2**2)*T + (a3**2)*(T**3)/3 + a3*b3*(T**2) + (b3**2)*T
print(latex(J))

dJdt = diff(J, T)
print(latex(dJdt))

gT = collect(expand(simplify(dJdt * (T**4))), syms = T)
print(latex(gT))

gT.as_poly(T)


# dp, dv, da = symbols('dp, dv, da')

# M = Matrix([
#     [720, -360*T, 60*(T**2)],
#     [-360*T, 168*(T**2), -24*(T**3)],
#     [60*(T**2), -24*(T**3), 3*(T**4)],
# ]
# ) / (T**5)

# print(M)

# d = Matrix([
#     [dp],
#     [dv],
#     [da]
# ]
# )

# coe = M * d 
# print(coe)

# a=coe[0]
# b=coe[1]
# c=coe[2]

# J2 = c**2 + b*c*T + (b**2)*(T**2)/3.0 + a*c*(T**2)/3 + a*b*(T**3)/4 + (a**2)*(T**4)/20

# print(latex(collect(expand(simplify(J2)), syms=T)))

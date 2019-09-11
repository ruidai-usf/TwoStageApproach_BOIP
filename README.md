# TwoStageApproach_BOIP
A two-stage approach for solving bi-objective pure integer linear programs

This is an algorithm to solve a class of Bi-objective Pure Integer Linear Programs (BOILP).

A BOILP can be stated as follows:
min {f1(x), f2(x)} 
s.t. x ∈ D

where X := 
x ∈ Z+ n : Ax ≤ b represents the feasible set in the decision space, and z1 (x) := c1 x and
z2(x) := c2 x are two linear objective functions. Note that Zn + := {s ∈ Zn : s ≥ 0}. The image Y of
X under vector-valued function z = (z1, z2) represents the feasible set in the objective/criterion space, i.e.,
Y := z(X ) := {y ∈ R2 : y = z(x)}. It is assumed that X is bounded, and all coefficients/parameters are
integers, i.e., A ∈ Zm×n , b ∈ Zm . and ci ∈ Zn for i = 1, 2.

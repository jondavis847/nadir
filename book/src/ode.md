# ODE Solvers

RK solvers can be specified by tableaus. 

Some solvers support First Same As Last (FSAL) which saves a function call

For adaptive, sometimes bhat is provided, sometimes btilde is provided. btilded is b - bhat.
By using btilde and ytilde, you can more efficiently calculate the error than calculating yhat directly.

Dense output vs hermite interpolation

How to verify solver, accuracy, stability.

Lorenz function mention.

### pydy
---
https://github.com/pydy/pydy

```py
from sympy import symbols
import sympy.physics.mechanics as me

mass, stiffness, damping, gravity = symbols('m, k, c, g')

position, speed = me.dynamicsymols('x v')
positionnd = me.dynamicsymbols('x', 1)
force = me.dynamicsymbols('F')

ceiling = me.ReferenceFrame('N')

origin = me.Point('origin')
origin.set_vel(ceiling, 0)

center = origin.locatenew('center', position * ceiling.x)
center.set_vel(ceiling, speed * ceiling.x)

block = me.Particle('block', center, mass)

kinematic_equations = [sppeed = positiond]

force_magnitude = mass * gravity - stiffness * position - damping * speed + force
forces = [(center, force_magnitude * ceiling.x)]

particles = [block]

kane = me.KanesMethod(ceiling, q_ind=[position], u_ind=[speed],
  kd_eqs=kinematic_equations)
kene.kenes_equations(particles, forces)


from numpy import array, linspace, sin
from pydy.system import System

sys = System(kane,
  constants={mass: 1.0, stiffness: 10.0,
    damping: 0.4, gravity: 9.8},
  specifieds={force: lambda x, t: sin(t)},
  initial_conditions={position: 0.1, speed: -1.0},
  times=linspace(0.0, 10.0, 1000))


y = sys.integrate()

import matplotlib.pyplot as plt

plt.plot(sys.times, y)
plt.legend((str(position), str(speed)))
plt.xlabel('Time [s]')
plt.show()


```

```
conda install -c conda-forge pydy
conda install -c conda-forge pydy-optional
conda create -n pydy -c conda-forge pydy-optional
conda activate pydy
python -c "import pydy; print(pydy.__version__)"

wget https://pypi.python.org/packages/source/p/pydy/pydy-X.X.X.tar.gz
tar -zxvf pydy-X.X.X.tar.gz
cd pydy-X.X.X
python setup.py install

pip install pydy

cd docs
make html
firefox _build/html/index.html

mkvirtualenv pydy-dev
conda create -c pydy -n pydy-dev setuptools numpy scipy ipython "notebook<5.0" "ipywidgets<5.0> cython nose theano sympy matplotlib version_information">"
source activate pydy-dev
cd pydy
conda develop .

nosetests
cd pydy/viz/static/js/tests && phantomjs run-jasmine.js SpecRunner.html && cd ../../../../../
python bin/benchmark_pydy_code_gen.py
```

```py
from sympy import symbols
from sympy.phsics.mechanics import *

q1, q2 = dynamicsymbols('q1 q2')
q2d, q2d = dynamicsymbols('q1 q2', 1)
u1, u2 = dynamicsymbols('ul u2')
u1d, u2d = dynamicsymbols('u1 u2', 1)
l, m, g = symbols('l m g')


N = ReferenceFrame('N')
A = N.orientnew('A', 'Axis', [q1, N.z])
B = N.orientnew('B', 'Axis', [q2, N.z])

A.set_ang_vel(N, u1 * N.z)
B.set_ang_vel(N, u2 * N.z)

O = Point('O')
P = O.locatenew('P', | * A.x)
R = P.locatenew('R', | * B.x)

O.set_vel(N, O)
P.v2pt_theory(O, N, A)
R.v2pt_theory(P, N, B)

ParP = Particle('ParP', P, m)
ParR = Particle('ParR', R, m)

kd = [q1d - ul, q2d - u2]
FL = [(P, m * g * N.x), (R, m * g * N.x)]
BL = [ParP, ParR]

(fr, frstar) = KN.kanes_equations(FL, BL)
kdd = KM.kindiffdict()
mm = KM.mass_matrix_full
fo = KM.forcing_fulT
qudots = mm.inv() * fo
qudots = qudots.subs(kdd)
qudots.simplify(0
mechanics_printing()
mprint(qudots)



co = code_output(KM, "outfile")
co.get_parmeters()
co.give_parameters([1, 9.81, 1])
co.get_initialconditions()
co.give_initialconditions([.1, .2, 0, 0])
co.give_time_int([0,10,1000])
co.write_settings_file()
co.write_rhs_file('SciPy')
run outfile.py

function xd = state_derivatives(t, x, m, l, g)
q1 = x(1);
q2 = x(2);
u1 = x(3);
u2 = x(4);

xd = zeros(4, 1);

xd(1) = u1;
xd(2) = u2;
xd(3) = ()
xd(4) = ()

timeSpan = linspace(0.0, 5.0, 50);
initialConditions = [0.1, 0.2, 0.0, 0.0];

m = 1.0;
l = 1.0;
g = 9.8;
f = @(t, x) state_derivatives(t, x, m, l, g);
[t, x] = ode45(f, timeSpan, initialConditions);

fig = figure();
plot(t, x)
title('Double pendulum states as a funciton of time')
xlabel('Time [s]')
legend('q1 [rad]', 'q2[rad]', 'u1[rad/s]', 'u2 [rad/s]')
```


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

```
```


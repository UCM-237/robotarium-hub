import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import matplotlib
import math
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
PI=math.pi
# New Antecedent/Consequent objects hold universe variables and membership
# functions
AngleError = ctrl.Antecedent(np.arange(-PI, PI, 0.02), 'AngleError')
AngleError['bigNegative'] = fuzz.trimf(AngleError.universe, [-PI, -PI,-PI/2])
AngleError['negative'] = fuzz.trimf(AngleError.universe, [-PI, -PI/4,0.0])
AngleError['zero'] = fuzz.trimf(AngleError.universe, [-0.3, 0 ,0.3])
AngleError['positive'] = fuzz.trimf(AngleError.universe, [0, PI/4, PI])
AngleError['bigPositive'] = fuzz.trimf(AngleError.universe, [PI/2, PI, PI])
AngleError.view()
leyenda = ['bigNegative', 'negative', 'zero', 'positive', 'bigPositive']
plt.legend(leyenda, loc='center right')
plt.grid()
plt.show()




W = ctrl.Consequent(np.arange(-5, 5, 0.01), 'W')

# Membership functions for the consequent
W['farLeft'] = fuzz.trimf(W.universe, [-5, -5, -3])
W['left'] = fuzz.trimf(W.universe, [-4, -3, -1])
W['center'] = fuzz.trimf(W.universe, [-3.5, 0, 3.5])
W['right'] = fuzz.trimf(W.universe, [1, 3, 4])
W['farRight'] = fuzz.trimf(W.universe, [3, 5, 5])

W.view()
leyenda = ['farLeft', 'left', 'center', 'right', 'farRight']
plt.legend(leyenda, loc='best')
plt.grid()
plt.show()
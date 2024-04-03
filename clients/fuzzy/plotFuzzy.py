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
AngleError = ctrl.Antecedent(np.arange(-PI/2, PI/2, 0.1), 'AngleError')
AngleError['bigNegative'] = fuzz.trimf(AngleError.universe, [-PI/2, -PI/2,-PI/4])
AngleError['negative'] = fuzz.trimf(AngleError.universe, [-PI/2, -PI/4,-0.2])
AngleError['zero'] = fuzz.trimf(AngleError.universe, [-0.5,0,0.65,])
AngleError['positive'] = fuzz.trimf(AngleError.universe, [0.2, PI/4, PI/2])
AngleError['bigPositive'] = fuzz.trimf(AngleError.universe, [PI/4, PI/2, PI/2])
AngleError.view()
# Mostrar la gráfica
plt.show()


W = ctrl.Consequent(np.arange(-PI, PI, 0.01), 'W')

# Membership functions for the consequent
W['farLeft'] = fuzz.trimf(W.universe, [-PI, -PI, -PI/2])
W['left'] = fuzz.trimf(W.universe, [-PI, -PI/2, 0])
W['center'] = fuzz.trimf(W.universe, [-PI/2, 0, PI/2])
W['right'] = fuzz.trimf(W.universe, [0, PI/2, PI])
W['farRight'] = fuzz.trimf(W.universe, [PI/2, PI, PI])

W.view()
plt.show()
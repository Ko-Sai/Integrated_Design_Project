from mpmath import *
from sympy import *

### Creates Homogeneous Transform Matrix
def tf_matrix(q, d, a, alpha):
    T = Matrix([[            cos(q),           -sin(q),           0,             a],
                [ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                [ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                [                 0,                 0,           0,             1]])
    return T

# Joint angle (variable for revolute)
q1, q2, q3, q4, q5 = symbols('q1:6')
# Link offset (variable for prismatic)
d1, d2, d3, d4, d5 = symbols('d1:6')
# Link length
a0, a1, a2, a3, a4 = symbols('a0:5')
# Twist angles
alpha0,alpha1, alpha2, alpha3, alpha4 = symbols('alpha0:5')

# Create Modified DH parameters
DH = {alpha0:     0,  a0:      0,  d1:   0.75,  q1:      q1,
      alpha1:    30,  a1:   0.35,  d2:      0,  q2: q2-pi/2,
      alpha2:     0,  a2:   1.25,  d3:      0,  q3:      q3,
      alpha3:     0,  a3: 0.0536,  d4:      0,  q4:      q4,
      alpha4:     0,  a4: 0.0536,  d5:      0,  q5:       0}

# Create individual transformation matrices
T0_1  = tf_matrix(q1, d1, a0, alpha0).subs(DH)
T1_2  = tf_matrix(q2, d2, a1, alpha1).subs(DH)
T2_3  = tf_matrix(q3, d3, a2, alpha2).subs(DH)
T3_4  = tf_matrix(q4, d4, a3, alpha3).subs(DH)
T4_EE  = tf_matrix(q5, d5, a4, alpha4).subs(DH)

# Generalized homogeneous transform
T0_EE = T0_1 * T1_2 * T2_3 * T3_4 * T4_5
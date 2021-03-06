# Demo
# Written by: Kristina Miller
import numpy as np
import pypoman as ppm
import matplotlib.pyplot as plt
from plot_polytope3d import *
import copy

def problem():
    A_rect = np.array([[-1,0,0],
                       [1,0,0],
                       [0,-1,0],
                       [0,1,0],
                       [0,0,-1],
                       [0,0,1]])

    # b1 = np.array([[-00],  [9], [-00], [9], [-00], [36]])
    # b2 = np.array([[-15],  [21], [-00], [9], [-00], [24]])
    # b3 = np.array([[-27],  [33], [-00], [9], [-00], [36]])
    # b4 = np.array([[-39],  [45], [-00], [9], [-00], [12]])
       
    # b5 = np.array([[-15],  [21], [-15], [21], [-00], [36]])
    # b6 = np.array([[-27],  [33], [-15], [21], [-00], [36]])
    # b7 = np.array([[-39],  [45], [-15], [21], [-00], [36]])
    
    # b8 = np.array([[-00],  [9], [-27], [33], [-00], [36]])
    # b9 = np.array([[-15],  [21], [-27], [33], [-00], [36]])
    # b10 = np.array([[-27], [33], [-27], [33], [-00], [36]])
    # b11 = np.array([[-39], [45], [-27], [33], [-00], [36]])
    
    # b12 = np.array([[-00], [9], [-39], [45], [-00], [12]])
    # b13 = np.array([[-15], [21], [-39], [45], [-00], [36]])
    # b14 = np.array([[-27], [33], [-39], [45], [-00], [36]])
    # b15 = np.array([[-39], [45], [-39], [45], [-00], [24]])
    
    # b16 = np.array([[-00], [9], [-00], [33], [-24], [36]])
    # b17 = np.array([[-27], [33], [-00], [21], [-12], [36]])
    # b18 = np.array([[-27], [45], [-15], [21], [-24], [36]])
    # b19 = np.array([[-00], [21], [-27], [33], [-00], [24]])
    # b20 = np.array([[-15], [21], [-27], [45], [-12], [36]])
    # b21 = np.array([[-27], [33], [-27], [45], [-24], [36]])
    # b22 = np.array([[-27], [33], [-27], [45], [-00], [12]])
    
    # b23 = np.array([[-00], [9], [-51], [57], [-00], [36]])
    # b24 = np.array([[-15], [21], [-51], [57], [-00], [36]])
    # b25 = np.array([[-27], [33], [-51], [57], [-00], [12]])
    # b25 = np.array([[-39], [45], [-51], [57], [-00], [36]])
    # b26 = np.array([[-51], [57], [-51], [57], [-00], [36]])
    # b27 = np.array([[-63], [72], [-51], [57], [-00], [36]])
    
    # b28 = np.array([[-00], [9], [-63], [72], [-00], [24]])
    # b29 = np.array([[-15], [21], [-63], [72], [-00], [36]])
    # b30 = np.array([[-27], [33], [-63], [72], [-00], [36]])
    # b31 = np.array([[-39], [45], [-63], [72], [-00], [36]])
    # b32 = np.array([[-51], [57], [-63], [72], [-00], [36]])

    # b46 = np.array([[-51], [57], [-0], [9], [-00], [36]])
    # b47 = np.array([[-51], [57], [-15], [21], [-00], [36]])
    # b48 = np.array([[-51], [57], [-27], [33], [-00], [36]])
    # b49 = np.array([[-51], [57], [-39], [45], [-00], [36]])
    
    # b50 = np.array([[-63], [72], [-0], [9], [-00], [24]])
    # b51 = np.array([[-63], [72], [-15], [21], [-00], [36]])
    # b52 = np.array([[-63], [72], [-27], [33], [-00], [12]])
    # b53 = np.array([[-63], [72], [-39], [45], [-00], [36]])

    # b33 = np.array([[-00], [21], [-51], [57], [-00], [36]])
    # b34 = np.array([[-15], [21], [-51], [72], [-00], [12]])
    # b35 = np.array([[-15], [21], [-51], [72], [-24], [36]])
    # b36 = np.array([[-15], [45], [-51], [57], [-24], [36]])
    # b37 = np.array([[-39], [45], [-39], [57], [-12], [24]])
    # b38 = np.array([[-51], [57], [-51], [72], [-24], [36]])
    # b39 = np.array([[-51], [72], [-51], [57], [-12], [24]])
    # b40 = np.array([[-51], [72], [-39], [45], [-00], [36]])
    # b41 = np.array([[-51], [57], [-27], [45], [-24], [36]])
    # b42 = np.array([[-51], [57], [-15], [21], [-12], [24]])
    # b43 = np.array([[-51], [57], [-15], [21], [-12], [24]])
    # b44 = np.array([[-51], [72], [-15], [21], [-00], [36]])
    # b45 = np.array([[-51], [57], [-0], [21], [-24], [36]])
    # b54 = np.array([[-51], [57], [-15], [33], [-12], [24]])

    # b1_1 = np.array([[-72-00], [72+9], [-00], [9], [-00], [36]])
    # b1_2 = np.array([[-72-15], [72+21], [-00], [9], [-00], [24]])
    # b1_3 = np.array([[-72-27], [72+33], [-00], [9], [-00], [36]])
    # b1_4 = np.array([[-72-39], [72+45], [-00], [9], [-00], [12]])
       
    # b1_5 = np.array([[-72-15], [72+21], [-15], [21], [-00], [36]])
    # b1_6 = np.array([[-72-27], [72+33], [-15], [21], [-00], [36]])
    # b1_7 = np.array([[-72-39], [72+45], [-15], [21], [-00], [36]])
    
    # b1_8 = np.array([[-72-00], [72+9], [-27], [33], [-00], [36]])
    # b1_9 = np.array([[-72-15], [72+21], [-27], [33], [-00], [36]])
    # b1_10 = np.array([[-72-27], [72+33], [-27], [33], [-00], [36]])
    # b1_11 = np.array([[-72-39], [72+45], [-27], [33], [-00], [36]])
    
    # b1_12 = np.array([[-72-00], [72+9], [-39], [45], [-00], [12]])
    # b1_13 = np.array([[-72-15], [72+21], [-39], [45], [-00], [36]])
    # b1_14 = np.array([[-72-27], [72+33], [-39], [45], [-00], [36]])
    # b1_15 = np.array([[-72-39], [72+45], [-39], [45], [-00], [24]])
    
    # b1_16 = np.array([[-72-00], [72+9], [-00], [33], [-24], [36]])
    # b1_17 = np.array([[-72-27], [72+33], [-00], [21], [-12], [36]])
    # b1_18 = np.array([[-72-27], [72+45], [-15], [21], [-24], [36]])
    # b1_19 = np.array([[-72-00], [72+21], [-27], [33], [-00], [24]])
    # b1_20 = np.array([[-72-15], [72+21], [-27], [45], [-12], [36]])
    # b1_21 = np.array([[-72-27], [72+33], [-27], [45], [-24], [36]])
    # b1_22 = np.array([[-72-27], [72+33], [-27], [45], [-00], [12]])
    
    # b1_23 = np.array([[-72-00], [72+9], [-51], [57], [-00], [36]])
    # b1_24 = np.array([[-72-15], [72+21], [-51], [57], [-00], [36]])
    # b1_25 = np.array([[-72-27], [72+33], [-51], [57], [-00], [12]])
    # b1_25 = np.array([[-72-39], [72+45], [-51], [57], [-00], [36]])
    # b1_26 = np.array([[-72-51], [72+57], [-51], [57], [-00], [36]])
    # b1_27 = np.array([[-72-63], [72+72], [-51], [57], [-00], [36]])
    
    # b1_28 = np.array([[-72-00], [72+9], [-63], [72], [-00], [24]])
    # b1_29 = np.array([[-72-15], [72+21], [-63], [72], [-00], [36]])
    # b1_30 = np.array([[-72-27], [72+33], [-63], [72], [-00], [36]])
    # b1_31 = np.array([[-72-39], [72+45], [-63], [72], [-00], [36]])
    # b1_32 = np.array([[-72-51], [72+57], [-63], [72], [-00], [36]])

    # b1_46 = np.array([[-72-51], [72+57], [-0], [9], [-00], [36]])
    # b1_47 = np.array([[-72-51], [72+57], [-15], [21], [-00], [36]])
    # b1_48 = np.array([[-72-51], [72+57], [-27], [33], [-00], [36]])
    # b1_49 = np.array([[-72-51], [72+57], [-39], [45], [-00], [36]])
    
    # b1_50 = np.array([[-72-63], [72+72], [-0], [9], [-00], [24]])
    # b1_51 = np.array([[-72-63], [72+72], [-15], [21], [-00], [36]])
    # b1_52 = np.array([[-72-63], [72+72], [-27], [33], [-00], [12]])
    # b1_53 = np.array([[-72-63], [72+72], [-39], [45], [-00], [36]])

    # b1_33 = np.array([[-72-00], [72+21], [-51], [57], [-00], [36]])
    # b1_34 = np.array([[-72-15], [72+21], [-51], [72], [-00], [12]])
    # b1_35 = np.array([[-72-15], [72+21], [-51], [72], [-24], [36]])
    # b1_36 = np.array([[-72-15], [72+45], [-51], [57], [-24], [36]])
    # b1_37 = np.array([[-72-39], [72+45], [-39], [57], [-12], [24]])
    # b1_38 = np.array([[-72-51], [72+57], [-51], [72], [-24], [36]])
    # b1_39 = np.array([[-72-51], [72+72], [-51], [57], [-12], [24]])
    # b1_40 = np.array([[-72-51], [72+72], [-39], [45], [-00], [36]])
    # b1_41 = np.array([[-72-51], [72+57], [-27], [45], [-24], [36]])
    # b1_42 = np.array([[-72-51], [72+57], [-15], [21], [-12], [24]])
    # b1_43 = np.array([[-72-51], [72+57], [-15], [21], [-12], [24]])
    # b1_44 = np.array([[-72-51], [72+72], [-15], [21], [-00], [36]])
    # b1_45 = np.array([[-72-51], [72+57], [-0], [21], [-24], [36]])
    # b1_54 = np.array([[-72-51], [72+57], [-15], [33], [-12], [24]])
    # TODO: Create more obstacles

    tmp = [np.array([[-00],  [9], [-00], [9], [-00], [36]]),
    np.array([[-15],  [21], [-00], [9], [-00], [24]]),
    np.array([[-27],  [33], [-00], [9], [-00], [36]]),
    np.array([[-39],  [45], [-00], [9], [-00], [12]]),
       
    np.array([[-15],  [21], [-15], [21], [-00], [36]]),
    np.array([[-27],  [33], [-15], [21], [-00], [36]]),
    np.array([[-39],  [45], [-15], [21], [-00], [36]]),
    
    np.array([[-00],  [9], [-27], [33], [-00], [36]]),
    np.array([[-15],  [21], [-27], [33], [-00], [36]]),
    np.array([[-27], [33], [-27], [33], [-00], [36]]),
    np.array([[-39], [45], [-27], [33], [-00], [36]]),
    
    np.array([[-00], [9], [-39], [45], [-00], [12]]),
    np.array([[-15], [21], [-39], [45], [-00], [36]]),
    np.array([[-27], [33], [-39], [45], [-00], [36]]),
    np.array([[-39], [45], [-39], [45], [-00], [24]]),
    
    np.array([[-00], [9], [-00], [33], [-24], [36]]),
    np.array([[-27], [33], [-00], [21], [-12], [36]]),
    np.array([[-27], [45], [-15], [21], [-24], [36]]),
    np.array([[-00], [21], [-27], [33], [-00], [24]]),
    np.array([[-15], [21], [-27], [45], [-12], [36]]),
    np.array([[-27], [33], [-27], [45], [-24], [36]]),
    np.array([[-27], [33], [-27], [45], [-00], [12]]),
    
    np.array([[-00], [9], [-51], [57], [-00], [36]]),
    np.array([[-15], [21], [-51], [57], [-00], [36]]),
    np.array([[-27], [33], [-51], [57], [-00], [12]]),
    np.array([[-39], [45], [-51], [57], [-00], [36]]),
    np.array([[-51], [57], [-51], [57], [-00], [36]]),
    np.array([[-63], [72], [-51], [57], [-00], [36]]),
    
    np.array([[-00], [9], [-63], [72], [-00], [24]]),
    np.array([[-15], [21], [-63], [72], [-00], [36]]),
    np.array([[-27], [33], [-63], [72], [-00], [36]]),
    np.array([[-39], [45], [-63], [72], [-00], [36]]),
    np.array([[-51], [57], [-63], [72], [-00], [36]]),

    np.array([[-51], [57], [-0], [9], [-00], [36]]),
    np.array([[-51], [57], [-15], [21], [-00], [36]]),
    np.array([[-51], [57], [-27], [33], [-00], [36]]),
    np.array([[-51], [57], [-39], [45], [-00], [36]]),
    
    np.array([[-63], [72], [-0], [9], [-00], [24]]),
    np.array([[-63], [72], [-15], [21], [-00], [36]]),
    np.array([[-63], [72], [-27], [33], [-00], [12]]),
    np.array([[-63], [72], [-39], [45], [-00], [36]]),

    np.array([[-00], [21], [-51], [57], [-00], [36]]),
    np.array([[-15], [21], [-51], [72], [-00], [12]]),
    np.array([[-15], [21], [-51], [72], [-24], [36]]),
    np.array([[-15], [45], [-51], [57], [-24], [36]]),
    np.array([[-39], [45], [-39], [57], [-12], [24]]),
    np.array([[-51], [57], [-51], [72], [-24], [36]]),
    np.array([[-51], [72], [-51], [57], [-12], [24]]),
    np.array([[-51], [72], [-39], [45], [-00], [36]]),
    np.array([[-51], [57], [-27], [45], [-24], [36]]),
    np.array([[-51], [57], [-15], [21], [-12], [24]]),
    np.array([[-51], [57], [-15], [21], [-12], [24]]),
    np.array([[-51], [72], [-15], [21], [-00], [36]]),
    np.array([[-51], [57], [-0], [21], [-24], [36]]),
    np.array([[-51], [57], [-15], [33], [-12], [24]]),
    np.array([[-63], [72], [-51], [72], [-24], [36]]),
    np.array([[-51], [72], [-63], [72], [-24], [36]])]

    # obstacles = [(A_rect, b1),
    #              (A_rect, b2),
    #              (A_rect, b3),
    #              (A_rect, b4),
    #              (A_rect, b5),
    #              (A_rect, b6),
    #              (A_rect, b7),
    #              (A_rect, b8),
    #              (A_rect, b9),
    #              (A_rect, b10),
    #              (A_rect, b11),
    #              (A_rect, b12),
    #              (A_rect, b13),
    #              (A_rect, b14),
    #              (A_rect, b15),
    #              (A_rect, b16),
    #              (A_rect, b17),
    #              (A_rect, b18),
    #              (A_rect, b19),
    #              (A_rect, b20),
    #              (A_rect, b21),
    #              (A_rect, b22),
    #              (A_rect, b23),
    #              (A_rect, b24),
    #              (A_rect, b25),
    #              (A_rect, b26),
    #              (A_rect, b27),
    #              (A_rect, b28),
    #              (A_rect, b29),
    #              (A_rect, b30),
    #              (A_rect, b31),
    #              (A_rect, b32),
    #              (A_rect, b33),
    #              (A_rect, b34),
    #              (A_rect, b35),
    #              (A_rect, b36),
    #              (A_rect, b37),
    #              (A_rect, b38),
    #              (A_rect, b39),
    #              (A_rect, b40),
    #              (A_rect, b41),
    #              (A_rect, b42),
    #              (A_rect, b43),
    #              (A_rect, b44),
    #              (A_rect, b45),
    #              (A_rect, b46),
    #              (A_rect, b47),
    #              (A_rect, b48),
    #              (A_rect, b49),
    #              (A_rect, b50),
    #              (A_rect, b51),
    #              (A_rect, b52),
    #              (A_rect, b53),
    #              (A_rect, b54),
    #              (A_rect, b1_1),
    #              (A_rect, b1_2),
    #              (A_rect, b1_3),
    #              (A_rect, b1_4),
    #              (A_rect, b1_5),
    #              (A_rect, b1_6),
    #              (A_rect, b1_7),
    #              (A_rect, b1_8),
    #              (A_rect, b1_9),
    #              (A_rect, b1_10),
    #              (A_rect, b1_11),
    #              (A_rect, b1_12),
    #              (A_rect, b1_13),
    #              (A_rect, b1_14),
    #              (A_rect, b1_15),
    #              (A_rect, b1_16),
    #              (A_rect, b1_17),
    #              (A_rect, b1_18),
    #              (A_rect, b1_19),
    #              (A_rect, b1_20),
    #              (A_rect, b1_21),
    #              (A_rect, b1_22),
    #              (A_rect, b1_23),
    #              (A_rect, b1_24),
    #              (A_rect, b1_25),
    #              (A_rect, b1_26),
    #              (A_rect, b1_27),
    #              (A_rect, b1_28),
    #              (A_rect, b1_29),
    #              (A_rect, b1_30),
    #              (A_rect, b1_31),
    #              (A_rect, b1_32),
    #              (A_rect, b1_33),
    #              (A_rect, b1_34),
    #              (A_rect, b1_35),
    #              (A_rect, b1_36),
    #              (A_rect, b1_37),
    #              (A_rect, b1_38),
    #              (A_rect, b1_39),
    #              (A_rect, b1_40),
    #              (A_rect, b1_41),
    #              (A_rect, b1_42),
    #              (A_rect, b1_43),
    #              (A_rect, b1_44),
    #              (A_rect, b1_45),
    #              (A_rect, b1_46),
    #              (A_rect, b1_47),
    #              (A_rect, b1_48),
    #              (A_rect, b1_49),
    #              (A_rect, b1_50),
    #              (A_rect, b1_51),
    #              (A_rect, b1_52),
    #              (A_rect, b1_53),
    #              (A_rect, b1_54)]

    obstacles = []

    for i in range(len(tmp)):
        b = copy.deepcopy(tmp[i])
        obstacles.append((A_rect, b))

    # for i in range(len(tmp)):
    #     b = copy.deepcopy(tmp[i])
    #     b[0,0] -= 72
    #     b[1,0] += 72
    #     obstacles.append((A_rect, b))

    # for i in range(len(tmp)):
    #     b = copy.deepcopy(tmp[i])
    #     b[2,0] -= 72
    #     b[3,0] += 72
    #     obstacles.append((A_rect, b))
    
    # for i in range(len(tmp)):
    #     b = copy.deepcopy(tmp[i])
    #     b[0,0] -= 72
    #     b[1,0] += 72
    #     b[2,0] -= 72
    #     b[3,0] += 72
    #     obstacles.append((A_rect, b))
    
    # for i in range(len(tmp)):
    #     b = copy.deepcopy(tmp[i])
    #     b[0,0] += 72
    #     b[1,0] -= 72
    #     obstacles.append((A_rect, b))

    # for i in range(len(tmp)):
    #     b = copy.deepcopy(tmp[i])
    #     b[2,0] += 72
    #     b[3,0] -= 72
    #     obstacles.append((A_rect, b))
    
    # for i in range(len(tmp)):
    #     b = copy.deepcopy(tmp[i])
    #     b[0,0] += 72
    #     b[1,0] -= 72
    #     b[2,0] += 72
    #     b[3,0] -= 72
    #     obstacles.append((A_rect, b))

    # for i in range(len(tmp)):
    #     b = copy.deepcopy(tmp[i])
    #     b[0,0] += 72
    #     b[1,0] -= 72
    #     b[2,0] -= 72
    #     b[3,0] += 72
    #     obstacles.append((A_rect, b))

    # for i in range(len(tmp)):
    #     b = copy.deepcopy(tmp[i])
    #     b[0,0] -= 72
    #     b[1,0] += 72
    #     b[2,0] += 72
    #     b[3,0] -= 72
    #     obstacles.append((A_rect, b))

    #TODO: Create initial set
    b0 = np.array([[-5], [7], [-41], [42], [-17], [19]])
    Theta = [(A_rect, b0)]
    #TODO: Create goal set
    tmp_g = [np.array([[- 3], [9], [-15], [21], [-3], [9]]),
    np.array([[-15], [21], [-3], [9], [-27], [33]]),
    np.array([[-39], [45], [-3], [9], [-15], [21]]),
    np.array([[-39], [45], [-39], [45], [-27], [33]]),
    np.array([[- 3], [9], [-63], [69], [-27], [33]]),
    np.array([[-27], [33], [-51], [57], [-15], [21]]),
    np.array([[-63], [69], [-63], [69], [-3], [9]]),
    np.array([[-63], [69], [-27], [33], [-15], [21]]),
    np.array([[-63], [69], [-3], [9], [-27], [33]])]

    # goal = []

    # bg1_1 = np.array([[-72 - 3], [72+9], [-15], [21], [-3], [9]])
    # bg1_2 = np.array([[-72 -15], [72+21], [-3], [9], [-27], [33]])
    # bg1_3 = np.array([[-72 -39], [72+45], [-3], [9], [-15], [21]])
    # bg1_4 = np.array([[-72 -39], [72+45], [-39], [45], [-27], [33]])
    # bg1_5 = np.array([[-72 - 3], [72+9], [-63], [69], [-27], [33]])
    # bg1_6 = np.array([[-72 -27], [72+33], [-51], [57], [-15], [21]])
    # bg1_7 = np.array([[-72 -63], [72+69], [-63], [69], [-3], [9]])
    # bg1_8 = np.array([[-72 -63], [72+69], [-27], [33], [-15], [21]])
    # bg1_9 = np.array([[-72 -63], [72+69], [-3], [9], [-27], [33]])
    # goal = [(A_rect, bg1_1),
    #         (A_rect, bg1_2),
    #         (A_rect, bg1_3),
    #         (A_rect, bg1_4),
    #         (A_rect, bg1_5),
    #         (A_rect, bg1_6),
    #         (A_rect, bg1_7),
    #         (A_rect, bg1_8),
    #         (A_rect, bg1_9)]
    goal = []

    for i in range(len(tmp_g)):
        goal.append((A_rect,tmp_g[i]))

    # for i in range(len(tmp_g)):
    #     b = copy.deepcopy(tmp_g[i])
    #     b[2,0] -= 72
    #     b[3,0] += 72
    #     goal.append((A_rect, b))

    # for i in range(len(tmp_g)):
    #     b = copy.deepcopy(tmp_g[i])
    #     b[0,0] -= 72
    #     b[1,0] += 72
    #     b[2,0] -= 72
    #     b[3,0] += 72
    #     goal.append((A_rect, b))

    # for i in range(len(tmp_g)):
    #     b = copy.deepcopy(tmp_g[i])
    #     b[0,0] += 72
    #     b[1,0] -= 72
    #     goal.append((A_rect, b))

    # for i in range(len(tmp_g)):
    #     b = copy.deepcopy(tmp_g[i])
    #     b[2,0] += 72
    #     b[3,0] -= 72
    #     goal.append((A_rect, b))
    
    # for i in range(len(tmp_g)):
    #     b = copy.deepcopy(tmp_g[i])
    #     b[0,0] += 72
    #     b[1,0] -= 72
    #     b[2,0] += 72
    #     b[3,0] -= 72
    #     goal.append((A_rect, b))

    # for i in range(len(tmp_g)):
    #     b = copy.deepcopy(tmp_g[i])
    #     b[0,0] += 72
    #     b[1,0] -= 72
    #     b[2,0] -= 72
    #     b[3,0] += 72
    #     goal.append((A_rect, b))

    # for i in range(len(tmp_g)):
    #     b = copy.deepcopy(tmp_g[i])
    #     b[0,0] -= 72
    #     b[1,0] += 72
    #     b[2,0] += 72
    #     b[3,0] -= 72
    #     goal.append((A_rect, b))

    return obstacles , Theta, goal

if __name__ == '__main__':
    obs, Theta, goal = problem()

    fig = plt.figure()
    axes = fig.add_subplot(111, projection='3d')
    
    for A,b in obs:
        # ppm.polygon.plot_polygon(ppm.duality.compute_polytope_vertices(A,b), color = 'red')
        plot_polytope_3d(A, b, ax = axes, color = 'red')

    for A,b in goal:
        plot_polytope_3d(A, b, ax = axes, color = 'green')

    for A,b in Theta:
        plot_polytope_3d(A, b, ax = axes, color = 'blue')

    plot_line_3d([0,0,0], [0,0,18],ax = axes)
    # plt.xlim(0, 24)
    # plt.ylim(0, 24)
    plt.grid()
    plt.show()

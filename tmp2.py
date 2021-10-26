import numpy as np
import math 
import polytope as pc 

class PolyUtils:

    @staticmethod
    def ceil(val, factor = 1):
        # factor = int(math.pow(10,factor))
        return math.ceil(val*factor)/factor

    @staticmethod
    def floor(val, factor = 1):
        # factor = int(math.pow(10, factor))
        return math.floor(val * factor) / factor

    @staticmethod
    def does_rect_contain(rect1, rect2): # does rect2 contains rect1
        for i in range(len(rect1[0][:])):
            # if PolyUtils.ceil(rect1[0][i],1) < PolyUtils.floor(rect2[0][i],1) and PolyUtils.floor(rect1[1][i],1) > PolyUtils.ceil(rect2[1][i],1):
            if PolyUtils.ceil(rect1[0][i],1) <= PolyUtils.floor(rect2[0][i],1) and PolyUtils.floor(rect1[1][i],1) >= PolyUtils.ceil(rect2[1][i],1):
            # if PolyUtils.floor(rect1[0][i],0.5) < PolyUtils.ceil(rect2[0][i],0.5) and PolyUtils.ceil(rect1[1][i],0.5) > PolyUtils.floor(rect2[1][i],0.5):
            # if math.ceil(rect1[0][i]) < math.floor(rect2[0][i]) or math.floor(rect1[1][i]) > math.ceil(rect2[1][i]):
                #print("containement: ", math.ceil(rect1[0][i]), "non rounded:", rect1[0][i], "rounded: ",  math.floor(rect2[0][i]), "non rounded: ", rect2[0][i])
                #print("containement: ", math.floor(rect1[1][i]), "non rounded:", rect1[1][i], "rounded: ", math.ceil(rect2[1][i]), "non rounded: ", rect2[1][i])
                return False
        return True

if __name__ == "__main__":
    initset_virtual = [[-3.9895,  -1.00002, -0.00999],[-1.9895,   0.99998,  0.01001]] 
    
    initset_union_box =  [[-4.2132,  -1.28679, -0.0708 ], [-1.7132,   1.21321,  0.0292 ]]
    print(PolyUtils.does_rect_contain(initset_virtual, initset_union_box))
import numpy as np 
import polytope as pc 

tmp1 = np.array([[-2.76228,-1.56976,-0.345,0.69458, -0.72302,-0.001],[-1.34807,-0.15554,0.655,0.69741,-0.72019,0.001]]) 
tmp2 = np.array([[-0.70711,-3.70711,-0.5,  -0.01414,-1.01414,-0.01 ],[ 0.70711,-2.29289,0.5,  0.01414,-0.98586,0.01 ]])
tmp1_poly = pc.box2poly(tmp1.T)
tmp2_poly = pc.box2poly(tmp2.T)
res = pc.is_subset(tmp1_poly, tmp2_poly)
print(res)
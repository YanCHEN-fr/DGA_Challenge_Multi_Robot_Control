import matplotlib.pyplot as plt
import numpy as np
from dron_path import nodes


plt.figure()
for i in range(len(nodes)):
    path_nodes=np.array(nodes[i])
    plt.plot(path_nodes[:,0],path_nodes[:,1],"-o",c=np.random.rand(3,))

plt.show()
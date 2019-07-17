import matplotlib.pyplot as plt
import numpy as np
vision = [0, 7, 0, 3, 2, 0, 3, 0, 0, 0, 0, 0, 0, 6, 2, 0, 0, 0, 3, 0]
print(len(vision))

x = np.arange(len(vision))
plt.bar(x, vision)
plt.xticks(x + 1)
# plt.yticks(np.arange(0, 54 + 2, 2))
plt.xlabel('ID experimento')
plt.ylabel('Cantidad de facelets incorrectamente clasificados.')
plt.show()

vision = np.array(vision)
print(vision.mean(), vision.std()) # 1.3, 2.0760539492026697
print(len(vision[vision == 0]) / len(vision))

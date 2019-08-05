import matplotlib.pyplot as plt
import numpy as np
vision = [0, 8, 0, 3, 2, 0, 3, 0, 0, 0, 0, 0, 0, 6, 2, 0, 0, 0, 3, 0]
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


# giros bien ejecutados
# manip =       [ 1,  2,  2,  4,  5,  6,  7,  8,  5, 10, 11,  7,  2,  3, 13,  4,  9, 13, 11,  7]
# manip =       [ 1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11,  12,  13,  14, 15,  16,  17, 18, 19,  20]
manip =       [ 1/1,  2/2,  2/3,  4/4,  5/5,  6/6,  7/7,  8/8,  5/9, 10/10, 11/11,  7/12,  2/13,  3/14, 13/15,  4/16,  9/17, 13/18, 11/19,  7/20]
# cambios de mano hasta el giro fallido
# cambios_mano = [ 1,  0,  1,  2,  2,  1,  4,  2,  4,  5,  7,  6,  1,  3,  5,  3,  5,  8,  7,  3]
# cambios de mano totales
cambios_mano2 = [ 1,  0,  1,  2,  2,  1,  4,  2,  5,  5,  7,  8,  10,  10,  6,  8,  10,  10,  11,  12]
# plt.scatter(cambios_mano, manip)
plt.plot(cambios_mano2, manip, 'o')
plt.xlabel('Cambios de mano durante la secuencia.')
plt.ylabel('Fracción de la secuencia ejecutada exitosamente.')
print(len(manip))
plt.show()

#
#ROBOTS MOVILES Y AGENTES INTELIGENTES
#PRACTICA 7
#LOCALIZACION POR EL METODO DEL HISTOGRAMA
#
import numpy as np

np.set_printoptions(precision=3)
map=[['white','black','black','white','black'],
     ['white','white','black','black','black'],
     ['white','black','black','white','white'],
     ['black','black','white','white','black'],
     ['black','white','black','white','black']]

p = np.ndarray(shape=(len(map),len(map[0])))
for i in range(len(map)):
    for j in range(len(map[0])):
        p[i][j] = 1.0/(len(map)*len(map[0]))

pHit  = 0.7
pMiss = 0.3

pCorrect = 0.6
pUnder   = 0.2
pOver    = 0.2

def move_x(cells, p):
    updated_p = np.ndarray(shape=(len(map),len(map[0])))
    #
    # Escriba el codigo necesario para calcular la nueva distribucion
    # de probabilidad despues de un movimiento horizontal, esto es,
    # el parametro 'cells' indica las columnas que el robot se movio a lo
    # largo de un solo renglon.
    # Almacenar las nuevas probabilidades en el arreglo 'updated_p'
    #
    for i in range(len(p)):
        for j in range(len(p[i])):
            cell_if_correct    = j - cells
            cell_if_undershoot = j - cells + 1
            cell_if_overshoot  = j - cells - 1

            cell_if_correct    %= len(p[i])
            cell_if_undershoot %= len(p[i])
            cell_if_overshoot  %= len(p[i])

            updated_p[i][j]  = p[i][cell_if_undershoot]*pUnder
            updated_p[i][j] += p[i][cell_if_correct]*pCorrect
            updated_p[i][j] += p[i][cell_if_overshoot]*pOver
    return updated_p

def move_y(cells, p):
    updated_p = np.ndarray(shape=(len(map),len(map[0])))
    #
    # Escriba el codigo necesario para calcular la nueva distribucion
    # de probabilidad despues de un movimiento vertical, esto es,
    # el parametro 'cells' indica los renglones que el robot se movio a lo
    # largo de una sola columna.
    # Almacenar las nuevas probabilidades en el arreglo 'updated_p'
    #
    for i in range(len(p)):
        for j in range(len(p[i])):
            cell_if_correct    = j - cells
            cell_if_undershoot = j - cells + 1
            cell_if_overshoot  = j - cells - 1

            cell_if_correct    %= len(p)
            cell_if_undershoot %= len(p)
            cell_if_overshoot  %= len(p)

            updated_p[j][i] = p[cell_if_undershoot][i]*pUnder
            updated_p[j][i] += p[cell_if_correct][i]*pCorrect
            updated_p[j][i] += p[cell_if_overshoot][i]*pOver
    return updated_p

def observe(landmark, p):
    posterior_p = np.ndarray(shape=(len(map),len(map[0])))
    #
    # Escribir el codigo para calcular la distribucion de probabilidad posterior
    # a realizar una observacion. 
    # Almacenar las nuevas probabilidades en el arreglo 'posterior_p'
    #
    pTotal = 0
    for i in range(len(p)):
        for j in range(len(p[i])):
            if landmark == map[i][j]:
                posterior_p[i][j] = pHit * p[i][j]
            else:
                posterior_p[i][j] = pMiss * p[i][j]
            pTotal += posterior_p[i][j]

    for i in range(len(p)):
        for j in range(len(p[i])):
            posterior_p[i][j] /= pTotal

    return posterior_p
            
cmd = ''
print 'World:'
for i in range(len(map)):
    print map[i]
        
print 'Initial distribution:'
print p

while cmd != 'quit' and cmd != 'exit' and cmd != 'q':
    print 'Enter a command:'
    cmd = raw_input()
    if cmd == 'left':
        p = move_x(-1, p)
    elif cmd == 'right':
        p = move_x(1, p)
    elif cmd == 'up':
        p = move_y(-1, p)
    elif cmd == 'down':
        p = move_y(1, p)
    elif cmd == 'black':
        p = observe(cmd, p)
    elif cmd == 'white':
        p = observe(cmd, p)
    else:
        print 'Pleas enter a valid command'
    print 'The new probability distribution is:'
    print p
        
    
    

#
#ROBOTS MOVILES Y AGENTES INTELIGENTES
#PRACTICA 7
#LOCALIZACION POR EL METODO DEL HISTOGRAMA
#
import numpy

numpy.set_printoptions(precision=3)
map = ['black', 'black', 'white', 'white', 'black', 'white', 'black', 'black', 'black', 'white']
p = [1.0/len(map)]*len(map)
p = numpy.array(p)

pHit  = 0.7
pMiss = 0.2

pCorrect = 0.7
pUnder   = 0.15
pOver    = 0.15

def move(cells, p):
    updated_p = numpy.array(p)
    for i in range(len(p)):
        cell_if_correct    = i - cells
        cell_if_undershoot = i - cells + 1
        cell_if_overshoot  = i - cells - 1

        cell_if_correct    %= len(p)
        cell_if_undershoot %= len(p)
        cell_if_overshoot  %= len(p)

        updated_p[i]  = p[cell_if_undershoot]*pUnder
        updated_p[i] += p[cell_if_correct]*pCorrect
        updated_p[i] += p[cell_if_overshoot]*pOver
    return updated_p

def observe(landmark, p):
    posterior_p = numpy.array(p)
    pTotal = 0
    for i in range(len(p)):
        if landmark == map[i]:
            posterior_p[i] = pHit * p[i]
        else:
            posterior_p[i] = pMiss * p[i]
        pTotal += posterior_p[i]
    for i in range(len(p)):
        posterior_p[i] /= pTotal
    return posterior_p
            
cmd = ''
print "World:"
print map
print 'Initial distribution:'
print p

while cmd != 'quit' and cmd != 'exit' and cmd != 'q':
    print 'Enter a command:'
    cmd = raw_input()
    if cmd == 'left':
        p = move(-1, p)
    elif cmd == 'right':
        p = move(1, p)
    elif cmd == 'black':
        p = observe(cmd, p)
    elif cmd == 'white':
        p = observe(cmd, p)
    else:
        print 'Pleas enter a valid command'
    print 'The new probability distribution is:'
    print p
        
    
    

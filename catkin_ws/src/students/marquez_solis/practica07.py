#!/usr/bin/env python3
#
#ROBOTS MOVILES Y AGENTES INTELIGENTES
#PRACTICA 7
#LOCALIZACION POR EL METODO DEL HISTOGRAMA
#
import numpy

numpy.set_printoptions(precision=3)
smap=[['white','black','black','white','black'],
     ['white','white','black','black','black'],
     ['white','black','black','white','white'],
     ['black','black','white','white','black'],
     ['black','white','black','white','black']]

amap = numpy.array(smap)


p = numpy.ndarray(shape=(len(smap),len(smap[0])))
for i in range(len(smap)):
    for j in range(len(smap[0])):
        p[i][j] = 1.0/(len(smap)*len(smap[0]))

pHit  = 0.8
pMiss = 0.2

pCorrect = 0.6
pUnder   = 0.2
pOver    = 0.2

def move_y(cells, p):
    updated_p = numpy.zeros(shape=(len(smap),len(smap[0])))
    rows, cols = numpy.indices(p.shape) 
    n = 1 if cells > 0 else -1
    updated_p += pUnder*p[(rows-cells+n)%p.shape[0],cols]
    updated_p += pCorrect*p[(rows-cells)%p.shape[0],cols]
    updated_p += pOver*p[(rows-cells-n)%p.shape[0],cols]
    return updated_p

def move_x(cells, p):
    updated_p = numpy.zeros(shape=(len(smap),len(smap[0])))
    rows, cols = numpy.indices(p.shape)
    n = 1 if cells > 0 else -1
    updated_p += pUnder*p[rows,(cols-cells+n)%p.shape[1]]
    updated_p += pCorrect*p[rows,(cols-cells)%p.shape[1]]
    updated_p += pOver*p[rows,(cols-cells-n)%p.shape[1]]
    return updated_p

def observe(landmark, p):
    new_p = numpy.array(p)
    mask = numpy.where(amap == landmark,pHit,pMiss)
    new_p *= mask
    new_p /= numpy.sum(new_p)
    return new_p
            
cmd = ''
print ('World:')
for i in range(len(smap)):
    print (smap[i])
        
print ('Initial distribution:')
print (p)

while cmd != 'quit' and cmd != 'exit' and cmd != 'q':
    print ('Enter a command:')
    cmd = input()
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
    elif cmd == 'q':
        break
    else:
        print ('Please enter a valid command')
    print ('The new probability distribution is:')
    print (p)

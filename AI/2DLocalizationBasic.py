colors = [['red', 'green', 'green', 'red' , 'red'],
          ['red', 'red', 'green', 'red', 'red'],
          ['red', 'red', 'green', 'green', 'red'],
          ['red', 'red', 'red', 'red', 'red']]

measurements = ['green', 'green', 'green' ,'green', 'green']


motions = [[0,0],[0,1],[1,0],[1,0],[0,1]]

sensor_right = 0.7
p_move = 0.8

def show(p):
    for i in range(len(p)):
        print p[i]



def move(p, U):
    #move L or R if U[2] != 0
    # if U[1] is 1, shift probabilities right
    # if U[1] is -1, shift probabilities left
    #if U[0] is 1, shift down
    #if U[0] is -1, shift up
    q = [[0 for x in xrange(cols)]for x in xrange(rows)]
    # q[row][col]

    #shift right or left
    for i in range(len(p)):
        for j in range(len(p[i])):
		#Add the probabilities of coming from all possible other grid cells
            q[i][j] = p_move * ( p[(i-U[0]) % len(p)][(j - U[1]) % len(p[i]) ] ) + (1-p_move)*p[i][j]
    return q

def sense(p, M, colors):
    
    q = [[0 for x in xrange(cols)]for x in xrange(rows)]
    
    for mycol in range(cols):
        for myrow in range(rows):
            iscorrect = (M == colors[myrow][mycol])
            q[myrow][mycol] = p[myrow][mycol] * (iscorrect * sensor_right + (1-iscorrect) * (1-sensor_right))
            
    mysum = sum(sum(x) for x in  q)
    for mycol in range(cols):
        for myrow in range(rows):
            q[myrow][mycol] = q[myrow][mycol]/mysum
    return q
    

y = len(colors)
x = len(colors[0])
uniform = 1/(float(x)*float(y))
rows = y
cols = x
# Init p with uniform prior distribution
p = [[uniform for x in xrange(5)]for x in xrange(4)]
show(p)
for action in range(len(measurements)):
    p = move(p, motions[action])
    p = sense(p, measurements[action], colors)

print "space"

#Your probability array must be printed 
#with the following code.

show(p)





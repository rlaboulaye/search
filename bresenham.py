import matplotlib.pyplot as plt

def get_points(p0, p1):
    x0 = p0[0]
    y0 = p0[1]
    x1 = p1[0]
    y1 = p1[1]
    deltax = x1 - x0
    deltay = y1 - y0

    if (abs(deltay) < abs(deltax)):
        a_index = 0
        b_index = 1
        a0 = x0
        a1 = x1
        b0 = y0
        b1 = y1
        delta_a = deltax
        delta_b = deltay
        deltaerr = abs(deltay / deltax)
    else:
        a_index = 1
        b_index = 0
        a0 = y0
        a1 = y1
        b0 = x0
        b1 = x1
        delta_a = deltay
        delta_b = deltax
        deltaerr = abs(deltax / deltay)

    b_increment = 1
    if (delta_b < 0):
        b_increment = -1

    a_increment = 1
    if (delta_a < 0):
        a_increment = -1

    arrays = [[],[]]
    error = deltaerr - .5
    b = b0
    for a in range(a0, a1, a_increment):
        arrays[a_index].append(a)
        arrays[b_index].append(b)
        error += deltaerr
        if error >= .5:
            b += b_increment
            error -= 1
    return arrays

p0 = [0,0]
p1 = [-4,2]
xs, ys = get_points(p0, p1)
plt.scatter(xs, ys, marker='o', color='b')
ax = plt.gca()
ax.set_xlim([-10,10])
ax.set_ylim([-10,10])
plt.show()

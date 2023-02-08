Web VPython 3.2

# Background
scene.autoscale = False
scene.width = 1400
scene.height = 600
sphere(pos=vector(0,0,0),texture="https://i.imgur.com/1nVWbbd.jpg",radius=55,shininess=0)

# Graphs
gke = graph(title='<b>Kinetic energies</b>',
      xtitle='<i>t-time</i>', ytitle='<i>Ke-Kinetic energy</i>',
      foreground=color.black, background=color.white)

gtotal = graph(title='<b>Total energy</b>',
      xtitle='<i>t-time</i>', ytitle='<i>Ke+Pe</i>',
      foreground=color.black, background=color.white)
total = gcurve(color=color.black,label="total energy")

planets = []

# KINEMATICS

# multiplication for matrix without using np
def matmul(A, B):
    rows_A = len(A)
    cols_A = len(A[0])
    rows_B = len(B)
    cols_B = len(B[0])
    
    if cols_A != rows_B:
      raise ValueError("Cannot multiply the two matrices. Incompatible dimensions.")

    # Create the result matrix C with the same number of rows as A and the same number of columns as B
    C = [[0 for row in range(cols_B)] for col in range(rows_A)]

    for i in range(rows_A):
        for j in range(cols_B):
            for k in range(cols_A):
                C[i][j] += A[i][k] * B[k][j]
    return C
    
def rotation_matrix(euler_angles):
    # Making the rotation matrix
    R_x = [[1, 0, 0],
       [0, cos(euler_angles[1]), -sin(euler_angles[1])],
       [0, sin(euler_angles[1]), cos(euler_angles[1])]]
    
    R_y = [[cos(euler_angles[0]), 0, sin(euler_angles[0])],
       [0, 1, 0],
       [-sin(euler_angles[0]), 0, cos(euler_angles[0])]]
    
    R_z = [[cos(euler_angles[2]), -sin(euler_angles[2]), 0],
       [sin(euler_angles[2]), cos(euler_angles[2]), 0],
       [0, 0, 1]]
    
    return matmul(matmul(R_z, R_y), R_x)
   
def extract_angles(rotation_matrix):
    # Extracting the angle of rotation from the sum of diagonal elements of the rotation matrix
    angle = acos((R[0][0] + R[1][1] + R[2][2] - 1)/2)  
    # Compute the axis of rotation from the matrix
    rotation_axis = [R[2][1] - R[1][2], R[0][2] - R[2][0], R[1][0] - R[0][1]]
    # normalize the axis
    axis_normalized = [x/sin(angle) for x in rotation_axis]
    return angle, axis_normalized   
 
def gforce(p1,p2):
    # Calculate the gravitational force exerted on p1 by p2.
    G = 1
    # Calculate distance vector between p1 and p2.
    r_vec = p1.pos-p2.pos
    # Calculate magnitude of distance vector.
    r_mag = mag(r_vec)
    # Calcualte unit vector of distance vector.
    r_hat = r_vec/r_mag
    # Calculate force magnitude.
    force_mag = G*p1.mass*p2.mass/r_mag**2
    # Calculate force vector.
    force_vec = -force_mag*r_hat
    return force_vec
        
def cal_total_force(goal_planet, planets):
    total_force = vec(0,0,0)
    for planet in planets:
        if planet.mass != goal_planet.mass:
            total_force += gforce(goal_planet, planet)
    return total_force
    
def ke(p1):
    # Calculate the kinetic energy of p1.
    ke = 0.5*p1.mass*mag(p1.velocity)**2
    
    return ke
    
def gpe(p1,p2):
    # Calculate the gravitational potential energy between p1 and p2.
    G = 1
    # Calculate distance vector between p1 and p2.
    r_vec = p1.pos-p2.pos
    # Calculate magnitude of distance vector.
    r_mag = mag(r_vec)
    # Calculate gravitational potential energy.
    gpe = -G*p1.mass*p2.mass/r_mag
    
    return gpe

def cal_total_energy(goal_planet, planets, is_ke):
    total_energy = 0
    for planet in planets:
        if is_ke:
            total_energy += ke(planet)
        else:
            if goal_planet.mass != planet.mass:
                total_energy += gpe(goal_planet, planet)
    return total_energy

# COLLISIONS

# SPHERES-SPHERES
def sphere_collisions(sp1, sp2):
    collision = False
    
    # Calculate the distance between centers of two spheres
    d = sqrt((sp1.pos.x - sp2.pos.x)**2 + (sp1.pos.y - sp2.pos.y)**2 + (sp1.pos.z - sp2.pos.z)**2)
    
    if d - (sp1.radius + sp2.radius) > 0:
        #print("d = " + str(d - (sp1.radius + sp2.radius)))
        pass
    else:
        print("collision")
        collision = True
    
    return collision

def check_all_sphere_colls(spheres_list):
    collision_check = []
    
    for i in range(len(spheres_list) - 1):
        for j in range(i + 1, len(spheres_list)):
            temp_col_check = sphere_collisions(spheres_list[i], spheres_list[j])
            collision_check.append(temp_col_check)
    
    return collision_check

# SPHERES-POLYHEDRONS
def calc_vertices(b):
        vertices = []
        
        A = b.pos + vec(-b.size.x/2,-b.size.y/2,b.size.z/2)
        B = b.pos + vec(b.size.x/2,-b.size.y/2,b.size.z/2)
        C = b.pos + vec(b.size.x/2,-b.size.y/2,-b.size.z/2)
        D = b.pos + vec(-b.size.x/2,-b.size.y/2,-b.size.z/2)
        A1 = b.pos + vec(-b.size.x/2,b.size.y/2,b.size.z/2)
        B1 = b.pos + vec(b.size.x/2,b.size.y/2,b.size.z/2)
        C1 = b.pos + vec(b.size.x/2,b.size.y/2,-b.size.z/2)
        D1 = b.pos + vec(-b.size.x/2,b.size.y/2,-b.size.z/2)
    
        vertices.append(A)
        vertices.append(B)
        vertices.append(C)
        vertices.append(D)
        vertices.append(A1)
        vertices.append(B1)
        vertices.append(C1)
        vertices.append(D1)
        
        return vertices
    
def calc_faces(vertices):
    faces = []
    
    face0 = [vertices[0], vertices[1], vertices[5], vertices[4]]
    face1 = [vertices[3], vertices[2], vertices[6], vertices[7]]
    face2 = [vertices[0], vertices[1], vertices[2], vertices[3]]
    face3 = [vertices[4], vertices[5], vertices[6], vertices[7]]
    face4 = [vertices[0], vertices[3], vertices[7], vertices[4]]
    face5 = [vertices[1], vertices[2], vertices[6], vertices[5]]
    
    faces.append(face0)
    faces.append(face1)
    faces.append(face2)
    faces.append(face3)
    faces.append(face4)
    faces.append(face5)
    
    return faces
    
# Calculate projection
def projection(vertex, normal):
    return (dot(vertex, normal) / mag(normal))
    
def check_collision(b, s):
    # Define the faces of the polyhedron
    vertices = calc_vertices(b)
    #print("vertices = " + str(vertices))
    
    faces = calc_faces(vertices)
    #print("faces = " + str(faces))
    
    # Define the normal vectors of the faces
    normal_vectors = []
    for face in faces:    
        v0 = face[1] - face[0]
        v1 = face[2] - face[0]

        #print("crossed" + str(crossed))
        normal_vectors.append(cross(v0, v1))
    
    #print("normal vectors = " + str(normal_vectors))

    is_collision = True
    for normal in normal_vectors:
        min_sep = (float)1000000
        
        sphere_min = projection(s.pos, normal) - s.radius
        sphere_max = projection(s.pos, normal) + s.radius
       # print(sphere_min, sphere_max)
        poly_min = 100000000
        poly_max = (float)(-100000000)
        for vertex in vertices:
            #print(projection(vertex, normal))
            poly_min = min(projection(vertex, normal), poly_min)
            poly_max = max(projection(vertex, normal), poly_max)
        
       # print(poly_min, poly_max)
        overlap = max(poly_min, sphere_min) - min(poly_max, sphere_max)
        
        if overlap > 0:
            is_collision = False    
            #print("no collision")
            break
        
    return is_collision

#initializing bodies
sun = sphere( pos=vector(0,0,0), radius=0.08, color=color.yellow,  rotation = vec(0,0,0),
               mass = 2000, velocity = vec(0,0,3), force = vec(0,0,0), make_trail=True, grph=gcurve(color=color.yellow, label="sun KE", graph = gke), texture = "https://upload.wikimedia.org/wikipedia/commons/thumb/9/99/Map_of_the_full_sun.jpg/1280px-Map_of_the_full_sun.jpg" ) # mass is a scaled down mass, velocity is just a ranfom velocity that works well here
earth = sphere( pos=vector(1,0,0), radius=0.03, color=color.cyan, rotation = vec(0,23.5,0),
               mass = 1, velocity = vec(0,29.78,0), make_trail=True, grph=gcurve(color=color.cyan, label="earth KE", graph = gke), texture = "https://media.istockphoto.com/id/182058785/photo/world-topographic-map.jpg?s=612x612&w=0&k=20&c=eWrcGjNB9o-KrzW4TC2yxUII7k5E26QIqlN3JEJu1e4=" ) # mass is a scaled down mass using the same principle as suns mass
mars = sphere( pos=vector(1.524,0,0), radius=0.025, force = vec(0,0,0), color=color.red, rotation = vec(0,25.19,0),
               mass = 0.107, velocity = vec(0,24.077,0), make_trail=True, grph=gcurve(color=color.red, label="mars KE", graph = gke), texture = "data:image/jpeg;base64,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" )
pluto = sphere( pos=vector(39.44,0,0), radius=0.05, force = vec(0,0,0), color=color.purple, rotation = vec(0,119.6,0),
               mass = 0.002, velocity = vec(0,4.74 ,0), make_trail=True, grph=gcurve(color=color.purple, label="pluto KE", graph = gke), texture = "data:image/jpeg;base64,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" )               
neptune = sphere( pos=vector(30.07,0,0), radius=0.1, force = vec(0,0,0), color=color.blue, rotation = vec(0,29.6,0),
               mass = 17.2, velocity = vec(0,5.43,0), make_trail=True, grph=gcurve(color=color.blue, label="neptune KE", graph = gke), texture = "data:image/jpeg;base64,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" )
jupiter = sphere( pos=vector(5.2,0,0), radius=0.1, force = vec(0,0,0), color=color.orange, rotation = vec(0,3.13,0),
               mass = 317.8, velocity = vec(0,13.07,0), make_trail=True, grph=gcurve(color=color.orange, label="jupiter KE", graph = gke), texture = "https://upload.wikimedia.org/wikipedia/commons/b/be/Solarsystemscope_texture_2k_jupiter.jpg" )
saturn = sphere( pos=vector(9.58,0,0), radius=0.08, force = vec(0,0,0), color=color.rgb_to_hsv(vec(255,181,51)), rotation = vec(0,26.73,0),
               mass = 95.2, velocity = vec(0,9.69,0), make_trail=True, grph=gcurve(color=color.rgb_to_hsv(vec(255,181,51)), label="saturn KE", graph = gke), texture = "https://upload.wikimedia.org/wikipedia/commons/e/ea/Solarsystemscope_texture_2k_saturn.jpg" )
venus = sphere( pos=vector(0.72,0,0), radius=0.02, force = vec(0,0,0), color=color.orange, rotation = vec(0,177.36,0),
               mass = 0.815, velocity = vec(0,35.02,0), make_trail=True, grph=gcurve(color=color.orange, label="venus KE", graph = gke), texture = "data:image/jpeg;base64,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" )
mercury = sphere( pos=vector(0.39,0,0), radius=0.01, force = vec(0,0,0), color=color.white, rotation = vec(0,0,0),
               mass = 0.055, velocity = vec(0,47.87,0), make_trail=True, grph=gcurve(color=color.white, label="mercury KE", graph = gke), texture = "https://upload.wikimedia.org/wikipedia/commons/9/92/Solarsystemscope_texture_2k_mercury.jpg" )

#asteroid = box( pos=vector(0.8,0,0), size= vec(0.03,0.03,0.03), force = vec(0,0,0), color=color.purple, rotation = vec(0,0,0),
#               mass = 5e-5, velocity = vec(0,10,0), make_trail=True, grph=gcurve(color=color.white, label="mercury KE", graph = gke), texture = "https://upload.wikimedia.org/wikipedia/commons/9/92/Solarsystemscope_texture_2k_mercury.jpg" )

#adding all planets to list
planets.append(sun)
planets.append(earth)
planets.append(mars)
planets.append(pluto)
planets.append(saturn)
planets.append(jupiter)
planets.append(venus)
planets.append(mercury)
planets.append(neptune)

# Making many asteroids
asteroids = []
m_min = 0.01
m_max = 0.10
r_min = 4
r_max = 9
for i in range(0,20):
    mass = m_min + random()*(m_max-m_min)
    r = r_min + random()*(r_max-r_min)
    theta = random()*2*pi
    velocity = mass*sqrt(sun.mass/r)
    asteroids.append(box(pos=r*vector(cos(theta),sin(theta),random(-3,3)),velocity=velocity*vector(-sin(theta),cos(theta),0),mass=mass, color=color.white, size = vec(0.09,0.09,0.09)))

# Defining euler angles and angular velocities for all the bodies
# Euler angles (roll, pitch, yaw)
euler_angles_sun = [7e-4, 0, 2e-7] #0.1, 0.2, 0.3
# Angular velocity of the planet (roll_dot, pitch_dot, yaw_dot)
angular_velocity_sun = vec(1.16e-7, 0, 0)

euler_angles_earth = [2e-2, 23.5e-7, 2e-7]
angular_velocity_earth = vec(0.0041780, 0, 0)

euler_angles_mars = [2e-2, 25.19e-7, 2e-7]
angular_velocity_mars = vec(0.00068, 0, 0)

euler_angles_pluto = [2e-2, 119.6e-7, 2e-7]
angular_velocity_pluto = vec(0.0001, 0, 0)

euler_angles_saturn = [2e-2, 26.73e-7, 2e-7]
angular_velocity_saturn = vec(0.0012, 0, 0)

euler_angles_jupiter = [2e-2, 3.13e-7, 2e-7]
angular_velocity_jupiter = vec(0.0014, 0, 0)

euler_angles_venus = [-2e-2, 177.36e-7, 2e-7]
angular_velocity_venus = vec(-0.0001, 0, 0)

euler_angles_mercury = [2e-2, 0, 2e-7]
angular_velocity_mercury = vec(0.0107, 0, 0)

euler_angles_neptune = [2e-2, 29.6e-7, 2e-7]
angular_velocity_neptune = vec(0.0007, 0, 0)

dt = 0.0001 # timestep - deltatime
t = 0
total_e = 0
while (True):
    rate(5000) #og je 500 a dt je 0.0001
    
    # Check for collisions
    list_of_collisions = check_all_sphere_colls(planets)
    happened = False
    for collision in list_of_collisions:
        if collision:
            happened = True
            break
    #print(list_of_collisions)
    if happened:
        break
    
    # Check other collisions
#    for planet in planets:
#        check = check_collision(asteroid, planet)
#        if check:
#            print("collision")
#            happened = True
#            break
#    if happened:
#        break
    
    for a in asteroids:
        for planet in planets:
            check = check_collision(a, planet)
            if check:
                print("collision")
                happened = True
                break
        if happened:
            break
    if happened:
        break
    
    # Calculate forces
    total_sun_force = cal_total_force(sun, planets)
    sun.force = total_sun_force
    
    total_earth_force = cal_total_force(earth, planets)
    earth.force = total_earth_force
    
    total_mars_force = cal_total_force(mars, planets)
    mars.force = total_mars_force
 
    total_jupiter_force = cal_total_force(jupiter, planets)
    jupiter.force = total_jupiter_force
    
    total_saturn_force = cal_total_force(saturn, planets)
    saturn.force = total_saturn_force
    
    total_venus_force = cal_total_force(venus, planets)
    venus.force = total_venus_force
   
    total_mercury_force = cal_total_force(mercury, planets)
    mercury.force = total_mercury_force
    
    total_pluto_force = cal_total_force(pluto, planets)
    pluto.force = total_pluto_force
    
    total_neptune_force = cal_total_force(neptune, planets)
    neptune.force = total_neptune_force
    
#    total_asteroid_force = cal_total_force(asteroid, planets)
#    asteroid.force = total_asteroid_force
    
    for a in asteroids:
        a.force = cal_total_force(a, planets)
    
    # Update velocity
    sun.velocity = sun.velocity + sun.force/sun.mass*dt
    earth.velocity = earth.velocity + earth.force/earth.mass*dt
    mars.velocity = mars.velocity + mars.force/mars.mass*dt
    jupiter.velocity = jupiter.velocity + jupiter.force/jupiter.mass*dt
    saturn.velocity = saturn.velocity + saturn.force/saturn.mass*dt
    venus.velocity = venus.velocity + venus.force/venus.mass*dt
    mercury.velocity = mercury.velocity + mercury.force/mercury.mass*dt
    pluto.velocity = pluto.velocity + pluto.force/pluto.mass*dt
    neptune.velocity = neptune.velocity + neptune.force/neptune.mass*dt
#    asteroid.velocity = asteroid.velocity + asteroid.force/asteroid.mass*dt
    for a in asteroids:
        a.velocity = a.velocity + a.force/a.mass*dt
    
    # Update positions
    sun.pos = sun.pos + sun.velocity*dt
    earth.pos = earth.pos + earth.velocity*dt
    mars.pos = mars.pos + mars.velocity*dt
    jupiter.pos = jupiter.pos + jupiter.velocity*dt
    saturn.pos = saturn.pos + saturn.velocity*dt
    venus.pos = venus.pos + venus.velocity*dt
    mercury.pos = mercury.pos + mercury.velocity*dt
    pluto.pos = pluto.pos + pluto.velocity*dt
    neptune.pos = neptune.pos + neptune.velocity*dt
#    asteroid.pos = asteroid.pos + asteroid.velocity*dt
    for a in asteroids:
        a.pos = a.pos + a.velocity*dt

    # Update energy graphs
    sun.grph.plot(pos=(t,ke(sun)))
    earth.grph.plot(pos=(t,ke(earth)))
    mars.grph.plot(pos=(t,ke(mars)))
    pluto.grph.plot(pos=(t,ke(pluto)))
    venus.grph.plot(pos=(t,ke(venus)))
    mercury.grph.plot(pos=(t,ke(mercury)))
    saturn.grph.plot(pos=(t,ke(saturn)))
    neptune.grph.plot(pos=(t,ke(neptune)))
    jupiter.grph.plot(pos=(t,ke(jupiter)))
    
    total_e = cal_total_energy(None, planets, True) # planet = None, planets list of astral bodies, is_ke = True
    total_e += cal_total_energy(sun, planets, False)
    total_e += cal_total_energy(earth, planets, False)
    total_e += cal_total_energy(mars, planets, False)
    total_e += cal_total_energy(jupiter, planets, False)
    total_e += cal_total_energy(saturn, planets, False)
    total_e += cal_total_energy(venus, planets, False)
    total_e += cal_total_energy(mercury, planets, False)
    total_e += cal_total_energy(pluto, planets, False)
    total_e += cal_total_energy(neptune, planets, False)
    
    total.plot(pos=(t,total_e))
    
    ################## ROTATION ##################
    R = rotation_matrix(euler_angles_sun)    
    angle, axis_normalized = extract_angles(R)
    # Update the Euler angles
    euler_angles_sun += angular_velocity_sun * dt
    # Use angle-axis representation in GlowScript
    sun.rotate(angle=angle, axis=vec(axis_normalized[0], axis_normalized[1], axis_normalized[2]))
    
    R = rotation_matrix(euler_angles_earth)    
    angle, axis_normalized = extract_angles(R)
    # Update the Euler angles
    euler_angles_earth += angular_velocity_earth * dt
    # Use angle-axis representation in GlowScript
    earth.rotate(angle=angle, axis=vec(axis_normalized[0], axis_normalized[1], axis_normalized[2]))
    
    R = rotation_matrix(euler_angles_mars)    
    angle, axis_normalized = extract_angles(R)
    # Update the Euler angles
    euler_angles_mars += angular_velocity_mars * dt
    # Use angle-axis representation in GlowScript
    mars.rotate(angle=angle, axis=vec(axis_normalized[0], axis_normalized[1], axis_normalized[2]))
    
    R = rotation_matrix(euler_angles_jupiter)    
    angle, axis_normalized = extract_angles(R)
    # Update the Euler angles
    euler_angles_jupiter += angular_velocity_jupiter * dt
    # Use angle-axis representation in GlowScript
    jupiter.rotate(angle=angle, axis=vec(axis_normalized[0], axis_normalized[1], axis_normalized[2]))
    
    R = rotation_matrix(euler_angles_saturn)    
    angle, axis_normalized = extract_angles(R)
    # Update the Euler angles
    euler_angles_saturn += angular_velocity_saturn * dt
    # Use angle-axis representation in GlowScript
    saturn.rotate(angle=angle, axis=vec(axis_normalized[0], axis_normalized[1], axis_normalized[2]))
    
    R = rotation_matrix(euler_angles_venus)    
    angle, axis_normalized = extract_angles(R)
    # Update the Euler angles
    euler_angles_venus += angular_velocity_venus * dt
    # Use angle-axis representation in GlowScript
    venus.rotate(angle=angle, axis=vec(axis_normalized[0], axis_normalized[1], axis_normalized[2]))
    
    R = rotation_matrix(euler_angles_mercury)    
    angle, axis_normalized = extract_angles(R)
    # Update the Euler angles
    euler_angles_mercury += angular_velocity_mercury * dt
    # Use angle-axis representation in GlowScript
    mercury.rotate(angle=angle, axis=vec(axis_normalized[0], axis_normalized[1], axis_normalized[2]))
    
    R = rotation_matrix(euler_angles_pluto)    
    angle, axis_normalized = extract_angles(R)
    # Update the Euler angles
    euler_angles_pluto += angular_velocity_pluto * dt
    # Use angle-axis representation in GlowScript
    pluto.rotate(angle=angle, axis=vec(axis_normalized[0], axis_normalized[1], axis_normalized[2]))
    
    R = rotation_matrix(euler_angles_neptune)    
    angle, axis_normalized = extract_angles(R)
    # Update the Euler angles
    euler_angles_neptune += angular_velocity_neptune * dt
    # Use angle-axis representation in GlowScript
    neptune.rotate(angle=angle, axis=vec(axis_normalized[0], axis_normalized[1], axis_normalized[2]))

    #update time with tmestep
    t = t + dt
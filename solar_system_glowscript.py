Web VPython 3.2

gke = graph(title='<b>Kinetic energies</b>',
      xtitle='<i>t-time</i>', ytitle='<i>Ke-Kinetic energy</i>',
      foreground=color.black, background=color.white)

gtotal = graph(title='<b>Total energy</b>',
      xtitle='<i>t-time</i>', ytitle='<i>Ke+Pe</i>',
      foreground=color.black, background=color.white)
total = gcurve(color=color.black,label="total energy")

planets = []

def gforce(p1,p2):
    # Calculate the gravitational force exerted on p1 by p2.
    G = 1 #6.67408e-11# Smaller value of G
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
    G = 1 # Change to 6.67e-11 to use real-world values.
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

#initializing bodies
sun = sphere( pos=vector(0,0,0), radius=0.08, color=color.yellow,
               mass = 2000, velocity = vec(0,0,3), force = vec(0,0,0), make_trail=True, grph=gcurve(color=color.yellow, label="sun KE", graph = gke) ) # mass is a scaled down mass, velocity is just a ranfom velocity that works well here
earth = sphere( pos=vector(1,0,0), radius=0.06, color=color.blue,
               mass = 1, velocity = vec(0,29.78,0), make_trail=True, grph=gcurve(color=color.blue, label="earth KE", graph = gke) ) # mass is a scaled down mass using the same principle as suns mass
mars = sphere( pos=vector(1.524,0,0), radius=0.05, force = vec(0,0,0), color=color.red,
               mass = 0.107, velocity = vec(0,24.077,0), make_trail=True, grph=gcurve(color=color.red, label="mars KE", graph = gke) )
pluto = sphere( pos=vector(39.44,0,0), radius=0.05, force = vec(0,0,0), color=color.purple,
               mass = 0.002, velocity = vec(0,4.74 ,0), make_trail=True, grph=gcurve(color=color.purple, label="pluto KE", graph = gke) )               
neptune = sphere( pos=vector(30.07,0,0), radius=0.1, force = vec(0,0,0), color=color.blue,
               mass = 17.2, velocity = vec(0,5.43,0), make_trail=True, grph=gcurve(color=color.blue, label="neptune KE", graph = gke) )
jupiter = sphere( pos=vector(5.2,0,0), radius=0.1, force = vec(0,0,0), color=color.orange,
               mass = 317.8, velocity = vec(0,13.07,0), make_trail=True, grph=gcurve(color=color.orange, label="jupiter KE", graph = gke) )
saturn = sphere( pos=vector(9.58,0,0), radius=0.08, force = vec(0,0,0), color=color.rgb_to_hsv(vec(255,181,51)),
               mass = 95.2, velocity = vec(0,9.69,0), make_trail=True, grph=gcurve(color=color.rgb_to_hsv(vec(255,181,51)), label="saturn KE", graph = gke) )
venus = sphere( pos=vector(0.72,0,0), radius=0.02, force = vec(0,0,0), color=color.rgb_to_hsv(vec(255,91,51)),
               mass = 0.815, velocity = vec(0,35.02,0), make_trail=True, grph=gcurve(color=color.rgb_to_hsv(vec(255,91,51)), label="venus KE", graph = gke) )
mercury = sphere( pos=vector(0.39,0,0), radius=0.02, force = vec(0,0,0), color=color.white,
               mass = 0.055, velocity = vec(0,47.87,0), make_trail=True, grph=gcurve(color=color.white, label="mercury KE", graph = gke) )

planets.append(sun)
planets.append(earth)
planets.append(mars)
planets.append(pluto)
planets.append(saturn)
planets.append(jupiter)
planets.append(venus)
planets.append(mercury)
planets.append(neptune)

dt = 0.00005 # timestep - deltatime
t = 0
total_e = 0
while (True):
    rate(500)
    
    # Calculate forces.
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
    
    # Update momenta.
    sun.velocity = sun.velocity + sun.force/sun.mass*dt
    earth.velocity = earth.velocity + earth.force/earth.mass*dt
    mars.velocity = mars.velocity + mars.force/mars.mass*dt
    jupiter.velocity = jupiter.velocity + jupiter.force/jupiter.mass*dt
    saturn.velocity = saturn.velocity + saturn.force/saturn.mass*dt
    venus.velocity = venus.velocity + venus.force/venus.mass*dt
    mercury.velocity = mercury.velocity + mercury.force/mercury.mass*dt
    pluto.velocity = pluto.velocity + pluto.force/pluto.mass*dt
    neptune.velocity = neptune.velocity + neptune.force/neptune.mass*dt
    
    # Update positions.
    sun.pos = sun.pos + sun.velocity*dt
    earth.pos = earth.pos + earth.velocity*dt
    mars.pos = mars.pos + mars.velocity*dt
    jupiter.pos = jupiter.pos + jupiter.velocity*dt
    saturn.pos = saturn.pos + saturn.velocity*dt
    venus.pos = venus.pos + venus.velocity*dt
    mercury.pos = mercury.pos + mercury.velocity*dt
    pluto.pos = pluto.pos + pluto.velocity*dt
    neptune.pos = neptune.pos + neptune.velocity*dt

    # Update energy graphs.
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
    
    #update time with tmestep
    t = t + dt
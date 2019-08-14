from collision1d import *

env = PhysEnv()
env.add_obj(Wall(coord=0))
point1 = PointOfMass(coord=10, mass=1)
point1.velocity = -5
point2 = PointOfMass(coord=20, mass=50)
point2.velocity = -20
env.add_obj(point1)
env.add_obj(point2)
print(env)
collision = 0

while True:
    try:
        env.until_next_collision()
        collision += 1
        print(env)
        print(collision)
        print()
    except SimulationEndException:
        print(env)
        break

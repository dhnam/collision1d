#TEST OK

from collision1d import *

def main():
    env = PhysEnv()
    point1 = PointOfMass(coord=10, mass=1)
    point1.velocity = 10
    point2 = PointOfMass(coord=20, mass=5)
    point2.velocity = 0
    point3 = PointOfMass(coord=30, mass=1)
    point3.velocity = -10

    env.add_obj(point1)
    env.add_obj(point2)
    env.add_obj(point3)

    collision = 0
    print("======BEFORE======")
    print(env)
    while True:
        try:
            env.until_next_collision()
            collision += 1
            print("#", collision, " at time ", env.time, sep='')
            print(env)
        except SimulationEndException:
            print("No more collision after #", collision, sep='')
            #print(env)
            break
    print("======AFTER======")
    print(env)
    
if __name__ == '__main__':
    main()

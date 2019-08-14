from collision1d import *

def main():
    mass = int(input("mass? "))
    env = PhysEnv()
    env.add_obj(Wall(coord=0))
    point1 = PointOfMass(coord=10, mass=1)
    point1.velocity = 0
    env.add_obj(point1)
    point2 = PointOfMass(coord=20, mass=mass)
    point2.velocity = -1
    env.add_obj(point2)
    print(env)

    collision = 0
    while True:
        try:
            env.until_next_collision()
            collision += 1
            if collision % 100 == 0:
                print("#", collision, " at time ", env.time, sep = '')
                print(env)
                print()
        except SimulationEndException:
            print("No more collision after #", collision)
            print(env)
            break

if __name__ == '__main__':
    main()

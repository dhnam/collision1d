#Simple 1D physics engine.... collision engine. No accumulation.
#Uses fractions module (because it's accurate). == It is slow.

from fractions import *

class PhysObject:
    def __init__(self):
        self._restitution = None
        self._position = None
        self._velocity = None

    @property
    def restitution(self):
        return self._restitution

    @restitution.setter
    def restitution(self, value):
        if not isinstance(value, (float, int, Fraction)):
            raise TypeError
        if value < 0 or value > 1:
            raise ValueError
        self._restitution = Fraction(str(value))

    @property
    def position(self):
        return self._position

    @position.setter
    def position(self, value):
        if not isinstance(value, (float, int, Fraction)):
            raise TypeError

        self._position = Fraction(str(value))

    @property
    def velocity(self):
        return self._velocity

    @velocity.setter
    def velocity(self, value):
        if not isinstance(value, (float, int, Fraction)):
            raise TypeError

        self._velocity = Fraction(str(value))


class Wall(PhysObject):
    def __init__(self, coord=0, restitution=1):
        self.restitution = restitution
        self.position = coord
        self._velocity = Fraction(0)

    @property
    def velocity(self):
        return super().velocity

    @velocity.setter
    def velocity(self, value):
        raise AttributeError("can't set attribute")

    def __str__(self):
        return "Wall(e=" + str(self.restitution)\
               + ")@" + str(self.position)

    def __repr__(self):
        return self.__str__()

class PointOfMass(PhysObject):
    def __init__(self, coord, mass, restitution=1):
        if not isinstance(mass, (int, float, Fraction)):
            raise TypeError
        if mass < 0:
            raise ValueError
        self._mass = Fraction(str(mass))
        self.position = coord
        self.velocity = 0
        self.restitution = restitution

    @property
    def mass(self):
        return self._mass

    @property
    def kinetic_energy(self):
        return self._mass * self.velocity**2 / 2

    @property
    def momentum(self):
        return self._mass * self.velocity

    def __str__(self):
        return "PointOfMass(m=" + str(float(self.mass)) + "/e="\
               + str(float(self.restitution)) +")@"\
               + str(float(self.position)) +"/v="\
               + str(float(self.velocity))

    def __repr__(self):
        return self.__str__()

class SimulationEndException(Exception):
    pass

class PhysEnv:
    def __init__(self):
        self._objs = []
        self._time = Fraction('0')

    @property
    def time(self):
        return float(self._time)

    @property
    def net_kinetic(self):
        net = 0
        for next_obj in self._objs:
            if isinstance(next_obj, PointOfMass):
                net += next_obj.kinetic_energy

        return net

    @property
    def net_momentum(self):
        net = 0
        for next_obj in self._objs:
            if isinstance(next_obj, PointOfMass):
                net += next_obj.momentum

        return net

    def add_obj(self, obj):
        if not isinstance(obj, PhysObject):
            raise TypeError
        self._objs.append(obj)
        self._objs.sort(key=lambda objs: objs.position)

    def after_time(self, time):
        pass

    def until_next_collision(self):
        # TODO : When 3+ things collide, works like 2 things from left collide
        #        first. Can I fix it?
        while True:
            time_table = []
            for i in range(1, len(self._objs) - 1):
                this_vel = self._objs[i].velocity
                left_vel = self._objs[i - 1].velocity
                right_vel = self._objs[i + 1].velocity
                left_len = self._objs[i].position - self._objs[i - 1].position
                right_len = self._objs[i + 1].position - self._objs[i].position
                next_time = float('Inf')
                if left_vel > this_vel:
                    if left_len / (left_vel-this_vel) < next_time:
                        next_time = left_len / (left_vel-this_vel)
                if right_vel < this_vel:
                    if right_len / (this_vel-right_vel) < next_time:
                        next_time = right_len / (this_vel-right_vel)
                time_table.append(next_time)
            try:
                min_time = min(time_table)
            except ValueError:
                try:
                    obj1 = self._objs[0]
                    obj2 = self._objs[1]
                    len_ = obj2.position - obj1.position
                except IndexError:
                    raise SimulationEndException
                if obj1.velocity <= obj2.velocity:
                    min_time = float('Inf')
                else:
                    min_time = len_ / (obj1.velocity - obj2.velocity)

            if min_time == float('Inf'):
                raise SimulationEndException

            for next_obj in self._objs:
                next_obj.position += next_obj.velocity * min_time

            self.process_collision()
            self._time += min_time
            if not self.has_collision():
                break

    def has_collision(self):
        for i, this_obj in enumerate(self._objs):
            if i == len(self._objs) - 1: break
            next_obj = self._objs[i + 1]
            if this_obj.position == next_obj.position:
                if next_obj.velocity - this_obj.velocity <= 0:
                    return True
        return False
        
    def process_collision(self):
        collision_group = []
        for next_obj in self._objs:
            if len(collision_group) == 0:
                collision_group.append(next_obj)
                if isinstance(next_obj, Wall):
                    if len(collision_group) != 1:
                        PhysEnv._collision_in_group(collision_group)
                    collision_group = [next_obj]
            else:
                if collision_group[0].position == next_obj.position:
                    collision_group.append(next_obj)
                else:
                    if len(collision_group) != 1:
                        PhysEnv._collision_in_group(collision_group)
                    collision_group = [next_obj]
        if len(collision_group) != 1:
            PhysEnv._collision_in_group(collision_group)


    def _collision_in_group(group):
        #group : objects in same position, seperated by wall.(W/ wall)
        def calc_next_vel(this_vel, other_vel, this_mass, other_mass,\
                          this_momentum, other_momentum, repulsion):
            vel_diff_before = this_vel - other_vel
            vel_diff_after = -repulsion * vel_diff_before
            momentum_sum = this_momentum + other_momentum
            other_vel = (momentum_sum\
                         - vel_diff_after*this_mass)\
                         /(this_mass + other_mass)
            return other_vel + vel_diff_after


        next_v_list = []
        for i, next_obj in enumerate(group):
            if isinstance(next_obj, Wall):
                #Would be first or last
                next_v_list.append(0)
            else:
                #Could be first or last, but could be not
                next_v = 0
                
                group_left = [x for j, x in enumerate(group) if j < i]
                group_right = [x for j, x in enumerate(group) if j > i]
                mass_tot = sum([x.mass for j, x in enumerate(group) if j != i\
                                and not isinstance(x, Wall)])

                wall_repulsion = None
                if not len(group_left) == 0:
                    repulsion = group_left[-1].restitution\
                                * next_obj.restitution
                    if not isinstance(group_left[-1], Wall):
                        left_mass = sum([x.mass for x in group_left\
                                         if not isinstance(x, Wall)])
                        left_vel = sum([x.velocity * x.mass for x in\
                                        group_left if not isinstance(x, Wall)])\
                                        /left_mass #velocity of center of mass
                        left_momentum = mass_tot * left_vel
                        next_v += calc_next_vel(next_obj.velocity, left_vel,\
                                                next_obj.mass, mass_tot,\
                                                next_obj.momentum,\
                                                left_momentum, repulsion)\
                                                *left_mass
                    else:
                        wall_repulsion = repulsion
                if not len(group_right) == 0:
                    repulsion = group_right[0].restitution\
                                * next_obj.restitution
                    if not isinstance(group_right[0], Wall):
                        right_mass = sum([x.mass for x in group_right\
                                          if not isinstance(x, Wall)])
                        right_vel = sum([x.velocity * x.mass for x in\
                                         group_right if not\
                                         isinstance(x, Wall)])/right_mass
                                         #velocity of center of mass
                        right_momentum = mass_tot * right_vel
                        next_v += calc_next_vel(next_obj.velocity, right_vel,\
                                                next_obj.mass, mass_tot,\
                                                next_obj.momentum,\
                                                right_momentum, repulsion)\
                                                *right_mass
                    else:
                        wall_repulsion = repulsion

                if wall_repulsion is not None:
                    if next_v == 0:
                        next_v = next_obj.velocity
                    next_v = -next_v * wall_repulsion
                    if mass_tot == 0:
                        mass_tot = 1
                next_v_list.append(next_v / mass_tot)
        for i, next_obj in enumerate(group):
            if isinstance(next_obj, Wall):
                continue
            next_obj.velocity = next_v_list[i]
                        

    def __str__(self):
        return "PhysEnv t=" + str(float(self._time)) + "\n"\
               + str(self._objs) + "\nnet_kinetic="\
               + str(float(self.net_kinetic))\
               + "\nnet_momentum=" + str(float(self.net_momentum))

    def __repr__(self):
        return self.__str__()


if __name__ == "__main__":
    env = PhysEnv()
    point1 = PointOfMass(coord=0, mass=5)
    point1.velocity = 0
    point2 = PointOfMass(coord=20, mass=1)
    point2.velocity = -10
    env.add_obj(point1)
    env.add_obj(point2)
    print(env)
    while True:
        try:
            env.until_next_collision()
            print(env)
        except SimulationEndException:
            break

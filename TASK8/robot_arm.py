# == IMPORTING == #
import vectormath as vmath

# == CLASS: ROBOT ARM == #

class RobotArm:
    def __init__(self, joints):
        # define variables for storing arm data
        self.joints = joints
        self.joint_positions = []
        self.link_lengths = []
        self.link_angles = []

        # take user input and calculate link lengths
        self._input_joint_positions()
        self._calculate_link_lengths()

        # calculate initial configuration of arm
        self.root = self.joint_positions[0]
        self.init_root_link_dirn = (self.joint_positions[1] - self.joint_positions[0]).normalize()
        self.max_reach = sum(self.link_lengths)    

    def _input_joint_positions(self):
        print()
        prompt = "Enter the position for joint {}: "

        for i in range(self.joints):
            coords = [ float(x) for x in (input(prompt.format(i+1)).split(",")) ]
            self.joint_positions.append(vmath.Vector3(coords))

    def output_joint_positions(self):
        print()
        prompt = "Position of joint {} is: {}, {}, {}"
    
        for i in range(self.joints):
            pos = self.joint_positions[i]
            print(prompt.format(i, pos.x, pos.y, pos.z))

    def _calculate_link_lengths(self):
        # calculate the lengths of links from their initial positions
        for i in range(self.joints - 1):
            link_vector = self.joint_positions[i+1] - self.joint_positions[i]
            self.link_lengths.append(link_vector.length)

    def _get_link_dirn(self, link):
        # get the direction vector of a link
        link_vector = self.joint_positions[link+1] - self.joint_positions[link]
        return link_vector.normalize()

    def _is_reachable(self, target):
        # check if the target is within reach
        distance = (target - self.root).length
        return (distance <= self.max_reach)

    def _within_constraint(self, link):
        if (link == 0):
            prev_link_dirn = self.init_root_link_dirn
        else:
            prev_link_dirn = (self.joint_positions[i] - self.joint_positions[i-1]).normalize()

        curr_link_dirn = (self.joint_positions[i+1] - self.joint_positions[i]).normalize()

        pdt_of_magnitude = (prev_link_dirn.length) * (curr_link_dirn.length)
        cos_of_angle = prev_link_dirn.dot(curr_link_dirn)/pdt_of_magnitude

        return (0 <= cos_of_angle)

    def _f_move_link(self, link, target):
        # forward move a link to a target
        dirn_vec = (target - self.joint_positions[link]).normalize()
        magnitude = self.link_lengths[link]
        self.joint_positions[link] = target - magnitude * (dirn_vec)

    def _b_move_link(self, link, target):
        # backward move a link to a target
        dirn_vec = (target - self.joint_positions[link+1]).normalize()
        magnitude = self.link_lengths[link]
        self.joint_positions[link+1] = target - magnitude * (dirn_vec)

    def _move_to_target(self, target):
        if not(self._is_reachable(target)):
            return False
        else:
            self.joint_positions[self.joints - 1] = target
            
            for i in range(self.joints-2, -1, -1):
                self._f_move_link(i, self.joint_positions[i+1])

            self.joint_positions[0] = self.root

            for i in range(0, self.joints-1, 1):
                self._b_move_link(i, self.joint_positions[i])
            
            return True

    def fabrik(self, target, iterations):
        for i in range(iterations):
            if not(self._move_to_target(target)):
                return False
        return True
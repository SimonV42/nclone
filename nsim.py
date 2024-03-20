import math
from itertools import product

class Ninja:
    """This class is responsible for updating and storing the positions and velocities of each ninja.
    self.xposlog and self.yposlog contain all the coordinates used to generate the traces of the replays.
    """

    #Physics constants for the ninja.
    GRAVITY_FALL = 0.06666666666666665 
    GRAVITY_JUMP = 0.01111111111111111 
    GROUND_ACCEL = 0.06666666666666665
    AIR_ACCEL = 0.04444444444444444
    DRAG = 0.9933221725495059 # 0.99^(2/3)
    FRICTION_GROUND = 0.9459290248857720 # 0.92^(2/3)
    FRICTION_GROUND_SLOW = 0.8617738760127536 # 0.80^(2/3)
    FRICTION_WALL = 0.9113380468927672 # 0.87^(2/3)
    MAX_HOR_SPEED = 3.333333333333333
    MAX_JUMP_DURATION = 45
    RADIUS = 10

    def __init__(self, sim):
        """Initiate ninja position at spawn point, and initiate other values to their initial state"""
        self.sim = sim
        self.xpos = sim.map_data[1231]*6
        self.ypos = sim.map_data[1232]*6
        self.xspeed = 0
        self.yspeed = 0
        self.applied_gravity = self.GRAVITY_FALL
        self.applied_friction = self.FRICTION_GROUND
        self.state = 0 #0:Immobile, 1:Running, 2:Ground sliding, 3:Jumping, 4:Falling, 5:Wall sliding
        self.jump_input_old = 0
        self.jump_duration = 0
        self.jump_buffer = -1
        self.floor_buffer = -1
        self.wall_buffer = -1
        self.launch_pad_buffer = -1
        self.poslog = [(0, self.xpos, self.ypos)] #Used for debug
        self.speedlog = [(0,0,0)]
        self.xposlog = [self.xpos] #Used to produce trace
        self.yposlog = [self.ypos]
        
    def integrate(self):
        """Update position and speed by applying drag and gravity before collision phase."""
        self.xspeed *= self.DRAG
        self.yspeed *= self.DRAG
        self.yspeed += self.applied_gravity
        self.xpos_old = self.xpos
        self.ypos_old = self.ypos
        self.xpos += self.xspeed
        self.ypos += self.yspeed

    def pre_collision(self):
        """Reset some values used for collision phase."""
        self.xspeed_old = self.xspeed
        self.yspeed_old = self.yspeed
        self.floor_count = 0
        self.wall_count = 0
        self.floor_normal_x = 0
        self.floor_normal_y = 0

    def collide_vs_objects(self):
        """Gather all entities in neighbourhood and apply physical collisions if possible."""
        entities = gather_entities_from_neighbourhood(self.sim, self.xpos, self.ypos)
        for entity in entities:
            if entity.is_physical_collidable:
                depen = entity.physical_collision()
                if depen:
                    depen_x, depen_y = depen[0]
                    depen_len = depen[1][0]
                    self.xpos += depen_x * depen_len
                    self.ypos += depen_y * depen_len
                    if entity.type in (17, 20, 28): #Depenetration for bounce blocks, thwumps and shwumps.
                        self.xspeed += depen_x * depen_len
                        self.yspeed += depen_y * depen_len
                    if entity.type == 11: #Depenetration for one ways
                        if depen_len:
                            xspeed_new = (self.xspeed*depen_y - self.yspeed*depen_x) * depen_y
                            yspeed_new = (self.xspeed*depen_y - self.yspeed*depen_x) * (-depen_x)
                            self.xspeed = xspeed_new
                            self.yspeed = yspeed_new
                    if depen_y < 0: 
                        self.floor_count += 1
                        self.floor_normal_x += depen_x
                        self.floor_normal_y += depen_y

    def collide_vs_tiles(self):
        """Gather all tile segments in neighbourhood and handle collisions with those."""
        #Interpolation routine mainly to prevent from going through walls.
        dx = self.xpos - self.xpos_old
        dy = self.ypos - self.ypos_old
        time = sweep_circle_vs_tiles(self.sim, self.xpos_old, self.ypos_old, dx, dy, self.RADIUS * 0.5) 
        self.xpos = self.xpos_old + time * dx
        self.ypos = self.ypos_old + time * dy

        #Find the closest point from the ninja, apply depenetration and update speed. Loop 32 times. 
        for _ in range(32):
            result, closest_point = get_single_closest_point(self.sim, self.xpos, self.ypos, self.RADIUS)
            if result == 0:
                break
            a, b = closest_point
            dx = self.xpos - a
            dy = self.ypos - b
            #This part tries to reproduce corner cases in some positions. Band-aid solution, and not in the game's code.
            if abs(dx) <= 0.0000001:
                dx = 0
                if self.xpos in (50.51197510492316, 49.23232124849253):
                    dx = -2**-47
                if self.xpos == 49.153536108584795:
                    dx = 2**-47
            dist = math.sqrt(dx**2 + dy**2)
            if dist == 0 or (self.RADIUS - dist*result < 0.0000001): 
                return
            self.xpos = a + result*self.RADIUS*dx/dist
            self.ypos = b + result*self.RADIUS*dy/dist
            dot_product = self.xspeed * dx + self.yspeed * dy
            if dot_product < 0: #Project velocity onto surface only if moving towards surface
                xspeed_new = (self.xspeed*dy - self.yspeed*dx) / dist**2 * dy
                yspeed_new = (self.xspeed*dy - self.yspeed*dx) / dist**2 * (-dx)
                self.xspeed = xspeed_new
                self.yspeed = yspeed_new
            if dy < -0.0001: #Adjust floor variables if ninja is standing on ground
                self.floor_count += 1
                self.floor_normal_x += dx/dist
                self.floor_normal_y += dy/dist
    
    def post_collision(self):
        """Perform logical collisions with entities, check for airborn state,
        check for walled state and calculate floor normals.
        """
        #Perform LOGICAL collisions between the ninja and nearby entities.
        #Also check if the ninja can interact with the walls of entities when applicable.
        wall_normal = 0
        entities = gather_entities_from_neighbourhood(self.sim, self.xpos, self.ypos)
        for entity in entities:
            if entity.is_logical_collidable:
                collision_result = entity.logical_collision()
                if collision_result:
                    if entity.type == 10: #If collision with launch pad, update speed and position.
                        xboost = collision_result[0] * 2/3
                        yboost = collision_result[1] * 2/3
                        self.xpos += xboost
                        self.ypos += yboost
                        self.xspeed = xboost
                        self.yspeed = yboost
                        self.floor_count = 0
                        self.floor_buffer = -1
                        boost_scalar = math.sqrt(xboost**2 + yboost**2)
                        self.xlp_boost_normalized = xboost/boost_scalar
                        self.ylp_boost_normalized = yboost/boost_scalar
                        self.launch_pad_buffer = 0
                        if self.state == 3:
                            self.applied_gravity = self.GRAVITY_FALL
                        self.state = 4
                    else: #If touched wall of bounce block, oneway, thwump or shwump, retrieve wall normal.
                        wall_normal += collision_result                  

        #Check if the ninja can interact with walls from nearby tile segments.
        rad = self.RADIUS + 0.1
        segments = gather_segments_from_region(self.sim, self.xpos-rad, self.ypos-rad,
                                                         self.xpos+rad, self.ypos+rad)
        for segment in segments:
            result = segment.get_closest_point(self.xpos, self.ypos)
            a, b = result[1], result[2]
            dx = self.xpos - a
            dy = self.ypos - b
            dist = math.sqrt(dx**2 + dy**2)
            if abs(dy) < 0.00001 and 0 < dist <= rad:
                wall_normal += dx/dist

        #Check if airborn or walled.
        self.airborn = True
        self.walled = False
        if wall_normal:
            self.walled = True
            self.wall_normal = wall_normal/abs(wall_normal)

        #Calculate the combined floor normalized normal vector if the ninja has touched any floor.
        if self.floor_count > 0:
            self.airborn = False
            floor_scalar = math.sqrt(self.floor_normal_x**2 + self.floor_normal_y**2)
            if floor_scalar == 0:
                self.floor_normalized_x = 0
                self.floor_normalized_y = -1
            else:
                self.floor_normalized_x = self.floor_normal_x/floor_scalar
                self.floor_normalized_y = self.floor_normal_y/floor_scalar

    def floor_jump(self):
        """Perform floor jump depending on slope angle and direction."""
        self.jump_buffer = -1
        self.floor_buffer = -1
        self.launch_pad_buffer = -1
        self.state = 3
        self.applied_gravity = self.GRAVITY_JUMP
        if self.floor_normalized_x == 0: #Jump from flat ground
            jx = 0
            jy = -2
        else: #Slope jump
            dx = self.floor_normalized_x
            dy = self.floor_normalized_y
            if self.xspeed * dx >= 0: #Moving downhill
                if self.xspeed * self.hor_input >= 0:
                    jx = 2/3 * dx
                    jy = 2 * dy
                else:
                    jx = 0
                    jy = -1.4
            else: #Moving uphill
                if self.xspeed * self.hor_input > 0: #Forward jump
                    jx = 0
                    jy = -1.4
                else:
                    self.xspeed = 0 #Perp jump
                    jx = 2/3 * dx
                    jy = 2 * dy
        if self.yspeed > 0:
            self.yspeed = 0
        self.xspeed += jx
        self.yspeed += jy
        self.xpos += jx
        self.ypos += jy
        self.jump_duration = 0

    def wall_jump(self):
        """Perform wall jump depending on wall normal and if sliding or not."""
        if self.hor_input * self.wall_normal < 0 and self.state == 5: #Slide wall jump
            jx = 2/3
            jy = -1
        else: #Regular wall jump
            jx = 1
            jy = -1.4
        self.state = 3
        self.applied_gravity = self.GRAVITY_JUMP
        if self.xspeed * self.wall_normal < 0:
            self.xspeed = 0
        if self.yspeed > 0:
            self.yspeed = 0
        self.xspeed += jx * self.wall_normal
        self.yspeed += jy
        self.xpos += jx * self.wall_normal
        self.ypos += jy
        self.jump_buffer = -1
        self.wall_buffer = -1
        self.launch_pad_buffer = -1
        self.jump_duration = 0

    def lp_jump(self):
        """Perform launch pad jump."""
        self.floor_buffer = -1
        self.wall_buffer = -1
        self.jump_buffer = -1
        self.launch_pad_buffer = -1
        boost_scalar = 2 * abs(self.xlp_boost_normalized) + 2
        if boost_scalar == 2:
            boost_scalar = 1.7 #This was really needed. Thanks Metanet
        self.xspeed += self.xlp_boost_normalized * boost_scalar * 2/3
        self.yspeed += self.ylp_boost_normalized * boost_scalar * 2/3
        
    def think(self):
        """This function handle all the ninja's actions depending of the inputs and its environment."""
        #Logic to determine if you're starting a new jump.
        if not self.jump_input:
            new_jump_check = False
        else:
            new_jump_check = self.jump_input_old == 0
        self.jump_input_old = self.jump_input

        #Determine if within buffer ranges. If so, increment buffers.
        if -1 < self.launch_pad_buffer < 3:
            self.launch_pad_buffer += 1
        else:
            self.launch_pad_buffer = -1
        in_lp_buffer = -1 < self.launch_pad_buffer < 4
        if -1 < self.jump_buffer < 5:
            self.jump_buffer += 1
        else:
            self.jump_buffer = -1
        in_jump_buffer = -1 < self.jump_buffer < 5
        if -1 < self.wall_buffer < 5:
            self.wall_buffer += 1
        else:
            self.wall_buffer = -1
        in_wall_buffer = -1 < self.wall_buffer < 5
        if -1 < self.floor_buffer < 5:
            self.floor_buffer += 1
        else:
            self.floor_buffer = -1
        in_floor_buffer = -1 < self.floor_buffer < 5

        #Initiate jump buffer if beginning a new jump and airborn.
        if new_jump_check and self.airborn:
            self.jump_buffer = 0
        #Initiate wall buffer if touched a wall this frame.
        if self.walled:
            self.wall_buffer = 0
        #Initiate floor buffer if touched a floor this frame.
        if not self.airborn:
            self.floor_buffer = 0

        #This block deals with the case where the ninja is touching a floor.
        if not self.airborn: 
            xspeed_new = self.xspeed + self.GROUND_ACCEL * self.hor_input
            if abs(xspeed_new) < self.MAX_HOR_SPEED:
                self.xspeed = xspeed_new
            if self.state > 2:
                if self.xspeed * self.hor_input <= 0:
                    if self.state == 3:
                        self.applied_gravity = self.GRAVITY_FALL
                    self.state = 2
                else:
                    if self.state == 3:
                        self.applied_gravity = self.GRAVITY_FALL
                    self.state = 1
            if not in_jump_buffer and not new_jump_check: #if not jumping
                if self.state == 2:
                    projection = abs(self.yspeed * self.floor_normalized_x
                                     - self.xspeed * self.floor_normalized_y)
                    if self.hor_input * projection * self.xspeed > 0:
                        self.state = 1
                        return
                    if projection < 0.1 and self.floor_normalized_x == 0:
                        self.state = 0
                        return
                    if self.yspeed < 0 and self.floor_normalized_x != 0:
                        #Up slope friction formula, very dumb but that's how it is
                        speed_scalar = math.sqrt(self.xspeed**2 + self.yspeed**2)
                        fric_force = abs(self.xspeed * (1-self.FRICTION_GROUND) * self.floor_normalized_y)
                        fric_force2 = speed_scalar - fric_force * self.floor_normalized_y**2
                        self.xspeed = self.xspeed / speed_scalar * fric_force2
                        self.yspeed = self.yspeed / speed_scalar * fric_force2 
                        return
                    self.xspeed *= self.FRICTION_GROUND
                    return
                if self.state == 1:
                    projection = abs(self.yspeed * self.floor_normalized_x
                                     - self.xspeed * self.floor_normalized_y)
                    if self.hor_input * projection * self.xspeed > 0:
                        if self.hor_input * self.floor_normalized_x >= 0: #if holding inputs in downhill direction or flat ground
                            return
                        if abs(xspeed_new) < self.MAX_HOR_SPEED:
                            boost = self.GROUND_ACCEL/2 * self.hor_input
                            xboost = boost * self.floor_normalized_y * self.floor_normalized_y
                            yboost = boost * self.floor_normalized_y * -self.floor_normalized_x
                            self.xspeed += xboost
                            self.yspeed += yboost
                        return
                    self.state = 2
                else: #if you were in state 0 I guess
                    if self.hor_input:
                        self.state = 1
                        return
                    projection = abs(self.yspeed * self.floor_normalized_x
                                     - self.xspeed * self.floor_normalized_y)
                    if projection < 0.1:
                        self.xspeed *= self.FRICTION_GROUND_SLOW
                        return
                    self.state = 2
                return
            self.floor_jump() #if you're jumping
            return

        #This block deals with the case where the ninja didn't touch a floor
        else:
            xspeed_new = self.xspeed + self.AIR_ACCEL * self.hor_input
            if abs(xspeed_new) < self.MAX_HOR_SPEED:
                self.xspeed = xspeed_new
            if self.state < 3:
                self.state = 4
                return
            if self.state == 3:
                self.jump_duration += 1
                if not self.jump_input or self.jump_duration > self.MAX_JUMP_DURATION:
                    self.applied_gravity = self.GRAVITY_FALL
                    self.state = 4
                    return
            if in_jump_buffer or new_jump_check: #if able to perfrom jump
                if self.walled or in_wall_buffer:
                    self.wall_jump()
                    return
                if in_floor_buffer:
                    self.floor_jump()
                    return
                if in_lp_buffer and new_jump_check:
                    self.lp_jump()
                    return
            if not self.walled:
                if self.state == 5:
                    self.state = 4
            else:
                if self.state == 5:
                    if self.hor_input * self.wall_normal <= 0:
                        self.yspeed *= self.FRICTION_WALL
                    else:
                        self.state = 4
                else:
                    if self.yspeed > 0 and self.hor_input * self.wall_normal < 0:
                        if self.state == 3:
                            self.applied_gravity = self.GRAVITY_FALL
                        self.state = 5


class GridSegmentLinear:
    """Contains all the linear segments of tiles and doors that the ninja can interract with"""
    def __init__(self, p1, p2, oriented=True):
        """Initiate an instance of a linear segment of a tile. 
        Each segment is defined by the coordinates of its two end points.
        Tile segments are oreinted which means they have an inner side and an outer side.
        Door segments are not oriented : Collision is the same regardless of the side.
        """
        self.x1, self.y1 = p1
        self.x2, self.y2 = p2
        self.oriented = oriented
        self.active = True
        self.type = "linear"

    def get_closest_point(self, xpos, ypos):
        """Find the closest point on the segment from the given position.
        is_back_facing is false if the position is facing the segment's outter edge.
        """
        px = self.x2 - self.x1
        py = self.y2 - self.y1
        dx = xpos - self.x1
        dy = ypos - self.y1
        seg_lensq = px**2 + py**2
        u = (dx*px + dy*py)/seg_lensq
        u = max(u, 0)
        u = min(u, 1)
        #If u is between 0 and 1, position is closest to the line segment.
        #If u is exactly 0 or 1, position is closest to one of the two edges.
        a = self.x1 + u*px
        b = self.y1 + u*py
        is_back_facing = dy*px - dx*py < 0 and self.oriented #Note: can't be backfacing if segment belongs to a door.
        return is_back_facing, a, b
    
    def intersect_with_ray(self, xpos, ypos, dx, dy, radius):
        """Return the time of intersection (as a fraction of a frame) for the
        closest point in the ninja's path. Return 0 if the ninja is already 
        intersecting or 1 if won't intersect within the frame.
        """
        time1 = get_time_of_intersection_circle_vs_circle(xpos, ypos, dx, dy, self.x1, self.y1, radius)
        time2 = get_time_of_intersection_circle_vs_circle(xpos, ypos, dx, dy, self.x2, self.y2, radius)
        time3 = get_time_of_intersection_circle_vs_lineseg(xpos, ypos, dx, dy, self.x1, self.y1,
                                                           self.x2, self.y2, radius)
        return min(time1, time2, time3)

    
class GridSegmentCircular:
    """Contains all the circular segments of tiles that the ninja can interract with"""
    def __init__(self, center, quadrant, convex, radius=24):
        """Initiate an instance of a circular segment of a tile. 
        Each segment is defined by the coordinates of its center, a vector indicating which
        quadrant contains the qurater-circle, a boolean indicating if the tile is convex or
        concave, and the radius of the quarter-circle."""
        self.xpos = center[0]
        self.ypos = center[1]
        self.hor = quadrant[0]
        self.ver = quadrant[1]
        self.radius = radius
        #The following two variables are the position of the two extremities of arc.
        self.p_hor = (self.xpos + self.radius*self.hor, self.ypos)
        self.p_ver = (self.xpos, self.ypos + self.radius*self.ver)
        self.active = True
        self.type = "circular"
        self.convex = convex

    def get_closest_point(self, xpos, ypos):
        """Find the closest point on the segment from the given position.
        is_back_facing is false if the position is facing the segment's outter edge.
        """
        dx = xpos - self.xpos
        dy = ypos - self.ypos
        is_back_facing = False
        if dx * self.hor > 0 and dy * self.ver > 0: #This is true if position is closer from arc than its edges.
            dist = math.sqrt(dx**2 + dy**2)
            a = self.xpos + self.radius*dx/dist
            b = self.ypos + self.radius*dy/dist
            is_back_facing = dist < self.radius if self.convex else dist > self.radius
        else: #If closer to edges of arc, find position of closest point of the two.
            if dx * self.hor > dy * self.ver:
                a, b = self.p_hor
            else:
                a, b = self.p_ver
        return is_back_facing, a, b
    
    def intersect_with_ray(self, xpos, ypos, dx, dy, radius):
        """Return the time of intersection (as a fraction of a frame) for the closest point in the ninja's path.
        Return 0 if the ninja is already intersection or 1 if won't intersect within the frame.
        """
        time1 = get_time_of_intersection_circle_vs_circle(xpos, ypos, dx, dy, self.p_hor[0], self.p_hor[1], radius)
        time2 = get_time_of_intersection_circle_vs_circle(xpos, ypos, dx, dy, self.p_ver[0], self.p_ver[1], radius)
        time3 = get_time_of_intersection_circle_vs_arc(xpos, ypos, dx, dy, self.xpos, self.ypos,
                                                       self.hor, self.ver, self.radius, radius)
        return min(time1, time2, time3)


class Entity:
    """Class that all entity types (gold, bounce blocks, thwumps, etc.) inherit from."""
    def __init__(self, type, sim, xcoord, ycoord):
        """Inititate a member from map data"""
        self.type = type
        self.sim = sim
        self.xpos = xcoord*6
        self.ypos = ycoord*6
        self.active = True
        self.is_logical_collidable = False
        self.is_physical_collidable = False
        self.is_movable = False
        self.is_thinkable = False
        self.cell = clamp_cell(math.floor(self.xpos / 24), math.floor(self.ypos / 24))        
    
    def grid_move(self):
        """As the entity is moving, if its center goes from one grid cell to another,
        remove it from the previous cell and insert it into the new cell.
        """
        cell_new = clamp_cell(math.floor(self.xpos / 24), math.floor(self.ypos / 24))
        if cell_new != self.cell:
            self.sim.entity_dic[self.cell].remove(self)
            self.cell = cell_new
            self.sim.entity_dic[self.cell].append(self)


class EntityGold(Entity):
    RADIUS = 6

    def __init__(self, type, sim, xcoord, ycoord):
        super().__init__(type, sim, xcoord, ycoord)
        self.is_logical_collidable = True
        self.collected = False
        
    def logical_collision(self):
        """If the ninja is colliding with the piece of gold, store the collection frame."""
        ninja = self.sim.ninja
        if overlap_circle_vs_circle(self.xpos, self.ypos, self.RADIUS,
                                    ninja.xpos, ninja.ypos, ninja.RADIUS):
            self.collected = self.sim.frame
            self.active = False


class EntityExit(Entity):
    RADIUS = 12

    def __init__(self, type, sim, xcoord, ycoord):
        super().__init__(type, sim, xcoord, ycoord)
        self.is_logical_collidable = True
        self.ninja_exit = []

    def logical_collision(self):
        """If the ninja is colliding with the open door, store the collision frame."""
        ninja = self.sim.ninja
        if overlap_circle_vs_circle(self.xpos, self.ypos, self.RADIUS,
                                    ninja.xpos, ninja.ypos, ninja.RADIUS):
            self.ninja_exit.append(self.sim.frame)
            self.active = False


class EntityExitSwitch(Entity):
    RADIUS = 6

    def __init__(self, type, sim, xcoord, ycoord, parent):
        super().__init__(type, sim, xcoord, ycoord)
        self.is_logical_collidable = True
        self.collected = False
        self.parent = parent

    def logical_collision(self):
        """If the ninja is colliding with the switch, flag it as being collected, and open its associated door."""
        ninja = self.sim.ninja
        if overlap_circle_vs_circle(self.xpos, self.ypos, self.RADIUS,
                                    ninja.xpos, ninja.ypos, ninja.RADIUS):
            self.collected = True
            self.active = False
            self.sim.entity_dic[self.parent.cell].append(self.parent) #Add door to the entity grid so the ninja can touch it


class EntityDoorBase(Entity):
    def __init__(self, type, sim, xcoord, ycoord, orientation, sw_xcoord, sw_ycoord):
        super().__init__(type, sim, xcoord, ycoord)
        self.is_logical_collidable = True
        self.closed = True
        self.sw_xpos = 6 * sw_xcoord
        self.sw_ypos = 6 * sw_ycoord
        self.is_vertical = orientation in (0, 4)
        vec = map_orientation_to_vector(orientation)
        #Find the cell that the door is in for the grid segment.
        door_xcell = math.floor((self.xpos - 12*vec[0]) / 24)
        door_ycell = math.floor((self.ypos - 12*vec[1]) / 24)
        door_cell = clamp_cell(door_xcell, door_ycell)
        #Find the half cell of the door for the grid edges.
        door_half_xcell = 2*(door_cell[0] + 1)
        door_half_ycell = 2*(door_cell[1] + 1)
        #Create the grid segment and grid edges.
        self.grid_edges = []
        if self.is_vertical:
            self.segment = GridSegmentLinear((self.xpos, self.ypos-12), (self.xpos, self.ypos+12),
                                             oriented=False)
            self.grid_edges.append((door_half_xcell, door_half_ycell-2))
            self.grid_edges.append((door_half_xcell, door_half_ycell-1))
            for grid_edge in self.grid_edges:
                sim.ver_grid_edge_dic[grid_edge] += 1
        else:
            self.segment = GridSegmentLinear((self.xpos-12, self.ypos), (self.xpos+12, self.ypos),
                                             oriented=False)
            self.grid_edges.append((door_half_xcell-2, door_half_ycell))
            self.grid_edges.append((door_half_xcell-1, door_half_ycell))
            for grid_edge in self.grid_edges:
                sim.hor_grid_edge_dic[grid_edge] += 1
        sim.segment_dic[door_cell].append(self.segment)
        #Update position and cell so it corresponds to the switch and not the door.
        self.xpos = self.sw_xpos
        self.ypos = self.sw_ypos
        self.cell = clamp_cell(math.floor(self.xpos / 24), math.floor(self.ypos / 24))

    def change_state(self, closed):
        """Change the state of the door from closed to open or from open to closed."""
        self.closed = closed
        self.segment.active = closed
        for grid_edge in self.grid_edges:
            if self.is_vertical:
                self.sim.ver_grid_edge_dic[grid_edge] += 1 if closed else -1
            else:
                self.sim.hor_grid_edge_dic[grid_edge] += 1 if closed else -1


class EntityDoorRegular(EntityDoorBase):
    RADIUS = 10

    def __init__(self, type, sim, xcoord, ycoord, orientation, sw_xcoord, sw_ycoord):
        super().__init__(type, sim, xcoord, ycoord, orientation, sw_xcoord, sw_ycoord)
        self.is_thinkable = True
        self.open_timer = 0

    def think(self):
        """If the door has been opened for more than 5 frames without being touched by the ninja, close it."""
        if not self.closed:
            self.open_timer += 1
            if self.open_timer > 5:
                self.change_state(closed = True)

    def logical_collision(self):
        """If the ninja touches the activation region of the door, open it."""
        ninja = self.sim.ninja
        if overlap_circle_vs_circle(self.xpos, self.ypos, self.RADIUS,
                                    ninja.xpos, ninja.ypos, ninja.RADIUS):
            self.change_state(closed = False)
            self.open_timer = 0


class EntityDoorLocked(EntityDoorBase):
    RADIUS = 5

    def __init__(self, type, sim, xcoord, ycoord, orientation, sw_xcoord, sw_ycoord):
        super().__init__(type, sim, xcoord, ycoord, orientation, sw_xcoord, sw_ycoord)

    def logical_collision(self):
        """If the ninja collects the associated open switch, open the door."""
        ninja = self.sim.ninja
        if overlap_circle_vs_circle(self.xpos, self.ypos, self.RADIUS,
                                    ninja.xpos, ninja.ypos, ninja.RADIUS):
            self.change_state(closed = False)
            self.active = False


class EntityDoorTrap(EntityDoorBase):
    RADIUS = 5

    def __init__(self, type, sim, xcoord, ycoord, orientation, sw_xcoord, sw_ycoord):
        super().__init__(type, sim, xcoord, ycoord, orientation, sw_xcoord, sw_ycoord)
        self.change_state(closed = False)

    def logical_collision(self):
        """If the ninja collects the associated close switch, close the door."""
        ninja = self.sim.ninja
        if overlap_circle_vs_circle(self.xpos, self.ypos, self.RADIUS,
                                    ninja.xpos, ninja.ypos, ninja.RADIUS):
            self.change_state(closed = True)
            self.active = False


class EntityLaunchPad(Entity):
    RADIUS = 6
    BOOST = 36/7

    def __init__(self, type, sim, xcoord, ycoord, orientation):
        super().__init__(type, sim, xcoord, ycoord)
        self.is_logical_collidable = True
        self.normal_x, self.normal_y = map_orientation_to_vector(orientation)

    def logical_collision(self):
        """If the ninja is colliding with the launch pad (semi circle hitbox), return boost."""
        ninja = self.sim.ninja
        if overlap_circle_vs_circle(self.xpos, self.ypos, self.RADIUS,
                                    ninja.xpos, ninja.ypos, ninja.RADIUS):
            if ((self.xpos - (ninja.xpos - ninja.RADIUS*self.normal_x))*self.normal_x
                + (self.ypos - (ninja.ypos - ninja.RADIUS*self.normal_y))*self.normal_y) >= -0.1:
                yboost_scale = 1
                if self.normal_y < 0:
                    yboost_scale = 1 - self.normal_y
                xboost = self.normal_x * self.BOOST
                yboost = self.normal_y * self.BOOST * yboost_scale
                return (xboost, yboost)


class EntityOneWayPlatform(Entity):
    SEMI_SIDE = 12

    def __init__(self, type, sim, xcoord, ycoord, orientation):
        super().__init__(type, sim, xcoord, ycoord)
        self.is_logical_collidable = True
        self.is_physical_collidable = True
        self.normal_x, self.normal_y = map_orientation_to_vector(orientation)

    def calculate_depenetration(self, ninja):
        dx = ninja.xpos - self.xpos
        dy = ninja.ypos - self.ypos
        lateral_dist = dy * self.normal_x - dx * self.normal_y
        direction = (ninja.yspeed * self.normal_x - ninja.xspeed * self.normal_y) * lateral_dist
        radius_scalar = 0.91 if direction < 0 else 0.51 #The platform has a bigger width if the ninja is moving towards its center.
        if abs(lateral_dist) < radius_scalar * ninja.RADIUS + self.SEMI_SIDE:
            normal_dist = dx * self.normal_x + dy * self.normal_y
            if 0 < normal_dist <= ninja.RADIUS:
                normal_proj = ninja.xspeed * self.normal_x + ninja.yspeed * self.normal_y
                if normal_proj <= 0:
                    dx_old = ninja.xpos_old - self.xpos
                    dy_old = ninja.ypos_old - self.ypos
                    normal_dist_old = dx_old * self.normal_x + dy_old * self.normal_y
                    if ninja.RADIUS - normal_dist_old <= 1.1:
                        return (self.normal_x, self.normal_y), (ninja.RADIUS - normal_dist, 0)

    def physical_collision(self):
        return self.calculate_depenetration(self.sim.ninja)

    def logical_collision(self):
        collision_result = self.calculate_depenetration(self.sim.ninja)
        if collision_result:
            if abs(self.normal_x) == 1:
                return self.normal_x


class EntityBounceBlock(Entity):
    SEMI_SIDE = 9
    STIFFNESS = 0.02222222222222222
    DAMPENING = 0.98
    STRENGTH = 0.2

    def __init__(self, type, sim, xcoord, ycoord):
        super().__init__(type, sim, xcoord, ycoord)
        self.is_physical_collidable = True
        self.is_logical_collidable = True
        self.is_movable = True
        self.xspeed, self.yspeed = 0, 0
        self.xorigin, self.yorigin = self.xpos, self.ypos
        
    def move(self):
        """Update the position and speed of the bounce block by applying the spring force and dampening."""
        self.xspeed *= self.DAMPENING
        self.yspeed *= self.DAMPENING
        self.xpos += self.xspeed
        self.ypos += self.yspeed
        xforce = self.STIFFNESS * (self.xorigin - self.xpos)
        yforce = self.STIFFNESS * (self.yorigin - self.ypos)
        self.xpos += xforce
        self.ypos += yforce
        self.xspeed += xforce
        self.yspeed += yforce
        self.grid_move()

    def physical_collision(self):
        """Apply 80% of the depenetration to the bounce block and 20% to the ninja."""
        ninja = self.sim.ninja
        depen = penetration_square_vs_point(self.xpos, self.ypos, ninja.xpos, ninja.ypos,
                                            self.SEMI_SIDE + ninja.RADIUS)
        if depen:
            depen_x, depen_y = depen[0]
            depen_len = depen[1][0]
            self.xpos -= depen_x * depen_len * (1-self.STRENGTH)
            self.ypos -= depen_y * depen_len * (1-self.STRENGTH)
            self.xspeed -= depen_x * depen_len * (1-self.STRENGTH)
            self.yspeed -= depen_y * depen_len * (1-self.STRENGTH)
            return (depen_x, depen_y), (depen_len * self.STRENGTH, depen[1][1])
        
    def logical_collision(self):
        """Check if the ninja can interact with the wall of the bounce block"""
        ninja = self.sim.ninja
        depen = penetration_square_vs_point(self.xpos, self.ypos, ninja.xpos, ninja.ypos,
                                            self.SEMI_SIDE + ninja.RADIUS + 0.1)
        if depen:
            return depen[0][0]
        

class EntityThwump(Entity):
    SEMI_SIDE = 9
    FORWARD_SPEED = 20/7
    BACKWARD_SPEED = 8/7

    def __init__(self, type, sim, xcoord, ycoord, orientation):
        super().__init__(type, sim, xcoord, ycoord)
        self.is_movable = True
        self.is_thinkable = True
        self.is_logical_collidable = True
        self.is_physical_collidable = True
        self.is_horizontal = orientation in (0, 4)
        self.direction = 1 if orientation in (0, 2) else -1
        self.xorigin, self.yorigin = self.xpos, self.ypos
        self.state = 0 #0:immobile, 1:forward, -1:backward

    def move(self):
        if self.state: #If not immobile.
            speed = self.FORWARD_SPEED if self.state == 1 else self.BACKWARD_SPEED
            speed_dir = self.direction * self.state
            if not self.is_horizontal:
                ypos_new = self.ypos + speed * speed_dir
                #If the thwump as retreated past its starting point, set its position to the origin.
                if self.state == -1 and (ypos_new - self.yorigin) * (self.ypos - self.yorigin) < 0:
                    self.ypos = self.yorigin
                    self.state = 0
                    return
                cell_y = math.floor((self.ypos + speed_dir * 11) / 12)
                cell_y_new = math.floor((ypos_new + speed_dir * 11) / 12)
                if cell_y != cell_y_new:
                    cell_x1 = math.floor((self.xpos - 11) / 12)
                    cell_x2 = math.floor((self.xpos + 11) / 12)
                    if not is_empty_row(self.sim, cell_x1, cell_x2, cell_y, speed_dir):
                        self.state = -1
                        return
                self.ypos = ypos_new
            else:
                xpos_new = self.xpos + speed * speed_dir
                #If the thwump as retreated past its starting point, set its position to the origin.
                if self.state == -1 and (xpos_new - self.xorigin) * (self.xpos - self.xorigin) < 0:
                    self.xpos = self.xorigin
                    self.state = 0
                    return
                cell_x = math.floor((self.xpos + speed_dir * 11) / 12)
                cell_x_new = math.floor((xpos_new + speed_dir * 11) / 12)
                if cell_x != cell_x_new:
                    cell_y1 = math.floor((self.ypos - 11) / 12)
                    cell_y2 = math.floor((self.ypos + 11) / 12)
                    if not is_empty_column(self.sim, cell_x, cell_y1, cell_y2, speed_dir):
                        self.state = -1
                        return
                self.xpos = xpos_new
            self.grid_move()

    def think(self):
        """Make the thwump charge if it has sight of the ninja."""
        if not self.state:
            ninja = self.sim.ninja
            activation_range = 2 * (self.SEMI_SIDE + ninja.RADIUS)
            if not self.is_horizontal:
                if abs(self.xpos - ninja.xpos) < activation_range: #If the ninja is in the activation range
                    ninja_ycell = math.floor(ninja.ypos / 12)
                    thwump_ycell = math.floor((self.ypos - self.direction * 11) / 12)
                    thwump_xcell1 = math.floor((self.xpos - 11) / 12)
                    thwump_xcell2 = math.floor((self.xpos + 11) / 12)
                    dy = ninja_ycell - thwump_ycell
                    if dy * self.direction >= 0:
                        for i in range(100):
                            if not is_empty_row(self.sim, thwump_xcell1, thwump_xcell2, thwump_ycell, self.direction):
                                dy = ninja_ycell - thwump_ycell
                                break
                            thwump_ycell += self.direction
                        if i > 0 and dy * self.direction <= 0:
                            self.state = 1
            else:
                if abs(self.ypos - ninja.ypos) < activation_range: #If the ninja is in the activation range
                    ninja_xcell = math.floor(ninja.xpos / 12)
                    thwump_xcell = math.floor((self.xpos - self.direction * 11) / 12)
                    thwump_ycell1 = math.floor((self.ypos - 11) / 12)
                    thwump_ycell2 = math.floor((self.ypos + 11) / 12)
                    dx = ninja_xcell - thwump_xcell
                    if dx * self.direction >= 0:
                        for i in range(100):
                            if not is_empty_column(self.sim, thwump_xcell, thwump_ycell1, thwump_ycell2, self.direction):
                                dx = ninja_xcell - thwump_xcell
                                break
                            thwump_xcell += self.direction
                        if i > 0 and dx * self.direction <= 0:
                            self.state = 1

    def physical_collision(self):
        ninja = self.sim.ninja
        return penetration_square_vs_point(self.xpos, self.ypos, ninja.xpos, ninja.ypos,
                                           self.SEMI_SIDE + ninja.RADIUS)
    
    def logical_collision(self):
        ninja = self.sim.ninja
        depen = penetration_square_vs_point(self.xpos, self.ypos, ninja.xpos, ninja.ypos,
                                          self.SEMI_SIDE + ninja.RADIUS + 0.1)
        if depen:
            return depen[0][0]


class EntityBoostPad(Entity):
    RADIUS = 6

    def __init__(self, type, sim, xcoord, ycoord):
        super().__init__(type, sim, xcoord, ycoord)
        self.is_movable = True
        self.is_touching_ninja = False

    def move(self):
        """If the ninja starts touching the booster, add 2 to its velocity norm."""
        ninja = self.sim.ninja
        if overlap_circle_vs_circle(self.xpos, self.ypos, self.RADIUS,
                                    ninja.xpos, ninja.ypos, ninja.RADIUS):
            if not self.is_touching_ninja:
                vel_norm = math.sqrt(ninja.xspeed**2 + ninja.yspeed**2)
                if vel_norm > 0:
                    x_boost = 2 * ninja.xspeed/vel_norm
                    y_boost = 2 * ninja.yspeed/vel_norm
                    ninja.xspeed += x_boost
                    ninja.yspeed += y_boost
                self.is_touching_ninja = True
        else:
            self.is_touching_ninja = False


class EntityShoveThwump(Entity):
    SEMI_SIDE = 12

    def __init__(self, type, sim, xcoord, ycoord):
        super().__init__(type, sim, xcoord, ycoord)
        self.is_thinkable = True
        self.is_logical_collidable = True
        self.is_physical_collidable = True
        self.xorigin, self.yorigin = self.xpos, self.ypos
        self.xdir, self.ydir = 0, 0
        self.state = 0 #0:immobile, 1:activated, 2:launching, 3:retreating
        self.activated = False

    def think(self):
        """Update the state of the shwump and move it if possible."""
        if self.state == 1: 
            if self.activated:
                self.activated = False
                return
            self.state = 2
        if self.state == 3:
            origin_dist = abs(self.xpos - self.xorigin) + abs(self.ypos - self.yorigin)
            if origin_dist >= 1:
                self.move_if_possible(self.xdir, self.ydir, 1)
            else:
                self.xpos = self.xorigin
                self.ypos = self.yorigin
                self.state = 0
        elif self.state == 2:
            self.move_if_possible(-self.xdir, -self.ydir, 4)

    def move_if_possible(self, xdir, ydir, speed):
        """Move the shwump depending of state and orientation.
        Not called in Simulator.tick like other entity move functions.
        """
        if self.ydir == 0:
            xpos_new = self.xpos + xdir * speed
            cell_x = math.floor(self.xpos / 12)
            cell_x_new = math.floor(xpos_new / 12)
            if cell_x != cell_x_new:
                cell_y1 = math.floor((self.ypos - 8) / 12)
                cell_y2 = math.floor((self.ypos + 8) / 12)
                if not is_empty_column(self.sim, cell_x, cell_y1, cell_y2, xdir):
                    self.state = 3
                    return
            self.xpos = xpos_new
        else:
            ypos_new = self.ypos + ydir * speed
            cell_y = math.floor(self.ypos / 12)
            cell_y_new = math.floor(ypos_new / 12)
            if cell_y != cell_y_new:
                cell_x1 = math.floor((self.xpos - 8) / 12)
                cell_x2 = math.floor((self.xpos + 8) / 12)
                if not is_empty_row(self.sim, cell_x1, cell_x2, cell_y, ydir):
                    self.state = 3
                    return
            self.ypos = ypos_new
        self.grid_move()
            
    def physical_collision(self):
        ninja = self.sim.ninja
        if self.state <= 1:
            depen = penetration_square_vs_point(self.xpos, self.ypos, ninja.xpos, ninja.ypos,
                                                self.SEMI_SIDE + ninja.RADIUS)
            if depen:
                depen_x, depen_y = depen[0]
                if self.state == 0 or self.xdir * depen_x + self.ydir * depen_y >= 0.01:
                    return depen

    def logical_collision(self):
        ninja = self.sim.ninja
        depen = penetration_square_vs_point(self.xpos, self.ypos, ninja.xpos, ninja.ypos,
                                            self.SEMI_SIDE + ninja.RADIUS + 0.1)
        if depen and self.state <= 1:
            depen_x, depen_y = depen[0]
            if self.state == 0:
                self.activated = True
                if depen[1][1] > 0.2:
                    self.xdir = depen_x
                    self.ydir = depen_y
                    self.state = 1
            elif self.state == 1:
                if self.xdir * depen_x + self.ydir * depen_y >= 0.01:
                    self.activated = True
                else:
                    return               
            return depen_x


class Simulator:
    """TODO"""

    #This is a dictionary mapping every tile id to the grid edges it contains.
    #The first 6 values represent horizontal half-tile edges, from left to right then top to bottom.
    #The last 6 values represent vertical half-tile edges, from top to bottom then left to right.
    #1 if there is a grid edge, 0 otherwise.
    TILE_GRID_EDGE_MAP = {0:[0,0,0,0,0,0,0,0,0,0,0,0], 1:[1,1,0,0,1,1,1,1,0,0,1,1], #0-1: Empty and full tiles
                          2:[1,1,1,1,0,0,1,0,0,0,1,0], 3:[0,1,0,0,0,1,0,0,1,1,1,1], #2-5: Half tiles
                          4:[0,0,1,1,1,1,0,1,0,0,0,1], 5:[1,0,0,0,1,0,1,1,1,1,0,0], 
                          6:[1,1,0,1,1,0,1,1,0,1,1,0], 7:[1,1,1,0,0,1,1,0,0,1,1,1], #6-9: 45 degree slopes
                          8:[0,1,1,0,1,1,0,1,1,0,1,1], 9:[1,0,0,1,1,1,1,1,1,0,0,1], 
                          10:[1,1,0,0,1,1,1,1,0,0,1,1], 11:[1,1,0,0,1,1,1,1,0,0,1,1], #10-13: Quarter moons
                          12:[1,1,0,0,1,1,1,1,0,0,1,1], 13:[1,1,0,0,1,1,1,1,0,0,1,1], 
                          14:[1,1,0,1,1,0,1,1,0,1,1,0], 15:[1,1,1,0,0,1,1,0,0,1,1,1], #14-17: Quarter pipes
                          16:[0,1,1,0,1,1,0,1,1,0,1,1], 17:[1,0,0,1,1,1,1,1,1,0,0,1], 
                          18:[1,1,1,1,0,0,1,0,0,0,1,0], 19:[1,1,1,1,0,0,1,0,0,0,1,0], #18-21: Short mild slopes
                          20:[0,0,1,1,1,1,0,1,0,0,0,1], 21:[0,0,1,1,1,1,0,1,0,0,0,1], 
                          22:[1,1,0,0,1,1,1,1,0,0,1,1], 23:[1,1,0,0,1,1,1,1,0,0,1,1], #22-25: Raised mild slopes
                          24:[1,1,0,0,1,1,1,1,0,0,1,1], 25:[1,1,0,0,1,1,1,1,0,0,1,1], 
                          26:[1,0,0,0,1,0,1,1,1,1,0,0], 27:[0,1,0,0,0,1,0,0,1,1,1,1], #26-29: Short steep slopes
                          28:[0,1,0,0,0,1,0,0,1,1,1,1], 29:[1,0,0,0,1,0,1,1,1,1,0,0], 
                          30:[1,1,0,0,1,1,1,1,0,0,1,1], 31:[1,1,0,0,1,1,1,1,0,0,1,1], #30-33: Raised steep slopes
                          32:[1,1,0,0,1,1,1,1,0,0,1,1], 33:[1,1,0,0,1,1,1,1,0,0,1,1], 
                          34:[1,1,0,0,0,0,0,0,0,0,0,0], 35:[0,0,0,0,0,0,0,0,0,0,1,1], #34-37: Glitched tiles
                          36:[0,0,0,0,1,1,0,0,0,0,0,0], 37:[0,0,0,0,0,0,1,1,0,0,0,0]} 

    #This is a dictionary mapping every tile id to the orthogonal linear segments it contains, 
    #same order as grid edges.
    #0 if no segment, -1 if normal facing left or up, 1 if normal right or down.                    
    TILE_SEGMENT_ORTHO_MAP = {0:[0,0,0,0,0,0,0,0,0,0,0,0], 1:[-1,-1,0,0,1,1,-1,-1,0,0,1,1], #0-1: Empty and full tiles
                              2:[-1,-1,1,1,0,0,-1,0,0,0,1,0], 3:[0,-1,0,0,0,1,0,0,-1,-1,1,1], #2-5: Half tiles
                              4:[0,0,-1,-1,1,1,0,-1,0,0,0,1], 5:[-1,0,0,0,1,0,-1,-1,1,1,0,0], 
                              6:[-1,-1,0,0,0,0,-1,-1,0,0,0,0], 7:[-1,-1,0,0,0,0,0,0,0,0,1,1], #6-9: 45 degree slopes
                              8:[0,0,0,0,1,1,0,0,0,0,1,1], 9:[0,0,0,0,1,1,-1,-1,0,0,0,0], 
                              10:[-1,-1,0,0,0,0,-1,-1,0,0,0,0], 11:[-1,-1,0,0,0,0,0,0,0,0,1,1], #10-13: Quarter moons
                              12:[0,0,0,0,1,1,0,0,0,0,1,1], 13:[0,0,0,0,1,1,-1,-1,0,0,0,0], 
                              14:[-1,-1,0,0,0,0,-1,-1,0,0,0,0], 15:[-1,-1,0,0,0,0,0,0,0,0,1,1], #14-17: Quarter pipes
                              16:[0,0,0,0,1,1,0,0,0,0,1,1], 17:[0,0,0,0,1,1,-1,-1,0,0,0,0], 
                              18:[-1,-1,0,0,0,0,-1,0,0,0,0,0], 19:[-1,-1,0,0,0,0,0,0,0,0,1,0], #18-21: Short mild slopes
                              20:[0,0,0,0,1,1,0,0,0,0,0,1], 21:[0,0,0,0,1,1,0,-1,0,0,0,0], 
                              22:[-1,-1,0,0,0,0,-1,-1,0,0,1,0], 23:[-1,-1,0,0,0,0,-1,0,0,0,1,1], #22-25: Raised mild slopes
                              24:[0,0,0,0,1,1,0,-1,0,0,1,1], 25:[0,0,0,0,1,1,-1,-1,0,0,0,1], 
                              26:[-1,0,0,0,0,0,-1,-1,0,0,0,0], 27:[0,-1,0,0,0,0,0,0,0,0,1,1], #26-29: Short steep slopes
                              28:[0,0,0,0,0,1,0,0,0,0,1,1], 29:[0,0,0,0,1,0,-1,-1,0,0,0,0], 
                              30:[-1,-1,0,0,1,0,-1,-1,0,0,0,0], 31:[-1,-1,0,0,0,1,0,0,0,0,1,1], #30-33: Raised steep slopes
                              32:[0,-1,0,0,1,1,0,0,0,0,1,1], 33:[-1,0,0,0,1,1,-1,-1,0,0,0,0], 
                              34:[-1,-1,0,0,0,0,0,0,0,0,0,0], 35:[0,0,0,0,0,0,0,0,0,0,1,1], #34-37: Glitched tiles
                              36:[0,0,0,0,1,1,0,0,0,0,0,0], 37:[0,0,0,0,0,0,-1,-1,0,0,0,0]} 

    #This is a dictionary mapping every tile id to the diagonal linear segment it contains.
    #Segments are defined by two sets of point that need to be added to the position inside the grid.
    TILE_SEGMENT_DIAG_MAP = {6:((0, 24), (24, 0)), 7:((0, 0), (24, 24)),
                             8:((24, 0), (0, 24)), 9:((24, 24), (0, 0)),
                             18:((0, 12), (24, 0)), 19:((0, 0), (24, 12)),
                             20:((24, 12), (0, 24)), 21:((24, 24), (0, 12)),
                             22:((0, 24), (24, 12)), 23:((0, 12), (24, 24)),
                             24:((24, 0), (0, 12)), 25:((24, 12), (0, 0)),
                             26:((0, 24), (12, 0)), 27:((12, 0), (24, 24)),
                             28:((24, 0), (12, 24)), 29:((12, 24), (0, 0)),
                             30:((12, 24), (24, 0)), 31:((0, 0), (12, 24)),
                             32:((12, 0), (0, 24)), 33:((24, 24), (12, 0))}
    
    #This is a dictionary mapping every tile id to the circular segment it contains.
    #Segments defined by their center point and the quadrant.
    TILE_SEGMENT_CIRCULAR_MAP = {10:((0, 0), (1, 1), True), 11:((24, 0), (-1, 1), True),
                                 12:((24, 24), (-1, -1), True), 13:((0, 24), (1, -1), True),
                                 14:((24, 24), (-1, -1), False), 15:((0, 24), (1, -1), False),
                                 16:((0, 0), (1, 1), False), 17:((24, 0), (-1, 1), False)}

    def __init__(self):
        self.frame = 1

        #initiate a dictionary mapping each tile id to its cell. Start by filling it with full tiles (id of 1).
        self.tile_dic = {}
        for x in range(44):
            for y in range(25):
                self.tile_dic[(x, y)] = 1
        
        #Initiate dictionaries and list containing interactable segments and entities
        self.segment_dic = {}
        for x in range(45):
            for y in range(26):
                self.segment_dic[(x, y)] = []
        self.entity_dic = {}
        for x in range(44):
            for y in range(25):
                self.entity_dic[(x, y)] = []
        self.entity_list = []

        #Initiate dictionaries of grid edges and segments. They are all set to zero initialy,
        #except for the edges of the frame, which are solid.
        self.hor_grid_edge_dic = {}
        for x in range(88):
            for y in range(51):
                value = 1 if y in (0, 50) else 0
                self.hor_grid_edge_dic[(x, y)] = value
        self.ver_grid_edge_dic = {}
        for x in range(89):
            for y in range(50):
                value = 1 if x in (0, 88) else 0
                self.ver_grid_edge_dic[(x, y)] = value
        self.hor_segment_dic = {}
        for x in range(88):
            for y in range(51):
                value = 0
                if y == 0:
                    value = 1
                if y == 50:
                    value = -1
                self.hor_segment_dic[(x, y)] = value
        self.ver_segment_dic = {}
        for x in range(89):
            for y in range(50):
                value = 0
                if x == 0:
                    value = 1
                if x == 88:
                    value = -1
                self.ver_segment_dic[(x, y)] = value
        
    def load(self, map_data):
        self.map_data = map_data
        #extract tile data from map data
        tile_data = self.map_data[184:1150]

        #map each tile to its cell
        for x in range(42):
            for y in range(23):
                self.tile_dic[(x+1, y+1)] = tile_data[x + y*42]

        #This loops makes the inventory of grid edges and orthogonal linear segments,
        #and initiates non-orthogonal linear segments and circular segments.
        for coord, tile_id in self.tile_dic.items():
            xcoord, ycoord = coord
            #Assign every grid edge and orthogonal linear segment to the dictionaries.
            if tile_id in self.TILE_GRID_EDGE_MAP.keys() and tile_id in self.TILE_SEGMENT_ORTHO_MAP.keys():
                grid_edge_list = self.TILE_GRID_EDGE_MAP[tile_id]
                segment_ortho_list = self.TILE_SEGMENT_ORTHO_MAP[tile_id]
                for y in range(3):
                    for x in range(2):
                        self.hor_grid_edge_dic[(2*xcoord + x, 2*ycoord + y)] = (
                            (self.hor_grid_edge_dic[(2*xcoord + x, 2*ycoord + y)] + grid_edge_list[2*y + x]) % 2)
                        self.hor_segment_dic[(2*xcoord + x, 2*ycoord + y)] += segment_ortho_list[2*y + x]
                for x in range(3):
                    for y in range(2):
                        self.ver_grid_edge_dic[(2*xcoord + x, 2*ycoord + y)] = (
                            (self.ver_grid_edge_dic[(2*xcoord + x, 2*ycoord + y)] + grid_edge_list[2*x + y + 6]) % 2)
                        self.ver_segment_dic[(2*xcoord + x, 2*ycoord + y)] += segment_ortho_list[2*x + y + 6]

            #Initiate non-orthogonal linear and circular segments.
            xtl = xcoord * 24
            ytl = ycoord * 24
            if tile_id in self.TILE_SEGMENT_DIAG_MAP.keys():
                ((x1, y1), (x2, y2)) = self.TILE_SEGMENT_DIAG_MAP[tile_id]
                self.segment_dic[coord].append(GridSegmentLinear((xtl+x1, ytl+y1), (xtl+x2, ytl+y2)))
            if tile_id in self.TILE_SEGMENT_CIRCULAR_MAP.keys():
                ((x, y), quadrant, convex) = self.TILE_SEGMENT_CIRCULAR_MAP[tile_id]
                self.segment_dic[coord].append(GridSegmentCircular((xtl+x, ytl+y), quadrant, convex))                

        #Initiate segments from the dictionaries of orthogonal linear segments.
        #Note that two segments of the same position but opposite orientation cancel each other,
        #and no segment is initiated.
        for coord, state in self.hor_segment_dic.items():
            if state:
                xcoord, ycoord = coord
                cell = (math.floor(xcoord/2), math.floor((ycoord - 0.1*state) / 2))
                point1 = (12*xcoord, 12*ycoord)
                point2 = (12*xcoord+12, 12*ycoord)
                if state == -1:
                    point1, point2 = point2, point1
                self.segment_dic[cell].append(GridSegmentLinear(point1, point2))
        for coord, state in self.ver_segment_dic.items():
            if state:
                xcoord, ycoord = coord
                cell = (math.floor((xcoord - 0.1*state) / 2), math.floor(ycoord/2))
                point1 = (12*xcoord, 12*ycoord+12)
                point2 = (12*xcoord, 12*ycoord)
                if state == -1:
                    point1, point2 = point2, point1
                self.segment_dic[cell].append(GridSegmentLinear(point1, point2))

        #initiate player 1 instance of Ninja at spawn coordinates
        self.ninja = Ninja(self)

        #Initiate each entity (other than ninjas)
        index = 1230
        exit_door_count = self.map_data[1156]
        while (index < len(map_data)):
            type = self.map_data[index]
            xcoord = self.map_data[index+1]
            ycoord = self.map_data[index+2]
            orientation = self.map_data[index+3]
            mode = self.map_data[index+4]
            if type == 2:
                entity = EntityGold(type, self, xcoord, ycoord)
            elif type == 3:
                parent = EntityExit(type, self, xcoord, ycoord)
                self.entity_list.append(parent)
                child_xcoord = self.map_data[index + 5*exit_door_count + 1]
                child_ycoord = self.map_data[index + 5*exit_door_count + 2]
                entity = EntityExitSwitch(4, self, child_xcoord, child_ycoord, parent)
            elif type == 5:
                entity = EntityDoorRegular(type, self, xcoord, ycoord, orientation, xcoord, ycoord)
            elif type == 6:
                switch_xcoord = self.map_data[index + 6]
                switch_ycoord = self.map_data[index + 7]
                entity = EntityDoorLocked(type, self, xcoord, ycoord, orientation, switch_xcoord, switch_ycoord)
            elif type == 8:
                switch_xcoord = self.map_data[index + 6]
                switch_ycoord = self.map_data[index + 7]
                entity = EntityDoorTrap(type, self, xcoord, ycoord, orientation, switch_xcoord, switch_ycoord)
            elif type == 10:
                entity = EntityLaunchPad(type, self, xcoord, ycoord, orientation)
            elif type == 11:
                entity = EntityOneWayPlatform(type, self, xcoord, ycoord, orientation)
            elif type == 17:
                entity = EntityBounceBlock(type, self, xcoord, ycoord)
            elif type == 20:
                entity = EntityThwump(type, self, xcoord, ycoord, orientation)
            elif type == 24:
                entity = EntityBoostPad(type, self, xcoord, ycoord)
            elif type == 28:
                entity = EntityShoveThwump(type, self, xcoord, ycoord)
            else:
                entity = None
            if entity:
                self.entity_list.append(entity)
                self.entity_dic[entity.cell].append(entity)
            index += 5

    def tick(self, hor_input, jump_input):
        #Store inputs as ninja variables
        self.ninja.hor_input = hor_input
        self.ninja.jump_input = jump_input

        #Move all movable entities
        for entity in self.entity_list: 
            if entity.is_movable and entity.active:
                entity.move()
        #Make all thinkable entities think
        for entity in self.entity_list:
            if entity.is_thinkable and entity.active:
                entity.think()
        
        self.ninja.integrate() #Do preliminary speed and position updates.
        self.ninja.pre_collision() #Do pre collision calculations.
        for _ in range(4):
            self.ninja.collide_vs_objects() #Handle PHYSICAL collisions with entities.
            self.ninja.collide_vs_tiles() #Handle physical collisions with tiles.
        self.ninja.post_collision() #Do post collision calculations.
        self.ninja.think() #Make ninja think

        #Update all the logs for debugging purposes and for tracing the route.
        self.ninja.poslog.append((self.frame, round(self.ninja.xpos, 6), round(self.ninja.ypos, 6)))
        self.ninja.speedlog.append((self.frame, round(self.ninja.xspeed, 6), round(self.ninja.yspeed, 6)))
        self.ninja.xposlog.append(self.ninja.xpos)
        self.ninja.yposlog.append(self.ninja.ypos)

        #Increment the current frame
        self.frame += 1


def gather_segments_from_region(sim, x1, y1, x2, y2):
    """Return a list containing all collidable segments from the cells in a
    rectangular region bounded by 2 points.
    """
    cx1, cy1 = clamp_cell(math.floor(x1/24), math.floor(y1/24))
    cx2, cy2 = clamp_cell(math.floor(x2/24), math.floor(y2/24))
    cells = product(range(cx1, cx2 + 1), range(cy1, cy2 + 1))
    segment_list = []
    for cell in cells:
        segment_list += [segment for segment in sim.segment_dic[cell] if segment.active]
    return segment_list

def gather_entities_from_neighbourhood(sim, xpos, ypos):
    """Return a list that contains all active entities from the nine neighbour cells."""
    cx, cy = clamp_cell(math.floor(xpos/24), math.floor(ypos/24))
    cells = product(range(max(cx - 1, 0), min(cx + 1, 43) + 1), 
                    range(max(cy - 1, 0), min(cy + 1, 24) + 1))
    entity_list = []
    for cell in cells:
        entity_list += [entity for entity in sim.entity_dic[cell] if entity.active]
    return entity_list
    
def sweep_circle_vs_tiles(sim, xpos_old, ypos_old, dx, dy, radius):
    """Fetch all segments from neighbourhood. Return shortest intersection time from interpolation."""
    xpos_new = xpos_old + dx
    ypos_new = ypos_old + dy
    width = radius + 1
    x1 = min(xpos_old, xpos_new) - width
    y1 = min(ypos_old, ypos_new) - width
    x2 = max(xpos_old, xpos_new) + width
    y2 = max(ypos_old, ypos_new) + width
    segments = gather_segments_from_region(sim, x1, y1, x2, y2)
    shortest_time = 1
    for segment in segments:
        time = segment.intersect_with_ray(xpos_old, ypos_old, dx, dy, radius)
        shortest_time = min(time, shortest_time)
    return shortest_time

def get_time_of_intersection_circle_vs_circle(xpos, ypos, vx, vy, a, b, radius):
    """Return time of intersection by interpolation by sweeping a circle onto an other circle, given a combined radius."""
    dx = xpos - a
    dy = ypos - b
    dist_sq = dx**2 + dy**2
    vel_sq = vx**2 + vy**2
    dot_prod = dx * vx + dy * vy
    if dist_sq - radius**2 > 0:
        radicand = dot_prod**2 - vel_sq * (dist_sq - radius**2)
        if vel_sq > 0.0001 and dot_prod < 0 and radicand >= 0:
            return (-dot_prod - math.sqrt(radicand)) / vel_sq
        return 1
    return 0

def get_time_of_intersection_circle_vs_lineseg(xpos, ypos, dx, dy, a1, b1, a2, b2, radius):
    """Return time of intersection by interpolation by sweeping a circle onto a line segment."""
    wx = a2 - a1
    wy = b2 - b1
    seg_len = math.sqrt(wx**2 + wy**2)
    nx = wx / seg_len
    ny = wy / seg_len
    normal_proj = (xpos - a1) * ny - (ypos - b1) * nx
    hor_proj = (xpos - a1) * nx + (ypos - b1) * ny
    if abs(normal_proj) >= radius:
        dir = dx * ny - dy * nx
        if dir * normal_proj < 0:
            t = min((abs(normal_proj) - radius) / abs(dir), 1)
            hor_proj2 = hor_proj + t * (dx * nx  + dy * ny)
            if 0 <= hor_proj2 <= seg_len:
                return t
    else:
        if 0 <= hor_proj <= seg_len:
            return 0
    return 1

def get_time_of_intersection_circle_vs_arc(xpos, ypos, vx, vy, a, b, hor, ver,
                                           radius_arc, radius_circle):
    """Return time of intersection by interpolation by sweeping a circle onto a circle arc.
    This algorithm assumes the radius of the circle is lesser than the radius of the arc.
    """
    dx = xpos - a
    dy = ypos - b
    dist_sq = dx**2 + dy**2
    vel_sq = vx**2 + vy**2
    dot_prod = dx * vx + dy * vy
    radius1 = radius_arc + radius_circle
    radius2 = radius_arc - radius_circle
    t = 1
    if dist_sq > radius1**2:
        radicand = dot_prod**2 - vel_sq * (dist_sq - radius1**2)
        if vel_sq > 0.0001 and dot_prod < 0 and radicand >= 0:
            t = (-dot_prod - math.sqrt(radicand)) / vel_sq
    elif dist_sq < radius2**2:
        radicand = dot_prod**2 - vel_sq * (dist_sq - radius2**2)
        if vel_sq > 0.0001:
            t = min((-dot_prod + math.sqrt(radicand)) / vel_sq, 1)
    else:
        t = 0
    if (dx + t*vx) * hor > 0 and (dy + t*vy) * ver > 0:
        return t
    return 1

def get_single_closest_point(sim, xpos, ypos, radius):
    """Find the closest point belonging to a collidable segment from the given position.
    Return result and position of the closest point. The result is 0 if no closest point
    found, 1 if belongs to outside edge, -1 if belongs from inside edge.
    """
    segments = gather_segments_from_region(sim, xpos-radius, ypos-radius, xpos+radius, ypos+radius)
    shortest_distance = 9999999
    result = 0
    closest_point = None
    for segment in segments:
        is_back_facing, a, b = segment.get_closest_point(xpos, ypos)
        distance_sq = (xpos - a)**2 + (ypos - b)**2
        if not is_back_facing: #This is to prioritize correct side collisions when multiple close segments.
            distance_sq -= 0.1
        if distance_sq < shortest_distance:
            shortest_distance = distance_sq
            closest_point = (a, b)
            result = -1 if is_back_facing else 1
    return result, closest_point

def overlap_circle_vs_circle(xpos1, ypos1, radius1, xpos2, ypos2, radius2):
    """Given two cirles definied by their center and radius, return true if they overlap."""
    dist = math.sqrt((xpos1 - xpos2)**2 + (ypos1 - ypos2)**2)
    return dist < radius1 + radius2

def penetration_square_vs_point(s_xpos, s_ypos, p_xpos, p_ypos, semi_side):
    """If a point is inside an orthogonal square, return the orientation of the shortest vector
    to depenetate the point out of the square, and return the penetrations on both axis.
    The square is defined by its center and semi side length. In the case of depenetrating the
    ninja out of square entity (bounce block, thwump, shwump), we consider a square of with a
    semi side equal to the semi side of the entity plus the radius of the ninja.
    """
    dx = p_xpos - s_xpos
    dy = p_ypos - s_ypos
    penx = semi_side - abs(dx)
    peny = semi_side - abs(dy)
    if  penx > 0 and peny > 0:
        if peny <= penx:
            depen_normal = (0, -1) if dy < 0 else (0, 1)
            depen_values = (peny, penx)
        else:
            depen_normal = (-1, 0) if dx < 0 else (1, 0)
            depen_values = (penx, peny)
        return depen_normal, depen_values

def map_orientation_to_vector(orientation):
    """Return a normalized vector pointing in the direction of the orientation.
    Orientation is a value between 0 and 7 taken from map data.
    """
    diag = math.sqrt(2) / 2
    orientation_dic = {0:(1, 0), 1:(diag, diag), 2:(0, 1), 3:(-diag, diag), 
                       4:(-1, 0), 5:(-diag, -diag), 6:(0, -1), 7:(diag, -diag)}
    return orientation_dic[orientation]

def clamp_cell(xcell, ycell):
    """If necessary, adjust coordinates of cell so it is in bounds."""
    xcell = max(xcell, 0)
    xcell = min(xcell, 43)
    ycell = max(ycell, 0)
    ycell = min(ycell, 24)
    return (xcell, ycell)

def clamp_half_cell(xcell, ycell):
    """If necessary, adjust coordinates of half cell so it is in bounds."""
    xcell = max(xcell, 0)
    xcell = min(xcell, 88)
    ycell = max(ycell, 0)
    ycell = min(ycell, 50)
    return (xcell, ycell)

def is_empty_row(sim, xcoord1, xcoord2, ycoord, dir):
    """Return true if the cell has no solid horizontal edge in the specified direction."""
    xcoords = range(xcoord1, xcoord2+1)
    if dir == 1:
        return not any(sim.hor_grid_edge_dic[clamp_half_cell(xcoord, ycoord+1)] for xcoord in xcoords)
    if dir == -1:
        return not any(sim.hor_grid_edge_dic[clamp_half_cell(xcoord, ycoord)] for xcoord in xcoords)
    
def is_empty_column(sim, xcoord, ycoord1, ycoord2, dir):
    """Return true if the cell has no solid vertical edge in the specified direction."""
    ycoords = range(ycoord1, ycoord2+1)
    if dir == 1:
        return not any(sim.ver_grid_edge_dic[clamp_half_cell(xcoord+1, ycoord)] for ycoord in ycoords)
    if dir == -1:
        return not any(sim.ver_grid_edge_dic[clamp_half_cell(xcoord, ycoord)] for ycoord in ycoords)
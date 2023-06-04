import numpy as np
import copy

from .abstract_dynamic_system import AbstractDynamicSystem
from quaternion import Quaternion

# Animations parameters
delta = 0.01
g = -9.81

class SphereDynamicSystem(AbstractDynamicSystem):

    def __init__(self, sphere, ob_sphere, plane):
        super().__init__()

        # Thrown ball parameters
        self.ball = Ball(sphere, plane, m=1, inertia_coeff=2/5)
        self.ball.speed = np.array([0, 0, 0], np.float64)
        self.ball.rot_q = Quaternion(matrix=getRotMatrix([0, 0, 0]))
        self.ball.rspeed = np.array([0, 0, 0], np.float64)

        # Other ball parameters
        self.ob_ball = Ball(ob_sphere, plane, m=1, inertia_coeff=2/5)

        # Inelastic collision parameters
        self.contact_threshold = 1.5 # Threshold for Sphere/Ground contact
        self.speed_threshold = 0.1 # Threshold to stop Sphere
        self.rolling_resistance = 0.3 # Rolling resistance coefficient

        # Elastic ground collision parameters
        self.tangent_cor = 0.8 # Tangent coefficient of restitution (0 to 1)
        self.normal_cor = -0.2 # Normal coefficient of restitution (-1 to 1)

        # Ball collision parameters
        self.ball_cor = 0.8
        self.spheres_colliding = False

        # Find initial speed
        findVa(self.ball, self.ob_ball)
        
    def step(self):
        # Speed update
        self.ball.speed += delta * self.ball.forces
        self.ob_ball.speed += delta * self.ob_ball.forces
        
        # Ground intersection
        [self.ball.intersecting, self.ball.inelastic] = self.reactToGround(self.ball)
        [self.ob_ball.intersecting, self.ob_ball.inelastic] = self.reactToGround(self.ob_ball)

        # Other ball intersection
        intersect = intersectSphereSphere(self.ball.sphere, self.ob_ball.sphere)
        if intersect is not None and not self.spheres_colliding:
            self.collideBalls(self.ball, self.ob_ball, intersect)
            self.spheres_colliding = True
        if intersect is None:
            self.spheres_colliding = False

        # Sphere rolling on the ground
        self.doSphereRoll(self.ball)
        self.doSphereRoll(self.ob_ball)

        # Position and Rotation update
        self.ball.updatePosAndRot()
        self.ob_ball.updatePosAndRot()

    def stopSphere(self, ball):
        ball.forces[0:3] = [0., 0., 0.]
        ball.speed[0:3] = [0., 0., 0.]

    def doElasticCollision(self, ball, normalspeed, tangentspeed):
        final_normalspeed = self.normal_cor * np.linalg.norm(normalspeed)
        if np.linalg.norm(tangentspeed) != 0:
            final_tangentspeed = np.linalg.norm(tangentspeed) * ((1 - ball.inertia_coeff * self.tangent_cor) / (1. + ball.inertia_coeff))\
                + (ball.inertia_coeff * (1 + self.tangent_cor)) / (1 + ball.inertia_coeff)\
                * (ball.sphere.radius * ball.rspeed) / np.linalg.norm(tangentspeed)
        else:
            final_tangentspeed = 0
        ball.speed[0:3] = [0., 0., 0.]
        if np.linalg.norm(normalspeed) != 0:
            ball.speed += final_normalspeed * normalspeed / np.linalg.norm(normalspeed)
        if np.linalg.norm(tangentspeed) != 0:
            ball.speed += final_tangentspeed * tangentspeed / np.linalg.norm(tangentspeed)
        rspeed_norm = (np.linalg.norm(ball.rspeed) * (ball.inertia_coeff - self.tangent_cor) / (1 + ball.inertia_coeff)\
                + ((1 + self.tangent_cor) / (1 + ball.inertia_coeff)) * (np.linalg.norm(tangentspeed) / ball.sphere.radius))
        if np.linalg.norm(ball.rspeed) != 0:  
            ball.rspeed[0:3] = rspeed_norm * ball.rspeed / np.linalg.norm(ball.rspeed)
        else:
            [a, b, c, _] = getPlaneEq(ball.plane) # Le truc d'ahmed là
            normal = np.array([a, b, c], np.float64)
            normal /= np.linalg.norm(normal)
            axis = np.cross(normal, ball.speed)
            axis /= np.linalg.norm(axis)
            ball.rspeed[0:3] = rspeed_norm * axis

    def doInelasticCollision(self, ball, intersect):
        tmp_react = reactRigide(intersect[1], ball.forces)
        if np.linalg.norm(tmp_react) != 0:
            ball.inelastic_react[0:3] = tmp_react
            ball.forces -= ball.inelastic_react
        ball.speed[0:3] = projectSpeedOnPlane(ball.speed, ball.plane)
        if np.linalg.norm(ball.speed) != 0:
            ball.forces -= self.rolling_resistance * np.linalg.norm(ball.inelastic_react) * ball.speed / np.linalg.norm(ball.speed)
        ball.rspeed[0:3] = [0., 0., 0.]

    def doSphereRoll(self, ball):
        if ball.inelastic:
            if np.linalg.norm(ball.speed) < self.speed_threshold and np.linalg.norm(ball.speed) != 0:
                self.stopSphere(ball)
            if np.linalg.norm(ball.speed) >= self.speed_threshold:
                ball.rotateSphere()

    def reactToGround(self, ball):
        intersect = intersectPlaneSphere(ball.plane, ball.sphere, ball.sphere.radius)
        new_inelastic = False
        new_intersecting = False
        new_notintersecting = False
        if intersect is not None and not ball.inelastic:
            tangentspeed = projectSpeedOnPlane(ball.speed, ball.plane)
            normalspeed = ball.speed - tangentspeed
            if np.linalg.norm(normalspeed) > self.contact_threshold:
                self.doElasticCollision(ball, normalspeed, tangentspeed)
            else:
                self.doInelasticCollision(ball, intersect)
                new_inelastic = True
            new_intersecting = True
        if intersect is None and ball.intersecting:
            new_notintersecting = True
            ball.forces[0:3] = [0., ball.m * g, 0]
        return [(new_intersecting or ball.intersecting) and not new_notintersecting, (new_inelastic or ball.inelastic) and not new_notintersecting]

    def collideBalls(self, ball, ob_ball, intersect):
        # Before collision time
        collision_pt = intersect[0]
        normal = intersect[1]
        rotmatrix = copy.deepcopy(ball.sphere.rotation)
        ob_rotmatrix = copy.deepcopy(ob_ball.sphere.rotation)
        inertia = ball.inertia_coeff * ball.m * ball.sphere.radius**2
        ob_inertia = ob_ball.inertia_coeff * ob_ball.m * ob_ball.sphere.radius**2
        J_mat = np.array([[inertia, 0., 0.], [0., inertia, 0.], [0., 0., inertia]], np.float64)
        J_inv = np.linalg.inv(J_mat)
        ob_J_mat = np.array([[ob_inertia, 0., 0.], [0., ob_inertia, 0.], [0., 0., ob_inertia]], np.float64)
        ob_J_inv = np.linalg.inv(ob_J_mat)
        I_mat = np.matmul(np.matmul(rotmatrix, J_mat), np.matrix.transpose(rotmatrix))
        I_inv = np.matmul(np.matmul(rotmatrix, J_inv), np.matrix.transpose(rotmatrix))
        ob_I_mat = np.matmul(np.matmul(ob_rotmatrix, ob_J_mat), np.matrix.transpose(ob_rotmatrix))
        ob_I_inv = np.matmul(np.matmul(ob_rotmatrix, ob_J_inv), np.matrix.transpose(ob_rotmatrix))
        p = ball.m * ball.speed
        ob_p = ob_ball.m * ob_ball.speed
        b = np.matmul(I_mat, ball.rspeed)
        ob_b = np.matmul(ob_I_mat, ob_ball.rspeed)

        # After collision time
        ra_pt = ball.speed + np.cross(ball.rspeed, collision_pt - ball.pos)
        rb_pt = ob_ball.speed + np.cross(ob_ball.rspeed, collision_pt - ob_ball.pos)
        num = (-1 - self.ball_cor) * np.dot(normal, ra_pt - rb_pt)
        denom = 1/ball.m + 1/ob_ball.m + np.dot(normal, np.cross(np.matmul(I_inv, np.cross(collision_pt - ball.pos, normal)), collision_pt - ball.pos))\
            + np.dot(normal, np.cross(np.matmul(ob_I_inv, np.cross(collision_pt - ob_ball.pos, normal)), collision_pt - ob_ball.pos))
        impulse_vec = (num / denom) * normal
        new_p = impulse_vec + p
        ob_new_p = - impulse_vec + ob_p
        new_b = np.cross(collision_pt - ball.pos, impulse_vec) + b
        ob_new_b = np.cross(collision_pt - ob_ball.pos, - impulse_vec) + ob_b
        ob_ball.speed = ob_new_p / ob_ball.m
        ball.rspeed = np.matmul(I_inv, new_b)
        ball.speed = new_p / ball.m
        ob_ball.rspeed = np.matmul(ob_I_inv, ob_new_b)
        ob_ball.speed[1] *= -self.ball_cor
        ob_ball.intersecting = False
        ob_ball.inelastic = False   # Maraiste PEUT DE NOUVEAU changer de type (c'est fou)
        ob_ball.forces = np.array([0, ob_ball.m * g, 0], np.float64)

class Ball:

    def __init__(self, sphere, plane, m=1., inertia_coeff=2/5):
        self.sphere = sphere
        self.plane = plane

        self.m = m
        self.pos = copy.deepcopy(self.sphere.center)
        self.init_pos = copy.deepcopy(self.pos)
        self.speed = np.array([0, 0, 0], np.float64)
        self.rspeed = np.array([0, 0, 0], np.float64)
        self.forces = np.array([0, self.m * g, 0], np.float64)
        self.inelastic = False
        self.intersecting = False
        self.inertia_coeff = inertia_coeff
        self.inelastic_react = np.array([0, 0, 0], np.float64)
        self.rot_q = Quaternion(matrix=getRotMatrix([0, 0, 0]))

    def updatePosAndRot(self):
        self.pos += delta * self.speed
        self.rot_q *= eulerToQuaternion(delta * self.rspeed)
        self.sphere.center = self.pos
        self.sphere.rotation = self.rot_q.rotation_matrix

    def rotateSphere(self):
        [a, b, c, _] = getPlaneEq(self.plane)
        normal = np.array([a, b, c], np.float64)
        normal /= np.linalg.norm(normal)
        axis = np.cross(normal, self.speed)
        axis /= np.linalg.norm(axis)
        angle_speed = np.linalg.norm(self.speed) / self.sphere.radius
        self.rot_q *= Quaternion(axis=axis, radians=angle_speed * delta)

#####################
# UTILITY FUNCTIONS #
#####################

def getPlaneEq(plane):
    p1 = plane.vertices[0]
    p2 = plane.vertices[1]
    p3 = plane.vertices[-2]
    a1 = p2[0] - p1[0]
    b1 = p2[1] - p1[1]
    c1 = p2[2] - p1[2]
    a2 = p3[0] - p1[0]
    b2 = p3[1] - p1[1]
    c2 = p3[2] - p1[2]
    a = b1 * c2 - b2 * c1
    b = a2 * c1 - a1 * c2
    c = a1 * b2 - b1 * a2
    d = a * p1[0] + b * p1[1] + c * p1[2]
    return [a, b, c, d]

def getRotMatrix(rot):
    rxmat = np.array([[1., 0., 0.], [0., np.cos(rot[0]), -np.sin(rot[0])], [0., np.sin(rot[0]), np.cos(rot[0])]])
    rymat = np.array([[np.cos(rot[1]), 0., np.sin(rot[1])], [0., 1., 0.], [-np.sin(rot[1]), 0., np.cos(rot[1])]])
    rzmat = np.array([[np.cos(rot[2]), -np.sin(rot[2]), 0.], [np.sin(rot[2]), np.cos(rot[2]), 0.], [0., 0., 1.]])
    return np.matmul(np.matmul(rzmat, rymat), rxmat)

def projectSpeedOnPlane(speed, plane):
    [a, b, c, _] = getPlaneEq(plane)
    normal = np.array([a, b, c], np.float64)
    proj = speed - (np.dot(speed, normal) / np.dot(normal, normal)) * normal
    return proj

def reactRigide(dist_vec, forces):
    direction = dist_vec / np.linalg.norm(dist_vec)
    if np.linalg.norm(forces) != 0:
        cos_angle = np.dot(direction, forces) / np.linalg.norm(forces)
    else:
        cos_angle = 1
    norm = np.linalg.norm(forces) * cos_angle
    react = norm * direction
    return react

def reactElast(elasticity, dist_vec, speed):
    n = dist_vec / np.linalg.norm(dist_vec)
    direction = 2 * n * np.dot(n, -speed) + speed
    react = elasticity * direction
    return react

def intersectPlaneSphere(plane, sphere, radius):
    [a, b, c, d] = getPlaneEq(plane)
    x0 = sphere.center[0]
    y0 = sphere.center[1]
    z0 = sphere.center[2]
    dist = (a * x0 + b * y0 + c * z0 - d) / np.sqrt(a*a + b*b + c*c)
    if abs(dist) <= radius:
        dist_vec = dist * np.array([a, b, c], np.float64) / np.sqrt(a*a + b*b + c*c)
        proj = sphere.center + dist_vec
        return [proj, dist_vec]
    return None
        
def intersectSphereSphere(sphere1, sphere2):
    dist_vec = sphere1.center - sphere2.center
    if np.linalg.norm(dist_vec) < sphere1.radius + sphere2.radius:
        proj = sphere2.center + sphere2.radius * dist_vec / np.linalg.norm(dist_vec)
        return [proj, dist_vec/np.linalg.norm(dist_vec)]
    return None

def eulerToQuaternion(vec):
    [x, y, z] = vec
    cr = np.cos(x * 0.5)
    sr = np.sin(x * 0.5)
    cp = np.cos(y * 0.5)
    sp = np.sin(y * 0.5)
    cy = np.cos(z * 0.5)
    sy = np.sin(z * 0.5)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return Quaternion(w, x, y, z)

def findVa(ball, ob_ball):
    contact_angle = 5*np.pi/12
    contact_speed = 5.1
    rotation_speed = 5.

    [a, b, c, _] = getPlaneEq(ball.plane)
    p1 = ball.sphere.center
    p2 = ob_ball.sphere.center
    p3 = ob_ball.sphere.center + [a, b, c]/np.linalg.norm([a, b, c])
    r = ob_ball.sphere.radius
    h = findCollisionPoint(p1, p2, p3, r, contact_angle)
    ca = -(ball.sphere.radius/r)*(p2-h) + h

    dist = p2 - p1
    dist[1] = 0
    cos_angle = np.dot([1, 0, 0], dist) / np.linalg.norm(dist)
    angle = np.arccos(cos_angle)
    v_dir = np.array([2.3 * cos_angle, -8, -2.3 * np.sin(angle)], np.float64)
    v_dir /= np.linalg.norm(v_dir)
    va = contact_speed * v_dir

    r_dir = np.array([-np.sin(angle), 0, -cos_angle])
    ball.rspeed = r_dir * rotation_speed

    ball.sphere.center[1] = ca[1]
    ball.speed = va
    ball.speed[1] *= -1

    # v_dir = np.array([0, -3, -1], np.float64)
    # v_dir /= np.linalg.norm(v_dir)
    # nb = (h - p2)/np.linalg.norm(h - p2)
    # ra = h - ca
    # rb = h - p2
    # inertia = ball.inertia_coeff * ball.m * ball.sphere.radius**2
    # J_inv = np.linalg.inv(inertia * np.eye(3))
    # ob_inertia = ob_ball.inertia_coeff * ob_ball.m * r**2
    # ob_J_inv = np.linalg.inv(ob_inertia * np.eye(3))
    # num = (-1-ball_cor) * (np.dot(nb, v_dir))
    # x = 1/ball.m + 1/ob_ball.m + np.dot(nb, np.cross(np.matmul(J_inv, np.cross(ra, nb)), ra)) + np.dot(nb, np.cross(np.matmul(ob_J_inv, np.cross(rb, nb)), rb))
    # j = num / x
    # '''denom = -ball.m * x + (1 + ball_cor) * np.dot(nb, v_dir)
    # va = -(num / denom) * v_dir'''
    # va = -(j / ball.m) * nb
    # print(va)
    # tz = (ca[2]/va[2])
    # ty = (va[1] + np.sqrt(va[1]**2 + 2 * g * ca[1])) / -g
    # #print(t, (ca[2]/va[2]))
    # #v0 = va - tz*ball.forces
    # euler = -tz * ball.rspeed
    # ball.rot_q = eulerToQuaternion(euler)
    # ball.speed = -va
    # #print(v0)

def findCollisionPoint(p1, p2, p3, r, contact_angle):
    u = p1 - p2
    u[1] = 0
    u /= np.linalg.norm(u)

    a1 = p2[0] - p1[0]
    b1 = p2[1] - p1[1]
    c1 = p2[2] - p1[2]
    a2 = p3[0] - p1[0]
    b2 = p3[1] - p1[1]
    c2 = p3[2] - p1[2]
    a = b1 * c2 - b2 * c1
    b = a2 * c1 - a1 * c2
    c = a1 * b2 - b1 * a2
    d = a * p1[0] + b * p1[1] + c * p1[2]

    y = p2[1] + r * np.sin(contact_angle)
    z_num = ((b * y + d) / a + p2[0]) * u[0] - (y - p2[1]) * u[1] + p2[2] * u[2] + r * np.cos(contact_angle)
    z_denom = u[2] - (u[0] * c) / a
    z = z_num / z_denom
    x = (-b * y - c * z - d) / a
    
    return [x, y, z]

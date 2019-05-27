import gym
from gym import spaces
from gym.utils import seeding
import numpy as np
import math as math
from os import path
import matplotlib.pyplot as plt
import random

class UnicycleSYSEnv_v2(gym.Env):
    metadata = {
        'render.modes' : ['human', 'rgb_array'],
        'video.frames_per_second' : 30
    }

    """ 
    -----------------Define on state space form-----------------------
    This derivation was done in the matalab file Model_Calculations_V3.m
    states = [dalpha_w; dalpha_d; dphi; dtheta; phi; theta]
    u = [u_w, u_d]
    theta = pitch
    phi = roll
    ------------------------------------------------------------------

    """

    def __init__(self):
        self.max_speed_disk = 45 
        self.max_speed_wheel = 8.4
        self.max_volt = 12
        self.dt = .01
        self.max_theta = 25 * math.pi/180 
        self.max_phi = 9 * math.pi/180
        self.viewer_pitch = None
        self.viewer_roll = None
        self.t = 0
        self.last_u = None
        self.weight_input = 0
        self.weight_angvel_w = 0
        self.weight_angvel_d = 0
        self.u = 0
        self.count = 0
        self.c2r_wheel = 2*math.pi/(32*26.9)
        self.c2r_disk = 2*math.pi/(24*19.203)
        self.old_states = np.zeros((6, 1))
        
        self.kalman_Q = 1e-4
        self.kalman_R = 1e-6
        self.rollvel_pred = 0
        self.pitchvel_pred = 0
        self.rollacc_pred = 0
        self.pitchacc_pred = 0
        self.roll_P = np.array([[1,0],[0,1]])
        self.pitch_P = np.array([[1,0],[0,1]])

        action_high = np.array([self.max_volt, self.max_volt])
        action_low = -action_high
        observation_high = np.array([self.max_speed_wheel, self.max_speed_disk, 100 , 100 , self.max_phi, self.max_theta])
        observation_low = -observation_high
        
        #degines in what range action inputs are allowed
        self.action_space = spaces.Box(low=action_low, high=action_high, dtype=np.float32)
        print('actiorn_space: ', self.action_space, type(self.action_space))
        #sort of defines what the agent should expect, needs this to set up a policy in the beginning
        self.observation_space = spaces.Box(low=observation_low, high=observation_high, dtype=np.float32)
        
        #Runge kutta matrices
        self.ARK5 = np.array([[0,0,0,0,0,0],[1/4,0,0,0,0,0],[3/32,9/32,0,0,0,0],[1932/2197,-7200/2197,7296/2197,0,0,0],[439/216,-8,3680/513,-845/4104,0,0],[-8/27,2,-3544/2565,1859/4104,-11/40,0]])
        self.BRK5 = np.array([16/135,0,6656/12825,28561/56430,-9/50,2/55])
        self.BRK4 = np.array([25/216,0,1408/2565,2197/4101,-1/5,0])
        self.tol = 1e-3

        self.state = np.zeros((6, 1))
        self.seed()

    #Seeds the environment - returns a random seed
    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]
    
    #The non linear model
    def NL_model_v3(self,dalpha_w, dalpha_d, dphi, dtheta, phi, theta, u):

        self.last_u = u # for rendering
        u_w, u_d = u

        t2 = math.cos(theta)
        t3 = t2**2
        t4 = math.sin(theta)
        t5 = theta*2.0
        t6 = math.sin(t5)
        t7 = t4**2
        t8 = math.sin(phi)
        t9 = dphi**2
        t10 = t3**2
        t11 = dtheta**2
        t12 = math.cos(phi)
        t13 = t2*5.270113949764262e136
        t14 = t2*t4*2.9425384197083e135
        t15 = t3*6.83841392796149e136
        t18 = t2*t3*2.611232497919307e136
        t19 = t10*4.084668258920979e136
        t20 = t2*t3*t4*1.460114161456265e135
        t16 = t13+t14+t15-t18-t19-t20+2.879983171105363e136
        t17 = 1.0/t16


        new_dalpha_w = t17*(dalpha_d*2.042856929685399e136+dalpha_w*3.273073914696596e151+t8*3.054874977324171e138+t9*2.745784604300393e135-t12*3.741131523359285e137-u_d*7.790525636087062e136-u_w*3.056236531283865e151+dalpha_d*dphi*5.110933045413505e134+dphi*dtheta*4.434845622939034e135-dalpha_d*t2*2.546554051031046e144+dalpha_d*t3*8.443765036611527e146+dalpha_d*t4*1.659312290475356e149+dalpha_d*t6*8.296558755645479e148+dalpha_w*t2*6.973354523360509e151+dalpha_w*t3*1.11917613821661e152+dalpha_w*t4*9.920814586940586e149+dalpha_w*t6*1.672083684579905e150+t2*t8*5.403285701124564e138+t2*t9*8.546396552413335e148+t3*t8*1.260791801283757e149+t3*t9*3.082291665851177e149+t4*t8*4.38882201112563e151+t2*t11*5.198980458965509e149+t4*t9*7.831387532098359e150-t3*t12*2.099879547909213e151+t4*t11*1.963955932906972e151+t6*t9*7.642580937407093e149+t6*t11*4.65228607866632e150-t9*t10*4.623387333236904e149+t10*t12*2.099879547909251e151+t2*u_d*9.711396980352382e144-t3*u_d*3.220067300207267e147-t4*u_d*6.327861119091366e149-t6*u_d*3.163929531135224e149-t2*u_w*6.511377804269218e151-t3*u_w*1.045032005907471e152-t4*u_w*9.263572027676704e149-t6*u_w*1.561310063066676e150+dalpha_d*dphi*t3*2.869299109108257e148+dalpha_d*dphi*t4*5.130389377691243e149+dalpha_d*dphi*t6*1.422570415904624e149-dalpha_d*dphi*t10*2.867173259503638e148+dalpha_d*dtheta*t2*5.995773018682494e148+dalpha_d*dtheta*t4*3.05825282407086e146+dphi*dtheta*t2*6.44230753926698e149+dphi*dtheta*t3*2.075640248828218e150+dphi*dtheta*t4*3.272962051762236e147+dphi*dtheta*t6*1.801090557489944e148-dphi*dtheta*t10*2.075823281462639e150+dalpha_d*t2*t3*8.483321453815265e146-dalpha_d*t4*t7*1.659312290475356e149+dalpha_w*t2*t3*2.775347128028627e151-dalpha_w*t4*t7*9.920814586940409e149+t2*t3*t8*2.230015422084065e149+t2*t3*t9*5.809970216980451e149+t2*t4*t8*2.481324009745926e151-t2*t3*t11*5.202806029440341e149-t2*t4*t12*2.082603305443433e152-t3*t4*t12*3.755373286529812e152-t4*t7*t8*4.38882201112563e151-t4*t7*t9*7.831387532098359e150-t2*t9*t10*9.929936649941362e149-t4*t7*t11*1.455484740767907e151+t4*t9*t10*1.427396486577786e151-t2*t3*u_d*3.235152315600143e147+t4*t7*u_d*6.327861119091366e149-t2*t3*u_w*2.591483571937945e151+t4*t7*u_w*9.263572027676539e149+dalpha_d*dphi*t2*t3*1.003310798056701e145-dalpha_d*dphi*t4*t7*5.130389377691243e149-dalpha_d*dtheta*t2*t3*5.995772422896568e148-dalpha_d*dtheta*t4*t7*3.058252823997043e146-dphi*dtheta*t2*t3*6.442307539355977e149-dphi*dtheta*t4*t7*3.272962051682921e147+t2*t3*t4*t9*1.344752284689147e151-t2*t3*t4*t12*5.874411442148844e152+dalpha_d*dphi*t2*t3*t4*8.025311155993206e149-dphi*dtheta*t2*t3*t4*6.149918077496672e148)*(-9.473903143468002e-15)
        new_dalpha_d = (t17*(dalpha_d*-2.00578283507889e145+dalpha_w*4.535869836181238e139+t9*3.939890218737078e137+t11*1.535128178188098e137+u_d*7.649141929636702e145-u_w*4.235373674954326e139+dalpha_d*dtheta*2.40632343861931e139+dphi*dtheta*2.585534105815514e140-dalpha_d*t2*4.012004684890443e145-dalpha_d*t3*5.10425730901202e145+dalpha_d*t4*3.349423111908704e139-dalpha_d*t6*1.02460832617633e144+dalpha_d*t10*3.014049198503113e145+dalpha_w*t2*5.343666273823632e139-dalpha_w*t3*1.514783995610157e142-dalpha_w*t4*4.943575306958141e144-dalpha_w*t6*1.4777664946376e144-t2*t8*5.10826225411768e146-t2*t9*4.591414857690957e143-t3*t8*9.03519803597496e146-t3*t9*1.522035766435382e144+t4*t8*5.008705555614654e141-t4*t9*2.351888871625816e141+t2*t12*6.25580008875471e145+t3*t11*7.819963377378171e143-t4*t11*5.829022005191063e138-t6*t9*1.282605358469394e142-t4*t12*9.60080466931992e140-t6*t11*3.946488228761421e141+t8*t10*4.476753814852014e146+t9*t10*1.522166227801671e144+t10*t11*3.259182596632243e137+t2*u_d*1.529995806145552e146+t3*u_d*1.946531195660712e146-t4*u_d*1.27731738039265e140+t6*u_d*3.907389360474802e144-t10*u_d*1.149421048931814e146-t2*u_w*4.989654527420961e139+t3*u_w*1.414431297625323e142+t4*u_w*4.616069127078926e144+t6*u_w*1.379866163528847e144-dalpha_d*dphi*t2*8.546338237478359e142-dalpha_d*dphi*t3*1.268554810217114e133-dalpha_d*dphi*t4*4.372957737381978e140-dalpha_d*dphi*t6*3.243351978251713e137-dalpha_d*dtheta*t3*6.016267420322421e139-dalpha_d*dtheta*t6*6.171701249739726e143+dalpha_d*dtheta*t10*2.392079348699333e139-dphi*dtheta*t2*9.167894488551957e139-dphi*dtheta*t3*1.276969142180746e140-dphi*dtheta*t4*1.82699680362363e145-dphi*dtheta*t6*2.069115453761744e135+dalpha_d*t2*t3*1.987865369624555e145+dalpha_d*t4*t7*3.310003030937141e139-dalpha_w*t2*t3*2.527428905384356e142+dalpha_w*t4*t7*4.943575306958141e144+t2*t3*t8*2.531038328361135e146+t2*t3*t9*4.591416806204145e143+t2*t4*t8*1.761395109586819e142-t2*t3*t12*6.25580008875471e145+t2*t4*t12*4.748176592951429e140+t3*t4*t12*3.198310734451447e143+t4*t7*t8*4.949757022697733e141+t4*t7*t9*2.358935333768436e141+t4*t7*t11*5.829022005191063e138-t2*t3*u_d*7.580812879311477e145-t4*t7*u_d*1.26228435742746e140+t2*t3*u_w*2.359989646483279e142-t4*t7*u_w*4.616069127078926e144+dalpha_d*dphi*t2*t3*8.546337385561156e142+dalpha_d*dphi*t4*t7*4.386073841304174e140+dphi*dtheta*t2*t3*7.417244845893447e143+dphi*dtheta*t4*t7*2.551769745390927e145-dphi*dtheta*t2*t10*3.672128524768424e143+dphi*dtheta*t4*t10*1.089100150830331e145+dalpha_d*t2*t3*t4*1.01683971504959e144-t2*t3*t4*t8*8.754843174698436e141+t2*t3*t4*t9*4.356314504001058e142-t2*t3*t4*t11*9.660654032062111e138-t2*t3*t4*u_d*3.877763416895396e144+dalpha_d*dtheta*t2*t3*t4*6.115900725995475e143))/2.5165824e8
        new_dphi = (t17*(dalpha_d*4.072187777437995e136+dalpha_w*7.255395246408907e123+t8*6.089523139624089e138+t9*5.473388930291566e135-t12*7.457492417522259e137-u_d*1.552946895795423e137-u_w*6.774733653710868e123+dalpha_d*dphi*1.018802578702464e135+dphi*dtheta*8.840327424856864e135+dalpha_d*t2*4.072187778708779e136-dalpha_d*t3*2.017684847696989e136-dalpha_d*t4*3.992823495756034e131+dalpha_w*t2*1.790216963132867e134+dalpha_w*t3*2.994408252267666e134+dalpha_w*t4*3.523268733743087e136+dalpha_w*t6*2.946599543147274e136+t2*t8*1.077079538483071e139+t2*t9*1.814405532989389e136-t3*t8*3.017232809497279e138-t3*t9*5.473388930291566e135-t2*t11*9.32212849492801e135-t4*t9*2.138494221944713e134+t3*t12*7.457492417522259e137+t4*t11*9.387837242449826e133+t6*t9*1.390548727519534e133-t2*u_d*1.552946896280042e137+t3*u_d*7.694530783390944e136+t4*u_d*1.522680974474728e132-t2*u_w*1.671617147747152e134-t3*u_w*2.796032148576346e134-t4*u_w*3.289856231246775e136-t6*u_w*2.751390711463915e136-dalpha_d*dphi*t3*1.018802477466437e135+dalpha_d*dphi*t6*2.5982937093691e132+dalpha_d*dtheta*t2*2.877598174333752e131+dalpha_d*dtheta*t4*7.423760619162204e135-dphi*dtheta*t3*2.206085565875265e136+dphi*dtheta*t4*7.976644355534868e136+dphi*dtheta*t6*2.546734092093561e137+dphi*dtheta*t10*8.760399810657885e135-dalpha_d*t2*t3*2.017684847696989e136-dalpha_d*t4*t7*3.945831097289492e131-t2*t3*t8*5.336706417622583e138-t2*t3*t9*1.814561032564051e136+t2*t3*t11*1.26472086516694e125-t2*t4*t12*3.789245282490729e135+t4*t7*t9*5.198831604496955e134-t4*t7*t11*6.467099232824421e129+t2*t3*u_d*7.694530783390944e136+t4*t7*u_d*1.504760214599919e132-dalpha_d*dtheta*t2*t3*1.425790398537715e131+dalpha_d*dtheta*t4*t7*7.290721807878587e135+dphi*dtheta*t4*t7*7.83369749375792e136-dphi*dtheta*t2*t3*t4*2.52370817340349e137))/3.0
        new_dtheta = t17*(dalpha_d*-2.163554096130888e131+dalpha_w*8.359385607639321e137+t9*7.261024407595829e135+t11*3.249755459877475e133+u_d*8.250809641164917e131-u_w*7.805585922896616e137+dalpha_d*dtheta*5.094012893512318e135+dphi*dtheta*5.473388930291566e136+dalpha_d*t2*7.173828601045808e133+dalpha_d*t3*7.207435760394424e133+dalpha_d*t4*1.409752408616941e136+dalpha_d*t6*7.048764334229481e135+dalpha_w*t2*2.905607144247856e138+dalpha_w*t3*4.879246922701434e138+dalpha_w*t4*1.409832547514694e137+dalpha_w*t6*4.214365459011691e136+t2*t8*1.071169584278635e136+t2*t9*2.618717125963146e136+t3*t8*1.894622641245364e136+t3*t9*4.936154704989958e136+t4*t8*2.108134590692846e138+t4*t9*1.27236530964614e138-t2*t12*1.784059112735233e138+t3*t11*2.183671279409655e136+t4*t11*3.91684874687896e137+t6*t9*3.326776125304919e137-t4*t12*1.769381205216369e139+t6*t11*1.054093753937499e137-t9*t10*8.436481029042844e136-t10*t11*2.190171242184398e136-t2*u_d*2.735771400004439e134-t3*u_d*2.748587639490274e134-t4*u_d*5.376153425270514e136-t6*u_d*2.688077586373493e136-t2*u_w*2.713114011857828e138-t3*u_w*4.556002424313359e138-t4*u_w*1.316432762291229e137-t6*u_w*3.935168593101073e136+dalpha_d*dphi*t2*2.437758502798174e135+dalpha_d*dphi*t3*8.524135462726417e131+dalpha_d*dphi*t4*9.235543509006982e136+dalpha_d*dphi*t6*2.179391177523307e136-dalpha_d*dtheta*t3*5.094012387332184e135+dalpha_d*dtheta*t6*1.29914685468455e133+dphi*dtheta*t2*1.763465387511866e137-dphi*dtheta*t3*5.473388930367178e136-dphi*dtheta*t4*2.164559081200158e135+dphi*dtheta*t6*1.390355408676988e134+dalpha_w*t2*t3*3.944005482066795e138-dalpha_w*t4*t7*1.409832547514669e137-t2*t3*t9*3.928033068267405e136+t2*t4*t8*3.728746208761129e138+t2*t3*t12*1.784059112735233e138-t2*t4*t12*3.19056773529065e139-t3*t4*t12*4.990904014354774e139-t4*t7*t9*1.142502468896315e138-t4*t7*t11*3.91684874687896e137-t2*t3*u_w*3.682719653764288e138+t4*t7*u_w*1.316432762291205e137-dalpha_d*dphi*t2*t3*2.435952379507297e135-dalpha_d*dphi*t4*t7*6.818309895270318e136-dphi*dtheta*t2*t3*1.76362089216435e137+dphi*dtheta*t4*t7*5.224974641823912e135+t2*t3*t4*t9*1.212717039842043e138+t2*t3*t4*t11*6.127002388381468e137)*(-1.0/1.5e1)
        new_phi = dphi
        new_theta = dtheta


        f = np.array([new_dalpha_w, new_dalpha_d, new_dphi, new_dtheta, new_phi, new_theta])

        return f
    #general RK method for f dependent only on x. f(t,x) x can be vector.
    def general_RK_step(self, A, B, Ts, f, init):
        #init is nx1 initial state.
        xRK = np.zeros((B.shape[0], init.size))
        stage_max = A.shape[1]
        K = np.zeros((init.size, stage_max))

        for j in range(0, stage_max):
            K[:, j] = f(self,0, init.transpose() + Ts*A[j, :]@K.transpose())

        for i in range(0, B.shape[0]):
            xRK[i, :] = init.transpose() + Ts*B[i, :]@K.transpose()
        return xRK	
    
    #updates states given input with sample time Ts and calculate reward function
    def step(self,u):
        self.count = self.count + 1

        self.state = self._get_obs()
        self.old_states = self.state.copy()

        if u[0] is "reset":
            obs = self.reset_given_state(u[1])
            return obs,0,False, {}

        #clip voltage input
        u = np.clip(u, -self.max_volt, self.max_volt)
        #filtering voltage inputs
        #[u[0], dummy] = self.weighted_average(self.last_u, u , lambda_value=0.9)

        self.t = 0
        state_tmp = self.state.copy()

        def odefun(self,t,x):
            return self.NL_model_v3(x[0],x[1],x[2],x[3],x[4],x[5],u)

        while self.t < self.dt:
            Ts = self.dt - self.t
            state_45 = self.general_RK_step(self.ARK5, np.array(
                [self.BRK4, self.BRK5]), Ts, odefun, state_tmp)
            err45 = np.linalg.norm(state_45[0][:]-state_45[1][:])
            while err45 > self.tol:
                Ts = Ts*(self.tol*Ts/(2*np.abs(err45)))**(1/4)
                state_45 = self.general_RK_step(self.ARK5, np.array(
                    [self.BRK4, self.BRK5]), Ts, odefun, state_tmp)
                err45 = np.linalg.norm(state_45[0][:]- state_45[1][:])
            state_tmp = state_45[1][:]
            self.t = self.t+Ts

        self.state = state_tmp.copy()
        self.u = u.copy()

        terminal = False
        if abs(self.state[5]) > self.max_theta or abs(self.state[4]) > self.max_phi or self.max_speed_wheel < abs(self.state[0]) or self.max_speed_disk < abs(self.state[1]):
            terminal = True

        if terminal:
            reward = -1
            self.count = 0
        else:
            reward1 = (1 - math.pow((self.state[4]/self.max_phi),2)) + (1 - math.pow((self.state[5]/self.max_theta),2)) +  (1 - math.pow((self.state[0]/self.max_speed_wheel),2)) + (1 - math.pow((self.state[1]/self.max_speed_disk),2))
            reward2 = (1 - math.pow(self.state[2]/100,2)) + (1 - math.pow(self.state[3]/100, 2))# + 0.5*(1 - math.pow(abs(u[0])/12,2)) + 0.5*(1- math.pow(abs(u[1])/12,2))
            reward = reward1 + reward2 

            if self.count == 1500:
                print('count reset', self.count)
                terminal = True
                self.count = 0

        return self.state, reward, terminal, {}

        
    def reset(self):
        #select a random initial position, within the bounduries of [7;15]
        value = random.randint(1,4)
        if value == 1:
            high = np.array([0,0,0,0,8*math.pi/180, 22*math.pi/180])
            low = np.array([0,0,0,0,3*math.pi/180, 9*math.pi/180])
        elif value == 2:
            high = np.array([0,0,0,0,-3*math.pi/180, -9*math.pi/180])
            low = np.array([0,0,0,0, -8*math.pi/180, -22*math.pi/180])
        elif value == 3: 
            high = np.array([0,0,0,0, -3*math.pi/180, 22*math.pi/180])
            low = np.array([0,0,0,0, -8*math.pi/180, 9*math.pi/180])
        else:
            high = np.array([0,0,0,0, 8*math.pi/180, -22*math.pi/180])
            low = np.array([0,0,0,0, 3*math.pi/180, -9*math.pi/180])

        self.state = self.np_random.uniform(low=low, high=high)

        print('reset')
        self.last_u = None
        return self._get_obs()

    def _get_obs(self):
        state_tmp = self.state.copy()
        #motor sensors noise
        #state_tmp[0] = self.get_motor_noise(state_tmp[0],self.c2r_wheel) #dalpha_w
        #state_tmp[1] = self.get_motor_noise(state_tmp[1],self.c2r_disk) #dalpha_d
        #IMU noise
        #state_tmp[0] = np.random.normal(loc=self.state[0], scale=math.sqrt(10e-5), size=1) #dalpha_w
        #state_tmp[1] = np.random.normal(loc=self.state[1], scale=math.sqrt(10e-5), size=1) #dalpha_d
        #state_tmp[2] = np.random.normal(loc=self.state[2], scale=math.sqrt(10e-5), size=1) #dphi
        #state_tmp[3] = np.random.normal(loc=self.state[3], scale=math.sqrt(10e-5), size=1) #dtheta
        #state_tmp[4] = np.random.normal(loc=self.state[4], scale=math.sqrt(10e-5), size=1) #phi
        #state_tmp[5] = np.random.normal(loc=self.state[5], scale=math.sqrt(10e-5), size=1) #theta
        #filter states
        #[state_tmp[0], state_tmp[1]] = self.filter_motor_states(self.old_states[0:2], state_tmp[0:2], lambda_value=[0.9, 0.1])
        #[state_tmp[2], state_tmp[3]] = self.filter_uni_angularvelocities([state_tmp[2],state_tmp[3]])
    
        return state_tmp
           
    #for closeing the wiewer   
    def close(self):
        if self.viewer_pitch:
            self.viewer_pitch.close()
            self.viewer_pitch = None

        if self.viewer_roll:
            self.viewer_roll.close()
            self.viewer_roll = None

    #for viewing the xz plane of the system
    def print_pitch_view(self):
        screen_width = 600
        screen_height = 400
        scale =  3
        wheel_y = 100 
        polewidth = 11.0*3
        polelen = 35*3

        if self.viewer_pitch is None:
            from gym.envs.classic_control import rendering
            #wheel object initialization
            self.viewer_pitch = rendering.Viewer(screen_width, screen_height)
            wheel = rendering.make_circle(7.2*3, filled=True)
            self.wheeltrans = rendering.Transform()
            wheel.add_attr(self.wheeltrans)
            self.viewer_pitch.add_geom(wheel)

            #body object initialization
            l,r,t,b = -polewidth/2,polewidth/2,polelen-polewidth/2,-polewidth/2
            pole = rendering.FilledPolygon([(l,b), (l,t), (r,t), (r,b)])
            pole.set_color(.24, .7, .9)
            self.poletrans = rendering.Transform(translation=(0, 7.2))
            pole.add_attr(self.poletrans)
            pole.add_attr(self.wheeltrans)
            self.viewer_pitch.add_geom(pole)

            self.axle = rendering.make_circle(polewidth/2)
            self.axle.add_attr(self.poletrans)
            self.axle.add_attr(self.wheeltrans)
            self.axle.set_color(.5,.5,.8)
            self.viewer_pitch.add_geom(self.axle)

            self._pole_geom = pole
            self.old_wheel = 0

        #updates at every timestep to body and wheel
        pole = self._pole_geom
        l,r,t,b = -polewidth/2,polewidth/2,polelen-polewidth/2,-polewidth/2
        pole.v = [(l,b), (l,t), (r,t), (r,b)]
        x = self.state
        wheel_translation = self.old_wheel + (x[0] * self.dt * 72 * scale)
        wheel_x = wheel_translation  + screen_width/2.0 
        self.old_wheel =  wheel_translation
        self.wheeltrans.set_translation(wheel_x, wheel_y)
        self.poletrans.set_rotation(-x[5])

    #for viewing the yz plane of the system
    def print_roll_view(self):
        screen_width = 600
        screen_height = 400
        scale =  3
        polewidth = 11.0*scale
        polelen = 35*scale

        if self.viewer_roll is None:
            from gym.envs.classic_control import rendering
            self.viewer_roll = rendering.Viewer(screen_width, screen_height)
            
            #wheel object initialization
            wheel2 = rendering.FilledPolygon([(0,0), (0,14.4*scale), (3*scale,14.4*scale), (3*scale,0)])
            self.wheeltrans2 = rendering.Transform(translation=(0,0))
            wheel2.add_attr(self.wheeltrans2)
            self.viewer_roll.add_geom(wheel2)
            
            #body object initialization
            l,r,t,b = -polewidth/2,polewidth/2,polelen-polewidth/2,-polewidth/2
            pole2 = rendering.FilledPolygon([(l,b), (l,t), (r,t), (r,b)])
            pole2.set_color(.24, .7, .9)
            self.poletrans2 = rendering.Transform(translation=(0, 7.2*scale))
            pole2.add_attr(self.poletrans2)
            pole2.add_attr(self.wheeltrans2)
            self.viewer_roll.add_geom(pole2)

            #disk object initialization
            disk = rendering.make_circle(20*scale, filled=False)
            self.disktrans = rendering.Transform(translation=(0,35*scale+7.2*scale))
            disk.add_attr(self.poletrans2)
            disk.add_attr(self.disktrans)
            self.viewer_roll.add_geom(disk)

            self._pole_geom = wheel2
            self._pole_geom = pole2

        #wheel related updates
        x = self.state
        wheel2 = self._pole_geom
        pole2 = self._pole_geom
        wheel2.v = ([(0,0), (0,14.4*scale), (3*scale,14.4*scale), (3*scale,0)])
        self.wheeltrans2.set_rotation(x[4])
        self.wheeltrans2.set_translation(screen_width/2,100)

        #body lelated updates
        l,r,t,b = -polewidth/2,polewidth/2,polelen-polewidth/2,-polewidth/2
        pole2.v = [(l,b), (l,t), (r,t), (r,b)]
        self.poletrans2.set_rotation(x[4])

        #disk related updates
        disk_x = -np.sin(x[4])*(7.2*3 + 35*3) + screen_width/2#y
        disk_y = np.cos(x[4])*(7.2*3 + 35*3) + 22.4*scale #z
        self.disktrans.set_translation(disk_x, disk_y)
 
    #prints out a view of the system
    def render(self, mode='human'):
        self.print_pitch_view()
        self.print_roll_view()

        self.viewer_pitch.render(return_rgb_array = mode=='rgb_array')
        self.viewer_roll.render(return_rgb_array = mode=='rgb_array')

    #a weighted mooving average on the angular velocity of the motors
    def filter_motor_states(self, old_angular_vel, new_angular_vel, lambda_value):
        if old_angular_vel[0] is None:
            old_angular_vel = [0 ,0]

        self.weight_angvel_w = lambda_value[0] * self.weight_angvel_w + 1
        self.weight_angvel_d = lambda_value[1] * self.weight_angvel_d + 1
        filtered_angular_vel1 = (1 - 1/self.weight_angvel_w) * old_angular_vel[0] + (1/self.weight_angvel_w) * new_angular_vel[0]
        filtered_angular_vel2 = (1 - 1/self.weight_angvel_d) * old_angular_vel[1] + (1/self.weight_angvel_d) * new_angular_vel[1]
        return  [filtered_angular_vel1, filtered_angular_vel2] 

    #wheighted average filter for voltage input
    def weighted_average(self, old_value, new_value, lambda_value):
        if old_value is None:
            old_value = [0, 0]
        
        self.weight_input = lambda_value * self.weight_input + 1
        filtered_output1 = (1 - 1/self.weight_input) * old_value[0] + (1/self.weight_input) * new_value[0]
        filtered_output2 = (1 - 1/self.weight_input) * old_value[1] + (1/self.weight_input) * new_value[1]
        return [filtered_output1, filtered_output2]

    def return_u(self):
        return self.u

    def reset_given_state(self,state_inp):
        #print('reset_given: ',state_inp)
        self.state = state_inp.copy()
        self.last_u = np.zeros(2)
        self.steps_since_reset = 0
        self.old_states = np.zeros(6)
        return self._get_obs()

    def get_motor_noise(self, vel, c2r):
        Ts = self.dt
        counts = vel*Ts/c2r
        rand = random.uniform(0,1)
        if rand > abs(counts) - abs(math.floor(counts)):
            noise = math.floor(counts)
        else:
            noise = math.ceil(counts)

        noise = noise*c2r/Ts
        return noise

    def kalman_prediction(self,omega_hat,domega_hat,P):
        omega_hat = omega_hat + self.dt*domega_hat
        domega_hat = domega_hat
        P = np.array([[P[1,1]*self.dt*self.dt + 2*P[0,1]*self.dt + P[0,0], P[0,1] + P[1,1]*self.dt],[P[0,1] + P[1,1]*self.dt,P[1,1]+self.kalman_Q]])
        return omega_hat, domega_hat, P

    def kalman_update(self,omega_hat,domega_hat,P,meas):
        domega_hat = domega_hat - ((P[0,1]*(omega_hat-meas))/(P[0,0]+self.kalman_R))
        omega_hat = (P[0,0]*meas + (omega_hat)*self.kalman_R) / (P[0,0]+self.kalman_R)
        P = np.array([[P[0,0]*self.kalman_R/(P[0,0]+self.kalman_R),P[0,1]*self.kalman_R/(P[0,0]+self.kalman_R)],[P[0,1]*self.kalman_R/(P[0,0]+self.kalman_R),P[1,1]-(P[0,1]*P[0,1]/(P[0,0]+self.kalman_R))]])
        return omega_hat, domega_hat, P

    def filter_uni_angularvelocities(self, meas):
        upd_roll_vel, self.rollacc_pred,self.roll_P = self.kalman_update(self.rollvel_pred,self.rollacc_pred,self.roll_P,meas[0])
        upd_pitch_vel, self.pitchacc_pred,self.pitch_P = self.kalman_update(self.pitchvel_pred,self.pitchacc_pred,self.pitch_P,meas[1])
        self.rollvel_pred,self.rollacc_pred,self.roll_P = self.kalman_prediction(upd_roll_vel, self.rollacc_pred,self.roll_P)
        self.pitchvel_pred,self.pitchacc_pred,self.pitch_P = self.kalman_prediction(upd_pitch_vel, self.pitchacc_pred,self.pitch_P)
        return  [upd_roll_vel,upd_pitch_vel]


       

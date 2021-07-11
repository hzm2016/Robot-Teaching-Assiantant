import numpy as np
from .tcp_protocol import Client, Server


class MovementPrimitive():
    
    def __init__(self):
        
        pass
    
    def get_full_trajectory(self):
        
        pass


class TaskInterface:

    def __init__(self):
        """
        Initialize the environment. Specify the number of feature of the movement.
        We assume "ClassicSpace" for the rbf of the movement, as specified in romi library.
        :param n_features:
        """
        pass

    def get_context_dim(self):
        """
        Returns the dimension of the context variable.
        :return:
        """
        pass

    def send_movement(self, params):
        """
        Send the weights of the movement and its duration
        :param weights: set of weights
        :param duration: duration of the movement
        :return: returns the success and the dense_reward
        """
        pass

    def read_context(self):
        """
        Read the context (before sending the movement).
        :return:
        """
        pass

    def reset(self):
        """
        Reset the environment to a (random) initial position.
        :return:
        """
        pass

    def get_demonstrations(self):
        """
        Retrieve the matrix containing the context and the movement parameters stored for this task
        """
        pass
    

class TCPTask():

    def __init__(self, ip, port):
        # super.__init__()
        self._conn = Client(ip, port)

    def get_loop_run_done(self):
        return self._conn.wait_loop_run_done()

    def send_params(self, params):
        return self._conn.send_params(params)
    
    def send_params_request(self):
        return self._conn.send_params_request()
    
    def send_way_points(self, way_points):
        self._conn.wait_way_points()
        print("Length of Way Points :::", way_points.shape[0])
        for i in range(way_points.shape[0]):
            self._conn.send_way_points(way_points[i, :])
        
        self._conn.send_way_points_done()
        
    def send_way_points_request(self):
        return self._conn.send_way_points_request()
    
    def send_way_points_done(self):
        return self._conn.send_way_points_done()

    def reset(self):
        # position reset :::
        return self._conn.send_reset()
    
    def get_encoder_check(self):
        return self._conn.wait_encoder_check()
    
    def get_demonstrations(self):
        return self._conn.wait_demonstration()

    def read_context(self):
        return self._conn.send_context_request()
    

class TCPServerExample:
    """
        client
    """
    def __init__(self, port, state_dim):
        self._server = Server(port)

        # First thing, wait for context_dim_request
        self._server.wait_context_dim_request()

        # Wait impedance parameters :::
        self._server.wait_params_request()

        # send context_dim
        self._state_dim = state_dim
        self._server.send_context_dim(state_dim)

        # ask for number of demonstrations
        self._n_demos = self._server.read_n_features()

    def run(self):
        
        # send the dataset of writting
        self._server.wait_demonstration_request()

        self._server.wait_way_points_request()
        
        # receive way points
        way_points = []
        send_done = self._server.wait_send_done()
        while not send_done:
            way_point = self._server.read_way_points()
            way_points.append(way_point)
            
        # each trajectory and impedance parameter for n_demos
        for i in range(self._n_demos):
            
            # wait for a reset request
            self._server.wait_reset()
            
            # reset the environment
            self._server.reset_ack()

            # Then, wait for the client to ask for the context.
            self._server.wait_context_request()
            
            impedance_params = self._server.read_params()
        
            # lastly, perform the movement
            duration, weights = self._server.wait_movement()
            mp = MovementPrimitive(self._space, MovementPrimitive.get_params_from_block(self._space, weights))
            mp.get_full_trajectory(duration=5.)

            # Run the demonstration, .... and send the data
            self._server.send_demonstration(np.random.normal(size=(1000, self._space.n_params + self._state_dim + 1)))

            self._server.send_done()


class RunWritingModel:
    
    def __init__(self, task: TaskInterface, rl_model: None, dense_reward=False):
        self.task = task
        self._rl_model = rl_model
        self.dense_reward = dense_reward
    
    def collect_samples(self, n_episodes=50, noise=True, isomorphic_noise=False):
        
        success_list = []
        reward_list = []
        latent = []
        cluster = []
        observations = []
        parameters = []
        
        for i in range(n_episodes):
            
            # env reset
            self.task.reset()
            
            # get context
            context = self.task.read_context()
            observations.append(context)
            w, z, k = self._rl_model.generate_full(np.expand_dims(context, 0), noise=noise,
                                                   isomorphic_noise=isomorphic_noise)
            
            parameters.append(w)
            latent.append(z)
            cluster.append(k)
            
            success, tot_reward = self.task.send_movement(w[1:], w[0])
            print(success, tot_reward)
            success_list.append(success)
            
            if self.dense_reward:
                reward_list.append(tot_reward)
            else:
                reward_list.append(success)
        
        print("-" * 50)
        print("Total reward", np.mean(reward_list))
        print("-" * 50)
        return np.array(success_list), np.array(reward_list), np.array(parameters), np.array(latent), \
               np.array(cluster), np.array(observations)


class RunInteractionModel:
    
    def __init__(self, task: TaskInterface, rl_model: None, dense_reward=False):
        self.task = task
        self._rl_model = rl_model
        self.dense_reward = dense_reward
    
    def collect_samples(self, n_episodes=50, noise=True, isomorphic_noise=False):
        
        success_list = []
        reward_list = []
        latent = []
        cluster = []
        observations = []
        parameters = []
        
        for i in range(n_episodes):
            # env reset
            self.task.reset()
            
            # get context
            context = self.task.read_context()
            observations.append(context)
            w, z, k = self._rl_model.generate_full(np.expand_dims(context, 0), noise=noise,
                                                   isomorphic_noise=isomorphic_noise)
            
            parameters.append(w)
            latent.append(z)
            cluster.append(k)
            
            success, tot_reward = self.task.send_movement(w[1:], w[0])
            print(success, tot_reward)
            success_list.append(success)
            
            if self.dense_reward:
                reward_list.append(tot_reward)
            else:
                reward_list.append(success)
        
        print("-" * 50)
        print("Total reward", np.mean(reward_list))
        print("-" * 50)
        return np.array(success_list), np.array(reward_list), np.array(parameters), np.array(latent), \
               np.array(cluster), np.array(observations)
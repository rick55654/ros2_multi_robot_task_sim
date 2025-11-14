import sys
import abc

from multi_robot_sim.Simulation.utils import ControlState

class Simulator:
    @abc.abstractmethod
    def init_state(self, pose):
        return NotImplementedError

    @abc.abstractmethod
    def step(self, command:ControlState):
        return NotImplementedError
    
    @abc.abstractmethod
    def render(self, img):
        return NotImplementedError
        
import abc
import sys

from multi_robot_sim.Simulation.utils import State, ControlState

class KinematicModel:
    @abc.abstractmethod
    def step(self, state:State, cstate:ControlState) -> State:
        return NotImplementedError
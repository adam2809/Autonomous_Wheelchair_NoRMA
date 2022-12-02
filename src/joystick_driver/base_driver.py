from abc import abstractmethod

class BaseJoystickDriver():
    @abstractmethod
    def set_output(self,fb,lr):
        pass


    @abstractmethod
    def get_input(self):
        pass

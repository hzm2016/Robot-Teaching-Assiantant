from utils import skeletonize
from .utils import hungarian_matching

class Controller(object):

    def __init__(self, img_processor, impedance_level=10) -> None:
        self.img_processor = img_processor
        self.impedance_level = impedance_level
        pass

    def guide(self,):
        """[summary]

        Raises:
            NotImplementedError: [description]
        """
        raise NotImplementedError

    def update_impedance(self, input, target):
        """[summary]

        Args:
            input ([image]): written image
            target ([image]): target image to be learnt
        """ 
        input = self.img_processor.process(input)
        tgt_pts = skeletonize(target)
        in_pts = skeletonize(input)


    def update_period(self, ):
        """ update period with user's input

        Raises:
            NotImplementedError: [description]
        """
        raise NotImplementedError

if __name__ == "__main__":

    
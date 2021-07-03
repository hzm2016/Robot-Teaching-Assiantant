from tools import skeletonize
from utils import hungarian_matching
import numpy as np


class Controller(object):

    def __init__(self, img_processor=None, impedance_level=10) -> None:
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
        if self.img_processor is not None:
            input = self.img_processor.process(input)
        tgt_pts = skeletonize(target)
        in_pts = skeletonize(input)

        matching = self.key_point_matching(tgt_pts, in_pts)
        tgt_index = matching[:,0]
        in_index = matching[:,1]

        x_dis = sum(abs(tgt_pts[tgt_index][:,0] - in_pts[in_index][:,0]))
        y_dis = sum(abs(tgt_pts[tgt_index][:,1] - in_pts[in_index][:,1]))

        self.impedance_update_policy(x_dis,y_dis)

    def impedance_update_policy(self, x_dis, y_dis):

        raise NotImplementedError

    def key_point_matching(self, tgt_pts, in_pts):

        matching = hungarian_matching(tgt_pts, in_pts)
        matching = np.array(matching)

        return matching

    def update_period(self, ):
        """ update period with user's input

        Raises:
            NotImplementedError: [description]
        """
        raise NotImplementedError


if __name__ == "__main__":

    a = np.array([(3, 4), (7, 8)])
    b = np.array([(1, 2), (3, 4), (5, 6)])


    c = Controller()

    matching = c.key_point_matching(a, b)

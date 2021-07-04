from tools import skeletonize
from .utils import hungarian_matching
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

    def update_impedance(self, target, input):
        """[summary]

        Args:
            input ([image]): written image
            target ([image]): target image to be learnt
        """
        if self.img_processor is not None:
            input = self.img_processor.process(input)
        tgt_pts, _ = skeletonize(~target)
        in_pts, _ = skeletonize(~input)

        tgt_pts = np.squeeze(tgt_pts, axis=0)
        in_pts = np.squeeze(in_pts, axis=0)

        matching = self.key_point_matching(tgt_pts, in_pts)
        tgt_index = matching[:, 0]
        in_index = matching[:, 1]

        x_dis = sum(abs(tgt_pts[tgt_index][:, 0] - in_pts[in_index][:, 0]))
        y_dis = sum(abs(tgt_pts[tgt_index][:, 1] - in_pts[in_index][:, 1]))

        return x_dis, y_dis
        self.impedance_update_policy(x_dis, y_dis)

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
    from imgprocessor import Postprocessor
    c = Controller(Postprocessor(
        {'CROPPING': [478, 418, 1586, 672], 'BINARIZE': 128, 'RESCALE': 0.8}))
    import cv2
    written_stroke = cv2.imread('./example/example_feedback.png')
    sample_stroke = cv2.imread(
        './example/example_traj.png', cv2.IMREAD_GRAYSCALE)
    # cv2.imshow('',sample_stroke)
    # cv2.waitKey(0)

    x_dis, y_dis = c.update_impedance(sample_stroke, written_stroke)
    print(x_dis, y_dis)
    matching = c.key_point_matching(a, b)

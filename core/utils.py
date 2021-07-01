from utils import skeletonize


def load_class(class_name):
    """[summary]

    Args:
        class_name ([str): name of the class to be built

    Returns:
        [type]: [description]
    """
    components = class_name.split('.')
    mod = __import__(components[0])
    for comp in components[1:]:
        mod = getattr(mod, comp)
    return mod


def get_traj_discrepancy(input, target):
    """[summary]

    Args:
        input ([image]): written image
        target ([image]): target image to be learnt
    """

    












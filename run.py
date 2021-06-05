import argparse
from core import Executor
import yaml


def main(args):

    configs = open(args.config, 'r')

    try:
        configs = yaml.safe_load(configs)
    except yaml.YAMLError as exc:
        print(exc)    
    runner = Executor(configs)
    runner.pipeline()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', type=str, required=True, help='yaml file provides configuration')

    args = parser.parse_args()

    main(args)
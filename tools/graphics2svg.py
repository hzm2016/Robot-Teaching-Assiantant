import os.path as osp
import cv2
import numpy as np
from PIL import Image
import os
from cairosvg import svg2png
from tqdm import tqdm
from svgpathtools import parse_path, disvg, wsvg

def list_to_str(list):

    rt_str = ''

    for ele in list:
        rt_str += ele
        rt_str += '\n'

    return rt_str

def svg_2_img(out_path, path_list):
    """[summary]

    Args:
        svg ([type]): [description]
    """

    transform = [r'<g transform="scale(1, -1) translate(0, -900)">', r'</g>']

    # for index in range(len(path_list)):

    # img_name = osp.join(out_path, str(1)) + '.jpg'
    # svg_name = osp.join(out_path, str(1)) + '.svg'
        # paths = path_list[index]
    
    img_name = out_path
    svg_name = out_path.replace('jpg','svg')
        
    path_str = wsvg(path_list,filename=svg_name,dimensions=(1024,1024))#,viewbox='0 0 1024 1024')
    path_str_list = path_str.split('\n')
    path_str_list.insert(2, transform[0])
    path_str_list.insert(-2, transform[1])
    path_str = list_to_str(path_str_list)
    svg2png(bytestring=path_str,write_to=img_name,background_color='white',output_width=128, output_height=128)

def svg2img(paths):

    paths = parse_path(paths)

    transform = [r'<g transform="scale(1, -1) translate(0, -900)">', r'</g>']    
    path_str = wsvg(paths, dimensions=(1024,1024))
    path_str_list = path_str.split('\n')
    path_str_list.insert(2, transform[0])
    path_str_list.insert(-2, transform[1])
    path_str = list_to_str(path_str_list)
    io = svg2png(bytestring=path_str,background_color='white',output_width=128, output_height=128)
    io.seek(0)
    byteImg = np.asarray(Image.open(io))

    return byteImg[:,:,0]


def _parse_strokes(strokes):
    """[summary]

    Args:
        strokes ([type]): [description]
    """

    path_list = []

    for stroke in strokes:

        path = parse_path(stroke)
        path_list.append(path)

    return path_list

def main():
    """[summary]
    """

    input_file = './tools/src/graphics.txt'
    output_dir = './imgs/D'

    input_lines = open(input_file,'r').readlines()

    for idx, line in enumerate(tqdm(input_lines)):

        char_info = eval(line)

        strokes = char_info['strokes']
        medians = char_info['medians']
        char = char_info['character']

        out_path = osp.join(output_dir, str(idx)) + '.jpg'
        # os.makedirs(out_path, exist_ok = True)
        svg = _parse_strokes(strokes)
        svg_2_img(out_path, svg)

if __name__ == '__main__':
    main()
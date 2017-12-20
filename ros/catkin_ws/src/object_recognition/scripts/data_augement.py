import Augmentor


def main(path):
    p = Augmentor.Pipeline(path)
    p.rotate90(probability=0.5)
    p.rotate270(probability=0.5)
    # p.skew(probability=0.3, magnitude=0.5)
    # p.skew_tilt(probability=0.3, magnitude=0.5)
    # p.flip_left_right(probability=0.3)
    # p.flip_top_bottom(probability=0.3)
    p.rotate(probability=0.5, max_left_rotation=10, max_right_rotation=15)
    p.zoom(probability=0.3, min_factor=1.1, max_factor=1.5)
    # p.random_erasing(probability=0.2, rectangle_area=0.8)
    # p.random_distortion(probability=0.3, grid_width=4, grid_height=4, magnitude=5)
    p.resize(probability=1.0, width=256, height=256)
    # p.invert(probability=0.1)
    p.sample(3500)


if __name__ == '__main__':
    list_datasets = ['ChickenSoup', 'Chips', 'Coke', 'EraserBox', 'Fanta', 'Salt', 'Shampoo', 'SportsDrink',
                     'TomatoSoup', 'YellowContainer']
    for data in list_datasets:
        path = '/home/student/dataset/datasetLL/' + data
        main(path)

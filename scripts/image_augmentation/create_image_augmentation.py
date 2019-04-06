"""
Usage
    python create_image_augmentation.py --input /path/to/input/image --output /path/to/output/image --img_ext jpg
"""
import cv2
import os
import fnmatch
import argparse
import image_augmentation

def parse_arg():
    parser = argparse.ArgumentParser()
    parser.add_argument('--input', default=os.getcwd(), help='input image path')
    parser.add_argument('--output', default=os.getcwd()+'\\augmented_image\\', help='output image path')
    parser.add_argument('--img_ext', default='jpg', help='input image extension')

    args = vars(parser.parse_args())
    return args

def augment_images(args):
    images_list = os.listdir(args['input'])

    image_aug = image_augmentation.ImageAugmentation()

    # Create directory if not exist
    if not os.path.exists(args['output']):
        os.mkdir(args['output'])

    for image_file in os.listdir(args['input']):
        if fnmatch.fnmatch(image_file, '*.'+args['img_ext']):
            print("Generating augmentation for image: {0}".format(image_file))
            image_path = args['input'] + '\\' + image_file
            image = cv2.imread(image_path)

            # Generate augment image and save
            augmented_inc_brightness_image = image_aug.increase_random_brightness(image)
            cv2.imwrite(args['output'] + 'inc_bright_' + image_file, augmented_inc_brightness_image)

            augmented_dec_brightness_image = image_aug.decrease_random_brightness(image)
            cv2.imwrite(args['output'] + 'dec_bright_' + image_file, augmented_dec_brightness_image)

            augmented_shadow_image = image_aug.add_random_shadow(image)
            cv2.imwrite(args['output'] + 'shadow_' + image_file, augmented_shadow_image)

            augmented_snow_image = image_aug.add_snow(image)
            cv2.imwrite(args['output'] + 'snow_' + image_file, augmented_snow_image)

            augmented_rain_image = image_aug.add_rain(image)
            cv2.imwrite(args['output'] + 'rain_' + image_file, augmented_rain_image)

            augmented_fog_image = image_aug.add_fog(image)
            cv2.imwrite(args['output'] + 'fog_' + image_file, augmented_fog_image)

            augmented_rain_fog_image = image_aug.add_fog(image)
            augmented_rain_fog_image = image_aug.add_rain(augmented_rain_fog_image)
            cv2.imwrite(args['output'] + 'rain_fog_' + image_file, augmented_rain_fog_image)

def main():
    args = parse_arg()

    print(args)

    augment_images(args)


main()
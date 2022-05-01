import os
from shutil import rmtree
from PIL import Image
from imageio import mimsave

__author__ = "Hanna Krasowski, Lucas Lange, Fabian Danisch, Erik Schütz"
__copyright__ = "TUM Cyber-Physical System Group"
__credits__ = ["ConVeY"]
__version__ = "2022a"
__maintainer__ = "Hanna Krasowski"
__email__ = "commonocean@lists.lrz.de"
__status__ = "released"

def get_project_root():
    """
    Get the root path of project.
    :return: String of the project root path
    """
    return Path(__file__).parent.parent.parent.parent


'This Code Creates a gif from a Folder of png images.'
# HOW TO:
# -> Put the Folder of Images(default = png) you want to create a gif from into the creating_debugging_gif Folder
# -> Change the png_dir path to the path of your Folder of Images
# -> If needed, Change the name of the Gif in the mimsave function
# -> the Gif will be created in the given Folder (png_dir), in the Process of creating the GIF,
# the images are resized to width x height and are saved in resized_pics -> if you want to keep these pics,then comment out line 37

# path to folder of images
png_dir = os.path.join(str(get_project_root()),
                       'install/commonocean_io/commonocean/visualization/creating_debugging_gif/meta_USA_FLO-1_20190101_T-20/')
images = []
os.makedirs(png_dir + 'resized_pics', exist_ok=True)
png_resized_dir = os.path.join(png_dir, "resized_pics/")  # Folder for the Resized Images
width = 1000
height = 1000
# Max ~3000x3000 -> Results in SIGKILL if more Pixels are Used
# If the gif is choppy, play around with the width & height

# Creates the Gif by Appending the images in the Given Folder(png_dir)
for file_name in sorted(os.listdir(png_dir)):
    if file_name.endswith('.png'):  # Different Format Wanted? Change here
        file_path = os.path.join(png_dir, file_name)
        image = Image.open(file_path)
        image = image.resize((width, height), Image.ANTIALIAS)
        image.save(png_resized_dir + "resized_" + file_name, 'png')
        images.append(image)

# Saves the Gif -> Name giving Here:
try:
    mimsave(png_dir + 'gif_for_debugging.gif', images)
    rmtree(png_resized_dir)
    print("Gif completed!")
    print("& can be found at: ", png_dir + 'gif_for_debugging.gif')
except RuntimeError as r:
    print("RunTimeError!")
    print("Please Check if Folder exists & has images")
    exit(r)

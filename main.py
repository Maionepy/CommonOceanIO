import matplotlib.pyplot as plt

from commonocean.common.file_reader import CommonOceanFileReader
from commonocean.scenario.scenario import Tag
from commonocean.common.file_writer import CommonOceanFileWriter, OverwriteExistingFile
from commonocean.visualization.draw_dispatch_cr import draw_object

"""
Main function to simply read in and plot a CommonOcean Scenario.
For more examples, please see the CommonOceanMain.ipynb IPython notebook by typing "jupyter notebook" in the console.
"""

# generate path of the file to be read
path_file = "scenarios/CommonOcean_Test_Scenario-1.xml"

# read in the scenario and planning problem set
scenario, planning_problem_set = CommonOceanFileReader(path_file).open()
print(scenario.__str__())

# plot scenario and planning problem
plt.figure(figsize=(15, 5))
draw_object(scenario)
draw_object(planning_problem_set)
plt.gca().set_aspect('equal')
plt.margins(0, 0)
plt.show()

cof = CommonOceanFileWriter(scenario, planning_problem_set, author="Hanna Krasowski", affiliation="Technical University of Munich, Germany", source="handcrafted", tags=[Tag.OPENSEA], location=scenario.location)
cof.write_to_file("scenarios/test" + scenario.benchmark_id + '.xml', overwrite_existing_file=OverwriteExistingFile.ALWAYS)

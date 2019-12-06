import yaml
import os

class Parser:
    def __init__(self, file_name):
        self.file_name = file_name
        self.data = None
        with open(self.file_name) as f:
            self.data = yaml.load(f)

    def write_values(self, color_object, word, index, val):
        array = color_object[word]
        array[index] = val

        color_object[word] = array

        with open(self.file_name, 'w') as f:
            data = yaml.dump(self.data, f)
        

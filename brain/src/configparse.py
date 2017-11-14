import ConfigParser

""" Module to parse object file and store the options in a parameter dictionary"""

class ParameterDict():
    """
    Class to collect different sections each containing some options with
    their value
    """

    def __init__(self):
        """Initialize the option dictionary empty"""
        self.option_dict = {}

    def add_option(self, section, option, value):
        """
        Add a new option to the dictionary in the correct section.
        If the section didn't exist yet, it is created.
        If the option already existed, it is overwritten.
        """
        if section in self.option_dict.keys():
            self.option_dict[section][option] = value
        else:
            self.option_dict[section] = {option:value}

    def get_option(self, section, option, default=None):
        """
        Returns value from the dictionary, or None if the section or option
        does not exist.
        """
        if section in self.option_dict.keys():
            if option in self.option_dict[section].keys():
                return self.option_dict[section][option]
        return default

    def get_section(self, section):
        """
        Get a section (dictionary) from the entire option dictionary, or {}
        if that section doesn't exist
        """
        if section in self.option_dict.keys():
            return self.option_dict[section]
        return {}

def parse_config(filename, option_dict, overwrite=False):
    """
    Parses a config file and stores the results in the option dictionary
    (of type ParameterDict)
    """
    parser = ConfigParser.SafeConfigParser()
    parser.read(filename)
    sections = parser.sections()
    for section in sections:
        if overwrite:
            section_options = parser.items(section, 1)
        else:
            #Items are read first from the existing dict, then from the parsed section, so no overwriting.
            section_options = parser.items(section, 1, option_dict.get_section(section))
        for name, value in section_options:
            #print "setting option %s in section %s to value %s" % (name, section, value)
            option_dict.add_option(section, name, value)
            
def parse_args(args):
    """
    Parse module arguments provided in the vision config file.
    Format is: modulename.py param1=20 param2=99
    Returns the parsed arguments in dictionary format
    """
    params = {}
    for arg in args:
        param = arg.strip().split("=")
        if(len(param) == 2):
            params[param[0]] = param[1]
    return params

def parse_args_to_param_dict(args, section):
    """
    Parse module arguments provided in the vision config file.
    Format is: modulename.py param1=20 param2=99
    Returns the parsed arguments in dictionary format
    """
    paramDict = ParameterDict()
    loose = []
    for arg in args:
        param = arg.strip().split("=")
        if len(param) == 2:
            val = param[1]
            try:
                val = int(val)
            except:
                try:
                    val = float(val)
                except:
                    pass
            paramDict.add_option(section, param[0], val)
        elif len(param) == 1:
            val = param[0]
            try:
                val = int(val)
            except:
                try:
                    val = float(val)
                except:
                    pass
            loose.append(val)
    if loose:
        paramDict.add_option(section, 'loose', loose)
    return paramDict


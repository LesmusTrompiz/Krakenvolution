import yaml

def load_yaml_actions(file : str) -> dict:
    loaded_actions = {}
    yaml_file = open(file, 'r')
    yaml_content = yaml.load(yaml_file, yaml.SafeLoader)
    for key, value in yaml_content.items():
        loaded_actions[key] = value
    yaml_file.close()
    return loaded_actions

def dump_yaml_routine(file : str,content):
    output_file = open(file, 'w')
    yaml.dump(content,output_file)
    output_file.close()
    return    

def load_yaml_routine(file : str) -> list:
    yaml_file = open(file, 'r')
    yaml_content = yaml.load(yaml_file, yaml.SafeLoader)
    routine = []
    start = 0
    while start < len(yaml_content):
        routine += [(yaml_content[start], yaml_content[start+1])]
        start += 2
    yaml_file.close()
    return routine

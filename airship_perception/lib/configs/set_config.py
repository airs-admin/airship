def reset_cfg(config_path, key, value):
    with open(config_path, 'r') as file:
        lines = file.readlines()

    for i, line in enumerate(lines):
        if line.startswith(key):
            lines[i] = f'{key} = "{value}"\n'

    with open(config_path, 'w') as file:
        file.writelines(lines)
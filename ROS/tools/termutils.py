def error(msg):
    print(f'\n\033[31m{msg}\nSaliendo...\033[0m')
    exit(1)

def light_error(msg):
    print(f'\033[31mAviso: {msg}\033[0m')

def info(msg):
    print(f'\033[33m{msg}\033[0m')

def ok(msg):
    print(f'\033[32m{msg}\033[0m')

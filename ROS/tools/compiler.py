import termutils

from time import sleep
import sys
def compile(name, fun):
    progress = ['/', '-', '\\', '|']

    print(f'Compilando {name}')
    binary = bytearray()
    print('')
    for i, ins in enumerate(fun):
        opcode = ins[0]
        arg = ins[1]
        # Convert to binary
        print(f'\033[2K\033[G{progress[i % len(progress)]}\t{opcode}: {arg}', end='')
        sys.stdout.flush()
        sleep(0.01)
    termutils.ok('\033[2K\033[G\033[AOK')



def expand_loops(functions):
    def recursive(function):
        instructions = []
        times = 0
        for ins in function:
            opcode = list(ins.keys())[0]
            if opcode == 'loop':
                instructions += recursive(ins['loop'])
            else:
                if opcode == 'times':
                    times = ins['times']
                else:
                    instructions.append(ins)

        if times == 0:
            termutils.light_error(f'No se ha especificado el tamaño para el bucle con el contenido {function} por defecto será 0\n\
            para especificar un tamaño añade la etiqueta "times"')
        
        return instructions * times
                

    expanded = {}
    for fun in functions:
        expanded[fun] = []
        for ins in functions[fun]:
            opcode = list(ins.keys())[0]
            if opcode == 'loop':
                expanded[fun] += recursive(ins['loop'])
            else:
                expanded[fun].append(ins)

    return expanded

def expand_macros(functions):
    def recursive(name, fun, d):
        in_iter = set()
        for ins in fun:
            if (list(ins.keys())[0] == 'call'):
                called = ins['call']
                # Check if function exists
                if not called in functions:
                    termutils.error(f'No se ha encontrado la función {called}\nEn la función {name}')
                else:
                    # Detect circular reference
                    in_iter.add(called)
                    if name in d:
                        termutils.error(f'Se ha detectado una referencia circular en la función {name}\nStack: {in_iter}')
                    else:
                        recursive(name, functions[called], in_iter)
        return in_iter

    generators = {}
    for fun in functions:
        # Check for circular dependencies and get dependencies for each function 
        recursive(fun, functions[fun], {})
        # Create generator
        def generator(fun, functions):
            for ins in functions[fun]:
                opcode = list(ins.keys())[0]
                if opcode == 'call':
                    yield from generator(ins[opcode], functions)
                else:
                    yield ins 

        generators[fun] = lambda fun=fun, functions=functions: generator(fun, functions)
    
    for gen in generators:
        generators[gen] = generators[gen]()
    return generators
        

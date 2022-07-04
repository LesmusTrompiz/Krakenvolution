from copyreg import constructor
from subprocess import call


def compile(fun):
    print(fun)
    for ins in fun:
        print(ins)

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
            print('No se ha especificado el tamaño para el bucle ' + str(function) + ' por defecto será 0')
        
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
                    print(f'No se ha encontrado la función {ins["call"]} referenciada en {fun}')
                    exit(1)
                else:
                    # Detect circular reference
                    in_iter.add(called)
                    if name in d:
                        print('Se ha detectado una referencia circular')
                        exit(1)
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
        print(gen)
        generators[gen] = generators[gen]()
    return generators
        
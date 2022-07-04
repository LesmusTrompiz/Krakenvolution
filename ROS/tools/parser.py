import load
import compiler

BANNER ='\
    ____ \n\
   / __ \_________ ________  ______  ____ ______________  _____ \n\
  / / / / ___/ __ `/ ___/ / / / __ \/ __ `/ ___/ ___/ _ \/ ___/ \n\
 / /_/ / /  / /_/ / /__/ /_/ / /_/ / /_/ / /  (__  )  __/ /    \n\
/_____/_/   \__,_/\___/\__,_/ .___/\__,_/_/  /____/\___/_/     \n\
                           /_/                                 '

# Language definition
KEYWORDS = [
    'call',
    'delay',
    'avanzar',
    'girar',
    'loop'
]

if __name__ == '__main__':
    print(BANNER + '\n\n')
    files = load.load_files()

    functions = {} 
    constants = {}
    # Search for all functions
    for file in files:
        if (file != None):
            for token in file:
                if token[0] != '$':
                    functions[token] = file[token]
                else:
                    constants[token.replace('$', '')] = file[token]

    print('\nSe han encontrado las siguientes funciones: \n')
    for fun in functions:
        print('\t' + fun)

    print('\nSe han encontrado las siguientes constantes: \n')
    for const in constants:
        print('\t' + const + ': ' + str(constants[const]))

    print('\n\n == INICIANDO COMPILACIÓN == \n')

    print('Comprobando sintaxis...')
    for function in functions:
        for index, ins in enumerate(functions[function]):
            opcode = list(ins.keys())[0]
            if not opcode in KEYWORDS and opcode[0] != '$':
                print(opcode + ' no es una palabra definida...')
                print('Error en: ' + function + ' instrucción ' + str(index))
                exit(1)


    print('Expandiendo bucles...')
    functions = compiler.expand_loops(functions)
    print('Expandiendo macros...')
    expanded = compiler.expand_macros(functions)

    compiled = {}
    for fun in expanded:
        # Prepare function for compiling process
        compiled[fun] = compiler.compile(expanded[fun])

    print('\n == COMPILACIÓN TERMINADA ==')
    print('Escribiendo en archivo...')

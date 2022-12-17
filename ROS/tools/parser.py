import load
import compiler
import termutils

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
    termutils.ok(f'{BANNER}\n\n')
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

    termutils.info('\nSe han encontrado las siguientes funciones\n')
    for fun in functions:
        print('\t' + fun)
    
    if len(constants) != 0:
        termutils.info('\nSe han encontrado las siguientes constantes\n')
        for const in constants:
            print(f'\t{const}: {constants[const]}')

    termutils.ok('\n\n == INICIANDO COMPILACIÓN == \n')

    termutils.info('Comprobando sinaxis...')
    for function in functions:
        for index, ins in enumerate(functions[function]):
            opcode = list(ins.keys())[0]
            if not opcode in KEYWORDS and opcode[0] != '$':
                termutils.error(f'{opcode} no es una palabra del lenguaje\nError en {function}:{index}')

    termutils.ok('OK')
    
    termutils.info('Expandiendo bucles...')
    functions = compiler.expand_loops(functions)
    termutils.ok('OK')
    termutils.info('Expandiendo macros...')
    expanded = compiler.expand_macros(functions)
    # Simplify structure
    expanded = dict([(fun, [(list(e.keys())[0], e[list(e.keys())[0]]) for e in expanded[fun]]) for fun in expanded])

    termutils.ok('OK')

    termutils.info('Pasando a binario...\n')
    compiled = {}
    for fun in expanded:
        # Prepare function for compiling process
        compiled[fun] = compiler.compile(fun, expanded[fun])

    termutils.ok('\n == COMPILACIÓN TERMINADA ==')
    termutils.info('Escribiendo en archivo...')

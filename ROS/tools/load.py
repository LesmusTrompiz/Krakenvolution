import termutils
import os
import yaml

def load_files():
    def recursive(dir, files_list):
        files = os.scandir(dir)
        for file in files:
            if file.is_dir():
                files_list = recursive(file.path, files_list)
            elif file.is_file():
                # Try to open the file
                try:
                    print('\t' + file.name)
                    files_list.append(yaml.safe_load(open(file.path)))
                except Exception as e:
                    print(e)
                    
                    termutils.error(f'No se ha podido abrir el archivo {file.path}')
        return files_list

    termutils.info('Se han encontrado los siguientes archivos:\n')
    return recursive('./secuencias', [])
    


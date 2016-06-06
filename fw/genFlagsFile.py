import sys
import json

flagsFile = 'compileFlags.json'

if __name__ == '__main__':
    with open(flagsFile,'w') as fileOut:
        json.dump(sys.argv[1:], fileOut)

import os
import random

def main():

    tests = []
    for root, dirs, files in os.walk("."):  
        for filename in files:
            if not filename[-1]=="n":
                pass
            
            print("processing: " +filename)  
            fin = open(filename,"r")
            for line in fin:
                if line[0]=="v":
                    continue
                tests.append(line)
            fin.close()
    fout = open("sc1_all.scen","a")
    fout.write("version 1\n")
    #for i in range(0, len(tests)//10):
     #   choice = random.randint(0,9)
     #fout.write(tests[i*10+choice])
    for i in range(0, len(tests)):
        fout.write(tests[i])    
    fout.close()
             
main()
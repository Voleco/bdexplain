
def main():
    for i in range(2,9):
        percent = i*5
        fout = open("random512-"+str(percent) +".map.scen","a")
        fout.write("version 1\n")
        for j in range(0,10):
            filename = "random512-" + str(percent)+ "-" + str(j) +".map.scen"
            print("reading: "+filename)
            fin = open(filename,"r")
            for line in fin:
                if line[0]=="v":
                    continue
                fout.write(line)
            fin.close()
        fout.close()
            
        
        
main()
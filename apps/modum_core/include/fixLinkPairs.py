import fileinput, shutil

def writeCode(fileName,emptyLinkPair,newLine):
    shutil.copy(fileName, fileName+".old")
    print "Opening file: " + fileName
    count = 0
    for line in fileinput.input(fileName,inplace=1):
        if line.find(emptyLinkPair) >= 0:
        		print newLine
        		count = count + 1
        else:
        	print line[:-1]
    print "replaced " + str(count) + " empty link pairs"
writeCode("modum_nottingham_map.xml", "<idfrom></idfrom>", "<idfrom>in</idfrom>")
writeCode("modum_nottingham_map.xml", "<idto></idto>", "<idto>out</idto>")

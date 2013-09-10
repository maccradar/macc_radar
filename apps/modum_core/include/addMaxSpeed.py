import fileinput, shutil

def writeCode(fileName,lineAbove,newLine):
    shutil.copy(fileName, fileName+".old")
    print "--------------------------------------------------"
    print "Opening file: " + fileName
    print "--------------------------------------------------"
    addedLine = False
    for line in fileinput.input(fileName,inplace=1):
        print line[:-1]
        found = True
        for la in lineAbove:
            if line.find(la) >= 0:
                found = found and True
            else:
                found = False
        
        if found:
            print newLine
            addedLine = True

    if addedLine:
        print "--------------------------------------------------"
        print "Found tag: "
        print lineAbove
        print "Added line: " + newLine
        print "--------------------------------------------------"
    else:
        print "--------------------------------------------------"
        print "Error finding tag:"
        print lineAbove
        print "--------------------------------------------------"

writeCode("modum_nottingham_map.xml", ["<link>"], "<maxSpeed>50</maxSpeed>")

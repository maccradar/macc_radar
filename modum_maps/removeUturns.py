import xml.etree.ElementTree as ET 

def indent(elem, level=0):
    i = "\n" + level*"  "
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = i + "  "
        if not elem.tail or not elem.tail.strip():
            elem.tail = i
        for elem in elem:
            indent(elem, level+1)
        if not elem.tail or not elem.tail.strip():
            elem.tail = i
    else:
        if level and (not elem.tail or not elem.tail.strip()):
            elem.tail = i

def removeUturn(xml, outputfile):
    xmltree = ET.parse(xml)
    xmlroot = xmltree.getroot()
    i= 0
    for xmllink in xmlroot.iter('connection'):
        if xmllink.get('from').translate(None,'-') == xmllink.get('to').translate(None,'-'):
            xmlroot.remove(xmllink)
            i = i+1
    print 'Removed ' + str(i) + ' U-turns!'
    indent(xmlroot)
    xmltree.write(outputfile)
    print 'Done!'
    
removeUturn('nottingham-center-november.net.xml', 'output.net.xml')

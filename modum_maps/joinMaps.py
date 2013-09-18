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

def joinMaps(osm, xml, outputfile):
    print 'Parsing OSM data...'
    osmtree = ET.parse(osm)
    print 'Done!'
    osmroot = osmtree.getroot()
    osmdict = {}
    xmltree = ET.parse(xml)
    xmlroot = xmltree.getroot()
    
    print 'Creating node dictionary...'
    for osmnode in osmroot.iter('node'):
        osmdict[osmnode.get('id')]= osmnode.get('lat'), osmnode.get('lon')
    print 'Done!'
    linkdict = {}
    print 'Creating link dictionary...'
    for w in osmroot.iter('way'):
        ns = w.findall("./tag[@k='name']")
        rs = w.findall("./tag[@k='highway']")
        if len(ns) == 1 and len(rs) == 1:
            linkdict[w.get('id')]= ns[0].get('v'), rs[0].get('v')
        if len(ns) == 0 and len(rs) == 1:
            linkdict[w.get('id')]= rs[0].get('v')+ '_' + w.get('id'), rs[0].get('v')
    print 'Done!'           
    print len(linkdict)
    gpsdict = {}
    print 'Creating gps dictionary...'
    for xmllink in xmlroot.iter('link'):
        gps = xmllink.find('GPSshape').text
        latlons = gps.split(' ')
        lllist = []
        for ll in latlons:
            lon = ll.split(',')[0]
            lat = ll.split(',')[1]
            lllist.append((lat,lon))
        gpsdict[xmllink.find('id').text] = lllist
    print 'Done!'

    print len(gpsdict)
    
    print 'Looking up nodes...'
    for xmlnode in xmlroot.iter('node'):
        coordEl = ET.SubElement(xmlnode, 'coordinates')
        latEl = ET.SubElement(coordEl,'lat')
        lonEl = ET.SubElement(coordEl,'lon')
        nodeID = xmlnode.find('id').text            
        if nodeID.startswith('cluster'):
            nodeID = nodeID.split('_')[1]
        if osmdict.has_key(nodeID):
            #print 'Found node ' + nodeID + ' with lat/lon ' + str(osmdict[nodeID])
            latEl.text = osmdict[nodeID][0]
            lonEl.text = osmdict[nodeID][1]
        else:
            print 'Did not find node ' + nodeID
            latEl.text = 'undefined'
            lonEl.text = 'undefined'
        links = []
        for lp in xmlnode.iter('linkPair'):
            idFrom = lp.find('idfrom').text.split('#')[0].translate(None,'-')
            idTo = lp.find('idto').text.split('#')[0].translate(None,'-')
            if linkdict.has_key(idFrom):
                links.append(linkdict[idFrom][0])
            if linkdict.has_key(idTo):
                links.append(linkdict[idTo][0])
        data = set(links)
        sortedData = sorted(data, key=lambda item: (int(item.partition(' ')[0]) if item[0].isdigit() else float('inf'), item))
        nodeDesc = ""
        if len(sortedData) > 0:
            for street in sortedData[:-1]:
                nodeDesc = nodeDesc + street + " and "
            nodeDesc = nodeDesc + sortedData[-1]
        xmlnode.find('nodeDesc').text = nodeDesc
    print 'Done!'
    for xmllink in xmlroot.iter('link'):
        origID = xmllink.find('id').text
        linkID = origID.split('#')[0].translate(None,'-')
        descEl = ET.SubElement(xmllink, 'linkDesc')
        typeEl = ET.SubElement(xmllink, 'roadType')
        
        if linkdict.has_key(linkID):    
            descEl.text = linkdict[linkID][0]
            typeEl.text = linkdict[linkID][1]
        else:
            print 'Did not find link ' + linkID
            descEl.text = "unknown street"
        
        if gpsdict.has_key(origID):
            for ll in gpsdict[origID]:
                coordEl = ET.SubElement(xmllink, 'coordinates')
                latEl = ET.SubElement(coordEl,'lat')
                lonEl = ET.SubElement(coordEl,'lon')
                latEl.text = ll[0]
                lonEl.text = ll[1]
        else:
            print 'Did not find gps coord for link ' + origID
            latEl = ET.SubElement(coordEl,'lat')
            lonEl = ET.SubElement(coordEl,'lon')
            latEl.text = "undefined"
            lonEl.text = "undefined"
            
    print 'Done!'
    print 'Writing output file...'
    
    indent(xmlroot)
    xmltree.write(outputfile)
    print 'Done!'
    
joinMaps('nottinghamshire-justnottingham.osm', 'modum_nottingham_map.new', 'output.xml')
